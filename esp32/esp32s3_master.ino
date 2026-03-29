/**
 * UAV-AudioLoc — ESP32-S3 #1 (MASTER)
 * Board  : Seeed XIAO ESP32-S3
 * Mics   : Sipeed 6+1 actual pinout:
 *            MIC_D0 → mic0(L) / mic1(R)
 *            MIC_D1 → mic2(L) / mic3(R)
 *            MIC_D2 → mic4(L) / mic5(R)
 *            MIC_D3 → mic6(L) / empty(R)   ← centre mic
 *
 * Master captures: MIC_D0 → mic0, mic1  |  MIC_D1 → mic2, mic3
 * Channels captured: 0, 1, 2, 3  (4 channels)
 * Reference: mic6 centre — provided by Slave over sync (use local noise proxy)
 *
 * Filter : LMS (ego-noise reference = mic2, closest to centre on this board) +
 *          Spectral Subtraction (hover-band mask)
 * Output : UART @ 1,500,000 baud → Pi 4B
 *          Frame: [0xAA][0x55][board_id=0x01][ch=4][N*4 bytes PCM32][CRC16 2B]
 * Sync   : GPIO2 output — rising pulse at every DMA callback → Slave
 *
 * Wiring (XIAO ESP32-S3 pin labels):
 *   MIC_CK  → D7  (GPIO7)   BCLK
 *   MIC_WS  → D8  (GPIO8)   LRCLK — also wire to Slave D8
 *   MIC_D0  → D9  (GPIO9)   mic0(L) / mic1(R)
 *   MIC_D1  → D10 (GPIO10)  mic2(L) / mic3(R)
 *   SYNC    → D2  (GPIO2)   OUTPUT → Slave GPIO2
 *   TX      → D43 (GPIO43)  UART0 TX → Pi /dev/ttyUSB0
 *   GND, VIN connected
 *
 * MIC_CK and MIC_WS also wire to Slave board (shared clock).
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define I2S_BCLK       7
#define I2S_LRCLK      8
#define I2S_DATA0      9    // mic0(L) mic1(R)
#define I2S_DATA1      10   // mic2(L) only
#define SYNC_PIN       2    // GPIO output → Slave
#define UART_TX_PIN    43

// ── Audio config ─────────────────────────────────────────────────────────────
#define SAMPLE_RATE    22050
#define DMA_BUF_COUNT  4
#define DMA_BUF_LEN    256   // samples per DMA buffer per channel
#define NUM_CH         4     // mic0, mic1, mic2, mic3
#define REF_CH         2     // mic2(L of D1) used as ego-noise proxy on this board

// ── UART / framing ───────────────────────────────────────────────────────────
#define UART_BAUD      1500000
#define BOARD_ID       0x01
#define FRAME_MAGIC_0  0xAA
#define FRAME_MAGIC_1  0x55

// ── LMS config ───────────────────────────────────────────────────────────────
#define LMS_TAPS       64
#define LMS_MU         0.005f   // step size — tune if over/under-correcting

// ── Spectral Subtraction config ───────────────────────────────────────────────
#define FFT_SIZE       256
#define HOVER_FREQ_HZ  180     // dominant motor harmonic — adjust per drone
#define SS_ALPHA       2.0f    // over-subtraction factor
#define SS_BETA        0.01f   // spectral floor

// ── Globals ──────────────────────────────────────────────────────────────────
static int32_t  raw[NUM_CH][DMA_BUF_LEN];
static float    filtered[NUM_CH][DMA_BUF_LEN];

// LMS state for mic0, mic1, mic3 (not mic2 ref)
static float    lms_w[3][LMS_TAPS];
static float    lms_x[3][LMS_TAPS];

// Spectral subtraction noise estimate (magnitude spectrum)
static float    noise_est[FFT_SIZE / 2 + 1];
static bool     noise_calibrated = false;
static uint32_t noise_frames     = 0;
#define NOISE_CAL_FRAMES 50   // collect 50 frames before subtracting

// ── CRC16-CCITT ──────────────────────────────────────────────────────────────
uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// ── LMS filter (single sample) ───────────────────────────────────────────────
// ch_idx: 0 or 1  (ch2 is reference, not filtered)
float lms_update(int ch_idx, float input, float reference) {
    // shift delay line
    memmove(&lms_x[ch_idx][1], &lms_x[ch_idx][0], (LMS_TAPS - 1) * sizeof(float));
    lms_x[ch_idx][0] = reference;

    // estimate noise
    float y = 0.0f;
    for (int i = 0; i < LMS_TAPS; i++)
        y += lms_w[ch_idx][i] * lms_x[ch_idx][i];

    // error = clean signal estimate
    float e = input - y;

    // NLMS weight update (normalised for stability)
    float norm = 0.0f;
    for (int i = 0; i < LMS_TAPS; i++)
        norm += lms_x[ch_idx][i] * lms_x[ch_idx][i];
    norm += 1e-6f;

    for (int i = 0; i < LMS_TAPS; i++)
        lms_w[ch_idx][i] += (LMS_MU / norm) * e * lms_x[ch_idx][i];

    return e;
}

// ── Minimal DFT for spectral subtraction (Goertzel per bin not full FFT) ─────
// We only need magnitude at hover harmonics — Goertzel is cheap on ESP32-S3
float goertzel_mag(const float *buf, int N, float target_freq, float sample_rate) {
    float k     = target_freq * N / sample_rate;
    float omega = 2.0f * M_PI * k / N;
    float coeff = 2.0f * cosf(omega);
    float s0 = 0, s1 = 0, s2 = 0;
    for (int i = 0; i < N; i++) {
        s0 = buf[i] + coeff * s1 - s2;
        s2 = s1; s1 = s0;
    }
    return sqrtf(s2 * s2 + s1 * s1 - coeff * s1 * s2);
}

// ── Spectral subtraction (time-domain approximation via gain) ─────────────────
// Estimates noise power at hover harmonics, applies suppression gain per frame
void spectral_subtract(float *buf, int N, float noise_mag) {
    float signal_power = 0.0f;
    for (int i = 0; i < N; i++)
        signal_power += buf[i] * buf[i];
    signal_power /= N;

    float noise_power  = noise_mag * noise_mag;
    float clean_power  = signal_power - SS_ALPHA * noise_power;
    if (clean_power < SS_BETA * signal_power)
        clean_power = SS_BETA * signal_power;

    float gain = (signal_power > 1e-12f) ? sqrtf(clean_power / signal_power) : 1.0f;
    for (int i = 0; i < N; i++)
        buf[i] *= gain;
}

// ── I2S init ─────────────────────────────────────────────────────────────────
void i2s_init() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = true,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };

    i2s_pin_config_t pins = {
        .bck_io_num   = I2S_BCLK,
        .ws_io_num    = I2S_LRCLK,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = I2S_DATA0
    };

    i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pins);

    // Second I2S port for DATA1 (mic2)
    i2s_config_t cfg1 = cfg;
    cfg1.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);  // slave to same BCLK/WS

    i2s_pin_config_t pins1 = {
        .bck_io_num   = I2S_BCLK,
        .ws_io_num    = I2S_LRCLK,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = I2S_DATA1
    };

    i2s_driver_install(I2S_NUM_1, &cfg1, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &pins1);
}

// ── Read I2S into raw buffers ─────────────────────────────────────────────────
void read_i2s() {
    size_t bytes_read = 0;

    // I2S_NUM_0: MIC_D0 → mic0(L), mic1(R)
    int32_t buf0[DMA_BUF_LEN * 2];
    i2s_read(I2S_NUM_0, buf0, sizeof(buf0), &bytes_read, portMAX_DELAY);
    for (int i = 0; i < DMA_BUF_LEN; i++) {
        raw[0][i] = buf0[i * 2];       // mic0 L
        raw[1][i] = buf0[i * 2 + 1];   // mic1 R
    }

    // I2S_NUM_1: MIC_D1 → mic2(L), mic3(R)
    int32_t buf1[DMA_BUF_LEN * 2];
    i2s_read(I2S_NUM_1, buf1, sizeof(buf1), &bytes_read, portMAX_DELAY);
    for (int i = 0; i < DMA_BUF_LEN; i++) {
        raw[2][i] = buf1[i * 2];       // mic2 L — used as LMS reference
        raw[3][i] = buf1[i * 2 + 1];   // mic3 R
    }
}

// ── Build and send UART frame ─────────────────────────────────────────────────
void send_frame() {
    // Header: AA 55 board_id ch_count
    const uint8_t hdr[4] = { FRAME_MAGIC_0, FRAME_MAGIC_1, BOARD_ID, NUM_CH };
    Serial.write(hdr, 4);

    // Payload: NUM_CH * DMA_BUF_LEN * 4 bytes (int32)
    const size_t payload_bytes = NUM_CH * DMA_BUF_LEN * sizeof(int32_t);
    uint8_t payload[payload_bytes];

    // Interleave channels: [ch0_s0, ch1_s0, ch2_s0, ch0_s1, ...]
    for (int s = 0; s < DMA_BUF_LEN; s++) {
        for (int c = 0; c < NUM_CH; c++) {
            int32_t sample = (int32_t)filtered[c][s];
            memcpy(&payload[(s * NUM_CH + c) * 4], &sample, 4);
        }
    }

    Serial.write(payload, payload_bytes);

    // CRC over payload only
    uint16_t crc = crc16(payload, payload_bytes);
    Serial.write((uint8_t)(crc >> 8));
    Serial.write((uint8_t)(crc & 0xFF));
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(UART_BAUD);
    pinMode(SYNC_PIN, OUTPUT);
    digitalWrite(SYNC_PIN, LOW);

    memset(lms_w, 0, sizeof(lms_w));
    memset(lms_x, 0, sizeof(lms_x));
    memset(noise_est, 0, sizeof(noise_est));

    i2s_init();
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
    // 1. Read from I2S
    read_i2s();

    // 2. Fire sync pulse → Slave captures simultaneously
    digitalWrite(SYNC_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(SYNC_PIN, LOW);

    // 3. Convert to float, normalise to [-1.0, 1.0]
    float ref[DMA_BUF_LEN];
    for (int i = 0; i < DMA_BUF_LEN; i++) {
        ref[i] = (float)raw[REF_CH][i] / (float)INT32_MAX;
        for (int c = 0; c < NUM_CH; c++)
            filtered[c][i] = (float)raw[c][i] / (float)INT32_MAX;
    }

    // 4. LMS — apply to mic0, mic1, mic3 using mic2 as noise reference
    for (int i = 0; i < DMA_BUF_LEN; i++) {
        filtered[0][i] = lms_update(0, filtered[0][i], ref[i]);  // mic0
        filtered[1][i] = lms_update(1, filtered[1][i], ref[i]);  // mic1
        filtered[3][i] = lms_update(2, filtered[3][i], ref[i]);  // mic3
        // ch2 (mic2 reference) passes through unchanged
    }

    // 5. Spectral subtraction — noise calibration then suppression
    float hover_mag = goertzel_mag(ref, DMA_BUF_LEN, HOVER_FREQ_HZ, SAMPLE_RATE);

    if (!noise_calibrated) {
        // Accumulate noise estimate during first N frames (hover, no speech)
        noise_est[0] = (noise_est[0] * noise_frames + hover_mag) / (noise_frames + 1);
        noise_frames++;
        if (noise_frames >= NOISE_CAL_FRAMES) noise_calibrated = true;
    } else {
        // Slowly update noise estimate (track motor speed changes)
        noise_est[0] = 0.95f * noise_est[0] + 0.05f * hover_mag;

        // Apply suppression to mic0 and mic1
        spectral_subtract(filtered[0], DMA_BUF_LEN, noise_est[0]);  // mic0
        spectral_subtract(filtered[1], DMA_BUF_LEN, noise_est[0]);  // mic1
        spectral_subtract(filtered[3], DMA_BUF_LEN, noise_est[0]);  // mic3
    }

    // 6. Convert back to int32 and send frame
    for (int c = 0; c < NUM_CH; c++)
        for (int i = 0; i < DMA_BUF_LEN; i++)
            filtered[c][i] = filtered[c][i] * (float)INT32_MAX;

    send_frame();
}
