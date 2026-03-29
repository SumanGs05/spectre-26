/**
 * UAV-AudioLoc — ESP32-S3 #2 (SLAVE)
 * Board  : Seeed XIAO ESP32-S3
 * Mics   : Sipeed 6+1 actual pinout:
 *            MIC_D2 → mic4(L) / mic5(R)
 *            MIC_D3 → mic6(L) / empty(R)   ← centre mic (ego-noise reference)
 * Channels captured: 4, 5, 6  (3 channels)
 * Reference: mic6 (centre, MIC_D3 L channel) — true ego-noise reference
 *
 * Filter : LMS (ego-noise via mic6 centre reference) +
 *          Spectral Subtraction (hover-band mask)
 * Output : UART @ 1,500,000 baud → Pi 4B
 *          Frame: [0xAA][0x55][board_id=0x02][ch=3][N*4 bytes PCM32][CRC16 2B]
 * Sync   : GPIO2 INPUT — rising edge ISR from Master triggers capture
 *
 * Wiring (XIAO ESP32-S3 pin labels):
 *   MIC_CK  → D7  (GPIO7)   BCLK  shared from Master
 *   MIC_WS  → D8  (GPIO8)   LRCLK shared from Master
 *   MIC_D2  → D9  (GPIO9)   mic4(L) / mic5(R)
 *   MIC_D3  → D10 (GPIO10)  mic6(L) / empty(R)
 *   SYNC    → D2  (GPIO2)   INPUT ← Master GPIO2
 *   TX      → D43 (GPIO43)  UART0 TX → Pi /dev/ttyUSB1
 *   GND, VIN connected
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define I2S_BCLK       7
#define I2S_LRCLK      8
#define I2S_DATA2      9    // mic4(L) mic5(R)
#define I2S_DATA3      10   // mic6(L) centre ref / empty(R)
#define SYNC_PIN       2
#define UART_TX_PIN    43

// ── Audio config ─────────────────────────────────────────────────────────────
#define SAMPLE_RATE    22050
#define DMA_BUF_COUNT  4
#define DMA_BUF_LEN    256
#define NUM_CH         3    // mic4, mic5, mic6
#define REF_CH         2    // mic6 centre = index 2 in this board's array

// ── UART / framing ───────────────────────────────────────────────────────────
#define UART_BAUD      1500000
#define BOARD_ID       0x02
#define FRAME_MAGIC_0  0xAA
#define FRAME_MAGIC_1  0x55

// ── LMS config ───────────────────────────────────────────────────────────────
#define LMS_TAPS       64
#define LMS_MU         0.005f

// ── Spectral Subtraction config ───────────────────────────────────────────────
#define HOVER_FREQ_HZ  180
#define SS_ALPHA       2.0f
#define SS_BETA        0.01f

// ── Sync flag (set by ISR) ───────────────────────────────────────────────────
volatile bool sync_received = false;

// ── Globals ──────────────────────────────────────────────────────────────────
static int32_t  raw[NUM_CH][DMA_BUF_LEN];
static float    filtered[NUM_CH][DMA_BUF_LEN];

// LMS state for mic4, mic5 (not mic6 ref)
static float    lms_w[2][LMS_TAPS];
static float    lms_x[2][LMS_TAPS];

// Noise estimate
static float    noise_est    = 0.0f;
static bool     noise_calibrated = false;
static uint32_t noise_frames = 0;
#define NOISE_CAL_FRAMES 50

// ── Sync ISR ─────────────────────────────────────────────────────────────────
void IRAM_ATTR sync_isr() {
    sync_received = true;
}

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

// ── LMS filter ───────────────────────────────────────────────────────────────
float lms_update(int ch_idx, float input, float reference) {
    memmove(&lms_x[ch_idx][1], &lms_x[ch_idx][0], (LMS_TAPS - 1) * sizeof(float));
    lms_x[ch_idx][0] = reference;

    float y = 0.0f;
    for (int i = 0; i < LMS_TAPS; i++)
        y += lms_w[ch_idx][i] * lms_x[ch_idx][i];

    float e = input - y;

    float norm = 0.0f;
    for (int i = 0; i < LMS_TAPS; i++)
        norm += lms_x[ch_idx][i] * lms_x[ch_idx][i];
    norm += 1e-6f;

    for (int i = 0; i < LMS_TAPS; i++)
        lms_w[ch_idx][i] += (LMS_MU / norm) * e * lms_x[ch_idx][i];

    return e;
}

// ── Goertzel for hover harmonic magnitude ────────────────────────────────────
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

// ── Spectral subtraction (gain-based) ────────────────────────────────────────
void spectral_subtract(float *buf, int N, float noise_mag) {
    float signal_power = 0.0f;
    for (int i = 0; i < N; i++)
        signal_power += buf[i] * buf[i];
    signal_power /= N;

    float noise_power = noise_mag * noise_mag;
    float clean_power = signal_power - SS_ALPHA * noise_power;
    if (clean_power < SS_BETA * signal_power)
        clean_power = SS_BETA * signal_power;

    float gain = (signal_power > 1e-12f) ? sqrtf(clean_power / signal_power) : 1.0f;
    for (int i = 0; i < N; i++)
        buf[i] *= gain;
}

// ── I2S init (all slave — clock comes from Master board) ─────────────────────
void i2s_init() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = false,   // slave — no internal PLL needed
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };

    // Port 0: MIC_D2 → mic4(L) / mic5(R)
    i2s_pin_config_t pins0 = {
        .bck_io_num   = I2S_BCLK,
        .ws_io_num    = I2S_LRCLK,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = I2S_DATA2
    };
    i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pins0);

    // Port 1: MIC_D3 → mic6(L) centre ref / empty(R)
    i2s_pin_config_t pins1 = {
        .bck_io_num   = I2S_BCLK,
        .ws_io_num    = I2S_LRCLK,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = I2S_DATA3
    };
    i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &pins1);
}

// ── Read I2S into raw buffers ─────────────────────────────────────────────────
void read_i2s() {
    size_t bytes_read = 0;

    // Port 0: MIC_D2 → mic4(L), mic5(R)
    int32_t buf0[DMA_BUF_LEN * 2];
    i2s_read(I2S_NUM_0, buf0, sizeof(buf0), &bytes_read, portMAX_DELAY);
    for (int i = 0; i < DMA_BUF_LEN; i++) {
        raw[0][i] = buf0[i * 2];       // mic4 L
        raw[1][i] = buf0[i * 2 + 1];   // mic5 R
    }

    // Port 1: MIC_D3 → mic6(L) centre ref, empty(R)
    int32_t buf1[DMA_BUF_LEN * 2];
    i2s_read(I2S_NUM_1, buf1, sizeof(buf1), &bytes_read, portMAX_DELAY);
    for (int i = 0; i < DMA_BUF_LEN; i++)
        raw[2][i] = buf1[i * 2];       // mic6 L — centre reference
}

// ── Build and send UART frame ─────────────────────────────────────────────────
void send_frame() {
    const uint8_t hdr[4] = { FRAME_MAGIC_0, FRAME_MAGIC_1, BOARD_ID, NUM_CH };
    Serial.write(hdr, 4);

    const size_t payload_bytes = NUM_CH * DMA_BUF_LEN * sizeof(int32_t);
    uint8_t payload[payload_bytes];

    for (int s = 0; s < DMA_BUF_LEN; s++) {
        for (int c = 0; c < NUM_CH; c++) {
            int32_t sample = (int32_t)filtered[c][s];
            memcpy(&payload[(s * NUM_CH + c) * 4], &sample, 4);
        }
    }

    Serial.write(payload, payload_bytes);

    uint16_t crc = crc16(payload, payload_bytes);
    Serial.write((uint8_t)(crc >> 8));
    Serial.write((uint8_t)(crc & 0xFF));
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(UART_BAUD);
    pinMode(SYNC_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(SYNC_PIN), sync_isr, RISING);

    memset(lms_w, 0, sizeof(lms_w));
    memset(lms_x, 0, sizeof(lms_x));
    noise_est = 0.0f;

    i2s_init();
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
    // 1. Wait for sync pulse from Master
    while (!sync_received) yield();
    sync_received = false;

    // 2. Read I2S
    read_i2s();

    // 3. Normalise to float
    float ref[DMA_BUF_LEN];
    for (int i = 0; i < DMA_BUF_LEN; i++) {
        ref[i] = (float)raw[REF_CH][i] / (float)INT32_MAX;
        for (int c = 0; c < NUM_CH; c++)
            filtered[c][i] = (float)raw[c][i] / (float)INT32_MAX;
    }

    // 4. LMS on mic4, mic5 using mic6 centre as reference
    for (int i = 0; i < DMA_BUF_LEN; i++) {
        filtered[0][i] = lms_update(0, filtered[0][i], ref[i]);  // mic4
        filtered[1][i] = lms_update(1, filtered[1][i], ref[i]);  // mic5
        // filtered[2] = mic6 ref passes through
    }

    // 5. Spectral subtraction
    float hover_mag = goertzel_mag(ref, DMA_BUF_LEN, HOVER_FREQ_HZ, SAMPLE_RATE);

    if (!noise_calibrated) {
        noise_est = (noise_est * noise_frames + hover_mag) / (noise_frames + 1);
        noise_frames++;
        if (noise_frames >= NOISE_CAL_FRAMES) noise_calibrated = true;
    } else {
        noise_est = 0.95f * noise_est + 0.05f * hover_mag;
        spectral_subtract(filtered[0], DMA_BUF_LEN, noise_est);  // mic4
        spectral_subtract(filtered[1], DMA_BUF_LEN, noise_est);  // mic5
    }

    // 6. Convert back and send
    for (int c = 0; c < NUM_CH; c++)
        for (int i = 0; i < DMA_BUF_LEN; i++)
            filtered[c][i] = filtered[c][i] * (float)INT32_MAX;

    send_frame();
}
