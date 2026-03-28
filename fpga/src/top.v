/**
 * UAV-AudioLoc — Top-level module for Tang Nano 9K
 *
 * I2S master → Sipeed 6+1 mic array → mu-law → TDM frames → UART to Pi.
 * Single time-multiplexed mu-law encoder. No RAM — registers only.
 *
 * led_prog: onboard LED (pin 10) toggles ~0.8 Hz when bitstream is running.
 *
 * UART_BAUD = 1,500,000 (27 MHz / 18, exact — 0% baud error).
 */

module top (
    input  wire clk_27m,
    input  wire rst_n,
    input  wire i2s_d0,
    input  wire i2s_d1,
    input  wire i2s_d2,
    input  wire i2s_d3,
    output wire i2s_ck,
    output wire i2s_ws,
    output wire uart_tx,
    output wire led_prog
);

    localparam UART_BAUD = 1_500_000;

    // ---------------------------------------------------------------
    // I2S Receiver (master): generates CK/WS, deserializes 7ch PCM
    // ---------------------------------------------------------------
    wire [15:0] pcm_ch0, pcm_ch1, pcm_ch2, pcm_ch3,
                pcm_ch4, pcm_ch5, pcm_ch6;
    wire        pcm_valid;

    i2s_receiver u_i2s (
        .clk       (clk_27m),
        .rst_n     (rst_n),
        .i2s_d0    (i2s_d0),
        .i2s_d1    (i2s_d1),
        .i2s_d2    (i2s_d2),
        .i2s_d3    (i2s_d3),
        .i2s_ck    (i2s_ck),
        .i2s_ws    (i2s_ws),
        .pcm_ch0   (pcm_ch0),
        .pcm_ch1   (pcm_ch1),
        .pcm_ch2   (pcm_ch2),
        .pcm_ch3   (pcm_ch3),
        .pcm_ch4   (pcm_ch4),
        .pcm_ch5   (pcm_ch5),
        .pcm_ch6   (pcm_ch6),
        .pcm_valid (pcm_valid)
    );

    // ---------------------------------------------------------------
    // Sample Sync (re-latch for clean timing)
    // ---------------------------------------------------------------
    wire [15:0] sync_pcm0, sync_pcm1, sync_pcm2, sync_pcm3,
                sync_pcm4, sync_pcm5, sync_pcm6;
    wire        sync_valid;

    sample_sync u_sync (
        .clk        (clk_27m),
        .rst_n      (rst_n),
        .pcm_in_0   (pcm_ch0),
        .pcm_in_1   (pcm_ch1),
        .pcm_in_2   (pcm_ch2),
        .pcm_in_3   (pcm_ch3),
        .pcm_in_4   (pcm_ch4),
        .pcm_in_5   (pcm_ch5),
        .pcm_in_6   (pcm_ch6),
        .valid_in   (pcm_valid),
        .pcm_out_0  (sync_pcm0),
        .pcm_out_1  (sync_pcm1),
        .pcm_out_2  (sync_pcm2),
        .pcm_out_3  (sync_pcm3),
        .pcm_out_4  (sync_pcm4),
        .pcm_out_5  (sync_pcm5),
        .pcm_out_6  (sync_pcm6),
        .valid_out  (sync_valid)
    );

    // ---------------------------------------------------------------
    // Time-multiplexed Mu-Law Encoder (1 encoder, 7 cycles)
    // ---------------------------------------------------------------
    reg [15:0] hold0, hold1, hold2, hold3, hold4, hold5, hold6;
    reg [7:0]  mreg0, mreg1, mreg2, mreg3, mreg4, mreg5, mreg6;
    reg [2:0]  enc_idx;
    reg        enc_busy;
    reg        mulaw_done;

    wire [15:0] pcm_enc_in =
        (enc_idx == 3'd0) ? hold0 :
        (enc_idx == 3'd1) ? hold1 :
        (enc_idx == 3'd2) ? hold2 :
        (enc_idx == 3'd3) ? hold3 :
        (enc_idx == 3'd4) ? hold4 :
        (enc_idx == 3'd5) ? hold5 : hold6;

    wire [7:0] mulaw_byte;

    mulaw_encoder u_mulaw (
        .pcm_in    (pcm_enc_in),
        .mulaw_out (mulaw_byte)
    );

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n) begin
            enc_idx    <= 3'd0;
            enc_busy   <= 1'b0;
            mulaw_done <= 1'b0;
            hold0 <= 16'd0; hold1 <= 16'd0; hold2 <= 16'd0; hold3 <= 16'd0;
            hold4 <= 16'd0; hold5 <= 16'd0; hold6 <= 16'd0;
            mreg0 <= 8'd0; mreg1 <= 8'd0; mreg2 <= 8'd0; mreg3 <= 8'd0;
            mreg4 <= 8'd0; mreg5 <= 8'd0; mreg6 <= 8'd0;
        end else begin
            mulaw_done <= 1'b0;

            if (sync_valid && !enc_busy) begin
                hold0 <= sync_pcm0; hold1 <= sync_pcm1;
                hold2 <= sync_pcm2; hold3 <= sync_pcm3;
                hold4 <= sync_pcm4; hold5 <= sync_pcm5;
                hold6 <= sync_pcm6;
                enc_busy <= 1'b1;
                enc_idx  <= 3'd0;
            end else if (enc_busy) begin
                case (enc_idx)
                    3'd0: mreg0 <= mulaw_byte;
                    3'd1: mreg1 <= mulaw_byte;
                    3'd2: mreg2 <= mulaw_byte;
                    3'd3: mreg3 <= mulaw_byte;
                    3'd4: mreg4 <= mulaw_byte;
                    3'd5: mreg5 <= mulaw_byte;
                    3'd6: mreg6 <= mulaw_byte;
                    default: ;
                endcase
                if (enc_idx == 3'd6) begin
                    enc_busy   <= 1'b0;
                    mulaw_done <= 1'b1;
                end else begin
                    enc_idx <= enc_idx + 3'd1;
                end
            end
        end
    end

    reg mulaw_done_d1;
    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n)
            mulaw_done_d1 <= 1'b0;
        else
            mulaw_done_d1 <= mulaw_done;
    end

    // ---------------------------------------------------------------
    // TDM Frame Muxer (streams directly, no RAM)
    // ---------------------------------------------------------------
    wire [7:0] tdm_byte;
    wire       tdm_valid;
    wire       tdm_frame_done;
    wire       uart_ready;

    tdm_muxer u_tdm (
        .clk          (clk_27m),
        .rst_n        (rst_n),
        .uart_ready   (uart_ready),
        .sample_valid (mulaw_done_d1),
        .ch0          (mreg0),
        .ch1          (mreg1),
        .ch2          (mreg2),
        .ch3          (mreg3),
        .ch4          (mreg4),
        .ch5          (mreg5),
        .ch6          (mreg6),
        .tx_byte      (tdm_byte),
        .tx_valid     (tdm_valid),
        .frame_done   (tdm_frame_done)
    );

    // ---------------------------------------------------------------
    // UART TX (8N1, 1.5 Mbaud — exact from 27 MHz)
    // ---------------------------------------------------------------
    uart_tx #(
        .BAUD_RATE(UART_BAUD)
    ) u_uart (
        .clk        (clk_27m),
        .rst_n      (rst_n),
        .data       (tdm_byte),
        .data_valid (tdm_valid),
        .tx         (uart_tx),
        .ready      (uart_ready)
    );

    // Heartbeat: ~0.8 Hz toggle @ 27 MHz
    reg [24:0] hb_cnt;
    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n)
            hb_cnt <= 25'd0;
        else
            hb_cnt <= hb_cnt + 25'd1;
    end
    assign led_prog = hb_cnt[24];

endmodule
