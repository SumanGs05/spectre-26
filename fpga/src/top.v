/**
 * UAV-AudioLoc — Top-level module for Tang Nano 9K
 *
 * I2S master → Sipeed 6+1 mic array → mu-law → TDM frames → UART to Pi.
 * Single time-multiplexed mu-law encoder. No RAM — registers only.
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
    output wire uart_tx
);

    // ---------------------------------------------------------------
    // I2S Receiver (master): generates CK/WS, deserializes 7ch PCM
    // ---------------------------------------------------------------
    wire [15:0] pcm_ch [0:6];
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
        .pcm_ch0   (pcm_ch[0]),
        .pcm_ch1   (pcm_ch[1]),
        .pcm_ch2   (pcm_ch[2]),
        .pcm_ch3   (pcm_ch[3]),
        .pcm_ch4   (pcm_ch[4]),
        .pcm_ch5   (pcm_ch[5]),
        .pcm_ch6   (pcm_ch[6]),
        .pcm_valid (pcm_valid)
    );

    // ---------------------------------------------------------------
    // Sample Sync (re-latch for clean timing)
    // ---------------------------------------------------------------
    wire [15:0] sync_pcm [0:6];
    wire        sync_valid;

    sample_sync u_sync (
        .clk        (clk_27m),
        .rst_n      (rst_n),
        .pcm_in_0   (pcm_ch[0]),
        .pcm_in_1   (pcm_ch[1]),
        .pcm_in_2   (pcm_ch[2]),
        .pcm_in_3   (pcm_ch[3]),
        .pcm_in_4   (pcm_ch[4]),
        .pcm_in_5   (pcm_ch[5]),
        .pcm_in_6   (pcm_ch[6]),
        .valid_in   (pcm_valid),
        .pcm_out_0  (sync_pcm[0]),
        .pcm_out_1  (sync_pcm[1]),
        .pcm_out_2  (sync_pcm[2]),
        .pcm_out_3  (sync_pcm[3]),
        .pcm_out_4  (sync_pcm[4]),
        .pcm_out_5  (sync_pcm[5]),
        .pcm_out_6  (sync_pcm[6]),
        .valid_out  (sync_valid)
    );

    // ---------------------------------------------------------------
    // Time-multiplexed Mu-Law Encoder (1 encoder, 7 cycles)
    // ---------------------------------------------------------------
    reg [15:0] hold [0:6];
    reg [7:0]  mreg [0:6];
    reg [2:0]  enc_idx;
    reg        enc_busy;
    reg        mulaw_done;

    wire [15:0] pcm_enc_in =
        (enc_idx == 3'd0) ? hold[0] :
        (enc_idx == 3'd1) ? hold[1] :
        (enc_idx == 3'd2) ? hold[2] :
        (enc_idx == 3'd3) ? hold[3] :
        (enc_idx == 3'd4) ? hold[4] :
        (enc_idx == 3'd5) ? hold[5] : hold[6];

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
        end else begin
            mulaw_done <= 1'b0;

            if (sync_valid && !enc_busy) begin
                hold[0] <= sync_pcm[0];
                hold[1] <= sync_pcm[1];
                hold[2] <= sync_pcm[2];
                hold[3] <= sync_pcm[3];
                hold[4] <= sync_pcm[4];
                hold[5] <= sync_pcm[5];
                hold[6] <= sync_pcm[6];
                enc_busy <= 1'b1;
                enc_idx  <= 3'd0;
            end else if (enc_busy) begin
                mreg[enc_idx] <= mulaw_byte;
                if (enc_idx == 3'd6) begin
                    enc_busy   <= 1'b0;
                    mulaw_done <= 1'b1;
                end else begin
                    enc_idx <= enc_idx + 3'd1;
                end
            end
        end
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
        .sample_valid (mulaw_done),
        .ch0          (mreg[0]),
        .ch1          (mreg[1]),
        .ch2          (mreg[2]),
        .ch3          (mreg[3]),
        .ch4          (mreg[4]),
        .ch5          (mreg[5]),
        .ch6          (mreg[6]),
        .tx_byte      (tdm_byte),
        .tx_valid     (tdm_valid),
        .frame_done   (tdm_frame_done)
    );

    // ---------------------------------------------------------------
    // UART TX (2 Mbaud, 8N1)
    // ---------------------------------------------------------------
    uart_tx u_uart (
        .clk        (clk_27m),
        .rst_n      (rst_n),
        .data       (tdm_byte),
        .data_valid (tdm_valid),
        .tx         (uart_tx),
        .ready      (uart_ready)
    );

endmodule
