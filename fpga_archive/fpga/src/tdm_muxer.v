/**
 * TDM Frame Muxer — streams 32 samples into a TDM frame with zero RAM.
 *
 * Frame format:
 *   [0xAA] [0x55] [32 x (CH0..CH6)] [CRC16_HI] [CRC16_LO]
 *   228 bytes per frame, 500 frames/sec at 16 kHz / 32 samples
 *
 * Each sample set (7 mu-law bytes) is latched from inputs on sample_valid,
 * then sent byte-by-byte over UART. No RAM buffers — just 7 hold registers.
 */

module tdm_muxer (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_ready,
    input  wire       sample_valid,
    input  wire [7:0] ch0, ch1, ch2, ch3, ch4, ch5, ch6,
    output reg  [7:0] tx_byte,
    output reg        tx_valid,
    output reg        frame_done
);

    localparam SYNC_HI = 8'hAA;
    localparam SYNC_LO = 8'h55;

    reg [7:0] h0, h1, h2, h3, h4, h5, h6;
    reg [4:0] sample_count;
    reg [2:0] ch_idx;
    reg [15:0] crc;

    localparam S_IDLE         = 3'd0;
    localparam S_SYNC_HI      = 3'd1;
    localparam S_SYNC_LO      = 3'd2;
    localparam S_SEND_CH      = 3'd3;
    localparam S_WAIT_SAMPLE  = 3'd4;
    localparam S_CRC_HI       = 3'd5;
    localparam S_CRC_LO       = 3'd6;

    reg [2:0] state;

    wire [7:0] ch_data =
        (ch_idx == 3'd0) ? h0 :
        (ch_idx == 3'd1) ? h1 :
        (ch_idx == 3'd2) ? h2 :
        (ch_idx == 3'd3) ? h3 :
        (ch_idx == 3'd4) ? h4 :
        (ch_idx == 3'd5) ? h5 : h6;

    reg [7:0]  crc_din;
    reg        crc_en;
    wire [15:0] crc_next;

    crc16_ccitt u_crc (
        .crc_in  (crc),
        .data    (crc_din),
        .crc_out (crc_next)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            sample_count <= 5'd0;
            ch_idx       <= 3'd0;
            crc          <= 16'hFFFF;
            tx_byte      <= 8'd0;
            tx_valid     <= 1'b0;
            frame_done   <= 1'b0;
            crc_en       <= 1'b0;
            crc_din      <= 8'd0;
            h0 <= 8'd0; h1 <= 8'd0; h2 <= 8'd0; h3 <= 8'd0;
            h4 <= 8'd0; h5 <= 8'd0; h6 <= 8'd0;
        end else begin
            tx_valid   <= 1'b0;
            frame_done <= 1'b0;

            if (crc_en) begin
                crc    <= crc_next;
                crc_en <= 1'b0;
            end

            case (state)
                S_IDLE: begin
                    if (sample_valid) begin
                        h0 <= ch0; h1 <= ch1; h2 <= ch2; h3 <= ch3;
                        h4 <= ch4; h5 <= ch5; h6 <= ch6;
                        sample_count <= 5'd0;
                        crc          <= 16'hFFFF;
                        state        <= S_SYNC_HI;
                    end
                end

                S_SYNC_HI: begin
                    if (uart_ready) begin
                        tx_byte <= SYNC_HI;
                        tx_valid <= 1'b1;
                        crc_din <= SYNC_HI;
                        crc_en  <= 1'b1;
                        state   <= S_SYNC_LO;
                    end
                end

                S_SYNC_LO: begin
                    if (uart_ready) begin
                        tx_byte <= SYNC_LO;
                        tx_valid <= 1'b1;
                        crc_din <= SYNC_LO;
                        crc_en  <= 1'b1;
                        ch_idx  <= 3'd0;
                        state   <= S_SEND_CH;
                    end
                end

                S_SEND_CH: begin
                    if (uart_ready) begin
                        tx_byte  <= ch_data;
                        tx_valid <= 1'b1;
                        crc_din  <= ch_data;
                        crc_en   <= 1'b1;

                        if (ch_idx == 3'd6) begin
                            if (sample_count == 5'd31) begin
                                state <= S_CRC_HI;
                            end else begin
                                sample_count <= sample_count + 5'd1;
                                state        <= S_WAIT_SAMPLE;
                            end
                        end else begin
                            ch_idx <= ch_idx + 3'd1;
                        end
                    end
                end

                S_WAIT_SAMPLE: begin
                    if (sample_valid) begin
                        h0 <= ch0; h1 <= ch1; h2 <= ch2; h3 <= ch3;
                        h4 <= ch4; h5 <= ch5; h6 <= ch6;
                        ch_idx <= 3'd0;
                        state  <= S_SEND_CH;
                    end
                end

                S_CRC_HI: begin
                    if (uart_ready) begin
                        tx_byte  <= crc[15:8];
                        tx_valid <= 1'b1;
                        state    <= S_CRC_LO;
                    end
                end

                S_CRC_LO: begin
                    if (uart_ready) begin
                        tx_byte    <= crc[7:0];
                        tx_valid   <= 1'b1;
                        frame_done <= 1'b1;
                        state      <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

/**
 * CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF.
 * Purely combinational, one byte at a time.
 */
module crc16_ccitt (
    input  wire [15:0] crc_in,
    input  wire [7:0]  data,
    output wire [15:0] crc_out
);
    wire [15:0] c0  = crc_in ^ {data, 8'd0};
    wire [15:0] c1  = c0[15] ? {c0[14:0], 1'b0} ^ 16'h1021 : {c0[14:0], 1'b0};
    wire [15:0] c2  = c1[15] ? {c1[14:0], 1'b0} ^ 16'h1021 : {c1[14:0], 1'b0};
    wire [15:0] c3  = c2[15] ? {c2[14:0], 1'b0} ^ 16'h1021 : {c2[14:0], 1'b0};
    wire [15:0] c4  = c3[15] ? {c3[14:0], 1'b0} ^ 16'h1021 : {c3[14:0], 1'b0};
    wire [15:0] c5  = c4[15] ? {c4[14:0], 1'b0} ^ 16'h1021 : {c4[14:0], 1'b0};
    wire [15:0] c6  = c5[15] ? {c5[14:0], 1'b0} ^ 16'h1021 : {c5[14:0], 1'b0};
    wire [15:0] c7  = c6[15] ? {c6[14:0], 1'b0} ^ 16'h1021 : {c6[14:0], 1'b0};
    assign crc_out = c7;
endmodule
