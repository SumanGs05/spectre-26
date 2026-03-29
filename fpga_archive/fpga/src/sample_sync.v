/**
 * Sample Synchronization — aligns 7 PCM channels.
 *
 * All seven PCM channels share one clock and one valid strobe from the
 * I2S receiver. This module latches all 7 channels simultaneously on valid_in
 * to guarantee sample alignment for downstream TDM muxing.
 */

module sample_sync (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [15:0] pcm_in_0,
    input  wire [15:0] pcm_in_1,
    input  wire [15:0] pcm_in_2,
    input  wire [15:0] pcm_in_3,
    input  wire [15:0] pcm_in_4,
    input  wire [15:0] pcm_in_5,
    input  wire [15:0] pcm_in_6,
    input  wire        valid_in,

    output reg  [15:0] pcm_out_0,
    output reg  [15:0] pcm_out_1,
    output reg  [15:0] pcm_out_2,
    output reg  [15:0] pcm_out_3,
    output reg  [15:0] pcm_out_4,
    output reg  [15:0] pcm_out_5,
    output reg  [15:0] pcm_out_6,
    output reg         valid_out
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pcm_out_0 <= 16'd0;
            pcm_out_1 <= 16'd0;
            pcm_out_2 <= 16'd0;
            pcm_out_3 <= 16'd0;
            pcm_out_4 <= 16'd0;
            pcm_out_5 <= 16'd0;
            pcm_out_6 <= 16'd0;
            valid_out <= 1'b0;
        end else begin
            valid_out <= 1'b0;
            if (valid_in) begin
                pcm_out_0 <= pcm_in_0;
                pcm_out_1 <= pcm_in_1;
                pcm_out_2 <= pcm_in_2;
                pcm_out_3 <= pcm_in_3;
                pcm_out_4 <= pcm_in_4;
                pcm_out_5 <= pcm_in_5;
                pcm_out_6 <= pcm_in_6;
                valid_out <= 1'b1;
            end
        end
    end

endmodule
