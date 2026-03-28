/**
 * I2S master receiver for Sipeed 6+1 mic array (MSM261S4030H0R).
 *
 * FPGA drives MIC_CK (bit clock) and MIC_WS (word select); four data lines
 * carry stereo pairs: D0=mics 0+1, D1=2+3, D2=4+5, D3=center (mic 6) on L.
 *
 * BCK = 1.024 MHz from 27 MHz (fractional NCO), stereo frame = 64 BCK periods
 * => 16 kHz PCM. MSB-first 16-bit samples in upper half of 32-bit slots.
 */

module i2s_receiver (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i2s_d0,
    input  wire        i2s_d1,
    input  wire        i2s_d2,
    input  wire        i2s_d3,
    output reg         i2s_ck,
    output reg         i2s_ws,
    output reg  [15:0] pcm_ch0,
    output reg  [15:0] pcm_ch1,
    output reg  [15:0] pcm_ch2,
    output reg  [15:0] pcm_ch3,
    output reg  [15:0] pcm_ch4,
    output reg  [15:0] pcm_ch5,
    output reg  [15:0] pcm_ch6,
    output reg         pcm_valid
);

    localparam integer ACC_INC = 2048000;
    localparam integer ACC_MOD = 27_000_000;

    reg [31:0] acc;
    reg        i2s_ck_d;

    wire bck_rise = i2s_ck && !i2s_ck_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc      <= 32'd0;
            i2s_ck   <= 1'b0;
            i2s_ck_d <= 1'b0;
        end else begin
            i2s_ck_d <= i2s_ck;
            acc <= acc + ACC_INC;
            if (acc >= ACC_MOD) begin
                acc <= acc - ACC_MOD;
                i2s_ck <= ~i2s_ck;
            end
        end
    end

    reg [5:0] bit_idx;
    reg [31:0] sh0, sh1, sh2, sh3;

    wire [31:0] n0 = {sh0[30:0], i2s_d0};
    wire [31:0] n1 = {sh1[30:0], i2s_d1};
    wire [31:0] n2 = {sh2[30:0], i2s_d2};
    wire [31:0] n3 = {sh3[30:0], i2s_d3};

    wire [5:0] next_bidx = (bit_idx == 6'd63) ? 6'd0 : (bit_idx + 6'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_idx   <= 6'd0;
            sh0       <= 32'd0;
            sh1       <= 32'd0;
            sh2       <= 32'd0;
            sh3       <= 32'd0;
            i2s_ws    <= 1'b0;
            pcm_ch0   <= 16'd0;
            pcm_ch1   <= 16'd0;
            pcm_ch2   <= 16'd0;
            pcm_ch3   <= 16'd0;
            pcm_ch4   <= 16'd0;
            pcm_ch5   <= 16'd0;
            pcm_ch6   <= 16'd0;
            pcm_valid <= 1'b0;
        end else begin
            pcm_valid <= 1'b0;

            if (bck_rise) begin
                sh0 <= n0;
                sh1 <= n1;
                sh2 <= n2;
                sh3 <= n3;

                if (bit_idx == 6'd31) begin
                    pcm_ch0 <= n0[31:16];
                    pcm_ch2 <= n1[31:16];
                    pcm_ch4 <= n2[31:16];
                    pcm_ch6 <= n3[31:16];
                end

                if (bit_idx == 6'd63) begin
                    pcm_ch1 <= n0[31:16];
                    pcm_ch3 <= n1[31:16];
                    pcm_ch5 <= n2[31:16];
                    pcm_valid <= 1'b1;
                end

                bit_idx <= next_bidx;
                i2s_ws  <= (next_bidx >= 6'd32);
            end
        end
    end

endmodule
