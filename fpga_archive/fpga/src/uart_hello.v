/**
 * Minimal standalone UART test — sends "HELLO\r\n" at 115200, repeats.
 * 
 * Use this as the ONLY top-level module to prove pin 17 + BL702 + COM work.
 * No I2S, no TDM, no generate, no parameters.
 *
 * Gowin project: set this as the top module. Same .cst file.
 * PC: open COM3 at 115200 in PuTTY/RealTerm — you should see HELLO scrolling.
 */

module uart_hello (
    input  wire clk_27m,
    input  wire rst_n,
    output reg  uart_tx,
    output wire led_prog,
    // unused but declared to satisfy .cst
    input  wire i2s_d0,
    input  wire i2s_d1,
    input  wire i2s_d2,
    input  wire i2s_d3,
    output wire i2s_ck,
    output wire i2s_ws
);

    assign i2s_ck = 1'b0;
    assign i2s_ws = 1'b0;

    // 27 MHz / 115200 = 234.375 → 234. Actual = 115,385 baud (0.16% error)
    localparam CLKS_PER_BIT = 234;

    // Message: H E L L O \r \n (no initial block — Gowin may not support it)
    wire [7:0] msg_byte =
        (char_idx == 3'd0) ? 8'h48 : // H
        (char_idx == 3'd1) ? 8'h45 : // E
        (char_idx == 3'd2) ? 8'h4C : // L
        (char_idx == 3'd3) ? 8'h4C : // L
        (char_idx == 3'd4) ? 8'h4F : // O
        (char_idx == 3'd5) ? 8'h0D : // \r
                             8'h0A;  // \n

    localparam S_IDLE  = 3'd0;
    localparam S_START = 3'd1;
    localparam S_DATA  = 3'd2;
    localparam S_STOP  = 3'd3;
    localparam S_GAP   = 3'd4;

    reg [2:0]  state;
    reg [7:0]  shift;
    reg [3:0]  bit_idx;
    reg [15:0] clk_cnt;
    reg [2:0]  char_idx;
    reg [23:0] gap_cnt;

    // Heartbeat LED
    reg [24:0] hb;
    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n) hb <= 0;
        else        hb <= hb + 1;
    end
    assign led_prog = hb[24];

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            uart_tx  <= 1'b1;
            clk_cnt  <= 0;
            bit_idx  <= 0;
            char_idx <= 0;
            shift    <= 0;
            gap_cnt  <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    uart_tx <= 1'b1;
                    shift   <= msg_byte;
                    state   <= S_START;
                    clk_cnt <= 0;
                end

                S_START: begin
                    uart_tx <= 1'b0;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_DATA: begin
                    uart_tx <= shift[bit_idx];
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        if (bit_idx == 7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_STOP: begin
                    uart_tx <= 1'b1;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        if (char_idx == 3'd6) begin
                            char_idx <= 0;
                            gap_cnt  <= 0;
                            state    <= S_GAP;
                        end else begin
                            char_idx <= char_idx + 1;
                            state    <= S_IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_GAP: begin
                    uart_tx <= 1'b1;
                    if (gap_cnt == 24'd13_500_000) begin
                        state <= S_IDLE;
                    end else begin
                        gap_cnt <= gap_cnt + 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
