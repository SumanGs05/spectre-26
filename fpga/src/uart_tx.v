/**
 * UART Transmitter — 2 Mbaud, 8N1
 *
 * Sends bytes to the Tang Nano 9K's onboard USB-UART bridge.
 * At 27 MHz system clock, 2 Mbaud requires ~13.5 clocks per bit.
 * We use a clock divider of 14 (actual baud: 27M/14 ≈ 1.928 Mbaud,
 * within UART tolerance).
 */

module uart_tx #(
    parameter CLK_FREQ  = 27_000_000,
    parameter BAUD_RATE = 2_000_000
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       data_valid,   // Pulse to start transmission
    output reg        tx,           // UART TX line
    output wire       ready         // High when idle, can accept data
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE; // ~14

    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [7:0]  tx_shift;
    reg [3:0]  bit_idx;
    reg [4:0]  clk_cnt;    // counts up to CLKS_PER_BIT

    assign ready = (state == S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            tx       <= 1'b1;  // idle high
            clk_cnt  <= 0;
            bit_idx  <= 0;
            tx_shift <= 8'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    tx <= 1'b1;
                    if (data_valid) begin
                        tx_shift <= data;
                        state    <= S_START;
                        clk_cnt  <= 0;
                    end
                end

                S_START: begin
                    tx <= 1'b0;  // start bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_DATA: begin
                    tx <= tx_shift[bit_idx]; // LSB first
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
                    tx <= 1'b1;  // stop bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        state   <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
            endcase
        end
    end

endmodule
