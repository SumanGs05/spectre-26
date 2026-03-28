/**
 * Blind UART test pattern — alternates 0xAA / 0x55 as fast as uart_tx allows.
 * Use when the main audio chain may not be producing sample_valid; isolates
 * FPGA → pin 17 → BL702 → PC COM path.
 */

module uart_debug_pump #(
    parameter BAUD_RATE = 1_500_000
) (
    input  wire       clk,
    input  wire       rst_n,
    output wire       tx
);

    wire       ready;
    reg  [7:0] tx_byte;
    reg        tx_valid;
    reg        use_hi;

    uart_tx #(
        .BAUD_RATE(BAUD_RATE)
    ) u_uart (
        .clk        (clk),
        .rst_n      (rst_n),
        .data       (tx_byte),
        .data_valid (tx_valid),
        .tx         (tx),
        .ready      (ready)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            use_hi   <= 1'b1;
            tx_valid <= 1'b0;
            tx_byte  <= 8'hAA;
        end else begin
            tx_valid <= 1'b0;
            if (ready) begin
                tx_byte  <= use_hi ? 8'hAA : 8'h55;
                tx_valid <= 1'b1;
                use_hi   <= ~use_hi;
            end
        end
    end

endmodule
