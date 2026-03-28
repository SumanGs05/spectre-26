/**
 * ITU G.711 Mu-Law Encoder
 *
 * Compresses 16-bit signed linear PCM to 8-bit mu-law.
 * Preserves ~72 dB dynamic range in 8 bits.
 *
 * Combinational logic — no clock needed. Instantiated per channel.
 *
 * Mu-law encoding format:
 *   Bit 7:    Sign (1 = positive)
 *   Bits 6-4: Exponent (0-7)
 *   Bits 3-0: Mantissa (4 bits from significant magnitude)
 */

module mulaw_encoder (
    input  wire [15:0] pcm_in,      // 16-bit signed PCM
    output wire [7:0]  mulaw_out    // 8-bit mu-law encoded
);

    wire        sign;
    wire [15:0] magnitude;
    wire [15:0] biased;

    // Extract sign and magnitude
    assign sign = ~pcm_in[15]; // mu-law: 1 = positive
    assign magnitude = pcm_in[15] ? (~pcm_in + 1'b1) : pcm_in;

    // Add bias (0x84 = 132) to avoid log(0) and improve SNR for small signals
    assign biased = (magnitude > 16'd32635) ? 16'd32767 : (magnitude + 16'd132);

    // Find exponent (position of highest set bit above bit 7)
    reg [2:0] exponent;
    reg [3:0] mantissa;

    always @(*) begin
        casez (biased[15:7])
            9'b1????????: begin exponent = 3'd7; mantissa = biased[14:11]; end
            9'b01???????: begin exponent = 3'd6; mantissa = biased[13:10]; end
            9'b001??????: begin exponent = 3'd5; mantissa = biased[12:9];  end
            9'b0001?????: begin exponent = 3'd4; mantissa = biased[11:8];  end
            9'b00001????: begin exponent = 3'd3; mantissa = biased[10:7];  end
            9'b000001???: begin exponent = 3'd2; mantissa = biased[9:6];   end
            9'b0000001??: begin exponent = 3'd1; mantissa = biased[8:5];   end
            default:       begin exponent = 3'd0; mantissa = biased[7:4];   end
        endcase
    end

    // Assemble and complement (mu-law convention: output is bitwise inverted)
    assign mulaw_out = ~{sign, exponent, mantissa};

endmodule
