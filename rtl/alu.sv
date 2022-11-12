///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 ALU
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_ALU
( // Interface
    input rst,
    input clk,
    input [2:0] op,
    input [31:0] a,
    input [31:0] b,
    output reg [31:0] c
);
    always @(posedge clk) begin
        casez(op)
            3'b111: begin // ADD
                $display("%m: add");
                c = a + b;
                end
            3'b110: begin // SUB
                $display("%m: sub");
                c = a - b;
                end
            3'b011: begin // AND
                $display("%m: and");
                c = a & b;
                end
            3'b010: begin // XOR
                $display("%m: xor");
                c = a ^ b;
                end
            3'b001: begin // OR
                $display("%m: or");
                c = a | b;
                end
            3'b101: begin // SLT
                $display("%m: slt");
                c = { 31'h0, a < b };
                end
            3'b100: begin // SLTS
                $display("%m: slts");
                // tmp32 is positive, register is negative
                if((a & 32'h80000000) != 0 && (b & 32'h80000000) == 0) begin
                    c = 1;
                // tmp32 is negative, register is positive
                end else if((a & 32'h80000000) == 0 && (b & 32'h80000000) != 0) begin
                    c = 0;
                end else begin
                    c = { 31'h0, a < b };
                end
                end
            default: begin end
        endcase
    end
endmodule
