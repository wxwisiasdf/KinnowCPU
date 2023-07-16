///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 l2k_alu
//
///////////////////////////////////////////////////////////////////////////////
module l2k_alu
( // Interface
    input [2:0] op,
    input [31:0] a,
    input [31:0] b,
    output [31:0] c
);
    function [31:0] calculate();
        casez(op)
            3'b111: begin // ADD
                $display("%m: add");
                calculate = a + b;
                end
            3'b110: begin // SUB
                $display("%m: sub");
                calculate = a - b;
                end
            3'b011: begin // AND
                $display("%m: and");
                calculate = a & b;
                end
            3'b010: begin // XOR
                $display("%m: xor");
                calculate = a ^ b;
                end
            3'b001: begin // OR
                $display("%m: or");
                calculate = a | b;
                end
            3'b101: begin // SLT
                $display("%m: slt");
                calculate = { 31'h0, a < b };
                end
            3'b100: begin // SLTS
                $display("%m: slts");
                // tmp32 is positive, register is negative
                if((a & 32'h80000000) != 0 && (b & 32'h80000000) == 0) begin
                    calculate = 1;
                // tmp32 is negative, register is positive
                end else if((a & 32'h80000000) == 0 && (b & 32'h80000000) != 0) begin
                    calculate = 0;
                end else begin
                    calculate = { 31'h0, a < b };
                end
                end
            default: begin end
        endcase
    endfunction
    assign c = calculate();
endmodule
