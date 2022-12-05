///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 MMU
//
///////////////////////////////////////////////////////////////////////////////

`include "defines.sv"

module l2k_mmu
( // Interface
    input rst,
    input clk,
    input [31:0] addr_in,
    output [31:0] addr_out,
    input [31:0] entry_addr_in,
    input [63:0] entry_in,
    input [1:0] cmd
);
    typedef struct packed {
        bit [20:0] vpn;
        bit [(31 - 21):(21 - 21)] asid;
        bit v;
        bit write;
        bit k;
        bit nc;
        bit g;
        bit [(25 - 5):(5 - 5)] ppn;
        bit [(31 - 25):(25 - 25)] avail;
    } tlb_entry;

    reg enabled;
    tlb_entry tlb[0:63];
    integer i;

    localparam
        CMD_WRITE = 0;

    assign addr_out = enabled ? tlb[addr_in[17:12]] : addr_in;

    always @(posedge clk) begin
        if(rst) begin
            enabled <= 0;
            for (i = 0; i < 64; i++) begin
                tlb[i] <= 0;
            end
        end
    end

    always @(posedge clk) begin
        if(cmd == CMD_WRITE) begin
            for (i = 0; i < 64; i++) begin
                if (i == { 26'h0, entry_addr_in[17:12] }) begin
                    tlb[i] <= entry_in;
                end
            end
        end
    end
endmodule
