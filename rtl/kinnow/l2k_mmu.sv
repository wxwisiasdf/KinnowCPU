///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 MMU
//
///////////////////////////////////////////////////////////////////////////////

`include "defines.sv"
`include "l2k_cache.sv"

module l2k_mmu
( // Interface
    input rst,
    input clk,
    input irq,
    input flush,
    input [31:0] read_addr_in,
    output [31:0] read_addr_out,
    input [31:0] write_addr_in,
    output [31:0] write_addr_out
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

    // TLB cache
    reg tlb_we;
    wire [63:0] tlb_data_out;
    l2k_cache #(.NUM_ENTRIES(128), .DATA_WIDTH(64)) tlb_cache(
        .rst(rst),
        .clk(clk),
        .we(tlb_we),
        .addr_in(ctl_regs[CREG_TBINDEX]),
        .data_in({ ctl_regs[CREG_TBHI], ctl_regs[CREG_TBLO] }),
        .addr_out(ctl_regs[CREG_TBINDEX]),
        .data_out(tlb_data_out)
    );

endmodule
