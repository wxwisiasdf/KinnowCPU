`include "l2k_msched.sv"
`include "l2k_core.sv"
`include "l2k_mmu.sv"

///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 Core
//
// Executes an instruction, multiple instances of this module are used
// for executing the whole instruction queue
//
///////////////////////////////////////////////////////////////////////////////
module l2k_cpu
#( // Parameters
    parameter NUM_CORES = 4
)
( // Interface
    input rst,
    input clk,
    input irq,
    output [31:0] addr,
    input [31:0] data_in,
    output [31:0] data_out,
    input rdy, // Whetever we can fetch instructions
    output we, // Write-Enable (1 = we want to write, 0 = we want to read)
    output ce // Command-State (1 = memory commands active, 0 = memory commands ignored)
);
    wire [31:0] core1_read_addr;
    wire [31:0] core1_read_value;
    wire [1:0] core1_read_size;
    wire core1_read_enable;
    wire core1_read_rdy;
    wire [31:0] core1_read_addr_in;

    wire [31:0] core1_write_addr;
    wire [31:0] core1_write_value;
    wire [1:0] core1_write_size;
    wire core1_write_enable;
    wire core1_write_rdy;
    wire core1_full;
    wire core1_flush;

    wire [31:0] mmu_addr_in;
    l2k_mmu mmu(
        .rst(rst),
        .clk(clk),
        .addr_in(mmu_addr_in),
        .addr_out(addr),
        .entry_addr_in(0),
        .entry_in(0),
        .cmd(0)
    );
    
    l2k_msched memsched(
        .rst(rst),
        .clk(clk),
        .ram_addr(mmu_read_addr_in),
        .ram_data_in(data_in),
        .ram_data_out(data_out),
        .ram_rdy(rdy),
        .ram_we(we),
        .ram_ce(ce),
        .client_read_addr(core1_read_addr),
        .client_read_value(core1_read_value),
        .client_read_size(core1_read_size),
        .client_read_enable(core1_read_enable),
        .client_read_rdy(core1_read_rdy),
        .client_read_addr_in(core1_read_addr_in),
        .client_write_addr(core1_write_addr),
        .client_write_value(core1_write_value),
        .client_write_size(core1_write_size),
        .client_write_enable(core1_write_enable),
        .client_write_rdy(core1_write_rdy),
        .full(core1_full),
        .flush(core1_flush)
    );

    l2k_core core1(
        .rst(rst),
        .clk(clk),
        .irq(irq),
        .read_addr(core1_read_addr),
        .read_value(core1_read_value),
        .read_size(core1_read_size),
        .read_enable(core1_read_enable),
        .read_rdy(core1_read_rdy),
        .read_addr_in(core1_read_addr_in),
        .write_addr(core1_write_addr),
        .write_value(core1_write_value),
        .write_size(core1_write_size),
        .write_enable(core1_write_enable),
        .write_rdy(core1_write_rdy),
        .full(core1_full),
        .flush(core1_flush)
    );
endmodule
