`include "cpu.v"
`include "dram128k.v"

///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 System
//
///////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module limn2600_system;
    reg rst;
    reg clk;
    wire we;
    wire [31:0] addr;
    wire [31:0] data_from_cpu;
    wire [31:0] data_from_ram;
    wire irq;
    wire rdy;

    limn2600_cpu cpu(
        .rst(rst),
        .clk(clk),
        .we(we),
        .irq(irq),
        .rdy(rdy),
        .addr(addr),
        .data_in(data_from_ram),
        .data_out(data_from_cpu)
    );

    limn2600_dram ram(
        .rst(rst),
        .clk(clk),
        .we(we),
        .rdy(rdy),
        .addr(addr),
        .data_in(data_from_cpu),
        .data_out(data_from_ram)
    );

    initial begin
        clk = 1'b0;
        forever
            #2 clk = ~clk;
    end

    initial begin
        $display("Limn2600 Verilog SoC!");

        // "Press" reset button
        #0 rst = 1'b1;
        #1 rst = 1'b0;

        #10000 rst = 1'b0;

        $finish;
    end
endmodule
