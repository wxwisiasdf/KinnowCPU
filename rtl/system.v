`timescale 1ns / 1ps

`include "rtl/cache.v"
`include "rtl/cpu.v"
`include "rtl/sram.v"

///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 System
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_System;
    integer i;
    reg rst;
    reg clk;
    wire we;
    wire [31:0] addr;
    wire [31:0] data_from_cpu;
    wire [31:0] data_from_ram;
    wire irq;
    wire rdy;
    wire cs;

    limn2600_CPU CPU(
        .rst(rst),
        .clk(clk),
        .we(we),
        .cs(cs),
        .irq(irq),
        .rdy(rdy),
        .addr(addr),
        .data_in(data_from_ram),
        .data_out(data_from_cpu)
    );

    limn2600_SRAM SRAM(
        .rst(rst),
        .clk(clk),
        .we(we),
        .cs(cs),
        .rdy(rdy),
        .addr(addr),
        .data_in(data_from_cpu),
        .data_out(data_from_ram)
    );

    initial begin
        $display("Limn2600 Verilog SoC!");

        // "Press" reset button
        #0 rst = 1'b1;
        #1 rst = 1'b0;

        #4 clk = 1'b0;
        for(i = 0; i < 1000; i++) begin
            #0 $display("perf: Begin tick %8t", $time);
            #0 clk = ~clk;
            #1 $display("perf: End tick   %8t", $time);
            #1 clk = ~clk;
        end
        $finish;
    end
endmodule
