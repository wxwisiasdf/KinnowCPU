`timescale 1ns / 1ps

`include "l2k_cpu.sv"
`include "soc_nvram.sv"
`include "soc_rom.sv"
`include "soc_uart.sv"

///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 System
//
///////////////////////////////////////////////////////////////////////////////
module l2k_soc
(
    input rst,
    input clk
);
    wire soc_we;
    wire [31:0] addr;
    wire [31:0] data_from_cpu;
    wire [31:0] data_from_ram;
    reg irq;
    reg rdy;
    wire soc_ce;
    wire soc_oe;

    assign soc_oe = 1;
    l2k_cpu CPU(
        .rst(rst),
        .clk(clk),
        .we(soc_we),
        .ce(soc_ce),
        .irq(irq),
        .rdy(rdy),
        .addr(addr),
        .data_in(data_from_ram),
        .data_out(data_from_cpu)
    );

    wire rom_enable = (addr[31:16] == 16'hFFFE) ? 1 : 0;
    wire rom_rdy = 1;
    wire [31:0] rom_data_out;
    soc_rom ROM(
        .addr(addr[16:2]),
        .data_out(rom_data_out)
    );

    wire nvram_enable = (addr[31:16] == 16'hF800) ? 1 : 0;
    wire nvram_rdy = 1;
    wire [31:0] nvram_data_out;
    soc_nvram NVRAM(
        .clk(clk),
        .we(nvram_enable & soc_we),
        .addr(addr),
        .data_in(data_from_cpu),
        .data_out(nvram_data_out)
    );

    wire ram_enable = (addr[31:16] == 16'h0000) ? 1 : 0;
    wire ram_rdy = 1;
    wire [31:0] ram_data_out;
    soc_nvram RAM(
        .clk(clk),
        .we(ram_enable & soc_we),
        .addr(addr),
        .data_in(data_from_cpu),
        .data_out(ram_data_out)
    );

    wire uart_enable = (addr[31:16] == 16'hF800) ? 1 : 0;
    wire uart_rdy;
    wire [31:0] uart_data_out;
    soc_uart UAART(
        .clk(clk),
        .we(uart_enable & soc_we),
        .ce(uart_enable & soc_ce),
        .oe(uart_enable & soc_oe),
        .rdy(uart_rdy),
        .offset(addr[7:0]),
        .data_in(data_from_cpu[7:0]),
        .data_out(uart_data_out)
    );

    assign data_from_ram = uart_enable ? uart_data_out
        : nvram_enable ? nvram_data_out
        : ram_enable ? ram_data_out
        : rom_enable ? rom_data_out : 0;
    
    assign rdy = uart_enable ? uart_rdy
        : nvram_enable ? nvram_rdy
        : ram_enable ? ram_rdy
        : rom_enable ? rom_rdy : 0;
    
    always @(posedge clk) begin
        if(rst) begin
            irq <= 0;
        end
        if(uart_enable && !soc_we) begin
            if(uart_rdy) begin
                irq <= 1;
            end
        end
    end

    initial begin
        $display("Limn2600 Verilog SoC!");
    end
endmodule
