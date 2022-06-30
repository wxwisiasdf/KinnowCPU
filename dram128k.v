`ifndef LIMN2600_DRAM128_H
`define LIMN2600_DRAM128_H

///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 DRAM (128K)
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_dram(
    input rst,
    input clk,
    input we,
    output reg rdy,
    input [31:0] addr,
    input [31:0] data_in,
    output reg [31:0] data_out
);
    reg [31:0] rom[0:65535]; // Bank 1 - READ ONLY
    reg [31:0] ram[0:65535]; // Bank 2 - READ-WRITE

    always @(posedge clk) begin
        if(we) begin
            if(addr[31:16] == 16'h0000) begin
                ram[addr[15:0] >> 2] <= data_in;
`ifdef DEBUG
                $display("ram_write data=0x%8h=>addr=0x%8h", data_in, addr);
`endif
            end
        end
        if(addr[31:16] == 16'h0000) begin
            data_out <= ram[addr[15:0] >> 2];
`ifdef DEBUG
            $display("ram_read data=0x%8h=>addr=0x%8h", data_in, addr);
`endif
        end else begin
            data_out <= rom[addr[15:0] >> 2];
`ifdef DEBUG
            $display("rom_read data=0x%8h=>addr=0x%8h", data_in, addr);
`endif
        end
        rdy <= 1'b1;
    end

    // Initialize RAM with ROM
    initial begin
        $readmemh("rom.txt", rom);
        rdy = 1'b0;
    end

`ifdef DEBUG
    // Monitor
    always @(posedge clk) begin
        $display("RAM > addr=0x%8x,we=%1d,data_in=0x%8h,data_out=0x%8h", addr, we, data_in, data_out);
    end
`endif
endmodule

`endif
