///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 DRAM (128K)
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_SRAM(
    input rst,
    input clk,
    input we,
    input cs,
    output reg rdy,
    input [31:0] addr,
    input [31:0] data_in,
    output reg [31:0] data_out
);
    reg [31:0] rom[0:65535]; // Bank 1 - READ ONLY
    reg [31:0] ram[0:65535]; // Bank 2 - READ-WRITE

    integer i;

    always @(posedge clk) begin
        if(cs) begin
            if(we) begin
                if(addr[31:16] == 16'h0000 || addr[31:16] == 16'h00F8) begin
                    ram[addr[15:0] >> 2] <= data_in;
                    $display("sram: ram_write data_in=0x%8h=>addr=0x%8h", data_in, addr);
                end else begin
                    $display("sram: rom_write [shadow] data_in=0x%8h=>addr=0x%8h", data_in, addr);
                end
            end else begin
                if(addr[31:16] == 16'h0000 || addr[31:16] == 16'h00F8) begin
                    data_out <= ram[addr[15:0] >> 2];
                    $display("sram: ram_read data_out=0x%8h=>addr=0x%8h", data_out, addr);
                end else begin
                    data_out <= rom[addr[15:0] >> 2];
                    $display("sram: rom_read [shadow] data_out=0x%8h=>addr=0x%8h", data_out, addr);
                end
            end
            rdy <= 1;
        end else begin
            rdy <= 0;
        end
    end

    // Initialize RAM with ROM
    initial begin
        for(i = 0; i < 65535; i++) begin
            ram[i] = 32'h0;
        end
        $readmemh("rom.txt", rom);
        rdy = 1'b0;
    end
endmodule
