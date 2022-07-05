///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 SRAM
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_SRAM
#( // Parameter
    parameter DATA_WIDTH = 32,
    parameter ROM_SIZE = 32768,
    parameter RAM_SIZE = 32768
)
( // Interface
    input rst,
    input clk,
    input we,
    input cs,
    output reg rdy,
    input [31:0] addr,
    input [DATA_WIDTH - 1:0] data_in,
    output reg [DATA_WIDTH - 1:0] data_out
);
    reg [DATA_WIDTH - 1:0] rom[0:ROM_SIZE - 1]; // Bank 1 - READ ONLY
    reg [DATA_WIDTH - 1:0] ram[0:RAM_SIZE - 1]; // Bank 2 - READ-WRITE

    integer i;

    always @(posedge clk) begin
        if(cs) begin
            if(we) begin
                if(addr == 32'hF8000040) begin
                    $display("%m: serial_emul WRITE <%b>", data_in);
                end else if(addr[31:16] == 16'h0000) begin
                    ram[addr[14:0] >> 2] <= data_in;
                    $display("%m: ram_write data_in=0x%8h=>addr=0x%8h", data_in, addr);
                end else begin
                    $display("%m: write [shadow] data_in=0x%8h=>addr=0x%8h", data_in, addr);
                end
            end else begin
                if(addr == 32'hF8000040) begin
                    $display("%m: serial_emul READ <%b>", data_in);
                end else if(addr[31:16] == 16'h0000) begin
                    data_out <= ram[addr[14:0] >> 2];
                    $display("%m: ram_read data_out=0x%8h=>addr=0x%8h", data_out, addr);
                end else if(addr[31:16] == 16'hFFFE) begin
                    data_out <= rom[addr[14:0] >> 2];
                    $display("%m: rom_read [shadow] data_out=0x%8h=>addr=0x%8h", data_out, addr);
                end else begin
                    data_out <= 0;
                    $display("%m: read [shadow] data_in=0x%8h=>addr=0x%8h", data_in, addr);
                end
            end
            rdy <= 1;
        end else begin
            rdy <= 0;
        end
    end

    // Initialize RAM with ROM
    initial begin
        $display("%m: ram_size=%0dKB,data_width=%0d bits", ((DATA_WIDTH / 8) * RAM_SIZE) / 1000, DATA_WIDTH);
        $display("%m: rom_size=%0dKB,data_width=%0d bits", ((DATA_WIDTH / 8) * ROM_SIZE) / 1000, DATA_WIDTH);
        $readmemh("../rom.txt", rom);
    end
endmodule
