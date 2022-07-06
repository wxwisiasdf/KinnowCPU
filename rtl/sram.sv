///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 SRAM
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_SRAM
#( // Parameter
    parameter DATA_WIDTH = 32,
    parameter ROM_SIZE = 32768,
    parameter RAM_SIZE = 262140,
    parameter NVRAM_SIZE = 16384
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
    reg [DATA_WIDTH - 1:0] nvram[0:NVRAM_SIZE - 1]; // Bank 3 - WRITE-VOLATILE

    integer i;

    always @(posedge clk) begin
        data_out <= 0;
        rdy <= 0;
        if(cs) begin
            if(we) begin
                if(addr == 32'hF8000040) begin
                    $display("%m: serial_emul WRITE CMD <%b> %c", data_in[7:0], data_in[7:0]);
                end else if(addr == 32'hF8000044) begin
                    $display("%m: serial_emul WRITE 0x%h<%b> %c", data_in[7:0], data_in[7:0], data_in[7:0]);
                end else if(addr[31:16] == 16'hFFFE) begin
                    $display("%m: ROM_write [shadow] data_in=0x%8h=>addr=0x%8h", data_in, addr);
                end else if(addr[31:16] == 16'h0000) begin
                    ram[addr[19:2]] <= data_in;
                    $display("%m: SRAM_write data_in=0x%8h=>addr=0x%8h", data_in, addr);
                end else if(addr[31:16] == 16'hF800) begin
                    nvram[addr[15:2]] <= data_in;
                    $display("%m: NVRAM_write data_in=0x%8h=>addr=0x%8h", data_in, addr);
                end else begin

                end
            end else begin
                if(addr == 32'hF8000040) begin
                    $display("%m: serial_emul READ_CMD <%b>", data_out);
                    data_out <= 32'h00000000;
                end else if(addr == 32'hF8000044) begin
                    $display("%m: serial_emul READ <%b>", data_out);
                    data_out <= 32'hFFFF;
                end else if(addr[31:16] == 16'hFFFE) begin
                    data_out <= rom[addr[16:2]];
                    $display("%m: SROM_read [shadow] data_out=0x%8h=>addr=0x%8h", data_out, addr);
                end else if(addr[31:16] == 16'h0000) begin
                    data_out <= ram[addr[19:2]];
                    $display("%m: SRAM_read data_out=0x%8h=>addr=0x%8h", data_out, addr);
                end else if(addr[31:16] == 16'hF800) begin
                    data_out <= nvram[addr[15:2]];
                    $display("%m: NVRAM_read data_out=0x%8h=>addr=0x%8h", data_out, addr);
                end else begin

                end
            end
            rdy <= 1;
        end
    end

    // Initialize RAM with ROM
    initial begin
        $display("%m: ram_size=%0dKB,data_width=%0d bits", ((DATA_WIDTH / 8) * RAM_SIZE) / 1000, DATA_WIDTH);
        $display("%m: rom_size=%0dKB,data_width=%0d bits", ((DATA_WIDTH / 8) * ROM_SIZE) / 1000, DATA_WIDTH);
        $readmemh("../rom.txt", rom);

        for(i = 0; i < RAM_SIZE; i++) begin
            ram[i] = i[31:0];
        end
    end
endmodule
