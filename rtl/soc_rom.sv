///////////////////////////////////////////////////////////////////////////////
//
// System ROM
//
///////////////////////////////////////////////////////////////////////////////
module soc_rom
#( // Parameter
    parameter DATA_WIDTH = 32,
    parameter ROM_SIZE = 32768
)
( // Interface
    input [14:0] addr, // Address
    output [DATA_WIDTH - 1:0] data_out
);
    reg [DATA_WIDTH - 1:0] rom[0:ROM_SIZE - 1]; // Bank 1 - READ ONLY
    assign data_out = rom[addr];

    // Initialize RAM with ROM
    initial begin
        $display("%m: rom_size=%0dKB,data_width=%0d bits", ((DATA_WIDTH / 8) * ROM_SIZE) / 1000, DATA_WIDTH);
        $readmemh("../rom.txt", rom);
    end
endmodule
