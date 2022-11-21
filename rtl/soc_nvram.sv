///////////////////////////////////////////////////////////////////////////////
//
// System NVRAM
//
///////////////////////////////////////////////////////////////////////////////
module soc_nvram
#( // Parameter
    parameter DATA_WIDTH = 32,
    parameter NVRAM_SIZE = 16384
)
( // Interface
    input clk, // Clock
    input we, // Write enable
    input [31:0] addr, // Address
    input [DATA_WIDTH - 1:0] data_in,
    output [DATA_WIDTH - 1:0] data_out
);
    reg [DATA_WIDTH - 1:0] nvram[0:NVRAM_SIZE - 1]; // Bank 3 - WRITE-VOLATILE
    assign data_out = nvram[addr[15:2]];

    always @(posedge clk) begin
        if(we) begin
            $display("%m: write data_in=0x%8h=>addr=0x%8h", data_in, addr);
            nvram[addr[15:2]] <= data_in;
        end
    end

    // Initialize RAM with ROM
    initial begin
        $display("%m: nvram_size=%0dKB,data_width=%0d bits", ((DATA_WIDTH / 8) * NVRAM_SIZE) / 1000, DATA_WIDTH);
    end
endmodule
