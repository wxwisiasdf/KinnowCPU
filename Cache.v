`ifndef LIMN2600_CACHE_H
`define LIMN2600_CACHE_H

///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 Cache
//
// 64K Cache for memory (not MMU), saves an entire page, can be flushed with
// a proper RST
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_Cache(
    input rst,
    input clk,
    input we, // Write-Enable
    output reg [3:0] addr, // Address (except 12-bits)
    input [31:0] data_in, // Input data
    output reg [31:0] data_out // Output data
);
    reg [4095:0] cache[0:16]; // Up to 16-pages stored

    // Reset cache
    always @(rst) begin
        for(integer i = 0; i < 16; i++) begin
            cache[i] = 4096'h0;
        end
    end

    always @(posedge clk) begin
        if(we) begin
            cache[addr] <= data_in;
        end
        data_out <= cache[addr];
    end
endmodule

`endif
