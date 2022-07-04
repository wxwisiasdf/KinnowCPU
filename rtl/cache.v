///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 Cache
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_cache(
    input rst,
    input clk,
    input we, // Write enable
    input [31:0] addr_out, // Address
    input [31:0] data_in, // Data-in
    input [31:0] addr_in,
    output reg [31:0] data_out // Data-out
);
    localparam NUM_CACHE_ELEM = 4096;
    integer i;
    function [31:0] hash_result(
        input [31:0] x
    );
        // https://stackoverflow.com/questions/664014/what-integer-hash-function-are-good-that-accepts-an-integer-hash-key
        // x = ((x >> 16) ^ x) * 0x45d9f3b;
        // x = ((x >> 16) ^ x) * 0x45d9f3b;
        // x = (x >> 16) ^ x;
        hash_result = (((((((x >> 16) ^ x) * 32'h45d9f3b) >> 16) ^ (((x >> 16) ^ x) * 32'h45d9f3b)) * 32'h45d9f3b) >> 16) ^ ((((((x >> 16) ^ x) * 32'h45d9f3b) >> 16) ^ (((x >> 16) ^ x) * 32'h45d9f3b)) * 32'h45d9f3b);
    endfunction

    reg [31:0] inst_cache[0:NUM_CACHE_ELEM];

    always @(rst) begin
        for(i = 0; i < NUM_CACHE_ELEM; i++) begin
            inst_cache[i] = 32'h0;
        end
    end

    always @(posedge clk) begin
        $display("cache: k_out=0x%h,v_out=0x%h,k_in=0x%h,v_in=0x%h,we=%b,k_out_hash=0x%h,k_in_hash=0x%h", addr_out, data_out, addr_in, data_in, we, hash_result(addr_out), hash_result(addr_in));
        if(we) begin
            inst_cache[hash_result(addr_in) % NUM_CACHE_ELEM] <= data_in;
        end
        data_out <= inst_cache[hash_result(addr_out) % NUM_CACHE_ELEM];
    end
endmodule
