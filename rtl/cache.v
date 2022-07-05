///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 Cache
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_cache
#( // Parameter
    parameter DATA_WIDTH = 32,
    parameter NUM_ENTRIES = 1024
)
( // Interface
    input rst,
    input clk,
    input we, // Write enable
    input [31:0] addr_out, // Address
    input [31:0] data_in, // Data-in
    input [31:0] addr_in,
    output reg [31:0] data_out // Data-out
);
    integer i;
    function [DATA_WIDTH - 1:0] hash_result(
        input [DATA_WIDTH - 1:0] x
    );
        // https://stackoverflow.com/questions/664014/what-integer-hash-function-are-good-that-accepts-an-integer-hash-key
        // x = ((x >> 16) ^ x) * 0x45d9f3b;
        // x = ((x >> 16) ^ x) * 0x45d9f3b;
        // x = (x >> 16) ^ x;
        hash_result = (((((((x >> 16) ^ x) * 32'h45d9f3b) >> 16) ^ (((x >> 16) ^ x) * 32'h45d9f3b)) * 32'h45d9f3b) >> 16) ^ ((((((x >> 16) ^ x) * 32'h45d9f3b) >> 16) ^ (((x >> 16) ^ x) * 32'h45d9f3b)) * 32'h45d9f3b);
    endfunction

    reg [DATA_WIDTH - 1:0] cache[0:NUM_ENTRIES - 1];

    always @(posedge clk) begin
        if(rst) begin
            for(i = 0; i < NUM_ENTRIES; i++) begin
                cache[i] = 0;
            end
            data_out <= 0;
        end
    end

    always @(posedge clk) begin
        $display("%m: k_out=0x%h,v_out=0x%h,k_in=0x%h,v_in=0x%h,we=%b,k_out_hash=0x%h,k_in_hash=0x%h", addr_out, data_out, addr_in, data_in, we, hash_result(addr_out), hash_result(addr_in));

/*
        for(i = 0; i < NUM_ENTRIES; i++) begin
            if(i == (hash_result(addr_in) & (NUM_ENTRIES - 1))) begin
                $display("%m: cache #%2d inst=0x%h <-- IN", i, cache[i]);
            end else if(i == (hash_result(addr_out) & (NUM_ENTRIES - 1))) begin
                $display("%m: cache #%2d inst=0x%h <-- OUT", i, cache[i]);
            end else begin
                $display("%m: cache #%2d inst=0x%h", i, cache[i]);
            end
        end
*/
        
        if(we) begin
            cache[hash_result(addr_in) & (NUM_ENTRIES - 1)] <= data_in;
        end
        data_out <= cache[hash_result(addr_out) & (NUM_ENTRIES - 1)];
    end

    initial begin
        $display("%m: size=%0dKB,data_width=%0d bits", (NUM_ENTRIES / 1000) * (DATA_WIDTH / 8), DATA_WIDTH);
    end
endmodule
