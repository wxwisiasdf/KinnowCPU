///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 l2k_cache
//
///////////////////////////////////////////////////////////////////////////////
module l2k_cache
#( // Parameter
    parameter DATA_WIDTH = 32,
    parameter NUM_ENTRIES = 512
)
( // Interface
    input rst,
    input clk,
    input we, // Write enable
    input [31:0] addr_in, // Address-in
    input [DATA_WIDTH - 1:0] data_in, // Data-in
    input [31:0] addr_out, // Address-out
    output [DATA_WIDTH - 1:0] data_out // Data-out
    // TODO: RDY output state wire
);
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

    reg [DATA_WIDTH - 1:0] cache[0:NUM_ENTRIES - 1];
    always @(posedge clk) begin
        for(i = 0; i < NUM_ENTRIES; i++) begin
            if(rst) begin
                cache[i] = 0;
            end
        end
    end

    // This weird ternary saves us 1 cycle every read
    assign data_out = (addr_in == addr_out) ? data_in
        : cache[hash_result(addr_out) & (NUM_ENTRIES - 1)];
    
    always @(posedge clk) begin
        $display("%m: k_out=0x%h,v_out=0x%h,k_in=0x%h,v_in=0x%h,we=%b,k_out_hash=0x%h,k_in_hash=0x%h", addr_out, data_out, addr_in, data_in, we, hash_result(addr_out), hash_result(addr_in));
        for(i = 0; i < NUM_ENTRIES; i++) begin
            if(we && i == (hash_result(addr_in) & (NUM_ENTRIES - 1))) begin
                $display("%m: cache #%2d(0x%8h) data=0x%h <-- IN", i, addr_in, data_in);
            end else if(i == (hash_result(addr_out) & (NUM_ENTRIES - 1))) begin
                $display("%m: cache #%2d(0x%8h) data=0x%h --> OUT", i, addr_out, cache[i]);
            end else begin
                //$display("%m: cache #%2d data=0x%h", i, cache[i]);
            end
        end

        if(we) begin
            cache[hash_result(addr_in) & (NUM_ENTRIES - 1)] <= data_in;
        end
    end

    initial begin
        $display("%m: size=%0dKB,data_width=%0d bits", (NUM_ENTRIES * (DATA_WIDTH / 8)) / 1000, DATA_WIDTH);
    end
endmodule
