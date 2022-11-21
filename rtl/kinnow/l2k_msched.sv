///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 Memory Scheduler
//
///////////////////////////////////////////////////////////////////////////////

// TODO: Scheduler will start discarding things once queue is full
// TODO: Writes won't be reflected on reads thata re too close near each other!

`include "defines.sv"

module l2k_msched
#( // Parameters
    MAX_CMDS = 32
)
( // Interface
    input rst,
    input clk,
    output reg [31:0] ram_addr,
    input [31:0] ram_data_in,
    output reg [31:0] ram_data_out,
    input ram_rdy,
    output reg ram_we, // Write-enable
    output reg ram_ce, // Command-enable

    input [31:0] client_read_addr,
    output [31:0] client_read_value,
    input [1:0] client_read_size,
    input client_read_enable,
    output client_read_rdy,
    output [31:0] client_read_addr_in,

    input [31:0] client_write_addr,
    input [31:0] client_write_value,
    input [1:0] client_write_size,
    input client_write_enable,
    output client_write_rdy,

    output full,
    input flush
);
    typedef struct packed {
        bit write;
        bit enable; // 0=disabled,1=enabled
        bit [31:0] value;
        bit [31:0] addr;
        bit [1:0] size;
    } st_memory_command;
    st_memory_command cmds[0:MAX_CMDS - 1];
    st_memory_command exec_cmd;

    reg [4:0] curr_client_cmd;
    reg [4:0] curr_exec_cmd;

    // Remember, for non-32-bit non-aligned writes we have to fetch the motherfucking
    // thing before writing to it, ugh
    reg write_fetched;
    reg [31:0] write_fetched_data;

    integer i;

    assign client_read_value = ram_data_in;

    // Do the required masking for reading and writing back RAM values
    // Assumes address if already aligned to the 32-bit boundary
    function [31:0] memio_aligned_write_mask(
        input [1:0] write_size,
        input [31:0] write_addr,
        input [31:0] ram_data,
        input [31:0] write_data
    );
        // Check docs/isa.txt "Shortcuts - Trans size" for an explanation
        // TODO: This "optimization" might be broken, also it's unreadable wtf
        $display("%m: Write aligned write_size=%0d,mask=0x%h", (1 << write_size) * 8, (1 << ((1 << write_size) * 8)) - 1);
        case(write_size)
        2'b11: begin
            memio_aligned_write_mask <= 0;
            end
        2'b10: begin
            memio_aligned_write_mask <= write_data;
            end
        2'b01: begin
            memio_aligned_write_mask <=
                (ram_data & (32'hFFFF << (((write_addr + 1) & 1) * 16)))
                | ({ 16'h0, write_data[15:0] } << ((write_addr & 1) * 16));
            end
        2'b00: begin
            memio_aligned_write_mask <=
                (ram_data & `ROR(32'hFFFFFF, (((write_addr + 1) & 3) * 8), 32))
                | ({ 24'h0, write_data[7:0] } << ((write_addr & 1) * 16));
            end
        endcase
    endfunction

    always @(posedge clk) begin
        if(rst) begin
            for(i = 0; i < MAX_CMDS; i++) begin
                cmds[i].enable <= 0;
            end
        end
        if(flush) begin
            for(i = 0; i < MAX_CMDS; i++) begin
                if(!cmds[i].write) begin
                    cmds[i].enable <= 0;
                end
            end
        end
    end

    // Obtain commands from the client and add them to the queue
    always @(posedge clk) begin
        client_read_rdy <= 0;
        client_write_rdy <= 0;
        full <= 1; // Check if we're full or not
        for (i = 0; i < MAX_CMDS; i++) begin
            if(i == curr_client_cmd) begin
                if(!cmds[i].enable) begin
                    full <= 0; // Not full!
                    if(client_read_enable) begin
                        cmds[i].enable <= 1;
                        cmds[i].write <= 0;
                        cmds[i].addr <= client_read_addr;
                        cmds[i].size <= client_read_size;
                        if(client_write_enable) begin // Two read and write at the same time
                            cmds[i + 1].enable <= 1;
                            cmds[i + 1].write <= 1;
                            cmds[i + 1].addr <= client_write_addr;
                            cmds[i + 1].value <= client_write_value;
                            cmds[i + 1].size <= client_write_size;
                            client_write_rdy <= 1;
                            curr_client_cmd <= curr_client_cmd + 2;
                            $display("%m: (write command) 0x%0x=0x%0x", client_write_addr, client_write_value);
                        end else begin
                            curr_client_cmd <= curr_client_cmd + 1;
                        end
                        $display("%m: (read command) 0x%0x", client_read_addr);
                    end else if(client_write_enable) begin
                        cmds[i].enable <= 1;
                        cmds[i].write <= 1;
                        cmds[i].addr <= client_write_addr;
                        cmds[i].value <= client_write_value;
                        cmds[i].size <= client_write_size;
                        client_write_rdy <= 1;
                        curr_client_cmd <= curr_client_cmd + 1;
                        $display("%m: (write command) 0x%0x=0x%0x", client_write_addr, client_write_value);
                    end
                end else begin // Advance until a non-enabled command is found
                    curr_client_cmd <= curr_client_cmd + 1;
                end
            end
        end
        if(rst || flush) begin
            curr_client_cmd <= 1;
        end
    end

    // Process commands
    always @(posedge clk) begin
        client_read_rdy <= 0;
        client_write_rdy <= 0;
        ram_ce <= 0; // Disable unless enabled
        for (i = 0; i < MAX_CMDS; i++) begin
            if(i == curr_exec_cmd) begin
                ram_addr <= cmds[i].addr;
                // Make sure not to read what is being written at the same time ;)
                if(curr_exec_cmd != curr_client_cmd && cmds[i].enable) begin
                    if(cmds[i].write) begin // Write
                        if(cmds[i].size == 2'b11) begin // 32 bits, instantly-write
                            $display("%m: write-32 0x%0x=0x%0x", cmds[i].addr, cmds[i].value);
                            ram_ce <= 1;
                            ram_we <= 1;
                            ram_data_out <= cmds[i].value;
                            if(ram_rdy) begin
                                cmds[i].enable <= 0;
                            end
                        end else begin // 8 and 16 bits
                            if(!write_fetched) begin // Obtain the value from the RAM first
                                $display("%m: write-r 0x%0x=0x%0x", cmds[i].addr, cmds[i].value);
                                ram_ce <= 1;
                                ram_we <= 0;
                                if(ram_rdy) begin
                                    write_fetched_data <= ram_data_in;
                                    write_fetched <= 1;
                                end
                            end else begin // After obtaining the value from read, write it back
                                $display("%m: write-w 0x%0x=0x%0x", cmds[i].addr, cmds[i].value);
                                ram_ce <= 1;
                                ram_we <= 1;
                                ram_data_out <= memio_aligned_write_mask(cmds[i].size, cmds[i].addr, write_fetched_data, cmds[i].value);
                                if(ram_rdy) begin
                                    write_fetched <= 0;
                                    cmds[i].enable <= 0;
                                end
                            end
                        end
                    end else begin // Read
                        ram_ce <= 1;
                        ram_we <= 0;
                        if(ram_rdy) begin
                            cmds[i].enable <= 0;
                            client_read_addr_in <= cmds[i].addr;
                            client_read_rdy <= 1;
                            $display("%m: read 0x%0x=0x%0x", cmds[i].addr, ram_data_in);
                        end
                    end
                end else if(curr_exec_cmd != curr_client_cmd) begin // Advance until an enabled command is found
                    curr_exec_cmd <= curr_exec_cmd + 1;
                end
            end
        end
        if(rst) begin
            curr_exec_cmd <= 0;
            write_fetched <= 0;
        end else if(flush) begin // Only change execution when writing to a memory area
            if(write_fetched == 0) begin
                curr_exec_cmd <= 0;
            end
        end
        $display("%m: curr_exec_cmd=%d,curr_client_cmd=%d", curr_exec_cmd, curr_client_cmd);
    end
endmodule
