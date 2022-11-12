///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 Memory Scheduler
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_MemorySched
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
    output reg [31:0] client_read_value,
    input [1:0] client_read_size,
    input client_read_enable,
    output client_read_rdy,

    input [31:0] client_write_addr,
    input [31:0] client_write_value,
    input [1:0] client_write_size,
    input client_write_enable,
    output client_write_rdy
);
    typedef struct packed {
        bit write;
        bit enable; // 0=disabled,1=enabled
        bit [31:0] value;
        bit [31:0] addr;
        bit [1:0] size;
    } st_memory_command;
    st_memory_command cmds[0:31];
    reg [4:0] curr_client_cmd;
    reg [4:0] curr_exec_cmd;

    // Remember, for non-32-bit non-aligned writes we have to fetch the motherfucking
    // thing before writing to it, ugh
    reg write_fetched;
    reg [31:0] write_fetched_data;

    integer i;

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
            memio_aligned_write_mask <= ram_data;
            end
        2'b01: begin
            memio_aligned_write_mask <= (ram_data & (32'hFFFF << ((write_addr & 1) * 16))) >> ((write_addr & 1) * 16);
            end
        2'b00: begin
            memio_aligned_write_mask <= (ram_data & (32'hFF << ((write_addr & 3) * 8))) >> ((write_addr & 3) * 8);
            end
        endcase
    endfunction

    always @(posedge clk) begin
        if(rst) begin
            write_fetched <= 0;
            curr_client_cmd <= 0;
            for(i = 0; i < 32; i++) begin
                cmds[i].enable <= 0;
            end
        end
    end

    // Obtain commands from the client and add them to the queue
    always @(posedge clk) begin
        client_read_rdy <= 0;
        client_write_rdy <= 0;
        if(!cmds[curr_client_cmd].enable) begin
            if(client_read_enable) begin
                cmds[curr_client_cmd].enable <= 1;
                cmds[curr_client_cmd].write <= 0;
                cmds[curr_client_cmd].addr <= client_read_addr;
                cmds[curr_client_cmd].size <= client_read_size;
                if(client_write_enable) begin // Two read and write at the same time
                    cmds[curr_client_cmd + 1].enable <= 1;
                    cmds[curr_client_cmd + 1].write <= 1;
                    cmds[curr_client_cmd + 1].addr <= client_write_addr;
                    cmds[curr_client_cmd + 1].value <= client_write_value;
                    cmds[curr_client_cmd + 1].size <= client_write_size;
                    client_write_rdy <= 1;
                    curr_client_cmd <= curr_client_cmd + 2;
                    $display("%m: (write command) 0x%0x=0x%0x", client_write_addr, client_write_value);
                end else begin
                    curr_client_cmd <= curr_client_cmd + 1;
                end
                $display("%m: (read command) 0x%0x=0x%0x", client_read_addr, client_read_value);
            end else if(client_write_enable) begin
                cmds[curr_client_cmd].enable <= 1;
                cmds[curr_client_cmd].write <= 1;
                cmds[curr_client_cmd].addr <= client_write_addr;
                cmds[curr_client_cmd].value <= client_write_value;
                cmds[curr_client_cmd].size <= client_write_size;
                client_write_rdy <= 1;
                curr_client_cmd <= curr_client_cmd + 1;
                $display("%m: (write command) 0x%0x=0x%0x", client_write_addr, client_write_value);
            end
        end else begin // Advance until a non-enabled command is found
            curr_client_cmd <= curr_client_cmd + 1;
        end
    end

    // Process commands
    always @(posedge clk) begin
        client_read_rdy <= 0;
        client_write_rdy <= 0;
        ram_ce <= 0; // Disable unless enabled
        // Make sure not to read what is being written at the same time ;)
        if(cmds[curr_exec_cmd].enable && curr_exec_cmd != curr_client_cmd) begin
            if(cmds[curr_exec_cmd].write) begin // Write
                if(cmds[curr_exec_cmd].size == 2'b11) begin // 32 bits, instantly-write
                    ram_ce <= 1;
                    ram_we <= 1;
                    ram_addr <= cmds[curr_exec_cmd].addr;
                    ram_data_out <= cmds[curr_exec_cmd].value;
                    if(ram_rdy) begin
                        cmds[curr_exec_cmd].enable <= 0;
                    end
                end else begin // 8 and 16 bits
                    if(!write_fetched) begin // Obtain the value from the RAM first
                        ram_ce <= 1;
                        ram_we <= 0;
                        ram_addr <= cmds[curr_exec_cmd].addr;
                        if(ram_rdy) begin
                            write_fetched_data <= ram_data_in;
                            write_fetched <= 1;
                        end
                    end else begin // After obtaining the value from read, write it back
                        ram_ce <= 1;
                        ram_we <= 1;
                        ram_addr <= cmds[curr_exec_cmd].addr;
                        ram_data_out <= memio_aligned_write_mask(cmds[curr_exec_cmd].size, cmds[curr_exec_cmd].addr, write_fetched_data, cmds[curr_exec_cmd].value);
                        if(ram_rdy) begin
                            write_fetched <= 0;
                            cmds[curr_exec_cmd].enable <= 0;
                        end
                    end
                end
            end else begin // Read
                ram_ce <= 1;
                ram_we <= 0;
                ram_addr <= cmds[curr_exec_cmd].addr;
                if(ram_rdy) begin
                    cmds[curr_exec_cmd].enable = 0;
                    client_read_value <= ram_data_in;
                    client_read_rdy <= 1;
                    $display("%m: read 0x%0x=0x%0x", cmds[curr_exec_cmd].addr, ram_data_in);
                end
            end
        end else begin // Advance until an enabled command is found
            curr_exec_cmd <= curr_exec_cmd + 1;
        end
        $display("%m: curr_exec_cmd=%d,curr_client_cmd=%d", curr_exec_cmd, curr_client_cmd);
    end
endmodule
