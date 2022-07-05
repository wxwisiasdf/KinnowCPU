module limn2600_ALU
( // Parameter
    input rst,
    input clk,
    input [2:0] op,
    input [31:0] in0,
    input [31:0] in1,
    output reg [31:0] out
);
    always @(posedge clk) begin
        casez(op)
            3'b111: begin
                $display("%m: add");
                out <= in0 + in0;
                end
            3'b110: begin
                $display("%m: sub");
                out <= in0 - in1;
                end
            3'b011: begin
                $display("%m: and");
                out <= in0 & in1;
                end
            3'b010: begin
                $display("%m: xor");
                out <= in0 ^ in1;
                end
            3'b001: begin
                $display("%m: or");
                out <= in0 | in1;
                end
            3'b101: begin
                $display("%m: slt");
                out = { 31'h0, in0 < in1 };
                end
            3'b100: begin
                $display("%m: slts");
                // tmp32 is positive, register is negative
                if((in0 & 32'h80000000) != 0 && in1 & 32'h80000000 == 0) begin
                    out <= 1;
                // tmp32 is negative, register is positive
                end else if((in0& 32'h80000000) == 0 && in1 & 32'h80000000 != 0) begin
                    out <= 0;
                end else begin
                    out = { 31'h0, in0 < in1 };
                end
                end
            3'b000: begin
                out <= 0;
                end
        endcase
    end
endmodule

///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 Core
//
// Executes an instruction, multiple instances of this module are used
// for executing the whole instruction queue
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_Core
( // Interface
    input rst,
    input clk,
    input irq,
    output reg [31:0] addr,
    input [31:0] data_in,
    output reg [31:0] data_out,
    input rdy, // Whetever we can fetch instructions
    output reg we, // Write-Enable (1 = we want to write, 0 = we want to read)
    output reg cs // Command-State (1 = memory commands active, 0 = memory commands ignored)
);
    // Perform an ALU op
    function [31:0] alu_op_result(
        input [2:0] op,
        input [31:0] a,
        input [31:0] b
    );
        casez(op)
            3'b111: begin // ADD
                $display("%m: add");
                alu_op_result = a + b;
                end
            3'b110: begin // SUB
                $display("%m: sub");
                alu_op_result = a - b;
                end
            3'b011: begin // AND
                $display("%m: and");
                alu_op_result = a & b;
                end
            3'b010: begin // XOR
                $display("%m: xor");
                alu_op_result = a ^ b;
                end
            3'b001: begin // OR
                $display("%m: or");
                alu_op_result = a | b;
                end
            3'b101: begin // SLT
                $display("%m: slt");
                alu_op_result = { 31'h0, a < b };
                end
            3'b100: begin // SLTS
                $display("%m: slts");
                // tmp32 is positive, register is negative
                if((a & 32'h80000000) != 0 && (b & 32'h80000000) == 0) begin
                    alu_op_result = 1;
                // tmp32 is negative, register is positive
                end else if((a & 32'h80000000) == 0 && (b & 32'h80000000) != 0) begin
                    alu_op_result = 0;
                end else begin
                    alu_op_result = { 31'h0, a < b };
                end
                end
            default: begin end
        endcase
    endfunction

    // Do the required masking for reading and writing back RAM values
    // Assumes address if already aligned to the 32-bit boundary
    function [31:0] memio_aligned_write_mask(
        input [1:0] size, // Bits from the MOV instruction
        input [31:0] addr, // Address of write
        input [31:0] ramdata, // Data of the RAM
        input [31:0] data // Data to write
    );
        // Check docs/isa.txt "Shortcuts - Trans size" for an explanation
        $display("%m: Write aligned size=%0d,mask=0x%h", (1 << (~trans_size)) * 8, (1 << ((1 << (~trans_size)) * 8)) - 1);
        memio_aligned_write_mask = (ramdata & ~(((1 << ((1 << (~trans_size)) * 8)) - 1) << ((addr & ((1 << (~trans_size)) - 1)) * 8))) | ((data & ((1 << ((1 << (~trans_size)) * 8)) - 1)) << ((addr & ((1 << (~trans_size)) - 1)) * 8));
    endfunction

    // State
    reg [31:0] tmp32; // Temporal value
    reg [31:0] regs[0:31]; // Limnstation has 32 registers
    reg [31:0] ctl_regs[0:31];
    reg [31:0] pc; // Program counter
    reg [2:0] state;

    // I/O machine
    reg [1:0] trans_size; // Memory transfer size
    reg [31:0] write_value; // Write value
    reg [4:0] read_regno; // Register to place the read of memory into
    reg [31:0] memio_addr;  // Read-Write address, please when writing to this register
                            // make sure to also update addr at the same time!

    localparam S_FETCH = 0;
    localparam S_EXECUTE = 1;
    localparam S_PREWRITE = 2;
    localparam S_WRITE = 3;
    localparam S_READ = 4;
    localparam S_HALT = 5;
    localparam S_BRANCHED = 6;
    
    // Instruction fetcher
    wire [31:0] fetch_addr = pc; // Address to fetch on, reset on JAL/J/BR
    wire [31:0] execute_inst = icache_data_out; // Instruction to execute

    // Branch prediction (fetcher stage)
    reg [3:0] regs_predict[0:31]; // Flags for the BP to tag registers
    parameter
        RP_NON_ZERO = 4'b0001, // Register is non-zero
        RP_ZERO = 4'b0000, // Register might be zero
        RP_UNSPEC_MEM = 4'b0010; // Register depends on memory value

    // Instruction executor
    wire [28:0] imm29 = execute_inst[31:3];
    wire [4:0] imm5 = execute_inst[15:11];
    wire [15:0] imm16 = execute_inst[31:16];
    wire [22:0] imm21 = execute_inst[31:9];
    wire [21:0] imm22 = execute_inst[27:6]; // BRK and SYS
    wire [5:0] inst_lo = execute_inst[5:0];
    wire [4:0] opreg1 = execute_inst[10:6];
    wire [4:0] opreg2 = execute_inst[15:11];
    wire [4:0] opreg3 = execute_inst[20:16];
    wire [4:0] opreg4 = execute_inst[25:21];
    wire [5:0] inst_hi = execute_inst[31:26];
    wire [1:0] opg1_instmode = execute_inst[27:26];
    // Register numbers
    parameter
        REG_LR = 31;
    parameter
        CREG_RS = 0, // Processor status
        CREG_ERS = 1, // TODO: This isn't standard on Limn2600
        CREG_TBLO = 2, // TLB entry
        CREG_EPC = 3, // PC before except
        CREG_EVEC = 4,
        CREG_PGTB = 5,
        CREG_TBINDEX = 6,
        CREG_EBADADDR = 7,
        CREG_TBVEC = 8,
        CREG_FWVEC = 9,
        CREG_TBSCRATCH = 10,
        CREG_TBHI = 11;
    // MMU and exceptions
    parameter
        ECAUSE_INTERRUPT = 2,
        ECAUSE_SYSCALL = 3,
        ECAUSE_BUS_ERROR = 4,
        ECAUSE_BREAKPOINT = 6,
        ECAUSE_INVALID_INST = 7,
        ECAUSE_PRIV_VIOLAT = 8,
        ECAUSE_UNALIGNED = 9,
        ECAUSE_PAGE_FAULT = 12,
        ECAUSE_PAGE_FAULT_WR = 13,
        ECAUSE_TLB_REFILL = 15;

    integer i;

    reg icache_we;
    reg [31:0] icache_addr_in;
    wire [31:0] icache_data_out;
    limn2600_cache icache(
        .rst(rst),
        .clk(clk),
        .we(icache_we),
        .addr_in(icache_addr_in),
        .addr_out(pc),
        .data_in(data_in),
        .data_out(icache_data_out)
    );

    always @(posedge clk) begin
        if(rst) begin
            $display("%m: Reset");
            pc <= 32'hFFFE0000;
            state <= S_EXECUTE;
            icache_addr_in <= 0;
            cs <= 0;
            we <= 0;
            icache_we <= 0;
        end else if(irq) begin
            $display("%m: IRQ event!");
            ctl_regs[CREG_EBADADDR] <= pc;
            pc <= ctl_regs[CREG_EVEC];
            ctl_regs[CREG_RS][31:28] <= ECAUSE_INTERRUPT;
            state <= S_BRANCHED;
        end
        regs[0] <= 0;
        regs_predict[0] <= 0;
    end

    // Fetch
    always @(posedge clk) begin
        // Continous assignments, can be overriden by below statments
        cs <= 0;
        we <= 0;
        icache_we <= 0;

        // Read a from the SRAM (1/1 cycles)
        if(state == S_READ) begin
            $display("%m: S_READ");
            cs <= 1;
            if(rdy) begin // Appropriately apply masks
                // Check docs/isa.txt "Shortcuts - Trans size" for an explanation
                $display("%m: Read size=%0d,mask=0x%h", (1 << (~trans_size)) * 8, (1 << ((1 << (~trans_size)) * 8)) - 1);
                regs[read_regno] <= (data_in & ((1 << ((1 << (~trans_size)) * 8)) - 1)) << ((memio_addr & ((1 << (~trans_size)) - 1)) * ((1 << (~trans_size)) * 8));
                state <= S_EXECUTE;
            end
        // Fetch the element from SRAM with 32-bits per unit of data
        // rememeber that we also need to write bytes so unaligned accesses
        // are allowed by the CPU because fuck you, addr should already be set
        // (1/2 cycles)
        end else if(state == S_PREWRITE) begin
            // Prewrite is in charge of reading the value and then writting it back with the desired offset
            // so we can support unaligned accesses
            $display("%m: S_PREWRITE");
            cs <= 1;
            if(rdy) begin
                // Appropriately apply masks
                if((memio_addr & 3) == 0) begin // Aligned access
                    if(trans_size == 2'b00) begin // 4-bytes, 1-per-cell
                        $display("%m: prewrite long");
                        we <= 1; // Since data_width == 32 we simply send the whole thing
                        data_out <= write_value;
                        state <= S_EXECUTE;
                        cs <= 0; // Disable commands
                    end else begin
                        write_value <= memio_aligned_write_mask(trans_size, memio_addr, data_in, write_value);
                    end
                end else begin // Unaligned access
                    $display("%m: unaligned write!");
                    ctl_regs[CREG_EBADADDR] <= pc;
                    pc <= ctl_regs[CREG_EVEC];
                    ctl_regs[CREG_RS][31:28] <= ECAUSE_UNALIGNED;
                    state <= S_BRANCHED;
                end
                state <= S_WRITE;
                $display("%m: write_value=0x%h,memio_addr=0x%h,data_in=0x%h", write_value, memio_addr, data_in);
            end
        // After fetching the value and appropriately setting the masks
        // send it to the SRAM, assumes that addr is set by the one who set this state
        // (2/2 cycles)
        end else if(state == S_WRITE) begin
            $display("%m: S_WRITE");
            cs <= 1;
            we <= 1; // Write the value, then return to fetching
            data_out <= write_value;
            if(rdy) begin
                $display("%m: data_out=0x%h,write_value=0x%h,addr=0x%h", data_out, write_value, addr);
                state <= S_EXECUTE;
                cs <= 0; // Disable commands
            end
        end else if(state == S_FETCH) begin
            $display("%m: (Fetch) Fetching");
            cs <= 1; // Read from memory
            addr <= fetch_addr;
            icache_addr_in <= fetch_addr;
            // Once we can fetch instructions we save the state, but only if
            // we aren't overwriting something being used by the executor!
            if(rdy) begin
                $display("%m: (Fetch) Fetched inst=%b,fetch=0x%h", data_in, fetch_addr);
                icache_we <= 1; // Write the fetched instruction onto the i-cache
                state <= S_EXECUTE;
                cs <= 0; // Disable commands
            end
        // Execution thread
        end else if(state == S_BRANCHED) begin
            $display("%m: S_BRANCHED");
            state <= S_FETCH;
        end else if(state == S_EXECUTE) begin
            $display("%m: Execution icache_data_out=0x%h,inst=0x%h,pc=0x%h", icache_data_out, execute_inst, pc);
            state <= S_FETCH;
            casez(inst_lo)
                // This is an invalid opcode, but used internally as a "true no-op", no PC is modified
                // no anything is modified, good for continuing the executor without stalling
                //
                // NOTE: While the Limn2600 spec says this is not a vlid opcode
                // we say otherwise, and this is only used internally, so hopefully
                // nothing bad happens from this!
                6'b00_0000: begin
                    $display("%m: tnop");
                    end
                // JALR [rd], [ra], [imm29]
                6'b11_1000: begin
                    $display("%m: jalr r%d,r%d,[%h]", opreg1, opreg2, { 8'h0, imm16 } << 2);
                    regs[opreg1] <= pc + 4;
                    pc <= regs[opreg2] + ({ 16'h0, imm16 } << 2); // TODO: Sign extend
                    state <= S_BRANCHED;
                    end
                // JAL [imm29]
                6'b??_?11?: begin
                    $display("%m: jal [0x%8h],lr=0x%8h", { 3'h0, imm29 } << 2, pc + 4);
                    if(execute_inst[0] == 1) begin
                        regs[REG_LR] <= pc + 4;
                    end
                    pc <= (pc & 32'h80000000) | ({ 3'h0, imm29 } << 2);
                    end
                // BEQ ra, [imm21]
                6'b11_1101: begin
                    if(regs[opreg1] == 32'h0) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin
                            $display("%m: beq r%d,[-%0d]", opreg1, ~imm21[19:0]);
                            pc <= pc - ({ 12'h0, ~imm21[19:0] } << 2);
                        end else begin
                            $display("%m: beq r%d,[%0d]", opreg1, imm21[19:0]);
                            pc <= pc + ({ 12'h0, imm21[19:0] } << 2);
                        end
                    end else begin
                        pc <= pc + 4;
                    end
                    state <= S_BRANCHED;
                    end
                // BNE ra, [imm21]
                6'b11_0101: begin
                    if(regs[opreg1] != 32'h0) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin
                            $display("%m: bne r%d,[-%0d]", opreg1, ~imm21[19:0]);
                            pc <= pc - ({ 12'h0, ~imm21[19:0] } << 2);
                        end else begin
                            $display("%m: bne r%d,[%0d]", opreg1, imm21[19:0]);
                            pc <= pc + ({ 12'h0, imm21[19:0] } << 2);
                        end
                    end else begin
                        pc <= pc + 4;
                    end
                    state <= S_BRANCHED;
                    end
                // BLT ra, [imm21]
                6'b10_1101: begin
                    // Branch taken if it's less than 0 (signed comparison)
                    if(regs[opreg1][31] == 1) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin
                            $display("%m: blt r%d,[-%0d]", opreg1, ~imm21[19:0]);
                            pc <= pc - ({ 12'h0, ~imm21[19:0] } << 2);
                        end else begin
                            $display("%m: blt r%d,[%0d]", opreg1, imm21[19:0]);
                            pc <= pc + ({ 12'h0, imm21[19:0] } << 2);
                        end
                    end else begin
                        pc <= pc + 4;
                    end
                    state <= S_BRANCHED;
                    end
                // ADDI [rd], [rd], [imm16]
                // SUBI [rd], [rd], [imm16]
                // SLTI [rd], [rd], [imm16]
                // SLTIS [rd], [rd], [imm16]
                // ANDI [rd], [rd], [imm16]
                // XORI [rd], [rd], [imm16]
                // ORI [rd], [rd], [imm16]
                // LUI [rd], [rd], [imm16]
                6'b??_?100: begin
                    $display("%m: imm_alu_inst r%d,r%d,[0x%h]", opreg1, opreg2, imm16);
                    if(inst_lo[5:3] == 0) begin // LUI
                        $display("%m: lui r%d,r%d,[0x%h]", opreg1, opreg2, imm16);
                        regs[opreg1] <= regs[opreg2] | ({ 16'b0, imm16 } << 16);
                        // TODO: Fuse OPS for example LA comes as LUI+ORI
                    end else begin // Rest of ops
                        regs[opreg1] <= alu_op_result(inst_lo[5:3], regs[opreg2], { 16'b0, imm16 });
                    end
                    pc <= pc + 4;
                    end
                6'b1?_?011: begin // MOV rd, [rs + imm16]
                    $display("%m: mov(5)(R) [r%d+%h],[%d],sz=%b", opreg1, { 8'h0, imm16 }, imm5, inst_lo[4:3]);
                    trans_size <= inst_lo[4:3];
                    read_regno <= opreg1;
                    memio_addr <= regs[opreg2] + { 16'h0, imm16 << (~trans_size) };
                    addr <= regs[opreg2] + { 16'h0, imm16 << (~trans_size) };
                    state <= S_PREWRITE;
                    cs <= 1;
                    pc <= pc + 4;
                    end
                6'b??_?010: begin // MOV [ra + imm16], rb
                    trans_size <= inst_lo[4:3];
                    if(inst_lo[5] == 1) begin // Move register to memory
                        $display("%m: mov(16)(W) [r%d+%h],r%d,sz=%b", opreg1, { 16'h0, imm16 }, opreg2, inst_lo[4:3]);
                        write_value <= regs[opreg2];
                    end else begin // Move immediate to memory
                        $display("%m: mov(5)(W) [r%d+%h],r%d,sz=%b", opreg1, { 16'h0, imm16 }, opreg2, inst_lo[4:3]);
                        write_value <= { 27'h0, imm5 };
                    end
                    memio_addr <= regs[opreg1] + { 16'h0, imm16 << (~trans_size) };
                    addr <= regs[opreg1] + { 16'h0, imm16 << (~trans_size) };
                    state <= S_PREWRITE;
                    cs <= 1;
                    pc <= pc + 4;
                    end
                // Instructions starting with 111001
                6'b11_1001: begin
                    $display("%m: alu_inst r%d,r%d,r%d,instmode=%b,op=%b", opreg1, opreg2, opreg3, opg1_instmode, inst_hi);
                    // Instmode
                    casez(opg1_instmode)
                        2'b00: tmp32 <= regs[opreg3] >> { 26'h0, imm5};
                        2'b01: tmp32 <= regs[opreg3] << { 26'h0, imm5};
                        2'b10: tmp32 <= regs[opreg3] >>> { 26'h0, imm5}; // TODO: What is ASH?
                        2'b11: tmp32 <= regs[opreg3] >>> { 26'h0, imm5};
                    endcase

                    // Check for special operators
                    casez(inst_hi[5:2])
                        4'b0000: begin // NOR ra,rb,rd
                            $display("%m: nor");
                            regs[opreg1] <= ~(regs[opreg2] | tmp32);
                            end
                        4'b11??: begin // Move-From-Registers
                            $display("%m: mov(W)(REG) [r%d+r%d+%d],r%d,sz=%b", opreg2, opreg3, imm5, opreg1, inst_hi[3:2]);
                            trans_size <= inst_hi[3:2];
                            write_value <= regs[opreg1];
                            memio_addr <= regs[opreg2] + tmp32;
                            addr <= regs[opreg2] + tmp32;
                            state <= S_PREWRITE;
                            cs <= 1;
                            end
                        4'b10??: begin // Move-To-Register
                            $display("%m: mov(R)(REG) r%d,[r%d+r%d+%d],sz=%b", opreg1, opreg2, opreg3, imm5, inst_hi[3:2]);
                            trans_size <= inst_hi[3:2];
                            read_regno <= opreg1;
                            memio_addr <= regs[opreg2] + tmp32;
                            addr <= regs[opreg2] + tmp32;
                            state <= S_READ;
                            cs <= 1;
                            end
                        default: begin // Assume this is a general ALU OP and perform a normal operation
                            regs[opreg1] <= alu_op_result(inst_hi[4:2], regs[opreg2], tmp32);
                            end
                    endcase
                    pc <= pc + 4;
                    end
                6'b10_1001: begin
                    $display("%m: privileged_inst");
                    casez(inst_hi[5:2])
                    // MFCR [opreg1] [opreg3]
                    4'b1111: begin
                        $display("%m: mfcr r%d,cr%d", opreg1, opreg3);
                        regs[opreg1] <= ctl_regs[opreg3];
                        pc <= pc + 4;
                        end
                    // MTCR [opreg3] [opreg2]
                    4'b1110: begin
                        $display("%m: mtcr cr%d,r%d", opreg3, opreg2);
                        ctl_regs[opreg3] <= regs[opreg2];
                        pc <= pc + 4;
                        end
                    // CACHEI [imm22]
                    4'b1000: begin
                        $display("%m: cachei [%h]", imm22);
                        pc <= pc + 4;
                        end
                    // FWC [imm22]
                    4'b1010: begin
                        ctl_regs[CREG_EBADADDR] <= pc;
                        pc <= ctl_regs[CREG_FWVEC];
                        ctl_regs[CREG_RS][31:28] <= ECAUSE_SYSCALL;
                        end
                    // HLT [imm22]
                    4'b1100: begin
                        $display("%m: hlt [%h]", imm22);
                        pc <= pc + 4;
                        state <= S_HALT;
                        end
                    default: begin // Invalid instruction
                        $display("%m: invalid_grp2=0b%b", inst_hi[5:2]);
                        ctl_regs[CREG_EBADADDR] <= pc;
                        pc <= ctl_regs[CREG_EVEC];
                        ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                        end
                    endcase
                    end
                6'b11_0001: begin
                    $display("%m: advanced_cohost_alu");
                    casez(inst_hi[5:2])
                        4'b0001: begin
                            $display("%m: brk [%h]", imm22);
                            // TODO: Is imm22 used at all?
                            ctl_regs[CREG_EBADADDR] <= pc;
                            pc <= ctl_regs[CREG_EVEC];
                            ctl_regs[CREG_RS][31:28] <= ECAUSE_BREAKPOINT;
                            state <= S_BRANCHED;
                            end
                        4'b1101: begin
                            if(opreg4 != 5'b0) begin
                                // Raise UD
                                $display("%m: exception - invalid div inst=%b", execute_inst);
                                ctl_regs[CREG_EBADADDR] <= pc;
                                pc <= ctl_regs[CREG_EVEC];
                                ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                                state <= S_BRANCHED;
                            end else begin
                                regs[opreg1] <= regs[opreg2] / regs[opreg3];
                                pc <= pc + 4;
                            end
                            end
                        4'b1100: begin
                            if(opreg4 != 5'b0) begin
                                // Raise UD
                                $display("%m: exception - invalid divs inst=%b", execute_inst);
                                ctl_regs[CREG_EBADADDR] <= pc;
                                pc <= ctl_regs[CREG_EVEC];
                                ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                                state <= S_BRANCHED;
                            end else begin
                                // TODO: Properly perform signed division
                                regs[opreg1] <= { (regs[opreg2][31] | regs[opreg3][31]), regs[opreg2][30:0] / regs[opreg3][30:0] };
                                pc <= pc + 4;
                            end
                            end
                        4'b1001: begin
                            // TODO: Is this a memory access?
                            pc <= pc + 4;
                            end
                        4'b1011: begin
                            if(opreg4 != 5'b0) begin
                                // Raise UD
                                $display("%m: exception - invalid mod inst=%b", execute_inst);
                                ctl_regs[CREG_EBADADDR] <= pc;
                                pc <= ctl_regs[CREG_EVEC];
                                ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                                state <= S_BRANCHED;
                            end else begin
                                regs[opreg1] <= regs[opreg2] % regs[opreg3];
                                pc <= pc + 4;
                            end
                            end
                        4'b1111: begin
                            if(opreg4 != 5'b0) begin
                                // Raise UD
                                $display("%m: exception - invalid mul inst=%b", execute_inst);
                                ctl_regs[CREG_EBADADDR] <= pc;
                                pc <= ctl_regs[CREG_EVEC];
                                ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                                state <= S_BRANCHED;
                            end else begin
                                regs[opreg1] <= regs[opreg2] * regs[opreg3];
                                pc <= pc + 4;
                            end
                            end
                        4'b1000: begin
                            // TODO: Is this a memory access?
                            pc <= pc + 4;
                            end
                        4'b0000: begin
                            $display("%m: sys [%h]", imm22);
                            // TODO: Is imm22 used at all?
                            ctl_regs[CREG_EBADADDR] <= pc;
                            pc <= ctl_regs[CREG_EVEC];
                            ctl_regs[CREG_RS][31:28] <= ECAUSE_SYSCALL;
                            state <= S_BRANCHED;
                            end
                        default: begin
                            end
                    endcase
                    end
                default: begin
                    $display("%m: invalid_opcode,inst=%b", execute_inst);
                    ctl_regs[CREG_EBADADDR] <= pc;
                    pc <= ctl_regs[CREG_EVEC];
                    ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                    state <= S_BRANCHED;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        $display("%m: state=%d,pc=0x%8h,data_in=0x%8h,data_out=0x%8h,addr=0x%8h", state, pc, data_in, data_out, addr);
        for(i = 0; i < 32; i = i + 8) begin
            $display("%m: %2d=0x%8h,%2d=0x%8h,%2d=0x%8h,%2d=0x%8h,%2d=0x%8h,%2d=0x%8h,%2d=0x%8h,%2d=0x%8h", i, regs[i], i + 1, regs[i + 1], i + 2, regs[i + 2], i + 3, regs[i + 3], i + 4, regs[i + 4], i + 5, regs[i + 5], i + 6, regs[i + 6], i + 7, regs[i + 7]);
        end
    end
endmodule

///////////////////////////////////////////////////////////////////////////////
//
// Limn2600 Core
//
// Executes an instruction, multiple instances of this module are used
// for executing the whole instruction queue
//
///////////////////////////////////////////////////////////////////////////////
module limn2600_CPU
#( // Parameters
    parameter NUM_CORES = 4
)
( // Interface
    input rst,
    input clk,
    input irq,
    output [31:0] addr,
    input [31:0] data_in,
    output [31:0] data_out,
    input rdy, // Whetever we can fetch instructions
    output we, // Write-Enable (1 = we want to write, 0 = we want to read)
    output cs // Command-State (1 = memory commands active, 0 = memory commands ignored)
);

    limn2600_Core core1(
        .rst(rst),
        .clk(clk),
        .irq(irq),
        .addr(addr),
        .data_in(data_in),
        .data_out(data_out),
        .rdy(rdy),
        .we(we),
        .cs(cs)
    );
/*
    generate
        genvar i;
        for(i = 0; i < NUM_CORES; i++) begin
            limn2600_Core core[0:NUM_CORES - 1](
                .rst(rst),
                .clk(clk),
                .irq(irq),
                .addr(addr),
                .data_in(data_in),
                .data_out(data_out),
                .rdy(rdy),
                .we(we),
                .cs(cs)
            );
        end
    endgenerate
*/
endmodule
