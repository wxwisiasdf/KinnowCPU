`include "rtl/cache.sv"
`include "rtl/alu.sv"
`include "rtl/msched.sv"

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

    output reg [31:0] read_addr,
    input [31:0] read_value,
    output reg [1:0] read_size,
    output read_enable,
    input read_rdy,
    input [31:0] read_addr_in,

    output reg [31:0] write_addr,
    output reg [31:0] write_value,
    output reg [1:0] write_size,
    output write_enable,
    input write_rdy,
    input full,
    output reg flush
);
    function void perform_fetch(
        input [31:0] a_addr
    );
        io_state <= SI_REFETCH;
        pc <= a_addr;
    endfunction

    function void raise_exception(
        input [3:0] cause
    );
        ctl_regs[CREG_EBADADDR] <= pc;
        ctl_regs[CREG_EPC] <= pc;
        ctl_regs[CREG_RS][31:28] <= cause;
        ctl_regs[CREG_ERS] <= ctl_regs[CREG_RS];
        perform_fetch(ctl_regs[CREG_EVEC]); // Switch to fetch state
    endfunction

    // Branches into the given address, resets PC and tells to refetch properly
    function void branch_to(
        input [31:0] new_pc
    );
        $display("%m: Branched to 0x%h", new_pc);
        perform_fetch(new_pc);
    endfunction

    // Prepare a read of data, usually for MOV+R
    function void read_in(
        input [31:0] a_read_addr, // Address to read from
        input [4:0] a_read_reg, // Register to place read value into
        input [1:0] a_size // Size of transfer
    );
        io_state <= SI_READ;
        read_addr <= a_read_addr;
        read_size <= a_size;
        read_enable <= 1;
    endfunction

    function void write_out(
        input [31:0] a_write_addr, // Address to write into
        input [31:0] a_data, // Data to write
        input [1:0] a_size // Size of transfer
    );
        write_addr <= a_write_addr;
        write_value <= a_data;
        write_size <= a_size;
        write_enable <= 1;
    endfunction

    function [31:0] do_alu_shift(
        input [31:0] a,
        input [31:0] b,
        input [1:0] shift_mode
    );
        casez(shift_mode)
            2'b00: do_alu_shift = a << b; // Left shift
            2'b01: do_alu_shift = a >> b; // Right shift
            2'b10: do_alu_shift = { a[31], 31'h0 } | (a >> b); // Arithmethic shift
            2'b11: do_alu_shift = (a >> b) | (a << (32 - b)); // ROR
        endcase
    endfunction

    // State
    reg [31:0] tmp32; // Temporal value
    reg [31:0] regs[0:31]; // Limnstation has 32 registers
    reg [31:0] ctl_regs[0:31];
    reg [31:0] pc; // Program counter

    reg [11:0] fetch_offset; // Prefetcher program counter
    wire [31:0] fetch_pc = { pc[31:12], fetch_offset };

    // I/O machine
    reg [4:0] read_regno; // Register to place the read of memory into

    reg [3:0] io_state;
    localparam SI_STALL = 0;
    localparam SI_FETCH = 1;
    localparam SI_REFETCH = 2;
    localparam SI_READ = 3;
    reg [3:0] ex_state;
    localparam SE_STALL = 0;
    localparam SE_EXECUTE = 1;
    localparam SE_HALT = 2;
    localparam SE_BRANCHED = 3;
    localparam SE_GET_TLB_ENTRY = 4;
    localparam SE_GET_TLB_FN = 5;
    localparam SE_SET_TLB_ENTRY = 6;

    wire [31:0] inst; // Instruction to execute
    reg icache_we;
    limn2600_Cache #(.NUM_ENTRIES(8192)) icache_cache(
        .rst(rst),
        .clk(clk),
        .we(icache_we),
        .addr_in(read_addr_in),
        .data_in(read_value),
        .addr_out(pc),
        .data_out(inst)
    );

    wire [5:0] inst_lo = inst[5:0];
    wire [5:0] inst_hi = inst[31:26];
    wire [4:0] imm5 = inst[15:11];
    wire [4:0] imm5_lo = inst[25:21]; // imm5 above R3, used for group 111001
    wire [20:0] imm21 = inst[31:11];
    wire [15:0] imm16 = inst[31:16];
    wire [21:0] imm22 = inst[27:6]; // BRK and SYS
    wire [28:0] imm29 = inst[31:3];
    wire [4:0] opreg1 = inst[10:6];
    wire [4:0] opreg2 = inst[15:11];
    wire [4:0] opreg3 = inst[20:16];
    wire [4:0] opreg4 = inst[25:21];
    wire [1:0] opg1_instmode = inst[27:26];
    wire [1:0] mov_simple_tsz = inst[4:3]; // Transmission size for simple arithmethic move
    wire [1:0] mov_comp_tsz = inst_hi[3:2]; // Complex move (arithmethic) transmission size

    parameter // Register numbers
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
    parameter // MMU and exceptions
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

    // TLB cache
    reg tlb_we;
    reg [31:0] tlb_addr_in; // Input to the TLB
    reg [31:0] tlb_data_in;
    reg [31:0] tlb_addr_out; // Output from the TLB
    wire [31:0] tlb_data_out;
    limn2600_Cache tlb_cache(
        .rst(rst),
        .clk(clk),
        .we(tlb_we),
        .addr_in(tlb_addr_in),
        .data_in(tlb_data_in),
        .addr_out(tlb_addr_out),
        .data_out(tlb_data_out)
    );

    wire [31:0] alu_imm_out;
    limn2600_ALU alu_imm(
        .rst(rst),
        .clk(clk),
        .op(inst_lo[5:3]),
        .a(regs[opreg2]),
        .b({ 16'b0, imm16 }),
        .c(alu_imm_out)
    );

    wire [31:0] alu_reg_out;
    limn2600_ALU alu_reg(
        .rst(rst),
        .clk(clk),
        .op(inst_hi[4:2]),
        .a(regs[opreg2]),
        .b(do_alu_shift(regs[opreg3], { 27'h0, imm5_lo }, opg1_instmode)),
        .c(alu_reg_out)
    );

    always @(posedge clk) begin
        if(rst) begin
            $display("%m: Reset");
            io_state <= SI_FETCH;
            ex_state <= SE_EXECUTE;
        end else if(irq) begin
            $display("%m: Interruption!");
            raise_exception(ECAUSE_INTERRUPT);
        end
        regs[0] <= 0;
    end

    // Fetching
    always @(posedge clk) begin
        icache_we <= 0;
        read_enable <= 0;
        write_enable <= 0;
        flush <= 0;
        if(!full) begin
            if(io_state == SI_READ) begin
                $display("%m: SI_READ");
                ex_state <= SE_STALL;
                if(read_rdy) begin
                    regs[read_regno] <= read_value;
                    perform_fetch(pc);
                end
            end else if(io_state == SI_FETCH) begin
                read_enable <= 1;
                read_addr <= fetch_pc;
                if(read_rdy) begin
                    $display("%m: SI_FETCH; pc=0x%8h,ir=0x%8h", read_addr_in, read_value);
                    icache_we <= 1; // Update!
                end else begin
                    $display("%m: SI_FETCH");
                end
                fetch_offset <= fetch_offset + 4;
            end else if(io_state == SI_REFETCH) begin
                $display("%m: SI_REFETCH; pc=0x%8h", pc);
                read_addr <= pc;
                read_enable <= 1;
                fetch_offset <= pc[11:0];
                io_state <= SI_FETCH;
                flush <= 1; // Flush everything from the memsched
            end
        end else begin
            $display("%m: Waiting for memory scheduler buffers to flush");
            io_state <= SI_REFETCH;
        end
        if(rst) begin
            fetch_offset <= 0;//{ 32'hFFFE0000 }[11:0];
        end
    end

    // Execution of insn
    always @(posedge clk) begin
        if(ex_state == SE_EXECUTE) begin
            $display("%m: SE_EXECUTE; Execution inst=0x%h<%b>,pc=0x%h", inst, inst, pc);
            pc <= pc + 4; // By default advance one instruction, can of course be overriden because this is combinatorial ;)
            casez(inst_lo)
                // This is an invalid opcode, but used internally as a "true no-op", no PC is modified
                // no anything is modified, good for continuing the executor without stalling
                6'b00_0000: begin
                    $display("%m: tnop");
                    pc <= pc; // No change on PC
                    end
                6'b11_1000: begin // JALR [rd], [ra], [imm29]
                    $display("%m: jalr r%0d,r%0d,[%h]", opreg1, opreg2, { 8'h0, imm16 } << 2);
                    regs[opreg1] <= pc + 4;
                    if(imm21[15] == 1) begin // Negative
                        branch_to(regs[opreg2] - (4 + ({ 17'h0, ~imm16[14:0] } << 2)));
                    end else begin // Positive
                        branch_to(regs[opreg2] + ({ 16'h0, imm16 } << 2));
                    end
                    end
                6'b??_?11?: begin // JAL [imm29]
                    $display("%m: jal [0x%8h],lr=0x%8h", { 3'h0, imm29 } << 2, pc + 4);
                    if(inst[0] == 1) begin
                        regs[REG_LR] <= pc + 4;
                    end
                    branch_to((pc & 32'h80000000) | ({ 3'h0, imm29 } << 2));
                    end
                6'b11_1101: begin // BEQ ra, [imm21]
                    if(regs[opreg1] == 32'h0) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin // Negative
                            $display("%m: beq r%0d,[-%0d]", opreg1, 4 + ({ 12'h0, ~imm21[19:0] } << 2));
                            branch_to(pc - (4 + ({ 12'h0, ~imm21[19:0] } << 2)));
                        end else begin // Positive
                            $display("%m: beq r%0d,[%0d]", opreg1, { 12'h0, imm21[19:0] } << 2);
                            branch_to(pc + ({ 12'h0, imm21[19:0] } << 2));
                        end
                    end
                    end
                6'b11_0101: begin // BNE ra, [imm21]
                    if(regs[opreg1] != 32'h0) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin // Negative
                            $display("%m: bne r%0d,[-%0d]", opreg1, 4 + ({ 12'h0, ~imm21[19:0] } << 2));
                            branch_to(pc - (4 + ({ 12'h0, ~imm21[19:0] } << 2)));
                        end else begin // Positive
                            $display("%m: bne r%0d,[%0d]", opreg1, { 12'h0, imm21[19:0] } << 2);
                            branch_to(pc + ({ 12'h0, imm21[19:0] } << 2));
                        end
                    end
                    end
                6'b10_1101: begin // BLT ra, [imm21]
                    // Branch taken if it's less than 0 (signed comparison)
                    if(regs[opreg1][31] == 1) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin // Negative
                            $display("%m: blt r%0d,[-%0d]", opreg1, 4 + ({ 12'h0, ~imm21[19:0] } << 2));
                            branch_to(pc - (4 + ({ 12'h0, ~imm21[19:0] } << 2)));
                        end else begin // Positive
                            $display("%m: blt r%0d,[%0d]", opreg1, { 12'h0, imm21[19:0] } << 2);
                            branch_to(pc + ({ 12'h0, imm21[19:0] } << 2));
                        end
                    end
                    end
                6'b10_0101: begin // BGT ra, [imm21]
                    // Branch taken if it's bigger than 0 (signed comparison)
                    if(regs[opreg1][31] == 0 && regs[opreg1] != 0) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin // Negative
                            $display("%m: bgt r%0d,[-%0d]", opreg1, 4 + ({ 12'h0, ~imm21[19:0] } << 2));
                            branch_to(pc - (4 + ({ 12'h0, ~imm21[19:0] } << 2)));
                        end else begin // Positive
                            $display("%m: bgt r%0d,[%0d]", opreg1, { 12'h0, imm21[19:0] } << 2);
                            branch_to(pc + ({ 12'h0, imm21[19:0] } << 2));
                        end
                    end
                    end
                6'b01_1101: begin // BGE ra, [imm21]
                    // Branch taken if it's greater or equal to 0 (signed comparison)
                    if(regs[opreg1][31] == 0) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin // Negative
                            $display("%m: bge r%0d,[-%0d]", opreg1, 4 + ({ 12'h0, ~imm21[19:0] } << 2));
                            branch_to(pc - (4 + ({ 12'h0, ~imm21[19:0] } << 2)));
                        end else begin // Positive
                            $display("%m: bge r%0d,[%0d]", opreg1, { 12'h0, imm21[19:0] } << 2);
                            branch_to(pc + ({ 12'h0, imm21[19:0] } << 2));
                        end
                    end
                    end
                6'b01_0101: begin // BLE ra, [imm21]
                    // Branch taken if it's lesser or equal to 0 (signed comparison)
                    if(regs[opreg1][31] == 1 || regs[opreg1] == 0) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin // Negative
                            $display("%m: ble r%0d,[-%0d]", opreg1, 4 + ({ 12'h0, ~imm21[19:0] } << 2));
                            branch_to(pc - (4 + ({ 12'h0, ~imm21[19:0] } << 2)));
                        end else begin // Positive
                            $display("%m: ble r%0d,[%0d]", opreg1, { 12'h0, imm21[19:0] } << 2);
                            branch_to(pc + ({ 12'h0, imm21[19:0] } << 2));
                        end
                    end
                    end
                6'b00_1101: begin // BPE ra, [imm21]
                    // Branch taken if first bit is clear
                    if(regs[opreg1][0] == 0) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin // Negative
                            $display("%m: bpe r%0d,[-%0d]", opreg1, 4 + ({ 12'h0, ~imm21[19:0] } << 2));
                            branch_to(pc - (4 + ({ 12'h0, ~imm21[19:0] } << 2)));
                        end else begin // Positive
                            $display("%m: bpe r%0d,[%0d]", opreg1, { 12'h0, imm21[19:0] } << 2);
                            branch_to(pc + ({ 12'h0, imm21[19:0] } << 2));
                        end
                    end
                    end
                6'b00_0101: begin // BPO ra, [imm21]
                    // Branch taken if first bit is set
                    if(regs[opreg1][0] == 0) begin
                        $display("%m: Branch taken!");
                        if(imm21[20] == 1) begin // Negative
                            $display("%m: bpo r%0d,[-%0d]", opreg1, 4 + ({ 12'h0, ~imm21[19:0] } << 2));
                            branch_to(pc - (4 + ({ 12'h0, ~imm21[19:0] } << 2)));
                        end else begin // Positive
                            $display("%m: bpo r%0d,[%0d]", opreg1, { 12'h0, imm21[19:0] } << 2);
                            branch_to(pc + ({ 12'h0, imm21[19:0] } << 2));
                        end
                    end
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
                    $display("%m: imm_alu_inst r%0d,r%0d,[0x%h]", opreg1, opreg2, imm16);
                    if(inst_lo[5:3] == 0) begin // LUI
                        $display("%m: lui r%0d,r%0d,[0x%h]", opreg1, opreg2, imm16);
                        regs[opreg1] <= regs[opreg2] | ({ 16'b0, imm16 } << 16);
                        // TODO: Fuse OPS for example LA comes as LUI+ORI
                    end else begin // Rest of ops
                        regs[opreg1] <= alu_imm_out;
                    end
                    end
                6'b1?_?011: begin // MOV rd, [rs + imm16]
                    $display("%m: mov(5)(R) r%0d,[r%0d+%h],sz=%b", opreg1, opreg2, { 8'h0, imm16 }, mov_simple_tsz);
                    read_in(regs[opreg2] + ({ 16'h0, imm16} << (~mov_simple_tsz)), opreg1, mov_simple_tsz);
                    end
                6'b??_?010: begin // MOV [ra + imm16], rb
                    if(inst_lo[5] == 1) begin // Move register to memory
                        $display("%m: mov(16)(W) [r%0d+%h],r%0d,sz=%b", opreg1, { 16'h0, imm16 } << (~mov_simple_tsz), opreg2, mov_simple_tsz);
                        write_out(regs[opreg1] + ({ 16'h0, imm16} << (~mov_simple_tsz)), regs[opreg2], ~mov_simple_tsz);
                    end else begin // Move immediate to memory
                        $display("%m: mov(5)(W) [r%0d+%h],r%0d,sz=%b", opreg1, { 16'h0, imm16 } << (~mov_simple_tsz), opreg2, mov_simple_tsz);
                        write_out(regs[opreg1] + ({ 16'h0, imm16} << (~mov_simple_tsz)), { 27'h0, imm5 }, ~mov_simple_tsz);
                    end
                    end
                // Instructions starting with 111001
                6'b11_1001: begin
                    $display("%m: alu_inst r%0d,r%0d,(r%0d %b %0d),op=%b", opreg1, opreg2, opreg3, opg1_instmode, imm5_lo, inst_hi);
                    casez(inst_hi[5:2]) // Check for special operators
                        4'b0000: begin // NOR ra,rb,rd
                            $display("%m: nor");
                            regs[opreg1] <= ~(regs[opreg2] | do_alu_shift(regs[opreg3], { 27'h0, imm5_lo }, opg1_instmode));
                            end
                        4'b11??: begin // Move-From-Registers
                            $display("%m: mov(W)(REG) [r%0d+r%0d+%d],r%0d,sz=%b", opreg2, opreg3, imm5_lo, opreg1, mov_comp_tsz);
                            write_out(regs[opreg2] + (do_alu_shift(regs[opreg3], { 27'h0, imm5_lo }, opg1_instmode) << (~mov_comp_tsz)), regs[opreg1], ~mov_comp_tsz);
                            end
                        4'b10??: begin // Move-To-Register
                            $display("%m: mov(R)(REG) r%0d,[r%0d+r%0d+%d],sz=%b", opreg1, opreg2, opreg3, imm5_lo, mov_comp_tsz);
                            read_in(regs[opreg2] + (do_alu_shift(regs[opreg3], { 27'h0, imm5_lo }, opg1_instmode) << (~mov_comp_tsz)), opreg1, ~mov_comp_tsz);
                            end
                        default: begin // Assume this is a general ALU OP and perform a normal operation
                            regs[opreg1] <= alu_reg_out;
                            end
                    endcase
                    end
                6'b10_1001: begin
                    $display("%m: privileged_inst");
                    casez(inst_hi[5:2])
                    4'b1111: begin // MFCR [opreg1] [opreg3]
                        $display("%m: mfcr r%0d,cr%0d", opreg1, opreg3);
                        regs[opreg1] <= ctl_regs[opreg3];
                        end
                    4'b1110: begin // MTCR [opreg3] [opreg2]
                        $display("%m: mtcr cr%0d,r%0d", opreg3, opreg2);
                        ctl_regs[opreg3] <= regs[opreg2];
                        end
                    4'b1000: begin // CACHEI [imm22]
                        $display("%m: cachei [%h]", imm22);
                        end
                    4'b1010: begin // FWC [imm22]
                        $display("%m: exception firmware");
                        ctl_regs[CREG_EBADADDR] <= pc;
                        ctl_regs[CREG_RS][31:28] <= ECAUSE_SYSCALL;
                        branch_to(ctl_regs[CREG_FWVEC]);
                        end
                    4'b1100: begin // HLT [imm22]
                        $display("%m: hlt [%h]", imm22);
                        ex_state <= SE_HALT;
                        end
                    4'b0011: begin // TBLD
                        $display("%m: tbld [%h]", imm22);
                        ctl_regs[CREG_TBLO] <= { 12'h0, (ctl_regs[CREG_TBLO][24:5] << 12) } | { 22'h0, (ctl_regs[CREG_TBHI][9:0] << 2) };
                        end
                    4'b0010: begin // TBLO
                        $display("%m: tblo [%0d]", imm22);
                        ex_state <= SE_GET_TLB_ENTRY; // Next cycle is guaranteed to output in tlb_data_out the given tlb
                        tlb_addr_out <= ctl_regs[CREG_TBINDEX];
                        end
                    4'b0001: begin // TBFN
                        $display("%m: tbfn [%0d]", imm22);
                        ex_state <= SE_GET_TLB_FN;
                        tlb_data_in <= ctl_regs[CREG_TBLO];
                        end
                    4'b0000: begin // TBWR
                        $display("%m: tbwr [%0d]", imm22);
                        tlb_we <= 1; // Set TLB entry
                        tlb_addr_in <= ctl_regs[CREG_TBINDEX];
                        tlb_data_in <= ctl_regs[CREG_TBHI] | ctl_regs[CREG_TBLO];
                        end
                    default: begin // Invalid instruction
                        $display("%m: exception invalid_grp2=0b%b", inst_hi[5:2]);
                        raise_exception(ECAUSE_INVALID_INST);
                        end
                    endcase
                    end
                6'b11_0001: begin
                    $display("%m: advanced_cohost_alu");
                    casez(inst_hi[5:2])
                        4'b0001: begin
                            $display("%m: exception brk [%h]", imm22);
                            // TODO: Is imm22 used at all?
                            raise_exception(ECAUSE_BREAKPOINT);
                            end
                        4'b1101: begin
                            if(opreg4 != 5'b0) begin // Raise UD
                                $display("%m: exception invalid div inst=%b", inst);
                                raise_exception(ECAUSE_INVALID_INST);
                            end else begin
                                regs[opreg1] <= regs[opreg2] / regs[opreg3];
                            end
                            end
                        4'b1100: begin
                            if(opreg4 != 5'b0) begin // Raise UD
                                $display("%m: exception invalid divs inst=%b", inst);
                                raise_exception(ECAUSE_INVALID_INST);
                            end else begin // TODO: Properly perform signed division
                                regs[opreg1] <= { (regs[opreg2][31] | regs[opreg3][31]), regs[opreg2][30:0] / regs[opreg3][30:0] };
                            end
                            end
                        4'b1001: begin // TODO: Is this a memory access?
                            end
                        4'b1011: begin // MOD rd,ra,rb
                            if(opreg4 != 5'b0) begin // Raise UD
                                $display("%m: exception invalid mod inst=%b", inst);
                                raise_exception(ECAUSE_INVALID_INST);
                            end else regs[opreg1] <= regs[opreg2] % regs[opreg3];
                            end
                        4'b1111: begin // MUL rd,ra,rb
                            if(opreg4 != 5'b0) begin // Raise UD
                                $display("%m: exception invalid mul inst=%b", inst);
                                raise_exception(ECAUSE_INVALID_INST);
                            end else regs[opreg1] <= regs[opreg2] * regs[opreg3];
                            end
                        4'b1000: begin // TODO: Is this a memory access?
                            end
                        4'b0000: begin
                            $display("%m: exception sys [%h]", imm22); // TODO: Is imm22 used at all?
                            raise_exception(ECAUSE_SYSCALL);
                            end
                        default: begin
                            end
                    endcase
                    end
                default: begin
                    $display("%m: exception invalid_opcode,inst=%b", inst);
                    raise_exception(ECAUSE_INVALID_INST);
                end
            endcase
        end else if(ex_state == SE_GET_TLB_ENTRY) begin // TODO: RDY for TLB
            ctl_regs[CREG_TBHI] <= tlb_data_out; // TODO: Is this correct?
            ctl_regs[CREG_TBLO] <= tlb_data_out;
            pc <= pc;
        end else if(ex_state == SE_GET_TLB_FN) begin // TODO: RDY for TLB
            // By now data from the TLB has arrived, negative values will be given for indicating NOT-FOUND
            ctl_regs[CREG_TBINDEX] <= tlb_data_out;
            pc <= pc;
        end
        if(rst) begin
            pc <= 32'hFFFE0000;
        end
    end

    always @(posedge clk) begin
        if(write_enable) begin
            $display("%m: pc=0x%8h,write_value=0x%8h,write_addr=0x%8h", pc, write_value, write_addr);
        end else if(read_enable) begin
            $display("%m: pc=0x%8h,read_value=0x%8h,read_addr=0x%8h", pc, read_value, read_addr);
        end else begin
            $display("%m: pc=0x%8h", pc);
        end
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
    output ce // Command-State (1 = memory commands active, 0 = memory commands ignored)
);
    wire [31:0] core1_read_addr;
    wire [31:0] core1_read_value;
    wire [1:0] core1_read_size;
    wire core1_read_enable;
    wire core1_read_rdy;
    wire [31:0] core1_read_addr_in;

    wire [31:0] core1_write_addr;
    wire [31:0] core1_write_value;
    wire [1:0] core1_write_size;
    wire core1_write_enable;
    wire core1_write_rdy;
    wire core1_full;
    wire core1_flush;
    limn2600_MemorySched msched(
        .rst(rst),
        .clk(clk),
        .ram_addr(addr),
        .ram_data_in(data_in),
        .ram_data_out(data_out),
        .ram_rdy(rdy),
        .ram_we(we),
        .ram_ce(ce),
        .client_read_addr(core1_read_addr),
        .client_read_value(core1_read_value),
        .client_read_size(core1_read_size),
        .client_read_enable(core1_read_enable),
        .client_read_rdy(core1_read_rdy),
        .client_read_addr_in(core1_read_addr_in),
        .client_write_addr(core1_write_addr),
        .client_write_value(core1_write_value),
        .client_write_size(core1_write_size),
        .client_write_enable(core1_write_enable),
        .client_write_rdy(core1_write_rdy),
        .full(core1_full),
        .flush(core1_flush)
    );

    limn2600_Core core1(
        .rst(rst),
        .clk(clk),
        .irq(irq),
        .read_addr(core1_read_addr),
        .read_value(core1_read_value),
        .read_size(core1_read_size),
        .read_enable(core1_read_enable),
        .read_rdy(core1_read_rdy),
        .read_addr_in(core1_read_addr_in),
        .write_addr(core1_write_addr),
        .write_value(core1_write_value),
        .write_size(core1_write_size),
        .write_enable(core1_write_enable),
        .write_rdy(core1_write_rdy),
        .full(core1_full),
        .flush(core1_flush)
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
                .ce(ce)
            );
        end
    endgenerate
*/
endmodule
