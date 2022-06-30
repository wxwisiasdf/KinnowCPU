
///////////////////////////////////////////////////////////////////////////////
//
// Limine2600 CPU
//
///////////////////////////////////////////////////////////////////////////////
module limine2600_cpu(
    input rst,
    input clk,
    output reg we,
    input irq,
    input rdy,
    output reg [31:0] addr,
    input [31:0] data_in,
    output reg [31:0] data_out
);
    reg [31:0] tmp32; // Temporal value
    reg [31:0] regs[0:32]; // Limnstation has 32 registers
    reg [31:0] ctl_regs[0:32];
    reg [31:0] pc; // Program counter
    reg [3:0] state;

    localparam S_RESET = 0;
    localparam S_SELECT = 1;
    localparam S_DECODE = 2;
    localparam S_DECODE_WAIT = 3;
    localparam S_WRITE = 4;
    localparam S_READ = 5;
    localparam S_WRITE_WAIT = 6; // Wait to readback value (since RAM is handled in multiples of 4)
    localparam S_READ_WAIT = 7;
    localparam S_HALT = 8;

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
    // Instruction decoder
    wire [28:0] imm29 = data_in[31:3];
    wire [22:0] imm21 = data_in[31:9];
    wire [23:0] imm22 = data_in[25:6]; // BRK and SYS
    wire [4:0] imm5 = data_in[31:27];
    wire [15:0] imm16 = data_in[31:24];

    wire [5:0] inst_lo = data_in[5:0];
    wire [4:0] opreg1 = data_in[10:6];
    wire [4:0] opreg2 = data_in[15:11];
    wire [4:0] opreg3 = data_in[20:16];
    wire [4:0] opreg4 = data_in[25:21];
    wire [5:0] inst_hi = data_in[31:26];

    wire [1:0] opg1_instmode = data_in[27:26];
    parameter
        OP_JALR = 6'b11_1000,
        OP_JAL = 6'b??_?111,
        OP_J = 6'b??_?110,
        OP_BEQ = 6'b11_1101,
        OP_BNE = 6'b11_0101,
        OP_BLT = 6'b10_1101,
        OP_ADDI = 6'b11_1100,
        OP_SUBI = 6'b11_0100,
        OP_SLTI = 6'b10_1100,
        OP_SLTIS = 6'b10_0100,
        OP_ANDI = 6'b01_1100,
        OP_XORI = 6'b01_0100,
        OP_ORI = 6'b00_1100,
        OP_LUI = 6'b00_0100,
        OP_MOV16 = 6'b1?_?010,
        OP_MOV5 = 6'b0?_?010,
        OP_GRP1 = 6'b11_1001, // Basic arithmethic
        OP_GRP2 = 6'b10_1001, // Privileged insn
        OP_GRP3 = 6'b11_0001; // Advanced arithmethic, atomics, etc
    // Group 1
    // Move's length
    parameter
        OP_G1_MV_BYTE = 2'b11,
        OP_G1_MV_INT = 2'b10,
        OP_G1_MV_LONG = 2'b01,
        OP_G1_MV_BITMASK = 2'b11;
    // Opcodes
    parameter
        OP_G1_MOV_TR = 4'b11??,
        OP_G1_MOV_FR = 4'b10??,
        OP_G1_NOP = 4'b1000,
        OP_G1_SLT = 4'b0101, // TODO: What SLT does?
        OP_G1_SLTS = 4'b0100, // TODO: What SLTS does?
        OP_G1_ADD = 4'b0111,
        OP_G1_SUB = 4'b0110,
        OP_G1_AND = 4'b0011,
        OP_G1_XOR = 4'b0010,
        OP_G1_OR = 4'b0001,
        OP_G1_NOR = 4'b0000;
    // Opcode modes
    parameter
        OPM_G1_LHS = 2'b00,
        OPM_G1_RHS = 2'b01,
        OPM_G1_ASH = 2'b10,
        OPM_G1_ROR = 2'b11;
    // Group 2
    parameter
        OP_G2_MFCR = 4'b1111,
        OP_G2_MTCR = 4'b1110, // If opreg1 is zero then CR=REG, otherwise REG=CR
        OP_G2_CACHEI = 4'b1000;
    // Group 3
    parameter
        OP_G3_MUL = 4'b1111,
        OP_G3_DIV = 4'b1101,
        OP_G3_DIVS = 4'b1100,
        OP_G3_MOD = 4'b1011,
        OP_G3_LL = 4'b1001,
        OP_G3_SC = 4'b1000,
        OP_G3_BRK = 4'b0001,
        OP_G3_SYS = 4'b0000;
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
    
    // add zero, zero, zero LSH 0
    parameter OP_NOP = 32'b0111_0000_0000_0000_0000_0000_0011_1001;
    
    // Write options
    reg [31:0] write_value;
    reg [31:0] write_addr;
    // Read options
    reg [4:0] read_regno;
    reg [31:0] read_addr;

    reg [1:0] rw_memsize;

    // Execution engine
    always @(posedge rst) begin
        state <= S_RESET;
    end

    // Reset zero register to zero
    always @(posedge clk) begin
        regs[0] <= 32'h0;
    end

    always @(posedge irq) begin
        ctl_regs[CREG_EBADADDR] <= pc;
        pc <= ctl_regs[CREG_EVEC];
        ctl_regs[CREG_RS][31:28] = ECAUSE_INTERRUPT;

        if(state == S_HALT) begin
            state <= S_SELECT;
        end
    end

    // Non-blocking
    always @(posedge clk) begin
        case(state)
        S_RESET: begin
            we <= 1'b0;
            pc <= 32'hFFFE0000;
            state <= S_SELECT;
        end
        S_SELECT: begin
            we <= 1'b0;
            if(rdy) begin
                addr <= pc;
                state <= S_DECODE_WAIT; // Wait for RAM
            end
        end
        S_DECODE: begin
            we <= 1'b0;
            state <= S_SELECT;
            casez(inst_lo)
            // JALR [rd], [ra], [imm29]
            OP_JALR: begin
                $display("jalr r%d,r%d,[%h]", opreg1, opreg2, imm16 << 2);
                regs[opreg1] <= pc + 4;
                pc <= regs[opreg2] + (imm16 << 2); // TODO: Sign extend
                end
            // JAL [lr] [imm29]
            OP_JAL: begin
                regs[REG_LR] <= pc + 4;
                $display("jal [0x%8h],lr=0x%8h", imm29 << 2, pc + 4);
                pc <= (pc & 32'h80000000) | ({ 3'h0, imm29 } << 2);
                end
            // J [imm29]
            OP_J: begin
                $display("j [%h]", imm29 << 2);
                pc <= (pc & 32'h80000000) | ({ 3'h0, imm29 } << 2);
                end
            // BEQ ra, [imm21]
            OP_BEQ: begin
                $display("beq r%d,[%h]", opreg1, imm21);
                if(regs[opreg1] == 32'h0) begin
                    pc <= pc + ({ 9'h0, imm21 } << 2);
                end else begin
                    pc <= pc + 4;
                end
                end
            // BNE ra, [imm21]
            OP_BNE: begin
                $display("bne r%d,[%h]", opreg1, imm21);
                if(regs[opreg1] != 32'h0) begin
                    pc <= pc + ({ 9'h0, imm21 } << 2);
                end else begin
                    pc <= pc + 4;
                end
                end
            // BLT ra, [imm21]
            OP_BLT: begin
                $display("blt r%d,[%h]", opreg1, imm21);
                if((regs[opreg1] & 32'b10000000000000000000000000000000) == 0) begin
                    pc <= pc + ({ 9'h0, imm21 } << 2);
                end else begin
                    pc <= pc + 4;
                end
                end
            // ADDI [rd], [rd], [imm16]
            OP_ADDI: begin
                $display("addi r%d,r%d,[%h]", opreg1, opreg2, imm16);
                regs[opreg1] = regs[opreg2] + imm16;
                pc <= pc + 4;
                end
            // SUBI [rd], [rd], [imm16]
            OP_SUBI: begin
                $display("subi r%d,r%d,[%h]", opreg1, opreg2, imm16);
                regs[opreg1] = regs[opreg2] - imm16;
                pc <= pc + 4;
                end
            // SLTI [rd], [rd], [imm16]
            OP_SLTI: begin
                // TODO: What does SLTI do?
                $display("slti r%d,r%d,[%h]", opreg1, opreg2, imm16);
                regs[opreg1] = regs[opreg2] + imm16;
                pc <= pc + 4;
                end
            // SLTIS [rd], [rd], [imm16]
            OP_SLTIS: begin
                // TODO: What does SLTIS do?
                $display("sltis r%d,r%d,[%h]", opreg1, opreg2, imm16);
                regs[opreg1] = regs[opreg2] + imm16;
                pc <= pc + 4;
                end
            // ANDI [rd], [rd], [imm16]
            OP_ANDI: begin
                $display("andi r%d,r%d,[%h]", opreg1, opreg2, imm16);
                regs[opreg1] = regs[opreg2] & imm16;
                pc <= pc + 4;
                end
            // XORI [rd], [rd], [imm16]
            OP_XORI: begin
                $display("xori r%d,r%d,[%h]", opreg1, opreg2, imm16);
                regs[opreg1] = regs[opreg2] ^ imm16;
                pc <= pc + 4;
                end
            // ORI [rd], [rd], [imm16]
            OP_ORI: begin
                $display("ori r%d,r%d,[%h]", opreg1, opreg2, imm16);
                regs[opreg1] = regs[opreg2] | imm16;
                pc <= pc + 4;
                end
            // LUI [rd], [rd], [imm16]
            OP_LUI: begin
                $display("lui r%d,r%d,[%h]", opreg1, opreg2, imm16);
                regs[opreg1] = regs[opreg2] | (imm16 << 16);
                pc <= pc + 4;
                end
            // MOV [ra + imm16], rb
            OP_MOV16: begin
                $display("mov(16) [r%d+%h],r%d,sz=%b", opreg1, imm16, opreg2, inst_lo[4:3]);
                rw_memsize <= inst_lo[4:3]; // Check OP_G1_MV_BITMASK
                write_value <= regs[opreg2];
                write_addr <= regs[opreg1] + imm16;
                state <= S_WRITE_WAIT;
                pc <= pc + 4;
                end
            // MOV [ra + imm16], imm5
            OP_MOV5: begin
                $display("mov(16) [r%d+%h],r%d,sz=%b", opreg1, imm16, imm5, inst_lo[4:3]);
                rw_memsize <= inst_lo[4:3]; // Check OP_G1_MV_BITMASK
                write_value <= imm16;
                write_addr <= regs[opreg1] + imm16;
                state <= S_WRITE_WAIT;
                pc <= pc + 4;
                end
            // Instructions starting with 111001
            OP_GRP1: begin
                $display("alu_inst r%d,r%d,r%d,instmode=%b,op=%b", opreg1, opreg2, opreg3, opg1_instmode, inst_hi[5:2]);
                // Instmode
                casez(opg1_instmode)
                OPM_G1_LHS: tmp32 <= regs[opreg3] >> { 26'h0, imm5};
                OPM_G1_RHS: tmp32 <= regs[opreg3] << { 26'h0, imm5};
                OPM_G1_ASH: tmp32 <= regs[opreg3] >>> { 26'h0, imm5}; // TODO: What is ASH?
                OPM_G1_ROR: tmp32 <= regs[opreg3] >>> { 26'h0, imm5};
                endcase

                casez(inst_hi[5:2])
                OP_G1_NOP: regs[opreg1] <= tmp32;
                OP_G1_ADD: regs[opreg1] <= regs[opreg2] + tmp32;
                OP_G1_SUB: regs[opreg1] <= regs[opreg2] - tmp32;
                OP_G1_AND: regs[opreg1] <= regs[opreg2] & tmp32;
                OP_G1_XOR: regs[opreg1] <= regs[opreg2] ^ tmp32;
                OP_G1_OR: regs[opreg1] <= regs[opreg2] | tmp32;
                OP_G1_NOR: regs[opreg1] <= ~(regs[opreg2] | tmp32);
                default: begin end
                endcase

                // These low 2-bits are indicative of whatever or not this is a mov
                if(inst_hi[3:2] != 2'b00) begin
                    casez(inst_hi[5:2])
                    OP_G1_MOV_FR: begin // Move-From-Registers
                        $display("mov [r%d+r%d+%d],r%d,sz=%b", opreg2, opreg3, imm5, opreg1, inst_hi[3:2]);
                        rw_memsize <= inst_hi[3:2]; // Check OP_G1_MV_BITMASK
                        write_value <= regs[opreg1];
                        write_addr <= regs[opreg2] + tmp32;
                        state <= S_WRITE_WAIT;
                        end
                    OP_G1_MOV_TR: begin // Move-To-Register
                        $display("mov r%d,[r%d+r%d+%d],sz=%b", opreg1, opreg2, opreg3, imm5, inst_hi[3:2]);
                        rw_memsize <= inst_hi[3:2]; // Check OP_G1_MV_BITMASK
                        read_regno <= opreg1;
                        state <= S_READ_WAIT;
                        read_addr <= regs[opreg2] + tmp32;
                        end
                    default: begin end
                    endcase
                end
                pc <= pc + 4;
                end
            OP_GRP2: begin
                $display("privileged_inst");
                casez(inst_hi[5:2])
                OP_G2_MFCR: begin
                    $display("mtcr r%d,cr%d", opreg1, opreg3);
                    regs[opreg1] <= ctl_regs[opreg3];
                    pc <= pc + 4;
                    end
                OP_G2_MTCR: begin
                    $display("mtcr cr%d,r%d", opreg3, opreg2);
                    ctl_regs[opreg3] <= regs[opreg2];
                    pc <= pc + 4;
                    end
                OP_G2_CACHEI: begin
                    $display("cachei [%h]", imm22);
                    pc <= pc + 4;
                    end
                default: begin // Invalid instruction
                    $display("invalid_grp2=0b%b", inst_hi[5:2]);
                    ctl_regs[CREG_EBADADDR] <= pc;
                    pc <= ctl_regs[CREG_EVEC];
                    ctl_regs[CREG_RS][31:28] = ECAUSE_INVALID_INST;
                    end
                endcase
                end
            OP_GRP3: begin
                $display("advanced_cohost_alu");
                casez(inst_hi[5:2])
                OP_G3_BRK: begin
                    $display("brk [%h]", imm22);
                    // TODO: Is imm22 used at all?
                    ctl_regs[CREG_EBADADDR] <= pc;
                    pc <= ctl_regs[CREG_EVEC];
                    ctl_regs[CREG_RS][31:28] = ECAUSE_BREAKPOINT;
                    end
                OP_G3_DIV: begin
                    if(opreg4 != 5'b0) begin
                        // Raise UD
                        ctl_regs[CREG_EBADADDR] <= pc;
                        pc <= ctl_regs[CREG_EVEC];
                        ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                    end else begin
                        regs[opreg1] = regs[opreg2] / regs[opreg3];
                        pc <= pc + 4;
                    end
                    end
                OP_G3_DIVS: begin
                    if(opreg4 != 5'b0) begin
                        // Raise UD
                        ctl_regs[CREG_EBADADDR] <= pc;
                        pc <= ctl_regs[CREG_EVEC];
                        ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                    end else begin
                        // TODO: Properly perform signed division
                        regs[opreg1] = { (regs[opreg2][31] | regs[opreg3][31]), (regs[opreg2] / regs[opreg3]) };
                        pc <= pc + 4;
                    end
                    end
                OP_G3_LL: begin
                    // TODO: Is this a memory access?
                    pc <= pc + 4;
                    end
                OP_G3_MOD: begin
                    if(opreg4 != 5'b0) begin
                        // Raise UD
                        ctl_regs[CREG_EBADADDR] <= pc;
                        pc <= ctl_regs[CREG_EVEC];
                        ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                    end else begin
                        regs[opreg1] = regs[opreg2] % regs[opreg3];
                        pc <= pc + 4;
                    end
                    end
                OP_G3_MUL: begin
                    if(opreg4 != 5'b0) begin
                        // Raise UD
                        ctl_regs[CREG_EBADADDR] <= pc;
                        pc <= ctl_regs[CREG_EVEC];
                        ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                    end else begin
                        regs[opreg1] = regs[opreg2] * regs[opreg3];
                        pc <= pc + 4;
                    end
                    end
                OP_G3_SC: begin
                    // TODO: Is this a memory access?
                    pc <= pc + 4;
                    end
                OP_G3_SYS: begin
                    $display("sys [%h]", imm22);
                    // TODO: Is imm22 used at all?
                    ctl_regs[CREG_EBADADDR] <= pc;
                    pc <= ctl_regs[CREG_EVEC];
                    ctl_regs[CREG_RS][31:28] <= ECAUSE_SYSCALL;
                    end
                default: begin
                    end
                endcase
                end
            default: begin
                $display("invalid_opcode");
                ctl_regs[CREG_EBADADDR] <= pc;
                pc <= ctl_regs[CREG_EVEC];
                ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
            end
            endcase

            $display("CPU > state=%d,pc=0x%8h,data_in=0x%8h,data_out=0x%8h,addr=0x%8h", state, pc, data_in, data_out, addr);
            for(integer i = 0; i < 32; i = i + 8) begin
                $display("%2d=0x%8h,%2d=0x%8h,%2d=0x%8h,%2d=0x%8h,%2d=0x%8h,%2d=0x%8h,%2d=0x%8h,%2d=0x%8h", i, regs[i], i + 1, regs[i + 1], i + 2, regs[i + 2], i + 3, regs[i + 3], i + 4, regs[i + 4], i + 5, regs[i + 5], i + 6, regs[i + 6], i + 7, regs[i + 7]);
            end
            end
        // Wait until RAM indicates it's ready
        S_DECODE_WAIT: begin
            we <= 1'b0;
            if(rdy) state <= S_DECODE;
            end
        S_WRITE_WAIT: begin
            addr <= write_addr;
            case(rw_memsize)
            OP_G1_MV_BYTE: data_out <= data_in | (write_value >> ((write_addr % 4) * 8));
            OP_G1_MV_INT: data_out <= data_in | (write_value >> ((write_addr % 2) * 16));
            OP_G1_MV_LONG: data_out <= write_value;
            endcase
            if(rdy) begin
                we <= 1'b1; // Enable write
                state <= S_SELECT;
            end
            $display("write_wait addr=0x%h,sz=%d,v=0x%h,data_in=0x%h", write_addr, rw_memsize, write_value, data_in);
            end
        S_READ_WAIT: begin
            we <= 1'b0; // Disable write
            addr <= read_addr;
            case(rw_memsize)
            OP_G1_MV_BYTE: regs[read_regno] <= data_in >> ((read_addr % 4) * 8);
            OP_G1_MV_INT: regs[read_regno] <= data_in >> ((read_addr % 2) * 16);
            OP_G1_MV_LONG: regs[read_regno] <= data_in;
            endcase
            if(rdy) begin
                state <= S_SELECT;
            end
            $display("read_wait addr=0x%h,sz=%,r=%d,v=0x%h", read_addr, rw_memsize, read_regno, regs[read_regno]);
            end
        default: begin end
        endcase
    end
endmodule
