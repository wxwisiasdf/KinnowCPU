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
        casez(size)
            2'b11: begin // 1-bytes, 4-per-cell
                $display("%m: prewrite byte");
                memio_aligned_write_mask = (ramdata & ~(32'hFF << ((addr % 4) * 8))) | ((data & 32'hFF) << ((addr % 4) * 8));
                end
            2'b10: begin // 2-bytes, 2-per-cell
                $display("%m: prewrite int");
                memio_aligned_write_mask = (ramdata & ~(32'hFFFF << ((addr % 4) * 8))) | ((data & 32'hFFFF) << ((addr % 4) * 8));
                end
            2'b0?: begin // 4-bytes, 1-per-cell
                $display("%m: prewrite long");
                memio_aligned_write_mask = ramdata;
                end
        endcase
    endfunction

    // State
    reg [31:0] tmp32; // Temporal value
    reg [31:0] regs[0:31]; // Limnstation has 32 registers
    reg [31:0] ctl_regs[0:31];
    reg [31:0] pc; // Program counter
    reg [2:0] state;

    reg took_branch; // Whetever a branch was taken (state)

    // I/O machine
    reg [1:0] trans_size; // Memory transfer size
    reg [31:0] write_value; // Write value
    reg [4:0] read_regno; // Register to place the read of memory into
    reg [31:0] rw_addr; // Read-Write address

    localparam S_FETCH = 0;
    localparam S_EXECUTE = 1;
    localparam S_PREWRITE = 2;
    localparam S_WRITE = 3;
    localparam S_READ = 4;
    localparam S_HALT = 5;
    localparam S_BRANCHED = 6;
    
    // Instruction fetcher
    reg stall_sramio;
    reg [31:0] fetch_addr; // Address to fetch on, reset on JAL/J/BR
    wire [31:0] execute_inst = icache_data_out; // Instruction to execute

    // Branch prediction (fetcher stage)
    reg [3:0] regs_predict[0:31]; // Flags for the BP to tag registers
    parameter
        RP_NON_ZERO = 4'b0001, // Register is non-zero
        RP_ZERO = 4'b0000, // Register might be zero
        RP_UNSPEC_MEM = 4'b0010; // Register depends on memory value

    // Instruction executor
    reg stall_exec;
    wire [28:0] f_imm29 = data_in[31:3];
    wire [22:0] f_imm21 = data_in[31:9];
    wire [21:0] f_imm22 = data_in[27:6]; // BRK and SYS
    wire [4:0] f_imm5 = data_in[31:27];
    wire [15:0] f_imm16 = data_in[31:16];
    wire [5:0] f_inst_lo = data_in[5:0];
    wire [4:0] f_opreg1 = data_in[10:6];
    wire [4:0] f_opreg2 = data_in[15:11];
    wire [4:0] f_opreg3 = data_in[20:16];
    wire [4:0] f_opreg4 = data_in[25:21];
    wire [5:0] f_inst_hi = data_in[31:26];
    wire [1:0] f_opg1_instmode = data_in[27:26];

    wire [28:0] imm29 = execute_inst[31:3];
    wire [22:0] imm21 = execute_inst[31:9];
    wire [21:0] imm22 = execute_inst[27:6]; // BRK and SYS
    wire [4:0] imm5 = execute_inst[31:27];
    wire [15:0] imm16 = execute_inst[31:16];
    wire [5:0] inst_lo = execute_inst[5:0];
    wire [4:0] opreg1 = execute_inst[10:6];
    wire [4:0] opreg2 = execute_inst[15:11];
    wire [4:0] opreg3 = execute_inst[20:16];
    wire [4:0] opreg4 = execute_inst[25:21];
    wire [5:0] inst_hi = execute_inst[31:26];
    wire [1:0] opg1_instmode = execute_inst[27:26];
    parameter
        OP_JALR = 6'b11_1000,
        OP_J_OR_JAL = 6'b??_?11?,
        OP_BEQ = 6'b11_1101,
        OP_BNE = 6'b11_0101,
        OP_BLT = 6'b10_1101,
        OP_LUI = 6'b00_0100,
        OP_MOV16 = 6'b1?_?010,
        OP_MOV5 = 6'b0?_?010; // Advanced arithmethic, atomics, etc
    // Arithmethic mode
    parameter OP_NOR = 3'b000;
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
        OP_G1_NOP = 4'b1000;
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
        OP_G2_CACHEI = 4'b1000,
        OP_G2_HLT = 4'b1100; // Halt until IRQ
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
    // NOTE: While the Limn2600 spec says this is not a vlid opcode
    // we say otherwise, and this is only used internally, so hopefully
    // nothing bad happens from this!
    parameter OP_TRULY_NOP = 32'b0;

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
            stall_sramio <= 0;
            stall_exec <= 0;
            fetch_addr <= 32'hFFFE0000;
        end else if(irq) begin
            $display("%m: IRQ event!");
            ctl_regs[CREG_EBADADDR] <= pc;
            pc <= ctl_regs[CREG_EVEC];
            ctl_regs[CREG_RS][31:28] <= ECAUSE_INTERRUPT;
            stall_sramio <= 1;
            state <= S_BRANCHED;
        end
        regs[0] <= 0;
        regs_predict[0] <= 0;
    end

    // Fetch
    always @(posedge clk) begin
        if(!stall_sramio) begin
            cs <= 1;
            addr <= rw_addr;
            // Read a from the SRAM (1/1 cycles)
            if(state == S_READ) begin
                $display("%m: S_READ");
                we <= 0; // Read value and emplace on register
                if(rdy) begin // Appropriately apply masks
                    casez(trans_size)
                    OP_G1_MV_BYTE: begin // 1-bytes, 4-per-cell
                        regs[read_regno] <= (data_in & 32'hFF) << ((rw_addr & 32'd3) << 32'd3);
                        end
                    OP_G1_MV_INT: begin // 2-bytes, 2-per-cell
                        regs[read_regno] <= (data_in & 32'hFFFF) << ((rw_addr & 32'd1) << 32'd4);
                        end
                    OP_G1_MV_LONG: begin // 4-bytes, 1-per-cell
                        regs[read_regno] <= (data_in & 32'hFFFFFFFF);
                        end
                    default: begin
                        // TODO: Raise exception for unknown size, or use this size for 8-bytes?
                        end
                    endcase
                    state <= S_EXECUTE;
                    stall_sramio <= 0;
                end
            // Fetch the element from SRAM with 32-bits per unit of data
            // rememeber that we also need to write bytes so unaligned accesses
            // are allowed by the CPU because fuck you (1/2 cycles)
            end else if(state == S_PREWRITE) begin
                // Prewrite is in charge of reading the value and then writting it back with the desired offset
                // so we can support unaligned accesses
                $display("%m: S_PREWRITE");
                we <= 0; // Read the value first
                if(rdy) begin
                    // Appropriately apply masks
                    if((rw_addr & 3) == 0) begin // Aligned access
                        if(trans_size == 2'b00) begin // 4-bytes, 1-per-cell
                            $display("%m: prewrite long");
                            we <= 1; // Since data_width == 32 we simply send the whole thing
                            data_out <= write_value;
                            state <= S_EXECUTE;
                            stall_sramio <= 0;
                        end else begin
                            write_value <= memio_aligned_write_mask(trans_size, rw_addr, data_in, write_value);
                        end
                    end else begin // Unaligned access
                        $display("%m: unaligned write!");
                        ctl_regs[CREG_EBADADDR] <= pc;
                        pc <= ctl_regs[CREG_EVEC];
                        ctl_regs[CREG_RS][31:28] <= ECAUSE_UNALIGNED;
                        state <= S_BRANCHED;
                    end
                    state <= S_WRITE;
                    $display("%m: write_value=0x%h,rw_addr=0x%h,data_in=0x%h", write_value, rw_addr, data_in);
                end
            // After fetching the value and appropriately setting the masks
            // send it to the SRAM (2/2 cycles)
            end else if(state == S_WRITE) begin
                $display("%m: S_WRITE");
                we <= 1; // Write the value, then return to fetching
                data_out <= write_value;
                if(rdy) begin
                    $display("%m: data_out=0x%h,write_value=0x%h,addr=0x%h", data_out, write_value, addr);
                    state <= S_EXECUTE;
                    stall_sramio <= 0;
                end
            end else if(state == S_FETCH) begin
                $display("%m: (Fetch) Fetching");
                addr <= fetch_addr;
                we <= 0; // Read from memory
                icache_we <= 0;
                icache_addr_in <= fetch_addr;
                // Once we can fetch instructions we save the state, but only if
                // we aren't overwriting something being used by the executor!
                if(rdy) begin
                    $display("%m: (Fetch) Fetched inst=%b,fetch=0x%h", data_in, fetch_addr);
                    icache_we <= 1; // Send the fetched instruction onto the i-cache
                    fetch_addr <= fetch_addr + 4; // Advance to next op
                    casez(f_inst_lo)
                        // JALR [rd], [ra], [imm29]
                        OP_JALR: begin end // TODO: Prediction for JALR
                        // JAL [imm29]
                        OP_J_OR_JAL: begin // This one is good because it's guaranteed to branch
                            $display("%m: (Branch Predict) jal [0x%h],lr=0x%h", { 3'h0, f_imm29 } << 2, fetch_addr + 4);
                            if(f_inst_lo[0] == 0) begin // JAL variant clobbers LR
                                regs_predict[REG_LR] <= regs_predict[REG_LR] | RP_NON_ZERO;
                            end
                            fetch_addr <= (fetch_addr & 32'h80000000) | ({ 3'h0, f_imm29 } << 2);
                            end
                        // Branches's signs are checked, backwards branches are often used in loops
                        // BEQ ra, [imm21]
                        OP_BEQ: begin
                            $display("%m: (Branch Predict) beq r%d,[%h]", f_opreg1, f_imm21);
                            if(f_imm21[20] == 1 || 1) begin
                                if(f_imm21[20] == 0) fetch_addr <= fetch_addr + ({ 12'h0, f_imm21[19:0] } << 2);
                                else fetch_addr <= fetch_addr - ({ 12'h0, f_imm21[19:0] } << 2);
                                $display("%m: (Branch Predict) Will be taken");
                            end else begin
                                $display("%m: (Branch Predict) Will not be taken");
                            end
                            end
                        // BNE ra, [imm21]
                        OP_BNE: begin
                            $display("%m: (Branch Predict) bne r%d,[%h]", f_opreg1, f_imm21);
                            if(f_imm21[20] == 1 || 1) begin
                                if(f_imm21[20] == 0) fetch_addr <= fetch_addr + ({ 12'h0, f_imm21[19:0] } << 2);
                                else fetch_addr <= fetch_addr - ({ 12'h0, f_imm21[19:0] } << 2);
                                $display("%m: (Branch Predict) Will be taken");
                            end else begin
                                $display("%m: (Branch Predict) Will not be taken");
                            end
                            end
                        // BLT ra, [imm21]
                        OP_BLT: begin
                            $display("%m: (Branch Predict) blt r%d,[%h]", f_opreg1, f_imm21);
                            if(f_imm21[20] == 1 || (regs_predict[f_opreg1] & RP_ZERO) == 1 || 1) begin
                                if(f_imm21[20] == 0) fetch_addr <= fetch_addr + ({ 12'h0, f_imm21[19:0] } << 2);
                                else fetch_addr <= fetch_addr - ({ 12'h0, f_imm21[19:0] } << 2);
                                $display("%m: (Branch Predict) Will be taken");
                            end else begin
                                $display("%m: (Branch Predict) Will not be taken");
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
                            if(f_imm16 != 16'b0) begin
                                regs_predict[f_opreg1] <= RP_NON_ZERO;
                            end else begin
                                regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                            end
                            end
                        6'b1?_?011: begin // MOV rd, [ra + imm16]
                            regs_predict[f_opreg1] <= regs_predict[f_opreg1] | RP_UNSPEC_MEM;
                            end
                        6'b??_?010: begin end // MOV [ra + imm16], rd
                        // Instructions starting with 111001
                        6'b11_1001: begin
                            casez(f_inst_hi[4:2])
                                3'b111: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                3'b110: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                3'b011: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                3'b010: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                3'b001: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                3'b101: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                3'b100: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                default: begin end
                            endcase
                            // These high 1 bit is indicative of a MOV, the following 3 bytes MUST have atleast one set
                            if(f_inst_hi[5] == 1 && (f_inst_hi[4:2] & 3'b111) != 0 && (f_inst_hi[5:2] & 4'b1100) == 4'b1100) begin
                                regs_predict[f_opreg1] <= regs_predict[f_opreg1] | RP_UNSPEC_MEM;
                            end
                            end
                        6'b10_1001: begin
                            $display("%m: (Branch Predict) privileged_inst");
                            casez(f_inst_hi[5:2])
                            OP_G2_MFCR: begin end
                            OP_G2_MTCR: begin end
                            OP_G2_CACHEI: begin end
                            OP_G2_HLT: begin end
                            default: fetch_addr <= ctl_regs[CREG_EVEC];
                            endcase
                            end
                        6'b11_0001: begin
                            $display("%m: (Branch Predict) special_insn");
                            casez(f_inst_hi[5:2])
                                OP_G3_DIV: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                OP_G3_DIVS: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                OP_G3_MOD: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                OP_G3_MUL: regs_predict[f_opreg1] <= regs_predict[f_opreg2] | regs_predict[f_opreg3];
                                OP_G3_BRK: fetch_addr <= ctl_regs[CREG_EVEC];
                                OP_G3_SYS: fetch_addr <= ctl_regs[CREG_EVEC];
                                default: fetch_addr <= ctl_regs[CREG_EVEC];
                            endcase
                            end
                        default: begin
                            // ...
                            end
                    endcase
                end
            end
        end
    end

    always @(posedge clk) begin
        // Execution thread
        if(!stall_exec) begin
            if(state == S_BRANCHED) begin
                $display("%m: S_BRANCHED");
                // We have to decrement 1 the instruction number of the execution unit because it is automatically incremented by the
                // parallel executor thread (see below)
                if(1) begin
                    $display("%m: (Branch Predict) Failed prediction pc=0x%h", pc);
                    if(took_branch == 1) begin
                        $display("%m: (Branch Predict) I tought it wouldn't branch, but it did");
                    end else begin
                        $display("%m: (Branch Predict) I tought it would branch, but it did");
                    end
                    stall_sramio <= 0;
                    // This is used to update the fetch address after an exception or PC, meaning we had to stall
                    // so let's unstall the fetcher
                    fetch_addr <= pc;
                end else begin
                    $display("%m: (Branch Predict) Success! prediction pc=0x%h", pc);
                end
                state <= S_EXECUTE;
                stall_sramio <= 0;
            end else begin
                $display("%m: Execution icache_data_out=0x%h,inst=0x%h,pc=0x%h", icache_data_out, execute_inst, pc);
                took_branch <= 0;
                casez(inst_lo)
                    // This is an invalid opcode, but used internally as a "true no-op", no PC is modified
                    // no anything is modified, good for continuing the executor without stanling
                    6'b00_0000: begin
                        $display("%m: tnop");
                        end
                    // JALR [rd], [ra], [imm29]
                    OP_JALR: begin
                        $display("%m: jalr r%d,r%d,[%h]", opreg1, opreg2, { 8'h0, imm16 } << 2);
                        regs[opreg1] <= pc + 4;
                        pc <= regs[opreg2] + ({ 16'h0, imm16 } << 2); // TODO: Sign extend
                        // No need to stall or inform of a branch, since we can accurately
                        // predict that this jump will ALWAYS happen
                        state <= S_BRANCHED;
                        took_branch <= 0;
                        end
                    // JAL [imm29]
                    OP_J_OR_JAL: begin
                        $display("%m: jal [0x%8h],lr=0x%8h", { 3'h0, imm29 } << 2, pc + 4);
                        if(execute_inst[0] == 1) begin
                            regs[REG_LR] <= pc + 4;
                        end
                        pc <= (pc & 32'h80000000) | ({ 3'h0, imm29 } << 2);
                        // No need to stall or inform of a branch, since we can accurately
                        // predict that this jump will ALWAYS happen
                        took_branch <= 1;
                        end
                    // BEQ ra, [imm21]
                    OP_BEQ: begin
                        $display("%m: beq r%d,[%h]", opreg1, imm21);
                        if(regs[opreg1] == 32'h0) begin
                            $display("%m: Branch taken!");
                            if(imm21[20] == 1) pc <= pc - ({ 12'h0, imm21[19:0] } << 2);
                            else pc <= pc + ({ 12'h0, imm21[19:0] } << 2);
                            took_branch <= 1;
                        end else begin
                            pc <= pc + 4;
                        end
                        stall_sramio <= 1;
                        state <= S_BRANCHED;
                        end
                    // BNE ra, [imm21]
                    OP_BNE: begin
                        $display("%m: bne r%d,[%h]", opreg1, imm21);
                        if(regs[opreg1] != 32'h0) begin
                            $display("%m: Branch taken!");
                            if(imm21[20] == 1) pc <= pc - ({ 12'h0, imm21[19:0] } << 2);
                            else pc <= pc + ({ 12'h0, imm21[19:0] } << 2);
                            took_branch <= 1;
                        end else begin
                            pc <= pc + 4;
                        end
                        stall_sramio <= 1;
                        state <= S_BRANCHED;
                        end
                    // BLT ra, [imm21]
                    OP_BLT: begin
                        $display("%m: blt r%d,[%h]", opreg1, imm21);
                        if(regs[opreg1][31] == 0) begin
                            $display("%m: Branch taken!");
                            if(imm21[20] == 1) pc <= pc - ({ 12'h0, imm21[19:0] } << 2);
                            else pc <= pc + ({ 12'h0, imm21[19:0] } << 2);
                            took_branch <= 1;
                        end else begin
                            pc <= pc + 4;
                        end
                        stall_sramio <= 1;
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
                    6'b1?_?011: begin // MOV rd, [ra + imm16]
                        $display("%m: mov(16)(R) [r%d+%h],r%d,sz=%b", opreg1, { 8'h0, imm16 }, opreg2, inst_lo[4:3]);
                        trans_size <= inst_lo[4:3]; // Check OP_G1_MV_BITMASK
                        read_regno <= opreg1;
                        rw_addr <= regs[opreg2] + { 16'h0, imm16 };
                        state <= S_READ;
                        stall_sramio <= 1;
                        pc <= pc + 4;
                        end
                    6'b??_?010: begin // MOV [ra + imm16], rd
                        $display("%m: mov(5)(W) [r%d+%h],[%d],sz=%b", opreg1, { 8'h0, imm16 }, imm5, inst_lo[4:3]);
                        trans_size <= inst_lo[4:3]; // Check OP_G1_MV_BITMASK
                        if(inst_lo[5] == 0) begin // Write immediate
                            write_value <= { 27'h0, imm5 };
                        end else begin // Write register rb
                            write_value <= regs[opreg2];
                        end
                        rw_addr <= regs[opreg1] + { 16'h0, imm16 };
                        state <= S_PREWRITE;
                        stall_sramio <= 1;
                        pc <= pc + 4;
                        end
                    // Instructions starting with 111001
                    6'b11_1001: begin
                        $display("%m: alu_inst r%d,r%d,r%d,instmode=%b,op=%b", opreg1, opreg2, opreg3, opg1_instmode, inst_hi);
                        // Instmode
                        casez(opg1_instmode)
                            OPM_G1_LHS: tmp32 <= regs[opreg3] >> { 26'h0, imm5};
                            OPM_G1_RHS: tmp32 <= regs[opreg3] << { 26'h0, imm5};
                            OPM_G1_ASH: tmp32 <= regs[opreg3] >>> { 26'h0, imm5}; // TODO: What is ASH?
                            OPM_G1_ROR: tmp32 <= regs[opreg3] >>> { 26'h0, imm5};
                        endcase

                        if(inst_hi[4:2] == OP_NOR) begin
                            $display("%m: nor");
                            regs[opreg1] <= ~(regs[opreg2] | tmp32);
                        end else begin
                            regs[opreg1] <= alu_op_result(inst_hi[4:2], regs[opreg2], tmp32);
                        end

                        // These high 1 bit is indicative of a MOV, the following 3 bytes MUST have atleast one set
                        if(inst_hi[5] == 1 && (inst_hi[4:2] & 3'b111) != 0) begin
                            casez(inst_hi[5:2])
                                OP_G1_MOV_FR: begin // Move-From-Registers
                                    $display("%m: mov(FR) [r%d+r%d+%d],r%d,sz=%b", opreg2, opreg3, imm5, opreg1, inst_hi[3:2]);
                                    trans_size <= inst_hi[3:2]; // Check OP_G1_MV_BITMASK
                                    write_value <= regs[opreg1];
                                    rw_addr <= regs[opreg2] + tmp32;
                                    state <= S_PREWRITE;
                                    stall_sramio <= 1;
                                    end
                                OP_G1_MOV_TR: begin // Move-To-Register
                                    $display("%m: mov(TR) r%d,[r%d+r%d+%d],sz=%b", opreg1, opreg2, opreg3, imm5, inst_hi[3:2]);
                                    trans_size <= inst_hi[3:2]; // Check OP_G1_MV_BITMASK
                                    read_regno <= opreg1;
                                    rw_addr <= regs[opreg2] + tmp32;
                                    state <= S_READ;
                                    stall_sramio <= 1;
                                    end
                                default: begin end
                            endcase
                        end
                        pc <= pc + 4;
                        end
                    6'b10_1001: begin
                        $display("%m: privileged_inst");
                        casez(inst_hi[5:2])
                        // MFCR [opreg1] [opreg3]
                        OP_G2_MFCR: begin
                            $display("%m: mfcr r%d,cr%d", opreg1, opreg3);
                            regs[opreg1] <= ctl_regs[opreg3];
                            pc <= pc + 4;
                            end
                        // MTCR [opreg3] [opreg2]
                        OP_G2_MTCR: begin
                            $display("%m: mtcr cr%d,r%d", opreg3, opreg2);
                            ctl_regs[opreg3] <= regs[opreg2];
                            pc <= pc + 4;
                            end
                        // CACHEI [imm22]
                        OP_G2_CACHEI: begin
                            $display("%m: cachei [%h]", imm22);
                            fetch_addr <= pc;
                            stall_sramio <= 1;
                            pc <= pc + 4;
                            end
                        // FWC [imm22]
                        4'b1010: begin
                            ctl_regs[CREG_EBADADDR] <= pc;
                            pc <= ctl_regs[CREG_FWVEC];
                            ctl_regs[CREG_RS][31:28] <= ECAUSE_SYSCALL;
                            end
                        // HLT [imm22]
                        OP_G2_HLT: begin
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
                            OP_G3_BRK: begin
                                $display("%m: brk [%h]", imm22);
                                // TODO: Is imm22 used at all?
                                ctl_regs[CREG_EBADADDR] <= pc;
                                pc <= ctl_regs[CREG_EVEC];
                                ctl_regs[CREG_RS][31:28] <= ECAUSE_BREAKPOINT;
                                stall_sramio <= 1;
                                state <= S_BRANCHED;
                                end
                            OP_G3_DIV: begin
                                if(opreg4 != 5'b0) begin
                                    // Raise UD
                                    $display("%m: exception - invalid div inst=%b", execute_inst);
                                    ctl_regs[CREG_EBADADDR] <= pc;
                                    pc <= ctl_regs[CREG_EVEC];
                                    ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                                    stall_sramio <= 1;
                                    state <= S_BRANCHED;
                                end else begin
                                    regs[opreg1] <= regs[opreg2] / regs[opreg3];
                                    pc <= pc + 4;
                                end
                                end
                            OP_G3_DIVS: begin
                                if(opreg4 != 5'b0) begin
                                    // Raise UD
                                    $display("%m: exception - invalid divs inst=%b", execute_inst);
                                    ctl_regs[CREG_EBADADDR] <= pc;
                                    pc <= ctl_regs[CREG_EVEC];
                                    ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                                    stall_sramio <= 1;
                                    state <= S_BRANCHED;
                                end else begin
                                    // TODO: Properly perform signed division
                                    regs[opreg1] <= { (regs[opreg2][31] | regs[opreg3][31]), regs[opreg2][30:0] / regs[opreg3][30:0] };
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
                                    $display("%m: exception - invalid mod inst=%b", execute_inst);
                                    ctl_regs[CREG_EBADADDR] <= pc;
                                    pc <= ctl_regs[CREG_EVEC];
                                    ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                                    stall_sramio <= 1;
                                    state <= S_BRANCHED;
                                end else begin
                                    regs[opreg1] <= regs[opreg2] % regs[opreg3];
                                    pc <= pc + 4;
                                end
                                end
                            OP_G3_MUL: begin
                                if(opreg4 != 5'b0) begin
                                    // Raise UD
                                    $display("%m: exception - invalid mul inst=%b", execute_inst);
                                    ctl_regs[CREG_EBADADDR] <= pc;
                                    pc <= ctl_regs[CREG_EVEC];
                                    ctl_regs[CREG_RS][31:28] <= ECAUSE_INVALID_INST;
                                    stall_sramio <= 1;
                                    state <= S_BRANCHED;
                                end else begin
                                    regs[opreg1] <= regs[opreg2] * regs[opreg3];
                                    pc <= pc + 4;
                                end
                                end
                            OP_G3_SC: begin
                                // TODO: Is this a memory access?
                                pc <= pc + 4;
                                end
                            OP_G3_SYS: begin
                                $display("%m: sys [%h]", imm22);
                                // TODO: Is imm22 used at all?
                                ctl_regs[CREG_EBADADDR] <= pc;
                                pc <= ctl_regs[CREG_EVEC];
                                ctl_regs[CREG_RS][31:28] <= ECAUSE_SYSCALL;
                                stall_sramio <= 1;
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
                        stall_sramio <= 1;
                        state <= S_BRANCHED;
                    end
                endcase
            end
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
