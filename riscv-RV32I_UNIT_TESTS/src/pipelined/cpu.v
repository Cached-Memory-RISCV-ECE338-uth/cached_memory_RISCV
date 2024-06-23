`include "constants.v"
`include "config.vh"

/*****************************************************************************************/
/* Implementation of the 5-stage MIPS pipeline that supports the following instructions: */
/*  R-format: add, sub, and, or, xor, slt                                                */
/*  addi, lw, sw, beq, j                                                                 */
/*****************************************************************************************/
module cpu(	input clock,
			input reset,
			output overflow);
reg		[31:0]	IFID_instr;
reg		[31:0]	PC, PC_Hold0,PC_Hold1, IFID_PC, IDEX_PC,EXMEM_PC, MEMWB_PC;
wire	[31:0]	PCplus4, JumpAddress, PC_new;
wire	[31:0]	instr;
wire			inA_is_PC, branch_taken;
wire	[31:0]	BranchInA;
reg		[31:0]	IDEX_signExtend;
wire	[31:0]	signExtend;
wire	[31:0]	rdA, rdB;
reg		[31:0]	IDEX_rdA, IDEX_rdB;
reg		[2:0]	IDEX_funct3;
reg		[6:0]	IDEX_funct7;
reg		[4:0]	IDEX_instr_rs2, IDEX_instr_rs1, IDEX_instr_rd;
reg				IDEX_RegDst, IDEX_ALUSrc, IDEX_inA_is_PC, IDEX_Jump, IDEX_JumpJALR;
reg		[2:0]	IDEX_ALUcntrl;
reg				IDEX_MemRead, IDEX_MemWrite;
reg				IDEX_MemToReg, IDEX_RegWrite;
reg 	[2:0]	EXMEM_funct3, MEMWB_funct3;
reg 	[4:0]	EXMEM_RegWriteAddr;
reg 	[31:0]	EXMEM_ALUOut;
reg 	[31:0]	EXMEM_BranchALUOut;
reg				EXMEM_Zero, EXMEM_JumpJALR;
wire		[3:0]	byte_select_vector;
reg		[31:0]	EXMEM_MemWriteData;
wire	[31:0]	MemWriteData;
reg				EXMEM_MemRead, EXMEM_MemWrite, EXMEM_RegWrite, EXMEM_MemToReg;
reg		[31:0]	MEMWB_DMemOut;
reg		[4:0]	MEMWB_RegWriteAddr;
reg		[31:0]	MEMWB_ALUOut;
reg				MEMWB_MemToReg, MEMWB_RegWrite;
wire	[31:0]	ALUInA, ALUInB, ALUOut, BranchALUOut, DMemOut, MemOut, wRegData;
wire			Zero, RegDst, MemRead, MemWrite, MemToReg, ALUSrc, PCSrc, RegWrite, Jump, JumpJALR;
wire		[31:0] 	bypassOutA, bypassOutB; //changed from wire
wire			Branch;
reg				IDEX_Branch, EXMEM_Branch;
wire			bubble_ifid, bubble_idex, bubble_exmem;   // create a NOP in respective stages
wire			write_ifid, write_idex, write_exmem, write_memwb, write_pc;  // enable/disable pipeline registers
wire	[6:0]	opcode;
wire	[2:0]	funct3, ALUcntrl; 
wire	[6:0]	funct7;
wire	[4:0]	instr_rs1, instr_rs2, instr_rd, RegWriteAddr;
reg 	[31:0] debug_instrIFID, debug_instrIDEX, debug_instrEXMEM, debug_instrMEMWB; 
wire	[3:0]	ALUOp;
wire	[1:0]	bypassA, bypassB;
wire	[31:0]	imm_i, imm_s, imm_b, imm_u, imm_j;
wire gen_stall, inst_stall;

/********************** Instruction Fetch Unit (IF)  **********************/
always @(posedge clock or negedge reset)
begin 
	if (reset == 1'b0)     
		begin
		PC <= `INITIAL_PC;
		PC_Hold0 <= `INITIAL_PC;
		PC_Hold1 <= `INITIAL_PC;
		end     
	else if (write_pc == 1'b1)
	begin
		PC_Hold0 <= PC_new;
		PC_Hold1 <= PC_Hold0;
	end
		
end

always@(*)
begin
	PC = (mem_stall | (mem_stall_hold&data_read_done) )?PC_Hold1:PC_Hold0;
end

// PC adder
assign PCplus4 = PC + 32'd4;

// PCSrc multiplexer (branch or not)
assign PC_new = (PCSrc == 1'b0) ? 
				((Jump == 1'b0) ? PCplus4 : JumpAddress) :
				EXMEM_BranchALUOut;

assign JumpAddress = IFID_PC + signExtend;

// IFID pipeline register
always @(posedge clock or negedge reset)
begin 
	if ((reset == 1'b0) || (bubble_ifid == 1'b1)) begin
		IFID_PC			<= 32'b0;
		//IFID_instr		<= 32'b0;
	end 
	else if (write_ifid == 1'b1) begin
		IFID_PC			<= PC;
		//IFID_instr		<= instr;
	end
end

always @(*)begin
	if ((reset == 1'b0) || (bubble_ifid == 1'b1)) begin
		IFID_instr		= 32'b0;
		debug_instrIFID = 32'b0;
	end 
	else if (write_ifid == 1'b1) begin
		IFID_instr		= instr;
		debug_instrIFID = instr;
	end
end


wire mem_stall;
wire stall_mem_entire;
reg mem_stall_hold;
wire data_read_done,instr_read_done;
wire data_write_done;
wire [31:0] data_read;
wire [31:0] instr_read;

// // Instruction memory
// Imem cpu_IMem(
// 	.clk(clock),
// 	.reset(reset),
// 	.ren(write_ifid),
// 	.addr(PC[`TEXT_BITS-1:2]),
// 	.inst_stall(mem_stall),
// 	.dout(instr)
// );

/***************************** Instruction Decode Unit (ID)  *******************/
assign opcode		= IFID_instr[6:0];
assign funct3		= IFID_instr[14:12];
assign funct7		= IFID_instr[31:25];
assign instr_rs1	= IFID_instr[19:15];
assign instr_rs2	= IFID_instr[24:20];
assign instr_rd		= IFID_instr[11:7];

//Sign Extension Unit
signExtend signExtendUnit (
	.instr(IFID_instr[31:7]),
	.imm_i(imm_i),
	.imm_s(imm_s),
	.imm_b(imm_b),
	.imm_u(imm_u),
	.imm_j(imm_j)
);

// Register file
RegFile cpu_regs (
	.clock(clock),
	.reset(reset),
	.raA(instr_rs1),
	.raB(instr_rs2),
	.wa(MEMWB_RegWriteAddr),
	.wen(MEMWB_RegWrite),
	.wd(wRegData),
	.stall(mem_stall),
	.rdA(rdA),
	.rdB(rdB)
);

// Sign Extended Signal Selection
SignExtendSelector SignExtendSelector (
	.out(signExtend),
	.imm_i(imm_i),
	.imm_s(imm_s),
	.imm_b(imm_b),
	.imm_u(imm_u),
	.imm_j(imm_j),
	.opcode(opcode)
);

// IDEX pipeline register
always @(posedge clock or negedge reset)
begin 
	mem_stall_hold <= stall_mem_entire;
	if ((reset == 1'b0) || (bubble_idex == 1'b1)) begin
		IDEX_inA_is_PC	<= 1'b0;
		IDEX_Jump		<= 1'b0;
		IDEX_JumpJALR	<= 1'b0;
		IDEX_signExtend	<= 32'b0;
		IDEX_instr_rd	<= 5'b0;
		IDEX_instr_rs1	<= 5'b0;
		IDEX_instr_rs2	<= 5'b0;
		IDEX_RegDst		<= 1'b0;
		IDEX_ALUcntrl	<= 3'b0;
		IDEX_ALUSrc		<= 1'b0;
		IDEX_Branch		<= 1'b0;
		IDEX_MemRead	<= 1'b0;
		IDEX_MemWrite	<= 1'b0;
		IDEX_MemToReg	<= 1'b0;
		IDEX_RegWrite	<= 1'b0;
		IDEX_funct3		<= 3'b0;
		IDEX_funct7		<= 7'b0;
		IDEX_PC			<= 32'b0;
		IDEX_rdA		<= 32'b0;
		IDEX_rdB		<= 32'b0;
		debug_instrIDEX <= 32'b0;
		mem_stall_hold <= 1'b0;
	end
	else if (write_idex == 1'b1) begin
		IDEX_inA_is_PC	<= inA_is_PC;
		IDEX_Jump		<= Jump;
		IDEX_JumpJALR	<= JumpJALR;
		IDEX_signExtend	<= signExtend;
		IDEX_instr_rd	<= instr_rd;
		IDEX_instr_rs1	<= instr_rs1;
		IDEX_instr_rs2	<= instr_rs2;
		IDEX_RegDst		<= RegDst;
		IDEX_ALUcntrl	<= ALUcntrl;
		IDEX_ALUSrc		<= ALUSrc;
		IDEX_Branch		<= Branch;
		IDEX_MemRead	<= MemRead;
		IDEX_MemWrite	<= MemWrite;
		IDEX_MemToReg	<= MemToReg;
		IDEX_RegWrite	<= RegWrite;
		IDEX_funct3		<= funct3;
		IDEX_funct7		<= funct7;
		IDEX_PC			<= IFID_PC;
		IDEX_rdA		<= rdA;
		IDEX_rdB		<= rdB;
		debug_instrIDEX <= IFID_instr;
	end
end

// Main Control Unit
control_main control_main (
	.RegDst(RegDst),
	.Branch(Branch),
	.MemRead(MemRead),
	.MemWrite(MemWrite),
	.MemToReg(MemToReg),
	.ALUSrc(ALUSrc),
	.RegWrite(RegWrite),
	.Jump(Jump),
	.JumpJALR(JumpJALR),
	.inA_is_PC(inA_is_PC),
	.ALUcntrl(ALUcntrl),
	.opcode(opcode)
);

// Control Unit that generates stalls and bubbles to pipeline stages
control_stall_id control_stall_id (
	.clk(clock),
	.bubble_ifid	(bubble_ifid),
	.bubble_idex	(bubble_idex),
	.bubble_exmem	(bubble_exmem),
	.write_ifid		(write_ifid),
	.write_idex		(write_idex),
	.write_exmem	(write_exmem),
	.write_memwb	(write_memwb),
	.write_pc		(write_pc),
	.ifid_rs		(instr_rs1),
	.ifid_rt		(instr_rs2),
	.idex_rd		(IDEX_instr_rd),
	.idex_memread	(IDEX_MemRead),
	.Jump			(Jump),
	.PCSrc			(PCSrc),
	.gen_stall		(mem_stall));



// /************************ Execution Unit (EX)  ***********************************/

// always@(posedge clock or negedge reset)
// begin
// 	if (reset == 1'b0) begin
// 		bypassOutA <= 32'b0;
// 		bypassOutB <= 32'b0;
// 	end
// 	else  begin
// 		bypassOutA <= (bypassA==2'b00) ? IDEX_rdA :
// 					(bypassA==2'b01) ? wRegData :
// 										ALUOut;
// 		bypassOutB <= (bypassB==2'b00) ?	IDEX_rdB :
// 					(bypassB==2'b01) ?	wRegData :
// 										ALUOut;
// 	end
// end
assign bypassOutA = (bypassA==2'b00) ? IDEX_rdA :
					(bypassA==2'b01) ? wRegData :
					(bypassA==2'b10) ?	EXMEM_ALUOut :
										DMemOut;

assign bypassOutB = (bypassB==2'b00) ?	IDEX_rdB :
					(bypassB==2'b01) ?	wRegData :
					(bypassB==2'b10) ?	EXMEM_ALUOut :
										DMemOut;

assign ALUInA = (IDEX_inA_is_PC == 1'b1) ? IDEX_PC : bypassOutA;
		
assign ALUInB = (IDEX_Jump == 1'b1 || IDEX_JumpJALR == 1'b1) ? 	32'd4 :
				(IDEX_ALUSrc == 1'b0) ? bypassOutB :
										IDEX_signExtend;

assign BranchInA = (IDEX_JumpJALR == 1'b1) ? bypassOutA : IDEX_PC;

assign BranchALUOut = BranchInA + IDEX_signExtend;

// ALU
ALU cpu_alu(.out(ALUOut),
			.zero(Zero),	
			.overflow(overflow),
			.inA(ALUInA),
			.inB(ALUInB),
			.op(ALUOp));

assign RegWriteAddr = (IDEX_RegDst==1'b0) ? IDEX_instr_rs2 : IDEX_instr_rd;

// EXMEM pipeline register
always @(posedge clock or negedge reset)
begin
	if ((reset == 1'b0) || (bubble_exmem == 1'b1)) begin
		EXMEM_ALUOut		<= 32'b0;
		EXMEM_JumpJALR 		<= 1'b0;
		EXMEM_BranchALUOut	<= 32'b0;
		EXMEM_RegWriteAddr	<= 5'b0;
		EXMEM_MemWriteData	<= 32'b0;
		EXMEM_Zero			<= 1'b0;
		EXMEM_Branch		<= 1'b0;
		EXMEM_MemRead		<= 1'b0;
		EXMEM_MemWrite		<= 1'b0;
		EXMEM_MemToReg		<= 1'b0;
		EXMEM_RegWrite		<= 1'b0;
		EXMEM_funct3		<= 3'b111;
		EXMEM_PC			<= 32'b0;
		debug_instrEXMEM <= 32'b0;
	end 
	else if (write_exmem == 1'b1) begin
		EXMEM_ALUOut		<= ALUOut;
		EXMEM_JumpJALR		<= IDEX_JumpJALR;
		EXMEM_BranchALUOut	<= BranchALUOut;
		EXMEM_RegWriteAddr	<= RegWriteAddr;
		EXMEM_MemWriteData	<= bypassOutB;
		EXMEM_Zero			<= Zero;
		EXMEM_Branch		<= IDEX_Branch;
		EXMEM_MemRead		<= IDEX_MemRead;
		EXMEM_MemWrite		<= IDEX_MemWrite;
		EXMEM_MemToReg		<= IDEX_MemToReg;
		EXMEM_RegWrite		<= IDEX_RegWrite;
		EXMEM_funct3		<= IDEX_funct3;
		EXMEM_PC			<= IDEX_PC;
		debug_instrEXMEM <= debug_instrIDEX;
	end
end
// ALU control
control_alu control_alu(
	.ALUOp(ALUOp), 
	.ALUcntrl(IDEX_ALUcntrl), 
	.funct3(IDEX_funct3), 
	.funct7(IDEX_funct7)
);

// Bypass control
control_bypass_ex control_bypass_ex(
	.clk(clock),
	.bypassA(bypassA), 
	.bypassB(bypassB),
	.mem_done(data_read_done),
	.idex_rs1(IDEX_instr_rs1), 
	.idex_rs2(IDEX_instr_rs2),
	.exmem_rd(EXMEM_RegWriteAddr), 
	.memwb_rd(MEMWB_RegWriteAddr),
	.exmem_regwrite(EXMEM_RegWrite), 
	.memwb_regwrite(MEMWB_RegWrite)
);


/*********************************** Memory Unit (MEM)  ********************************************/
mem_write_selector mem_write_selector(
	.mem_select(EXMEM_funct3),
	.ALUin(EXMEM_MemWriteData),
	.offset(EXMEM_ALUOut[1:0]),
	.byte_select_vector(byte_select_vector),
	.out(MemWriteData)
);

// reg reading_latch;
// always @(posedge EXMEM_MemRead or negedge reset or posedge data_read_done)
// begin
//     if (reset == 1'b0) begin
//         reading_latch <= 1'b0;
//         $display("Time: %0t | Reset activated. reading_latch set to 0", $time);
//     end else begin
//         if (EXMEM_MemRead == 1'b1) begin
//             reading_latch <= 1'b1;
//             $display("Time: %0t | EXMEM_MemRead active. reading_latch set to 1", $time);
//         end
//         if (data_read_done == 1'b1) begin
//             reading_latch <= ~data_read_done;
//             $display("Time: %0t | Data read done", $time);
// 			$display("DATA DONE %b, latch is %b", data_read_done, reading_latch);
//         end
//     end
// end

assign mem_stall = (stall_mem_entire); 
mem_entire mem(
	.clk(clock),
	.rst(!reset),
	.stall(stall_mem_entire),
	.proc_data_read_enable(EXMEM_MemRead), 
	.proc_data_write_enable(EXMEM_MemWrite),
	.proc_data_read_done(data_read_done),
	.proc_data_write_done(data_write_done),
	.proc_data_address({{(32 - `DATA_BITS){1'b0}},{EXMEM_ALUOut[`DATA_BITS-1:0]}}),
	.proc_mask_vector(byte_select_vector),
	.proc_data_read(DMemOut),
	.proc_instr_read_enable(reset), // THIS IS VERY BAD PRACTICE, NEED TO FIX THIS
	.proc_instr_read_done(instr_read_done),
	// .proc_instr_address({{(32 - `TEXT_BITS+2){1'b0}},{PC[`TEXT_BITS-1:2]}}),
	.proc_instr_address(PC),
	.proc_instr_read(instr)
);
// // Data memory 1KB
// Dmem cpu_DMem(
// 	.clock(clock), 
// 	.reset(reset),
// 	.ren(EXMEM_MemRead), 
// 	.wen(EXMEM_MemWrite), 
// 	.byte_select_vector(byte_select_vector), 
// 	.addr(EXMEM_ALUOut[`DATA_BITS-1:2]), 
// 	.din(MemWriteData),
// 	.gen_stall(gen_stall), 
// 	.dout(DMemOut)
// );

// MEMWB pipeline register
always @(posedge clock or negedge reset or negedge mem_stall)
begin 
	if (reset == 1'b0) begin
		MEMWB_DMemOut		<= 32'b0;
		MEMWB_ALUOut		<= 32'b0;
		MEMWB_RegWriteAddr	<= 5'b0;
		MEMWB_MemToReg		<= 1'b0;
		MEMWB_RegWrite		<= 1'b0;
		MEMWB_funct3		<= 3'b111;
		MEMWB_PC			<= 32'b0;
		debug_instrMEMWB <= 32'b0;
	end 
	else if (write_memwb == 1'b1) begin
		MEMWB_DMemOut		<= DMemOut;
		MEMWB_ALUOut		<= EXMEM_ALUOut;
		MEMWB_RegWriteAddr	<= EXMEM_RegWriteAddr;
		MEMWB_MemToReg		<= EXMEM_MemToReg;
		MEMWB_RegWrite		<= EXMEM_RegWrite;
		MEMWB_funct3		<= EXMEM_funct3;
		MEMWB_PC			<= EXMEM_PC;
		debug_instrMEMWB <= debug_instrEXMEM;
	end
end
// 

// Branch control unit
control_branch control_branch (
	.branch_taken(branch_taken),
	.funct3(EXMEM_funct3),
	.Branch(EXMEM_Branch),
	.zero(EXMEM_Zero),
	.sign(EXMEM_ALUOut[31])
);

assign PCSrc = (EXMEM_JumpJALR) ? 1'b1 : branch_taken;

/**************************** WriteBack Unit (WB) **************************/  
mem_read_selector mem_read_selector(
	.mem_select(MEMWB_funct3),
	.DMemOut(MEMWB_DMemOut),
	.byte_index(MEMWB_ALUOut[1:0]),
	.out(MemOut)
);

assign wRegData = (MEMWB_MemToReg == 1'b0) ? MEMWB_ALUOut : DMemOut;

endmodule
