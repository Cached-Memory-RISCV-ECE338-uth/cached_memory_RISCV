`include "constants.v"
`include "config.vh"

/**************** control for Bypass Detection in EX pipe stage  *********/
module  control_bypass_ex(	input clk,
							output reg [1:0] bypassA,
							output reg [1:0] bypassB,
							input mem_done, //Used to bypass the memory read state
							input [4:0] idex_rs1,
							input [4:0] idex_rs2,
							input [4:0] exmem_rd,
							input [4:0] memwb_rd,
							input exmem_regwrite,
							input memwb_regwrite);

// reg [4:0] delay_memwb;
reg bypass_holdA, bypass_holdB;
// always@(posedge clk)
// begin 
// 	delay_memwb <= memwb_rd;
// end
always @(*)
begin
	
	// Bypassing the memory WB in order to execute the next instruction
	if(mem_done && exmem_rd == idex_rs1 && exmem_rd != 5'b0)
	begin
		bypassA = 2'b11;
	end
	else if (exmem_regwrite == 1'b1 && exmem_rd != 5'b0 && exmem_rd == idex_rs1) begin
		bypassA = 2'b10;
	end
	else if (memwb_regwrite == 1'b1 && memwb_rd != 5'b0 && memwb_rd == idex_rs1) begin
		bypassA = 2'b01;
	end
	else begin
		bypassA = 2'b00;
	end
end

always @(*)
begin
	if(mem_done && exmem_rd == idex_rs2 && exmem_rd != 5'b0)
	begin
		bypassA = 2'b11;
	end
	else if (exmem_regwrite == 1'b1 && exmem_rd != 5'b0 && exmem_rd == idex_rs2) begin
		bypassB = 2'b10;
	end
	else if (memwb_regwrite == 1'b1 && memwb_rd != 5'b0 && memwb_rd == idex_rs2) begin
		bypassB = 2'b01;
	end
	else begin
		bypassB = 2'b00;
	end
end

endmodule