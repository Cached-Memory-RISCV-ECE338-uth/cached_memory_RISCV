`include "constants.v"
`include "config.vh"

/**************** control for Stall Detection in ID pipe stage  **********/
/****************** Create bubbles for loads and branches ****************/
module  control_stall_id(
	input clk,	
	output reg bubble_ifid,
	output reg bubble_idex,
	output reg bubble_exmem,
	output reg write_ifid,
	output reg write_idex,
	output reg write_exmem,
	output reg write_memwb,
	output reg write_pc,
	input gen_stall,
	input [4:0] ifid_rs,
	input [4:0] ifid_rt,
	input [4:0] idex_rd,
	input idex_memread,
	input Jump,
	input PCSrc);

reg stall_delay;

always @(posedge clk)
begin
	stall_delay <= gen_stall;
end
always @(*)
begin
	bubble_ifid		= 1'b0;
	bubble_idex		= 1'b0;
	bubble_exmem	= 1'b0;
	write_pc		= 1'b1;
	write_ifid		= 1'b1;
	write_idex		= 1'b1;
	write_exmem		= 1'b1;
	write_memwb		= 1'b1;

	/*if ((idex_memread == 1'b1) && ((idex_rd==ifid_rs) || (idex_rd==ifid_rt))) begin // Load stall
		bubble_idex	= 1'b1;
		write_ifid	= 1'b0;
		write_pc	= 1'b0;
	end*/
	if (Jump == 1'b1) begin // j instruction in ID stage	
		bubble_ifid	= 1'b1;
		write_pc	= 1'b1;
	end

	if (PCSrc == 1'b1) begin // Taken Branch in MEM stage
		bubble_ifid		= 1'b1;
		bubble_idex		= 1'b1;
		bubble_exmem	= 1'b1;
		write_pc		= 1'b1;
	end
	if(gen_stall)
	begin
		write_pc		= 1'b0;
		write_ifid		= 1'b0;
		write_idex		= 1'b0;
		write_exmem		= 1'b0;
		// bubble_exmem	= 1'b1;
		write_memwb		= 1'b0;
	end
	// if(stall_delay)
	// begin
	// 	write_pc		= 1'b0;
	// 	write_ifid		= 1'b0;
	// 	write_idex		= 1'b0;
	// 	write_exmem		= 1'b0;
	// 	write_memwb		= 1'b0;

	// end
end
endmodule