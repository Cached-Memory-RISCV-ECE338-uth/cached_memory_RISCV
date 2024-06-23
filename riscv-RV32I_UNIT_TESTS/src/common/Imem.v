`include "constants.v"
`include "config.vh"

module Imem (	input clk,
				input reset, 
				input ren, 
				input  [`TEXT_BITS-3:0] addr,
				input inst_stall, 
				output reg[31:0] dout);

/****** SIGNALS ******/
reg [31:0] data[0:2**(`TEXT_BITS-2)-1];
reg [1:0] counter;
reg reading;

/****** LOGIC ******/
// assign dout = (reset == 1 && ren == 1) ? data[addr] : 32'b0;
always @(posedge clk or negedge reset) begin
	if(!reset) begin
		dout <= 32'b0;
		counter = 2'b0;
		reading<=0;
	end 
	else if(inst_stall == 1'b0) begin
		dout <= (reset == 1) ? data[addr] : 32'b0;
		// counter <= counter + 1;
		// inst_stall <= 1;
		// if(counter == 2'b11) begin 
		// 	inst_stall <= 0;
		// 	counter <= 0;
		// end
	end
end


// reg ren_reg;
// always@(*)
// begin
// 	if (ren_reg==1'b1 && reset == 1'b1) 
// 	begin
// 		inst_stall <= 1;
// 		reading<=1;
// 	end 
// end
// always @(posedge clk or negedge reset) begin
// 	ren_reg<=ren;
// 	if(!reset) begin
// 		counter = 2'b0;
// 		inst_stall<=;
// 		reading<=0;
// 	end 
// 	if(inst_stall)
// 	begin
// 		counter <= counter + 1;
// 		if(counter == 2'b11) begin 
// 			dout <= (reset == 1) ? data[addr] : 32'b0;
// 			$display("Should be zero");
// 			inst_stall <= 0;
// 		end
// 	end
// end

/****** SIMULATION ******/
`ifdef TEXT_HEX
initial $readmemh(`TEXT_HEX, data);
`endif

endmodule