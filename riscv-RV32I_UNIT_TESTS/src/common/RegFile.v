`include "constants.v"
`include "config.vh"
// Register File. Read ports: address raA, data rdA, renA
//                            address raB, data rdB, renB
//                Write port: address wa, data wd, enable wen.
module RegFile (input clock, reset,
				input [4:0] raA, raB, wa,
				input wen,
				input [31:0] wd,
				input stall,
				output [31:0] rdA, rdB);
/****** SIGNALS ******/
integer i;
reg [31:0] data[31:0];
reg stall_reg;
/****** LOGIC ******/

// The register file is written at the positive edge. Make sure that bypasssing is enabled. 
always @(posedge clock or negedge reset)
begin
	//	probably bad code! But fuck it!
	stall_reg<=stall;
	if (reset == 1'b0) begin
		for (i = 0; i < 32; i = i+1)
			data[i] <= 0;
	end
	else begin
		if (wen == 1'b1 && wa != 5'b0 && stall_reg == 1'b0)
			data[wa] <=  wd;
	end
end

assign rdA = (wen == 1'b1 && wa == raA && wa != 5'b0) ? wd : data[raA];
assign rdB = (wen == 1'b1 && wa == raB && wa != 5'b0) ? wd : data[raB];

endmodule