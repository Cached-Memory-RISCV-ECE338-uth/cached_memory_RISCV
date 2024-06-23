`include "constants.v"
`include "config.vh"

// Read : disable wen, enable ren, address addr, data dout
// Write: enable wen, disable ren, address addr, data din.
module Dmem (	input clock, reset,
				input ren, wen,
				input [3:0] byte_select_vector,
				input [`DATA_BITS-3:0] addr,
				input [31:0] din,
				output reg [31:0] dout,
				output reg gen_stall);

/****** SIGNALS ******/
reg [31:0] data[0:2**(`DATA_BITS-2)-1];
reg [2:0]  counter;

/****** LOGIC ******/
reg writing;
reg reading;

always@(*)
begin
  if (wen == 1'b1 && ren==1'b0 && reset == 1'b1) begin
		gen_stall <= 1;
		writing<=1;
	end
	else if (wen == 1'b0 && ren==1'b1 && reset == 1'b1) begin
		gen_stall <= 1;
		reading<=1;
	end 
end
/* Write memory in the negative edge of the clock */
always @(posedge clock or negedge reset)
begin
	if(!reset) begin
		counter = 2'b0;
		gen_stall<=0;
		writing<=0;
		reading<=0;
	end 
	if(gen_stall)
	begin
		counter <= counter + 1;
		if(counter == 3'b1000) begin 
			if(writing)
			begin
				if (byte_select_vector[3] == 1'b1)
					data[addr][31:24] <= din[31:24];
				if (byte_select_vector[2] == 1'b1)
					data[addr][23:16] <= din[23:16];
				if (byte_select_vector[1] == 1'b1)
					data[addr][15:8] <= din[15:8];
				if (byte_select_vector[0] == 1'b1)
					data[addr][7:0] <= din[7:0];
				writing<=0;
			end
			if(reading)
			begin
				dout <= (reset == 1) ? data[addr] : 32'b0;
				reading <= 0;
			end
			gen_stall <= 0;
			// data[addr] <= din;
		end
		else
		begin
			gen_stall <= 1;
		end
	end
end

/****** SIMULATION ******/

`ifdef DATA_HEX
initial $readmemh(`DATA_HEX, data);
`endif


always @(ren or wen)
begin
	if (ren & wen)
		$display ("\nMemory ERROR (time %0d): ren and wen both active!\n", $time);
end

endmodule