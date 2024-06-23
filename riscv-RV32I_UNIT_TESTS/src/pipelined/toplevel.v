`include "constants.v"
`include "config.vh"

module toplevel(input clock, 
				input reset, 
				output overflow);

cpu cpu(.clock(clock), 
		.reset(reset), 
		.overflow(overflow));

endmodule