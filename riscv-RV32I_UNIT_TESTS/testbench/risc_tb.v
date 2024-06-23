`timescale 1ns / 1ps

`define CYCLE 20
`define N 2000

module tb;
    
    reg clk;
    reg reset;
    wire overflow;
    cpu cpu(.clock(clk), 
		.reset(reset), 
		.overflow(overflow));
    // Clock generation
    initial begin
        clk = 1;
        forever #5 clk = ~clk; // 10ns clock period (100MHz)
    end
    initial begin
        reset=0;
        #20
        reset=1;
        #500
        $finish;
    end
endmodule
