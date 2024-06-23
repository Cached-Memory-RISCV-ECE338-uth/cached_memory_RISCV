`timescale 1ns/10ps

module word_to_block #(
    parameter ADDRESSIZE    = 32,  // Size in bits
    parameter WORDSIZE      = 32,    // Size in bits
    parameter OFFSETBITS    = 2,
    parameter BLOCKSIZE     =  16,    // Number of Words 
    parameter BLOCKSIZE_log = 4
) (
    input [WORDSIZE-1:0] data_in,
    input [ADDRESSIZE-1:0] mask_in,
    input [ADDRESSIZE-1:0] address,
    output reg [(BLOCKSIZE*WORDSIZE)-1:0] mask_out,
    output reg [(BLOCKSIZE*WORDSIZE)-1:0] data_out,
    input enable_in,
    output reg enable_out
);
wire [BLOCKSIZE_log-1:0] offset;
assign offset = address[2**BLOCKSIZE+OFFSETBITS-1:OFFSETBITS];

reg [(BLOCKSIZE*WORDSIZE)-1:0] mask_temp;
reg [(BLOCKSIZE*WORDSIZE)-1:0] data_temp;
integer i;
always@(*)
begin
    
    mask_temp={(BLOCKSIZE*WORDSIZE){1'b0}};
    data_temp={(BLOCKSIZE*WORDSIZE){1'b0}};
    // Set the appropriate segment of mask_temp and data_temp based on offset
    for (i = 0; i < (BLOCKSIZE); i = i + 1) begin
        if (i == offset) begin
            mask_temp[(i*WORDSIZE) +: WORDSIZE] = mask_in;
            data_temp[(i*WORDSIZE) +: WORDSIZE] = data_in;
        end
    end

    mask_out = mask_temp;
    data_out = data_temp;
    enable_out = enable_in;
end
endmodule