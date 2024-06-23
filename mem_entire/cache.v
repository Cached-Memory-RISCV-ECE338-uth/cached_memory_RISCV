`timescale 1ns/10ps

module L1_cache #(
    parameter ADDRESSIZE = 32,  // Size in bits
    parameter SETSIZE = 2,      // Number of bits
    parameter CACHESIZE = 1024, // Size in bytes
    parameter CACHESIZEBITS=2,
    parameter WORDSIZE = 32,
    parameter BLOCK_OFFSET = 4,         //LOGARITHMOS
    parameter NUM_OF_BLOCKS = 2**BLOCK_OFFSET,
    parameter BLOCKSIZE = NUM_OF_BLOCKS * WORDSIZE,
    parameter OFFSETSIZE = 2,   // Number of bits corrensponding to the offset_read
    parameter TAGSIZE = ADDRESSIZE - CACHESIZEBITS - OFFSETSIZE // Number of bits corresponding to the tag_read
) (
    input clk,
    input reset,
    input wr_en,
    input r_en,
    input [ADDRESSIZE-1:0] address_read,
    input [ADDRESSIZE-1:0] address_write,
    input [BLOCKSIZE-1:0] data_in,
    input [BLOCKSIZE-1:0] data_mask,
    input check_exist,
    output reg data_ready,
    output reg [WORDSIZE-1:0] data_out,
    output reg write_done,
    output reg read_done,
    
    output reg [15:0] cnt

);
    reg [WORDSIZE-1:0] data_out_reg;
    reg data_ready_reg;
    reg [BLOCKSIZE-1:0] word_mask = {{(BLOCKSIZE - WORDSIZE){1'b0}},{WORDSIZE{1'b1}}}; //512'hFFFFFFFF 

    reg wr_en_reg;
    reg r_en_reg;
    


    reg [TAGSIZE-1:0] cache_tag [0:2**CACHESIZEBITS-1][0: 2**SETSIZE -1];          // Tag array
    reg [BLOCKSIZE-1:0] cache_data [0:2**CACHESIZEBITS-1][0: 2**SETSIZE -1];  // Data array
    reg valid [0: 2**CACHESIZEBITS-1][0 : 2**SETSIZE - 1];                          // Valid bit array                         // Valid bit array

    wire [CACHESIZEBITS-1:0] set_read = address_read[BLOCK_OFFSET+OFFSETSIZE+CACHESIZEBITS-1:BLOCK_OFFSET+OFFSETSIZE];                                  // Set
    wire [TAGSIZE-1:0] tag_read = address_read[ADDRESSIZE-1:BLOCK_OFFSET+OFFSETSIZE+CACHESIZEBITS];                                   // Tag
    wire [OFFSETSIZE-1:0] offset_write = address_write[OFFSETSIZE-1:0];                                             // Offset
    wire [CACHESIZEBITS-1:0] set_write = address_write[BLOCK_OFFSET+OFFSETSIZE+CACHESIZEBITS-1:BLOCK_OFFSET+OFFSETSIZE];                                  // Set
    wire [TAGSIZE-1:0] tag_write = address_write[ADDRESSIZE-1:BLOCK_OFFSET+OFFSETSIZE+CACHESIZEBITS];                                  // Tag
    
    reg [BLOCKSIZE-1:0] data_buffer, data_read,data_test1,data_test2,data_test3;
    
    reg [31:0] seed;

    wire [SETSIZE-1:0] rnd;

    hash_param #(.OUTPUT_BITS(SETSIZE)) hash_4bit (         // Used as a random number generator
        .seed(seed),
        .hash_out(rnd)
    );           
    
    reg found;      // Flag to indicate if the data was found in the cache 
    integer i, j, k;   // Loop variables
    wire [BLOCK_OFFSET:0] offset;

    assign offset =address_read[BLOCK_OFFSET+OFFSETSIZE-1:OFFSETSIZE];

    always @(posedge clk or posedge reset) begin
    if (reset) begin    // Clear the cache and valid bits
        seed<=0;
        read_done = 0;
        write_done <= 0;
        for (i = 0; i < 2**SETSIZE; i=i+1) begin
            for (j = 0; j < 2**CACHESIZEBITS; j=j+1) begin
                cache_tag[j][i] <= 0;
                cache_data[j][i] <= 0;
                valid[j][i] <= 0;
                cnt = 0;
                found = 0;
            end
        end
    end 
    else begin
        if (wr_en & (~write_done)) begin
            write_done <= 1;
        end
        else begin
            write_done <= 0;
        end
        
        seed<=seed+1;
        data_out <= data_out_reg;
        data_ready <= data_ready_reg;
    end
    end
    always@(*)
    begin
    data_ready_reg=0;
    data_out_reg = 0;
    
    read_done = 0;
        if (wr_en) begin // Write-through: write to both cache and memory (not implemented here)
            found = 0;
            $display("Trying to load address: %h with data: %h ",address_read, data_in);
            for(k = 0; k < 2**SETSIZE; k = k + 1) begin : write_search
                if(valid[set_write][k] && (cache_tag[set_write][k] == tag_write)) begin // Cache hit on write
                    data_buffer = cache_data[set_write][k];
                    data_buffer = data_buffer & (~data_mask); //clearing the data inside
                    data_buffer = data_buffer | (data_in & data_mask);
                    cache_data[set_write][k] = data_buffer;
                    valid[set_write][k] = 1;               // Mark the location as valid
                    found=1;
                    $display("???---ADRESS REWRITTEN address: %h with data: %h ",address_read, data_in);
                    disable write_search;
                end
                else if((~check_exist)&&(!valid[set_write][k])&&(!found)) begin // Cache variable does not exist              
                    cache_data[set_write][k] = data_in;    // Write data to cache
                    valid[set_write][k] = 1;               // Mark the location as valid
                    cache_tag[set_write][k] = tag_write;         // Store the tag_write in the cache
                    found=1;
                    $display("!!!ADRESS WRITTEN to load address: %h with data: %h ",address_read, data_in);

                    disable write_search;
                end
            end
            if((~check_exist) &(!found)  ) begin              
                $display("------ RANDOM REPLACEMENT INITIATED to load address: %h with data: %h ",address_read, data_in);
                cnt=cnt+1;
                //Random replacement policy
                cache_data[set_write][rnd] = data_in;    // Write data to cache
                valid[set_write][rnd] = 1;               // Mark the location as valid
                cache_tag[set_write][rnd] = tag_write;         // Store the tag_read in the cache
            end
            
            end 
        if (r_en) begin  // Read operation
            found = 0;
            for(k=0; k < 2**SETSIZE; k=k+1) begin
                if(valid[set_read][k] && cache_tag[set_read][k] == tag_read) begin : read_search // Cache hit
                    data_test1= (cache_data[set_read][k]);
                    data_test2=0;
                    data_test2[WORDSIZE-1:0]=((cache_data[set_read][k] >> (4*WORDSIZE)));
//                    data_test2=data_test2[WORDSIZE-1:0];
                    data_read = ((cache_data[set_read][k] >> (offset*WORDSIZE)));
                    data_test3 = data_read;
                    data_out_reg = data_read[WORDSIZE-1:0];
                    data_ready_reg = 1;
                    read_done = 1;
                    found=1;
                    disable read_search;
                end
            end
            if(!found)  // Cache miss
            begin
                data_out_reg = 32'd0;
                data_ready_reg = 0;
            end
        end
    end

endmodule

