`timescale 1ns/10ps

module mem_entire
#(
    
    parameter ADDRESS_WIDTH = 32,  // Size in bits
    parameter BLOCK_WIDTH = 32*16,
    parameter DATA_WIDTH = 32,
    parameter DATASIZE = 4,     // Size in bytes
    parameter SETSIZE = 2,      // Number of bits
    parameter CACHESIZE = 1024, // Size in bytes
    parameter OFFSETSIZE = 2,   // Number of bits corrensponding to the offset
    parameter CACHESIZEBITS = 2,
    parameter TAGSIZE = ADDRESS_WIDTH    - SETSIZE - OFFSETSIZE, // Number of bits corresponding to the tag
    parameter BLOCK_WIDTH_WORD_log = 4

)
(

    input clk, 
    input rst,

    //proc stall sig
    output stall,

    
    //PROC I/O
        //DATA I/O
            //SIGNALS
                //INPUT
                    input proc_data_read_enable,            //proc data read enable
                    input proc_data_write_enable,           //proc data write enable
                //OUTPUT 
                    output proc_data_read_done,
                    output proc_data_write_done,
        
            //DATA
                //INPUT

                    input [ADDRESS_WIDTH-1:0]   proc_data_address,                //proc data address for read or write
        
                    
                    input [3:0] proc_mask_vector,                   //!!!!!! !!!!!! NOT PARAMETRISABLE, ONLY FOR 32bit WORD
                    input [DATA_WIDTH-1:0]      proc_data_write,                  //data to be written

                //OUTPUT
                    output [DATA_WIDTH-1:0]     proc_data_read,

        //INSTR I/O
            //SIGNALS
                //INPUTS
                    input proc_instr_read_enable,           //enable reading
                //OUTPUT
                    output proc_instr_read_done,            //data are ready to be received by proc
            //DATA
                //INPUT 
                    input [ADDRESS_WIDTH-1:0]   proc_instr_address,               //address to read from
                
                //OUTPUT
                    output [DATA_WIDTH-1:0]     proc_instr_read                 //data that had been read
    

);

    wire [OFFSETSIZE -1:0]    proc_data_offset_begin;           //offset begin
    wire [OFFSETSIZE-1:0]     proc_data_offset_end;             //offset end

    wire MEM_data_write_done;                   //HIGH when data are ready to be written
    wire MEM_data_read_done;                    //HIGH when data are ready to be read
    wire MEM_data_write_enable;                
    wire MEM_data_read_enable;
    wire [BLOCK_WIDTH-1:0]       MEM_data_get;                          //the output data from the cache
    wire  [ADDRESS_WIDTH-1:0]     MEM_data_read_address;                 //read address
    wire  [ADDRESS_WIDTH-1:0]     MEM_data_write_address;                //write address
    wire [BLOCK_WIDTH-1:0]        MEM_data_give;                        // giving data to MEM to write
    wire [BLOCK_WIDTH-1:0]        MEM_data_mask;                         //data mask




cache_entire
#(
    .ADDRESS_WIDTH(ADDRESS_WIDTH),
    .BLOCK_WIDTH(BLOCK_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .DATASIZE(DATASIZE),
    .SETSIZE(SETSIZE),
    .CACHESIZE(CACHESIZE),
    .OFFSETSIZE(OFFSETSIZE),
    .BLOCK_WIDTH_WORD_log(BLOCK_WIDTH_WORD_log),
    .CACHESIZEBITS(CACHESIZEBITS)

)
 cache_entire_inst
(
    .clk(clk), 
    .rst(rst),
    .stall(stall),                                     
    .proc_data_read_enable(proc_data_read_enable),
    .proc_data_write_enable(proc_data_write_enable),
    .proc_data_read_done(proc_data_read_done),
    .proc_data_write_done(proc_data_write_done),                            
    .proc_data_address(proc_data_address),
    .proc_data_offset_begin(proc_data_offset_begin),
    .proc_data_offset_end(proc_data_offset_end),
    .proc_data_write(proc_data_write),
    .proc_data_read(proc_data_read),                               
    .proc_instr_read_enable(proc_instr_read_enable),
    .proc_instr_read_done(proc_instr_read_done),
    .proc_instr_address(proc_instr_address),
    .proc_instr_read(proc_instr_read),
    .MEM_data_write_done(MEM_data_write_done),
    .MEM_data_read_done(MEM_data_read_done),
    .MEM_data_write_enable(MEM_data_write_enable),
    .MEM_data_read_enable(MEM_data_read_enable),                   
    .MEM_data_get(MEM_data_get),
    .MEM_data_read_address(MEM_data_read_address),
    .MEM_data_write_address(MEM_data_write_address),
    .MEM_data_give(MEM_data_give),
    .MEM_data_mask(MEM_data_mask)

);

///////////VECTOR TO OFFSETS//////////////

//!!!!!!!!!!!!! NOT PARAMETRISABLE, THIS ONE IS ONLY SUITABLE FOR 32 bit 
convert_vector_to_offset convet2offset_inst 
(
    proc_mask_vector,
    proc_data_offset_begin, 
    proc_data_offset_end
);

///////////VECTOR TO OFFSET END
bram 
#(
    .ADDR_SIZE(ADDRESS_WIDTH/2),
    .OFFSET_BITS(BLOCK_WIDTH_WORD_log+OFFSETSIZE),
    .stall_bram(100)
)
bram_inst
(
    .clk(clk),
    .reset(rst),
    .addr_write(MEM_data_write_address[(ADDRESS_WIDTH/2+BLOCK_WIDTH_WORD_log+OFFSETSIZE)-1:0]),
    .addr_read(MEM_data_read_address[(ADDRESS_WIDTH/2+BLOCK_WIDTH_WORD_log+OFFSETSIZE)-1:0]),
    .bram_enable(1'b1),
    .read_enable(MEM_data_read_enable),
    .write_enable(MEM_data_write_enable),
    .data_input(MEM_data_give),
    .data_mask(MEM_data_mask),
    .data_out(MEM_data_get),
    .data_ready(MEM_data_read_done)
);



endmodule






