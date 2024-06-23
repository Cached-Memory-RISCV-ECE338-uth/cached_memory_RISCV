`timescale 1ns/10ps

module cache_entire
#(
    parameter ADDRESS_WIDTH = 32,  // Size in bits
    parameter BLOCK_WIDTH = 32*16,
    parameter DATA_WIDTH = 32,
    parameter DATASIZE = 4,     // Size in bytes
    parameter SETSIZE = 2,      // Number of bits
    parameter CACHESIZE = 1024, // Size in bytes
    parameter OFFSETSIZE = 2,   // Number of bits corrensponding to the offset
    parameter TAGSIZE = ADDRESS_WIDTH - SETSIZE - OFFSETSIZE, // Number of bits corresponding to the tag
    parameter CACHESIZEBITS = 2,
    parameter BLOCK_WIDTH_WORD_log = 4               //the log of the number of words
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
        
                    input [OFFSETSIZE -1:0]    proc_data_offset_begin,           //offset begin
                    input [OFFSETSIZE-1:0]     proc_data_offset_end,             //offset end

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
                    output [DATA_WIDTH-1:0]     proc_instr_read,                 //data that had been read


    //MEM DATA I/O
        //SIGNALS
            //INPUTS
                input MEM_data_write_done,                   //HIGH when data are ready to be written
                input MEM_data_read_done,                    //HIGH when data are ready to be read
            //OUTPUT
                output MEM_data_write_enable,                
                output MEM_data_read_enable,
        
        //DATA
            //INPUTS 
                input [BLOCK_WIDTH-1:0]              MEM_data_get,                          //the output data from the cache
            //OUTPUTS
                output  [ADDRESS_WIDTH-1:0]     MEM_data_read_address,                 //read address
                output  [ADDRESS_WIDTH-1:0]     MEM_data_write_address,                //write address

                output [BLOCK_WIDTH-1:0]        MEM_data_give,                        // giving data to MEM to write

                output [BLOCK_WIDTH-1:0]        MEM_data_mask                       //data mask
        
    


);

//////////CONNECTING SINGALS/////////////////

        
//// L1 CONNECTIONS            
wire L1_data_write_done;               //HIGH when data are ready to be written
wire L1_data_read_done;                //HIGH when data are ready to be read
wire L1_data_hit;
wire L1_data_write_enable;
wire L1_data_read_enable;
wire [DATA_WIDTH-1:0] L1_data_get;     //the data from the cache
wire  [ADDRESS_WIDTH-1:0] L1_data_read_address;  //read address
wire  [ADDRESS_WIDTH-1:0] L1_data_write_address; //write address
wire [BLOCK_WIDTH-1:0] L1_data_give;    // giving data to L1 to write
wire [BLOCK_WIDTH-1:0] L1_data_mask;    //data mask

//// L2 CONNECTIONS for now left on the air
wire L2_data_write_done;               //HIGH when data are ready to be written
wire L2_data_read_done;                //HIGH when data are ready to be read
wire L2_data_hit;
wire L2_data_write_enable;
wire L2_data_read_enable;
wire [BLOCK_WIDTH-1:0] L2_data_get;    //the data from the cache
wire  [ADDRESS_WIDTH-1:0] L2_data_read_address;  //read address
wire  [ADDRESS_WIDTH-1:0] L2_data_write_address; //write address
wire [BLOCK_WIDTH-1:0] L2_data_give;    // giving data to L2 to write
wire [BLOCK_WIDTH-1:0] L2_data_mask;    //data mask

//// MEM CONNECTIONS
wire L1_instr_write_done;              //HIGH when data are ready to be written
wire L1_instr_read_done;               //HIGH when data are ready to be read
wire L1_instr_hit;
wire L1_instr_write_enable;
wire L1_instr_read_enable;
wire [DATA_WIDTH-1:0] L1_instr_get;    //the data from the cache
wire  [ADDRESS_WIDTH-1:0] L1_instr_read_address; //read address
wire  [ADDRESS_WIDTH-1:0] L1_instr_write_address;//write address
wire [BLOCK_WIDTH-1:0] L1_instr_give;   // giving data to L1 to write
wire [BLOCK_WIDTH-1:0] L1_instr_mask;   //data mask


wire check_exist;
cache_controller 
#(
    .BLOCK_WIDTH(BLOCK_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDRESS_WIDTH(ADDRESS_WIDTH),
    .OFFSET_SIZE(OFFSETSIZE),
    .BLOCK_WIDTH_WORD_log(BLOCK_WIDTH_WORD_log)

)
cache_controller_inst
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
    .L1_data_write_done(L1_data_write_done),                 
    .L1_data_read_done(1'b1),                  
    .L1_data_hit(L1_data_hit),
    .L1_data_write_enable(L1_data_write_enable),                
    .L1_data_read_enable(L1_data_read_enable),
    .L1_data_get(L1_data_get),                       
    .L1_data_read_address(L1_data_read_address),              
    .L1_data_write_address(L1_data_write_address),             
    .L1_data_give(L1_data_give),                      
    .L1_data_mask(L1_data_mask),                      
    .L2_data_write_done(L2_data_write_done),                 
    .L2_data_read_done(L2_data_read_done),                  
    .L2_data_hit(L2_data_hit),
    .L2_data_write_enable(L2_data_write_enable),                
    .L2_data_read_enable(L2_data_read_enable),
    .L2_data_get(L2_data_get),                      
    .L2_data_read_address(L2_data_read_address),               
    .L2_data_write_address(L2_data_write_address),              
    .L2_data_give(L2_data_give),                       
    .L2_data_mask(L2_data_mask),                        
    .MEM_data_write_done(MEM_data_write_done),                 
    .MEM_data_read_done(MEM_data_read_done),                  
    .MEM_data_write_enable(MEM_data_write_enable),                
    .MEM_data_read_enable(MEM_data_read_enable),
    .MEM_data_get(MEM_data_get),                         
    .MEM_data_read_address(MEM_data_read_address),                
    .MEM_data_write_address(MEM_data_write_address),              
    .MEM_data_give(MEM_data_give),                       
    .MEM_data_mask(MEM_data_mask),                       
    .L1_instr_write_done(L1_instr_write_done),                  
    .L1_instr_read_done(1'b1),                  
    .L1_instr_hit(L1_instr_hit),
    .L1_instr_write_enable(L1_instr_write_enable),                
    .L1_instr_read_enable(L1_instr_read_enable),
    .L1_instr_get(L1_instr_get),                       
    .L1_instr_read_address(L1_instr_read_address),              
    .L1_instr_write_address(L1_instr_write_address),             
    .L1_instr_give(L1_instr_give),                       
    .L1_instr_mask(L1_instr_mask),
    .check_exist(check_exist)                      

);

wire [15:0] dummy_wire1, dummy_wire2;

L1_cache
#(
    .ADDRESSIZE(ADDRESS_WIDTH),
    .WORDSIZE(DATA_WIDTH),
    .SETSIZE(SETSIZE),
    .CACHESIZE(CACHESIZE),
    .CACHESIZEBITS(CACHESIZEBITS),
    .OFFSETSIZE(OFFSETSIZE),
    .BLOCK_OFFSET(BLOCK_WIDTH_WORD_log)
    
) 
 L1_data_cache
(
    .clk(clk),
    .reset(rst),
    .wr_en(L1_data_write_enable),
    .r_en(L1_data_read_enable),
    .address_read(L1_data_read_address),
    .address_write(L1_data_write_address),
    .data_in(L1_data_give),
    .data_mask(L1_data_mask),
    .data_ready(L1_data_hit),
    .write_done(L1_data_write_done),
    .data_out(L1_data_get),
    .cnt(dummy_wire1),
    .check_exist(proc_data_write_enable | check_exist)
);

L1_cache 
#(
    .ADDRESSIZE(ADDRESS_WIDTH),
    .WORDSIZE(DATA_WIDTH),
    .SETSIZE(SETSIZE),
    .CACHESIZE(CACHESIZE),
    .CACHESIZEBITS(CACHESIZEBITS),
    .OFFSETSIZE(OFFSETSIZE),
    .BLOCK_OFFSET(BLOCK_WIDTH_WORD_log)
    
) 
L1_instr_cache
(
    .clk(clk),
    .reset(rst),
    .wr_en(L1_instr_write_enable),
    .r_en(L1_instr_read_enable),
    .address_read(L1_instr_read_address),
    .address_write(L1_instr_write_address),
    .data_in(L1_instr_give),
    .data_mask(L1_instr_mask),
    .data_ready(L1_instr_hit),
    .write_done(L1_instr_write_done),
    .data_out(L1_instr_get),
    .cnt(dummy_wire2),
    .check_exist(1'b0)
);




endmodule