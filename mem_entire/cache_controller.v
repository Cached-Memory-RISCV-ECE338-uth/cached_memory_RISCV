`timescale 1ns/10ps



module cache_controller 
# (
    parameter BLOCK_WIDTH = 32*16,
    parameter DATA_WIDTH  = 32,
    parameter ADDRESS_WIDTH = 32,
    
    parameter OFFSET_SIZE = 2,
    parameter BLOCK_WIDTH_WORD_log = 4,              //the log of the number of words
    parameter DATA_WIDTH_log = 5
   

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
        
                    input [OFFSET_SIZE -1:0]    proc_data_offset_begin,           //offset begin
                    input [OFFSET_SIZE-1:0]     proc_data_offset_end,             //offset end

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

    //L1 DATA I/O
        //SIGNALS
            //INPUTS
                input L1_data_write_done,                   //HIGH when data are ready to be written
                input L1_data_read_done,                    //HIGH when data are ready to be read

                input L1_data_hit,
            //OUTPUT
                output L1_data_write_enable,                
                output L1_data_read_enable,
        
        //DATA
            //INPUTS 
                input [DATA_WIDTH-1:0]              L1_data_get,                          //the output data from the cache
            //OUTPUTS
                output [ADDRESS_WIDTH-1:0]     L1_data_read_address,                 //read address
                output [ADDRESS_WIDTH-1:0]     L1_data_write_address,                //write address

                output [BLOCK_WIDTH-1:0]        L1_data_give,                        // giving data to L1 to write

                output [BLOCK_WIDTH-1:0]        L1_data_mask,                         //data mask

    //L2 DATA I/O
        //SIGNALS
            //INPUTS
                input L2_data_write_done,                   //HIGH when data are ready to be written
                input L2_data_read_done,                    //HIGH when data are ready to be read

                input L2_data_hit,
            //OUTPUT
                output L2_data_write_enable,                
                output L2_data_read_enable,
        
        //DATA
            //INPUTS 
                input [BLOCK_WIDTH-1:0]              L2_data_get,                          //the output data from the cache
            //OUTPUTS
                output [ADDRESS_WIDTH-1:0]     L2_data_read_address,                 //read address
                output [ADDRESS_WIDTH-1:0]     L2_data_write_address,                //write address

                output [BLOCK_WIDTH-1:0]        L2_data_give,                        // giving data to L2 to write

                output [BLOCK_WIDTH-1:0]        L2_data_mask,                         //data mask

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
                output [ADDRESS_WIDTH-1:0]     MEM_data_read_address,                 //read address
                output [ADDRESS_WIDTH-1:0]     MEM_data_write_address,                //write address

                output [BLOCK_WIDTH-1:0]        MEM_data_give,                        // giving data to MEM to write

                output [BLOCK_WIDTH-1:0]        MEM_data_mask,                         //data mask
        
    //L1 INSTR I/O
        //SIGNALS
            //INPUTS
                input L1_instr_write_done,                   //HIGH when data are ready to be written
                input L1_instr_read_done,                    //HIGH when data are ready to be read

                input L1_instr_hit,
            //OUTPUT
                output L1_instr_write_enable,                
                output L1_instr_read_enable,
        
        //DATA
            //INPUTS 
                input [DATA_WIDTH-1:0]              L1_instr_get,                          //the output data from the cache
            //OUTPUTS
                output  [ADDRESS_WIDTH-1:0]     L1_instr_read_address,                 //read address
                output  [ADDRESS_WIDTH-1:0]     L1_instr_write_address,                //write address

                output [BLOCK_WIDTH-1:0]        L1_instr_give,                        // giving data to L1 to write

                output [BLOCK_WIDTH-1:0]        L1_instr_mask,                        //data mask

            output check_exist

);

///WIRES COVERING NON EXISTENT BUFFER
wire dummy_enable1, dummy_enable2, dummy_enable3, dummy_enable4, dummy_enable5;
wire [ADDRESS_WIDTH-1:0]dummy_address1, dummy_address2, dummy_address3, dummy_address4;
wire [BLOCK_WIDTH-1:0] dummy_data1, dummy_data2;
wire [DATA_WIDTH-1:0] dummy_data3;
wire [DATA_WIDTH-1:0] dummy_mask;


////////////////  READING FSM     ///////////////////

/////////////// DATA READING FSM BEGIN ////////////////



wire [1:0]data_reading_fsm_current_state;

////CONNECTIONS TO PRIORIY HANDLER
wire [ADDRESS_WIDTH-1:0] reading_fsm_mem_data_address;

////CONNECTIONS TO L1 DATA WRITE BUFFER
wire reading_fsm_L1_data_buffer_enable;
wire [ADDRESS_WIDTH-1:0] reading_fsm_L1_data_buffer_address;


/////CONNECTIONS TO SIGNAL OBSCURING
wire reading_fsm_MEM_data_read_enable_command;
wire [BLOCK_WIDTH-1:0] reading_fsm_MEM_data_virtual_data_get;
wire reading_fsm_MEM_data_read_done_virtual;

////CONNECTIONS TO ADDRESS BUFFER SEARCH SWITCH
wire [BLOCK_WIDTH-1:0] data_buffer_data_found;
wire data_buffer_address_found;

wire stall_read_data;
reading_fsm 
#(
    .ADDRESS_WIDTH(ADDRESS_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .BLOCK_WIDTH(BLOCK_WIDTH),
    .DATA_WIDTH_log(DATA_WIDTH_log),
    .BLOCK_WORD_NUMBER_log(BLOCK_WIDTH_WORD_log)
    
)
data_reading_fsm
(
    .clk(clk), 
    .rst(rst),
    .address_d(proc_data_address),          //this is the same signals as the proc_address_d below
    .current_state_out(data_reading_fsm_current_state),
    .proc_enable_d(proc_data_read_enable),       
    .proc_enable_wr(proc_data_write_enable),
    .proc_address_d(proc_data_address),
    .proc_done_d(proc_data_read_done),         
    .proc_data_d_out(proc_data_read),     
    .L1_d_hit(L1_data_hit), 
    .L1_d_done(L1_data_read_done), 
    .L1_d_cache_out(1'b0),                  //useless sig              
    .L1_d_data_out(L1_data_get), 
    .L1_d_enable(L1_data_read_enable),                 
    .L1_d_address(L1_data_read_address),
    .L2_hit(L2_data_hit), 
    .L2_done(L2_data_read_done),
    .L2_data_out(L2_data_get),
    .L2_enable(L2_data_read_enable),           
    .L2_address(L2_data_read_address),
    .mem_done(reading_fsm_MEM_data_read_done_virtual),                 
    .mem_data_out(reading_fsm_MEM_data_virtual_data_get), 
    .mem_enable(reading_fsm_MEM_data_read_enable_command),   
    .mem_address(reading_fsm_mem_data_address), 
    .copy_enable_L2L1(dummy_enable1), 
    .copy_enable_MEML2(reading_fsm_L1_data_buffer_enable),                           //actually MEM->L1
    .copy_address_L2L1(dummy_address1),                           
    .copy_address_MEML2(reading_fsm_L1_data_buffer_address),                          
    .mux_enable(dummy_enable1),                   //for now, no mux
    .block_out(dummy_data1),                     //for now, no mux
    .mem_address_out(dummy_address1),              //for now, no mux
    .stall(stall_read_data),
    .buffer_address_exist(data_buffer_address_found),
    .buffer_data_found(data_buffer_data_found),
    .extra_stall(1'b1)
);

/////////////////////// DATA READING FSM ENDING

////////////////// INSTRUCTION READING FSM BEGIN //////////////////

wire [1:0] instruction_reading_fsm_current_state;

////CONNECTIONS FOR PRIORITY HANDLER
wire [ADDRESS_WIDTH-1:0] reading_fsm_mem_instr_address; 

/////CONNECTIONS TO SIGNAL OBSCURING
wire reading_fsm_MEM_instr_read_enable_command;
wire [BLOCK_WIDTH-1:0] reading_fsm_MEM_instr_virtual_data_get;
wire reading_fsm_MEM_instr_read_done_virtual;

////CONNECTIONS TO L1 INSTR WRITE BUFFER
wire reading_fsm_L1_instr_buffer_enable;
wire [ADDRESS_WIDTH-1:0] reading_fsm_L1_instr_buffer_address;

////CONNECTIONS TO INSTR BUFFER ADDRESS CHECK
wire instr_buffer_address_found;
wire [BLOCK_WIDTH-1:0] instr_buffer_data_found;

wire stall_read_instr;
reading_fsm 
#(
    .ADDRESS_WIDTH(ADDRESS_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .BLOCK_WIDTH(BLOCK_WIDTH),
    .DATA_WIDTH_log(DATA_WIDTH_log),
    .BLOCK_WORD_NUMBER_log(BLOCK_WIDTH_WORD_log)
)
instr_reading_fsm
(
    .clk(clk), 
    .rst(rst),
    .address_d(proc_instr_address),
    .current_state_out(instruction_reading_fsm_current_state),
    .proc_enable_d(proc_instr_read_enable),       
    .proc_enable_wr(1'b0),                                          //instructions cant write
    .proc_address_d(proc_instr_address  ),
    .proc_done_d(proc_instr_read_done),         
    .proc_data_d_out(proc_instr_read),     
    .L1_d_hit(L1_instr_hit), 
    .L1_d_done(L1_instr_read_done), 
    .L1_d_cache_out(1'b0),              
    .L1_d_data_out(L1_instr_get), 
    .L1_d_enable(L1_instr_read_enable),                 
    .L1_d_address(L1_instr_read_address),
    .L2_hit(L2_data_hit),                                                                   //ignore for now
    .L2_done(L2_data_read_done),                                                            //ignore for now
    .L2_data_out(L2_data_get),                                                              //ignore for now  
    .L2_enable(L2_data_read_enable),                                                        //ignore for now
    .L2_address(L2_data_read_address),                                                      //ignore for now
    .mem_done(reading_fsm_MEM_instr_read_done_virtual),                 
    .mem_data_out(reading_fsm_MEM_instr_virtual_data_get), 
    .mem_enable(reading_fsm_MEM_instr_read_enable_command),   
    .mem_address(reading_fsm_mem_instr_address), 
    .copy_enable_L2L1(dummy_enable4), 
    .copy_enable_MEML2(reading_fsm_L1_instr_buffer_enable),
    .copy_address_L2L1(dummy_address3), 
    .copy_address_MEML2(reading_fsm_L1_instr_buffer_address),
    .mux_enable(dummy_enable3),                  //for now, no mux
    .block_out(dummy_data2),    //for now, not used
    .mem_address_out(dummy_address2),
    .stall(stall_read_instr),
    .buffer_address_exist(instr_buffer_address_found),
    .buffer_data_found(instr_buffer_data_found),
    .extra_stall(1'b0)
);


///////////////INSTRUCTION READING FSM END

///////////////WRITING FSM BEGIN/////////////////////


wire [2:0] writing_fsm_current_state;

////L1 DATA BUFFER WRITE INTERMIDIATE
wire writing_fsm_L1_data_buffer_enable;
wire [ADDRESS_WIDTH-1:0] writing_fsm_L1_data_buffer_address;

wire [DATA_WIDTH-1:0] writing_fsm_L1_data_buffer_data_word_width;           //used to connect the fsm to word2block -> L1 buff
wire [DATA_WIDTH-1:0] writing_fsm_L1_data_buffer_mask_word_width;           //used to connect the fsm to word2block -> L1 buff
wire [BLOCK_WIDTH-1:0] writing_fsm_L1_data_buffer_data;                     //used to connect word2block to switch
wire [BLOCK_WIDTH-1:0] writing_fsm_L1_data_buffer_mask;                     //used to connect word2block to switch


////MEM DATA BUFFER CONNECTIONS
wire writing_fsm_MEM_data_buffer_enable;
wire [ADDRESS_WIDTH-1:0] writing_fsm_MEM_data_buffer_address;

wire [DATA_WIDTH-1:0]       writing_fsm_MEM_data_buffer_data_word_width;        //used to connect the fsm to word2block -> L1 buff
wire [DATA_WIDTH-1:0]       writing_fsm_MEM_data_buffer_mask_word_width;         //used to connect the fsm to word2block -> L1 buff
wire [BLOCK_WIDTH-1:0]      writing_fsm_MEM_data_buffer_data;                   //used to connect word2block to switch    
wire [BLOCK_WIDTH-1:0]      writing_fsm_MEM_data_buffer_mask;                    //used to connect word2block to switch


////L1 WORD TO BLOCK CONNECTIONS
wire [DATA_WIDTH-1:0] writing_fsm_data_buffer_word; 
wire [DATA_WIDTH-1:0] writing_fsm_data_buffer_word_mask;

wire L1_data_buffer_full;
wire MEM_data_buffer_full;

wire stall_write;
writing_fsm 
#(
    .ADDRESS_WIDTH(ADDRESS_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .BLOCK_WIDTH(BLOCK_WIDTH),
    .OFFSET_WIDTH(OFFSET_SIZE)

)
data_writing_fsm
(
    .clk(clk), 
    .rst(rst),
    .current_state_out(writing_fsm_current_state),
    .proc_enable_wr(proc_data_write_enable),
    .proc_address_wr(proc_data_address),            
    .proc_data_wr(proc_data_write),
    .proc_offset_bits_begin(proc_data_offset_begin),     
    .proc_offset_bits_end(proc_data_offset_end),       
    .L1_wr_enable(writing_fsm_L1_data_buffer_enable), 
    .L1_wr_address(writing_fsm_L1_data_buffer_address),
    .L1_wr_data(writing_fsm_L1_data_buffer_data_word_width),
    .L1_mask(writing_fsm_L1_data_buffer_mask_word_width), 
    .buff_L1_full(L1_data_buffer_full),
    .L2_wr_enable(dummy_enable5),
    .L2_wr_address(dummy_address4),
    .L2_wr_data(dummy_data3),
    .L2_mask(dummy_mask),
    .buff_L2_full(1'b0),
    .MEM_wr_enable(writing_fsm_MEM_data_buffer_enable), 
    .MEM_wr_address(writing_fsm_MEM_data_buffer_address),
    .MEM_wr_data(writing_fsm_MEM_data_buffer_data_word_width),
    .MEM_mask(writing_fsm_MEM_data_buffer_mask_word_width),
    .buff_mem_full(MEM_data_buffer_full),
    .write_busy(stall_write)

);

/////////////WRITING FSM END
/////////////////// STALL BEGIN //////////////////////

assign stall = stall_read_data | stall_read_instr | stall_write;

////////////////// STALL END

/////// PRIORITY HANDLER BEGIN ///////////

//// CONNECTIONS TO SIGNAL OBSCURING
wire sig_obscuring_MEM_data_read_enable, sig_obscuring_MEM_data_read_done;

wire sig_obscuring_MEM_instr_read_enable, sig_obscuring_MEM_instr_read_done;

wire [BLOCK_WIDTH-1:0] priority_handler_block_out;

priority_handler 
#(
    .DATA_WIDTH(DATA_WIDTH),
    .BLOCK_WIDTH(BLOCK_WIDTH),
    .ADDRESS_WIDTH(ADDRESS_WIDTH)

)
mem_priority_handler
(
    .clk(clk), 
    .rst(rst), 
    .read_en_instr(sig_obscuring_MEM_instr_read_enable), 
    .read_en_data(sig_obscuring_MEM_data_read_enable), 
    .read_done(MEM_data_read_done),
    .read_done_virtual_data(sig_obscuring_MEM_data_read_done), 
    .read_done_virtual_instr(sig_obscuring_MEM_instr_read_done),
    .addr_data(reading_fsm_mem_data_address), 
    .addr_instr(reading_fsm_mem_instr_address), 
    .memory_block(MEM_data_get), 
    .memory_address(MEM_data_read_address), 
    .mem_enable(MEM_data_read_enable),
    .block_return(priority_handler_block_out)
);


////// PRIORITY HANDLER END

////////BUFFER FULL SIGNALS BEGIN/////////////

wire data_buffers_full_sig;             


assign data_buffers_full_sig = L1_data_buffer_full | MEM_data_buffer_full; 

wire instr_buffer_full_sig;

////////BUFFER FULL SIGNALS END

//////// L1 DATA BUFFER INTERMIDIATE BEGIN ///////////

//no chance writing and reading fsm try to write simultaniously, 
//exists just to connect the correct cables when one or the other wants to write

//buffer connections
wire L1_data_buffer_enable_sig;
wire [ADDRESS_WIDTH-1:0]    L1_data_buffer_address;
wire [BLOCK_WIDTH-1:0]      L1_data_buffer_data_in;
wire [BLOCK_WIDTH-1:0]      L1_data_buffer_mask_in;

//logic
wire check_exist_internal;
assign {L1_data_buffer_enable_sig, L1_data_buffer_address, L1_data_buffer_data_in, L1_data_buffer_mask_in, check_exist_internal} =
        (reading_fsm_L1_data_buffer_enable)     ?       {1'b1, reading_fsm_L1_data_buffer_address, reading_fsm_MEM_data_virtual_data_get, {BLOCK_WIDTH{1'b1}}, 1'b0}                         :
        (writing_fsm_L1_data_buffer_enable)     ?       {word_to_block_L1_buff_enable, writing_fsm_L1_data_buffer_address, writing_fsm_L1_data_buffer_data, writing_fsm_L1_data_buffer_mask, 1'b1}           :
                                                        {1'b0, {ADDRESS_WIDTH{1'b0}}, {BLOCK_WIDTH{1'b0}}, {BLOCK_WIDTH{1'b0}}, 1'b0};


//////// L1 DATA BUFFER INTERMIDIATE END

/////// L1 DATA WRITE BUFFER BEGIN /////////////

////CONNECTIONS TO DATA BUFFER SEARCH SWITCH
wire L1_data_buffer_address_exist;
wire [BLOCK_WIDTH-1:0] L1_data_buffer_data_found;

write_buffer 
#(
    .ADDRESSIZE(ADDRESS_WIDTH),
    .DATASIZE( BLOCK_WIDTH/8),
    .BUFFERSIZE(2),              //do the power of two to find the size real
    .BUFFER_NUMBER(4)

)
L1_data_write_buffer
(
    .clk(clk),
    .reset(rst),
    .enable(L1_data_buffer_enable_sig),
    .done(L1_data_write_done),
    .address_in(L1_data_buffer_address),
    .data_in(L1_data_buffer_data_in),
    .data_mask(L1_data_buffer_mask_in),
    .buffer_full(L1_data_buffer_full),
    .data_out(L1_data_give),
    .address_out(L1_data_write_address),
    .mask_out(L1_data_mask),
    .write_enable(L1_data_write_enable),
    .address_to_check(L1_data_read_address),
    .address_exist(L1_data_buffer_address_exist),
    .data_found(L1_data_buffer_data_found),
    .check_exist_in(check_exist_internal),
    .check_exist_out(check_exist)

);



////// L1 BUFFER END 

//////// L1 INSTR WRITE BUFFER BEGIN /////////////

wire fake_check_exist1;

write_buffer 
#(
    .ADDRESSIZE(ADDRESS_WIDTH),
    .DATASIZE( BLOCK_WIDTH/8),
    .BUFFERSIZE(2)
)
L1_instr_write_buffer
(
    .clk(clk),
    .reset(rst),
    .enable(reading_fsm_L1_instr_buffer_enable),
    .done(L1_instr_write_done),
    .address_in(reading_fsm_L1_instr_buffer_address),
    .data_in(reading_fsm_MEM_instr_virtual_data_get),
    .data_mask({BLOCK_WIDTH{1'b1}}),             //we always want all the bits
    .buffer_full(instr_buffer_full_sig),
    .data_out(L1_instr_give),
    .address_out(L1_instr_write_address),
    .mask_out(L1_instr_mask),
    .write_enable(L1_instr_write_enable),
    .address_to_check(L1_instr_read_address),
    .address_exist(instr_buffer_address_found),
    .data_found(instr_buffer_data_found),
    .check_exist_in(1'b0),
    .check_exist_out(fake_check_exist1)

);

///////////// L1 INSTR BUFFER END




///////// MEM WRITE BUFFER BEGIN ////////////////

/////BUFFER SEARCH CONNECTION
wire fake_check_exist2;


wire MEM_data_buffer_address_exist;
wire [BLOCK_WIDTH-1:0] MEM_data_buffer_data_found;

write_buffer 
#(
    .ADDRESSIZE(ADDRESS_WIDTH),
    .DATASIZE( BLOCK_WIDTH/8),
    .BUFFERSIZE(2)

)
MEM_write_buffer
(
    .clk(clk),
    .reset(rst),
    .enable(word_to_block_MEM_buff_enable),
    .done(MEM_data_write_done),
    .address_in(writing_fsm_MEM_data_buffer_address),
    .data_in(reading_fsm_MEM_instr_virtual_data_get),
    .data_mask(writing_fsm_MEM_data_buffer_mask),
    .buffer_full(MEM_data_buffer_full),
    .data_out(MEM_data_give),
    .address_out(MEM_data_write_address),
    .mask_out(MEM_data_mask),
    .write_enable(MEM_data_write_enable),
    .address_to_check(L1_data_read_address),
    .address_exist(MEM_data_buffer_address_exist),
    .data_found(MEM_data_buffer_data_found),
    .check_exist_in(1'b0),
    .check_exist_out(fake_check_exist2)

);
//////// MEM WRITE BUFFER END 

//////// L1 DATA SIG OBSCURING BEGIN /////////////////////

done_sig_obscuring 
#(
    .BLOCK_WIDTH(BLOCK_WIDTH)
)
L1_data_done_sig_obscuring
(
    .clk(clk), 
    .rst(rst),
    .fsm_enabling(reading_fsm_MEM_data_read_enable_command),
    .real_enable(sig_obscuring_MEM_data_read_enable),
    .real_done(sig_obscuring_MEM_data_read_done), 
    .real_data(priority_handler_block_out),
    .fsm_virtual_done(reading_fsm_MEM_data_read_done_virtual), 
    .fsm_virtual_data(reading_fsm_MEM_data_virtual_data_get),
    .buff_full(data_buffers_full_sig)

);


///////// L1 DATA SIG OBSCURING END


///////// L1 INSTR SIG OBSCURING BEGIN /////////////////////////
done_sig_obscuring 
#(
    .BLOCK_WIDTH(BLOCK_WIDTH)
)
L1_instr_done_sig_obscuring
(
    .clk(clk), 
    .rst(rst),
    .fsm_enabling(reading_fsm_MEM_instr_read_enable_command),
    .real_enable(sig_obscuring_MEM_instr_read_enable),
    .real_done(sig_obscuring_MEM_instr_read_done), 
    .real_data(priority_handler_block_out),
    .fsm_virtual_done(reading_fsm_MEM_instr_read_done_virtual), 
    .fsm_virtual_data(reading_fsm_MEM_instr_virtual_data_get),
    .buff_full(instr_buffer_full_sig)

);

/////// L1 INSTR SIG OBSCURING END

////////////////WORD TO BLOCK WRITING FSM -> L1 BUFF BEGIN///////////////

word_to_block 
#(
    .ADDRESSIZE(ADDRESS_WIDTH),
    .WORDSIZE(DATA_WIDTH),  
    .OFFSETBITS(OFFSET_SIZE),
    .BLOCKSIZE(BLOCK_WIDTH/DATA_WIDTH),
    .BLOCKSIZE_log(BLOCK_WIDTH_WORD_log)  
) 
word2block_writing_FSMtoL1BUFF
(
    .data_in(writing_fsm_L1_data_buffer_data_word_width),
    .mask_in(writing_fsm_L1_data_buffer_mask_word_width),
    .address(writing_fsm_L1_data_buffer_address),
    .mask_out(writing_fsm_L1_data_buffer_mask),
    .data_out(writing_fsm_L1_data_buffer_data),
    .enable_in(writing_fsm_L1_data_buffer_enable),
    .enable_out(word_to_block_L1_buff_enable)
);

////////////WORD TO BLOCK END

////////////////WORD TO BLOCK WRITING FSM -> MEM BUFF BEGIN/////////////////

word_to_block 
#(
    .ADDRESSIZE(ADDRESS_WIDTH),
    .WORDSIZE(DATA_WIDTH),  
    .OFFSETBITS(OFFSET_SIZE),
    .BLOCKSIZE(BLOCK_WIDTH/DATA_WIDTH),
    .BLOCKSIZE_log(BLOCK_WIDTH_WORD_log) 
) 
word2block_writing_FSMtoMEMBUFF
(
    .data_in(writing_fsm_MEM_data_buffer_data_word_width),
    .mask_in(writing_fsm_MEM_data_buffer_mask_word_width),
    .address(writing_fsm_MEM_data_buffer_address),
    .mask_out(writing_fsm_MEM_data_buffer_mask),
    .data_out(writing_fsm_MEM_data_buffer_data),
    .enable_in(writing_fsm_MEM_data_buffer_enable),
    .enable_out(word_to_block_MEM_buff_enable)
);

/////////////WORD TO BLOCK END

///////////////////DATA BUFFER ADDRESS SEARCH SWITCH BEGIN/////////////////

assign data_buffer_address_found = L1_data_buffer_address_exist | MEM_data_buffer_address_exist;
assign data_buffer_data_found = L1_data_buffer_address_exist ? L1_data_buffer_data_found : MEM_data_buffer_data_found;

/////////////////DATA BUFFER ADDRESS CHECK SWITCH END

endmodule