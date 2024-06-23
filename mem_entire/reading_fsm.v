`timescale 1ns/10ps

module reading_fsm
#(  parameter ADDRESS_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter BLOCK_WIDTH = 32, 
    parameter DATA_WIDTH_log = 5,
    parameter BLOCK_WORD_NUMBER_log = 4,
    parameter DATA_NUMBER_BYTES_log=2 )
(

input clk, rst,
    
input [ADDRESS_WIDTH-1:0] address_d,

output [1:0] current_state_out,

//PROC 
input proc_enable_d,                    //SIG
input proc_enable_wr,
input [ADDRESS_WIDTH-1:0] proc_address_d,

output reg proc_done_d,                      //SIG
output [DATA_WIDTH-1:0] proc_data_d_out,     //DATA


//L1 CACHE
input L1_d_hit, L1_d_done, L1_d_cache_out,              //SIG
input [DATA_WIDTH-1:0] L1_d_data_out, // DATA

output reg L1_d_enable,                     //SIG
output reg [ADDRESS_WIDTH-1:0] L1_d_address,

//L2 CACHE
input L2_hit, L2_done,                  //SIG
input [BLOCK_WIDTH-1:0] L2_data_out,


output reg L2_enable,                       //SIG
output reg [ADDRESS_WIDTH-1:0]  L2_address,

//MEM
input mem_done,                         //SIG
input[BLOCK_WIDTH-1:0]  mem_data_out,   //DATA               

output reg mem_enable,                      //SIG
output reg [ADDRESS_WIDTH-1:0] mem_address, //DATA

//COPYING CONTROL
output reg copy_enable_L2L1, copy_enable_MEML2,
output reg [ADDRESS_WIDTH-1:0] copy_address_L2L1, copy_address_MEML2,

//MUX CONTROL
output reg mux_enable,
output reg [BLOCK_WIDTH-1:0] block_out,
output reg [ADDRESS_WIDTH-1:0] mem_address_out,
//BUFFER SIGS
input buffer_address_exist,
input [BLOCK_WIDTH-1:0] buffer_data_found,

output reg stall,
input extra_stall
//saving reg
);
reg [ADDRESS_WIDTH-1:0] prev_address;
reg [ADDRESS_WIDTH-1:0] L1_d__address;



//all the states
parameter   STATE_idle          =   2'b00,
            STATE_reading_L1    =   2'b01,
            STATE_reading_L2    =   2'b10,
            STATE_reading_mem   =   2'b11;

//state regs
reg [1:0]   current_state;
reg [1:0]   next_state;

assign current_state_out = current_state;
reg stall_reg, stall_reg2;
//Sequential Block of the FSM
always @(posedge clk) begin
    if (rst) begin
        current_state <= STATE_idle;
        prev_address <= {ADDRESS_WIDTH{1'b0}};
        stall_reg <= 1'b0;
        stall_reg2 <= 1'b0;
    end
    else begin
        if (((current_state == STATE_idle) || (current_state == STATE_reading_L1) & proc_enable_d)) begin
            prev_address <= proc_address_d;
        end
        stall_reg2 <= stall;
        stall_reg2 <= stall_reg;
        current_state <= next_state;
    end
end

wire [BLOCK_WIDTH-1:0] shifted_data;
assign shifted_data = block_out >> ({{DATA_WIDTH_log+1{1'b0}}, prev_address[BLOCK_WORD_NUMBER_log+DATA_NUMBER_BYTES_log-1:DATA_NUMBER_BYTES_log ]}<<DATA_WIDTH_log);
assign proc_data_d_out = ((current_state == STATE_reading_L1 & L1_d_hit)) ?L1_d_data_out: shifted_data[DATA_WIDTH-1:0];

//assign proc_data_d_out = (block_out>>{prev_address[ADDRESS_WIDTH-1:2], 2'b00})[DATA_WIDTH-1:0];
//Asynchronous Block
always @(*) begin
    
    next_state = current_state;
    
    //DEFAULT VALUES
    //PROC
    proc_done_d = 1'b0;     
    
    //L1
    L1_d_enable = 1'b0;
    L1_d_address = {ADDRESS_WIDTH{1'b0}};
    //L2
    L2_enable = 1'b0;
    L2_address = {ADDRESS_WIDTH{1'b0}};
    //MEM
    mem_enable = 1'b0;
    mem_address = {ADDRESS_WIDTH{1'b0}};
    //COPIER
    copy_enable_L2L1 = 1'b0;
    copy_enable_MEML2 = 1'b0;
    copy_address_L2L1 = {ADDRESS_WIDTH{1'b0}};
    copy_address_MEML2 = {ADDRESS_WIDTH{1'b0}};
    //MUX
    mux_enable = 1'b0;
    mem_address_out = {ADDRESS_WIDTH{1'b0}};
    block_out = {BLOCK_WIDTH{1'b0}};
    stall = 1'b1;

    case (current_state)
        STATE_idle: begin
            stall = 1'b0;
            if (proc_enable_d & ~proc_enable_wr) begin
                L1_d_enable = 1'b1;
                L1_d_address = proc_address_d;
                next_state = STATE_reading_L1;
                stall =1'b1;
            end
           

        end
        
        
        STATE_reading_L1: begin
            if (~L1_d_done) begin
                L1_d_enable = 1'b1;
                L1_d_address = proc_address_d;
                stall = 1'b1;
            end
            else begin
                if(L1_d_hit) begin
                    stall = 1'b0;
                    proc_done_d = 1'b1;
                     
                    if (~proc_enable_d) begin   
                        next_state = STATE_idle;
                    end
                    else begin
                        L1_d_enable = 1'b1;
                        L1_d_address = proc_address_d; 
                    end
                    
                end
                else if (buffer_address_exist) begin
                    mem_address_out = prev_address;
                    block_out = buffer_data_found;
                    proc_done_d = 1'b1;

                    if (proc_enable_d) begin
                        L1_d_enable = 1'b1;
                        L1_d_address = proc_address_d;
                        next_state = STATE_reading_L1;
                    end
                    else begin
                        next_state = STATE_idle;
                        L1_d_enable = 1'b0;
                    end

                end
                else begin
                    /*L2_enable = 1'b1;
                    L2_address = address_d;
                    prev_address = address_d;
                    next_state = STATE_reading_L2; */

                    mem_address = proc_address_d; // modification to skip L2
                    mem_enable = 1'b1;
                    next_state = STATE_reading_mem;
                end
        
            end
        end
        
        
        STATE_reading_L2: begin
            if (~L2_done) begin
                L2_enable = 1'b1;
                L2_address = address_d;
                L1_d_enable = 1'b0;
            end
            else begin
                if (L2_hit)begin
                    // stall = 1'b0;
                    proc_done_d = 1'b1;
                    mux_enable = 1'b1;
                    mem_address_out = prev_address;
                    block_out = L2_data_out;
                    
                    copy_enable_L2L1 = 1'b1;
                    copy_address_L2L1 = prev_address;

                    if (~proc_enable_d) begin
                        next_state = STATE_idle; 
                        L1_d_enable = 1'b0;
                    end
                    else begin
                        L1_d_enable = 1'b1;
                        L1_d__address = proc_address_d;
                        next_state = STATE_reading_L1;
                    end


                end
                else begin
                    mem_address = proc_address_d;
                    mem_enable = 1'b1;
                    next_state = STATE_reading_mem;
                    L1_d_enable = 1'b0;

                end
            end

        end

        STATE_reading_mem: begin
            if (~mem_done) begin
                L1_d_enable = 1'b0;
                mem_address = proc_address_d;
                mem_enable = 1'b1;
            end
            else begin
                mem_enable = 1'b0;
                proc_done_d = 1'b1;
                mux_enable = 1'b1;
                mem_address_out = prev_address;
                block_out = mem_data_out;
                stall = 1'b0;
                copy_enable_MEML2 = 1'b1;
                copy_address_MEML2 = prev_address;
                
                
                if (proc_enable_d) begin
                    L1_d_enable = 1'b1;
                    L1_d_address = proc_address_d;
                    next_state = STATE_reading_L1;
                end
                else begin
                    next_state = STATE_idle;
                    L1_d_enable = 1'b0;
                end

            end
        end

    
    endcase
  
end

endmodule