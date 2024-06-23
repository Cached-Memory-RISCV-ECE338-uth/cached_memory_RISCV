`timescale 1ns/10ps

module writing_fsm
#(  parameter ADDRESS_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter BLOCK_WIDTH = 32,
    parameter OFFSET_WIDTH = 2)
(
    clk, 
    rst,

    //fsm control sigs
    current_state_out,                      //fsm current state out

    //PROC SIGNALS
        //INPUTS
            //SIGNALS
                proc_enable_wr,             //write enable by processor
            //DATA
                proc_address_wr,            //writing address
                proc_data_wr,
                proc_offset_bits_begin,      //first bit offset
                proc_offset_bits_end,        //last bit offset
                                            
        //note that we do not inform processor that write was finished 

    //L1 CACHE SIGNALS
        
        //OUTPUTS
            //SIGNALS
                L1_wr_enable,               //write enable
            //DATA
                L1_wr_address,              //write address for L1
                L1_wr_data,                 //data to write to L1
                L1_mask,                    //L1 mask
    //L1 BUFF SIGNALS
        //INPUTS
            buff_L1_full,                   //high if L1 buffer is full


    //L2 CACHE SIGNALS
        //OUTPUTS
            //SIGNALS
                L2_wr_enable,                //write was done
            //SIGNALS
                L2_wr_address,             //write address for L2
                L2_wr_data,                //data to write to L2
                L2_mask,                   //L2 mask

    //L2 BUFF SIGNALS
        //INPUTS
            buff_L2_full,                 //high if L2 buffer is full

    //MEM_SIGNALS 
        //OUTPUTS
            //SIGNALS
                MEM_wr_enable,                //begin writing
            //SIGNALS
                MEM_wr_address,             //write address for L2
                MEM_wr_data,                //data to write to L2
                MEM_mask,                   //MEM mask

    //MEM BUFF SIGNALS
        //INPUTS
            buff_mem_full,                //high if mem buffer is full
    
    write_busy                              //high if fsm still on writing stage

);


input clk, rst;

output [2:0]current_state_out;
output write_busy;

//PROC
input proc_enable_wr;
input [ADDRESS_WIDTH-1:0] proc_address_wr;
input [DATA_WIDTH-1:0] proc_data_wr;

input [OFFSET_WIDTH -1:0] proc_offset_bits_begin;
input [OFFSET_WIDTH -1:0] proc_offset_bits_end;

//L1 CACHE
output reg L1_wr_enable;
output reg [ADDRESS_WIDTH-1:0] L1_wr_address;
output reg [DATA_WIDTH-1:0] L1_wr_data;
output reg [DATA_WIDTH-1:0] L1_mask;

//L1 BUFF SIGNALS
input buff_L1_full;

//L2 CACHE
output reg L2_wr_enable;
output reg [ADDRESS_WIDTH-1:0] L2_wr_address;
output reg [DATA_WIDTH-1:0] L2_wr_data;
output reg [DATA_WIDTH-1:0] L2_mask;

//L2 BUFF SIGNALS
input buff_L2_full;

//MEM
output reg MEM_wr_enable;
output reg [ADDRESS_WIDTH-1:0] MEM_wr_address;
output reg [DATA_WIDTH-1:0] MEM_wr_data;
output reg [DATA_WIDTH-1:0] MEM_mask;

//MEM BUFF SIGNALS
input buff_mem_full;

parameter   STATE_idle              =   3'b000,
            STATE_fullB1            =   3'b001,
            STATE_fullB2            =   3'b010,
            STATE_fullMEM           =   3'b011,
            STATE_fullB1B2          =   3'b100,
            STATE_fullB1MEM         =   3'b101,
            STATE_fullB2MEM         =   3'b110,
            STATE_fullALL           =   3'b111;
                       

//state regs
reg [2:0]   current_state;
reg [2:0]   next_state;

assign current_state_out = current_state;


reg [ADDRESS_WIDTH-1:0] save_address;
reg [DATA_WIDTH-1:0] save_data;
reg [DATA_WIDTH-1:0] save_mask;
reg [DATA_WIDTH/8:0] save_offset_begin;
reg [DATA_WIDTH/8:0] save_offset_end;

//sequential fsm block
always @(posedge clk, posedge rst) begin
    if (rst) begin
        current_state <= STATE_idle;
        
        //temp save regs
        save_address <= {ADDRESS_WIDTH{1'b0}};
        save_data <= {DATA_WIDTH{1'b0}};
        save_offset_begin <= {(DATA_WIDTH/8){1'b0}};
        save_offset_end <= {(DATA_WIDTH/8){1'b0}};
        save_mask <= {DATA_WIDTH{1'b0}};
    end
    else begin
        current_state <= next_state;
        
        //update temp save regs
        if (current_state == STATE_idle)begin
            if (buff_L1_full | buff_L2_full | buff_mem_full)begin
                save_address <= proc_address_wr;
                save_data <= proc_data_wr;
                
                if ( (|proc_offset_bits_begin) | (|proc_offset_bits_end)) begin
                        save_mask <= {DATA_WIDTH{1'b1}}  
                                                &({DATA_WIDTH{1'b1}} << {proc_offset_bits_begin, 2'b00}) 
                                                &({DATA_WIDTH{1'b1}} >> {{OFFSET_WIDTH{1'b1}} - proc_offset_bits_end, 2'b00});
                end
                else begin
                        save_mask <= {DATA_WIDTH{1'b0}};
                end 
            end
        end
    end
end




assign write_busy = current_state != 3'b000 ? 1'b1 : 1'b0;

//asynchronous fsm block
always @(*) begin

    next_state = current_state;

    //DEFAULT VALUES
    //L1 CACHE
    L1_wr_enable = 1'b0;
    L1_wr_address = {ADDRESS_WIDTH{1'b0}};
    L1_wr_data = {DATA_WIDTH{1'b0}};
    L1_mask = {DATA_WIDTH{1'b0}};
    //L2 CACHE
    L2_wr_enable = 1'b0;
    L2_wr_address = {ADDRESS_WIDTH{1'b0}};
    L2_wr_data = {DATA_WIDTH{1'b0}};
    L2_mask = {DATA_WIDTH{1'b0}};
    //MEM
    MEM_wr_enable = 1'b0;
    MEM_wr_address = {ADDRESS_WIDTH{1'b0}};
    MEM_wr_data = {DATA_WIDTH{1'b0}};
    MEM_mask = {DATA_WIDTH{1'b0}};
    

    case (current_state)
        STATE_idle: begin
            if (proc_enable_wr) begin
                if ((~buff_L1_full) & (~buff_L2_full) & (~buff_mem_full)) begin
                    //L1 CACHE
                    L1_wr_enable = 1'b1;
                    L1_wr_address = proc_address_wr;
                    L1_wr_data = proc_data_wr;
                    
                    //L2 CACHE
                    L2_wr_enable = 1'b1;
                    L2_wr_address = proc_address_wr;
                    L2_wr_data = proc_data_wr;
                    
                    //MEM
                    MEM_wr_enable = 1'b1;
                    MEM_wr_address = proc_address_wr;
                    MEM_wr_data = proc_data_wr;
                    
                    //masks
                    if ( (|proc_offset_bits_begin) | (|proc_offset_bits_end)) begin
                        L1_mask = {DATA_WIDTH{1'b1}}  
                                                &({DATA_WIDTH{1'b1}} << {proc_offset_bits_begin, 2'b00}) 
                                                &({DATA_WIDTH{1'b1}} >> {{OFFSET_WIDTH{1'b1}} - proc_offset_bits_end, 2'b00});
                        L2_mask =L1_mask;
                        MEM_mask = L1_mask;
                    end
                end
                else if (buff_L1_full & buff_L2_full & buff_mem_full) begin

                    next_state = STATE_fullALL;
                end
                else if (buff_L1_full & buff_L2_full) begin
                    //MEM
                    MEM_wr_enable = 1'b1;
                    MEM_wr_address = proc_address_wr;
                    MEM_wr_data = proc_data_wr;
                    //masks
                    if ( (|proc_offset_bits_begin) | (|proc_offset_bits_end)) begin
                        MEM_mask = {DATA_WIDTH{1'b1}}  
                                                &({DATA_WIDTH{1'b1}} << {proc_offset_bits_begin, 2'b00}) 
                                                &({DATA_WIDTH{1'b1}} >> {{OFFSET_WIDTH{1'b1}} - proc_offset_bits_end, 2'b00});
                    end  

                    next_state = STATE_fullB1B2;                  
                end
                else if (buff_L1_full & buff_mem_full)begin
                    //L2 CACHE
                    L2_wr_enable = 1'b1;
                    L2_wr_address = proc_address_wr;
                    L2_wr_data = proc_data_wr;

                    if ( (|proc_offset_bits_begin) | (|proc_offset_bits_end)) begin
                        L2_mask = {DATA_WIDTH{1'b1}}  
                                                &({DATA_WIDTH{1'b1}} << {proc_offset_bits_begin, 2'b00}) 
                                                &({DATA_WIDTH{1'b1}} >> {OFFSET_WIDTH - proc_offset_bits_end, 2'b00});
                    end
                    next_state = STATE_fullB1MEM;

                end
                else if (buff_L2_full & buff_mem_full) begin
                    //L1 CACHE
                    L1_wr_enable = 1'b1;
                    L1_wr_address = proc_address_wr;
                    L1_wr_data = proc_data_wr;

                     if ( (|proc_offset_bits_begin) | (|proc_offset_bits_end)) begin
                        L1_mask = {DATA_WIDTH{1'b1}}  
                                                &({DATA_WIDTH{1'b1}} << {proc_offset_bits_begin, 2'b00}) 
                                                &({DATA_WIDTH{1'b1}} >> {{OFFSET_WIDTH{1'b1}} - proc_offset_bits_end, 2'b00});
                    end 

                    next_state = STATE_fullB2MEM;

                end
                else if (buff_L1_full) begin
                    //L2 CACHE
                    L2_wr_enable = 1'b1;
                    L2_wr_address = proc_address_wr;
                    L2_wr_data = proc_data_wr;
                    //MEM
                    MEM_wr_enable = 1'b1;
                    MEM_wr_address = proc_address_wr;
                    MEM_wr_data = proc_data_wr;
                    //masks
                    if ( (|proc_offset_bits_begin) | (|proc_offset_bits_end)) begin
                        L2_mask =  {DATA_WIDTH{1'b1}}  
                                                &({DATA_WIDTH{1'b1}} << {proc_offset_bits_begin, 2'b00}) 
                                                &({DATA_WIDTH{1'b1}} >> {{OFFSET_WIDTH{1'b1}} - proc_offset_bits_end, 2'b00});
                        MEM_mask = L2_mask;
                    end  

                    next_state = STATE_fullB1;
                end
                else if (buff_L2_full) begin
                    //L1 CACHE
                    L1_wr_enable = 1'b1;
                    L1_wr_address = proc_address_wr;
                    L1_wr_data = proc_data_wr;
                    //MEM
                    MEM_wr_enable = 1'b1;
                    MEM_wr_address = proc_address_wr;
                    MEM_wr_data = proc_data_wr;
                    //masks
                    if ( (|proc_offset_bits_begin) | (|proc_offset_bits_end)) begin
                        L1_mask =  {DATA_WIDTH{1'b1}}  
                                                &({DATA_WIDTH{1'b1}} << {proc_offset_bits_begin, 2'b00}) 
                                                &({DATA_WIDTH{1'b1}} >> {{OFFSET_WIDTH{1'b1}} - proc_offset_bits_end, 2'b00});
                        MEM_mask = L1_mask;
                    end 

                    next_state = STATE_fullB2;
                end
                else if (buff_mem_full) begin
                    //L1 CACHE
                    L1_wr_enable = 1'b1;
                    L1_wr_address = proc_address_wr;
                    L1_wr_data = proc_data_wr;
                    
                    //L2 CACHE
                    L2_wr_enable = 1'b1;
                    L2_wr_address = proc_address_wr;
                    L2_wr_data = proc_data_wr;

                    if ( (|proc_offset_bits_begin) | (|proc_offset_bits_end)) begin
                        L1_mask =  {DATA_WIDTH{1'b1}}  
                                                &({DATA_WIDTH{1'b1}} << {proc_offset_bits_begin, 2'b00}) 
                                                &({DATA_WIDTH{1'b1}} >> {{OFFSET_WIDTH{1'b1}} - proc_offset_bits_end, 2'b00});
                        L2_mask = L1_mask;
                    end 

                    next_state = STATE_fullMEM;
                end
                
                
            end
            

        end
        STATE_fullB1: begin
            if (~buff_L1_full) begin
                //L1 CACHE
                L1_wr_enable = 1'b1;
                L1_wr_address = save_address;
                L1_wr_data = save_data;
                L1_mask = save_mask;
                next_state = STATE_idle;
            end
        end
        STATE_fullB2: begin
            
            if (~buff_L2_full) begin
                //L2 CACHE
                L2_wr_enable = 1'b1;
                L2_wr_address = save_address;
                L2_wr_data = save_data;
                L2_mask = save_mask;
                next_state = STATE_idle;
            end
        end
        STATE_fullMEM: begin
            if (~buff_mem_full) begin
                //MEM
                MEM_wr_enable = 1'b1;
                MEM_wr_address = save_address;
                MEM_wr_data = save_data;
                MEM_mask = save_mask;
                next_state = STATE_idle;

            end

        end
        STATE_fullB1B2: begin
            if ((~buff_L1_full) & (~buff_L2_full))begin
                //L1 CACHE
                L1_wr_enable = 1'b1;
                L1_wr_address = save_address;
                L1_wr_data = save_data;
                L1_mask = save_mask;
                
                //L2 CACHE
                L2_wr_enable = 1'b1;
                L2_wr_address = save_address;
                L2_wr_data = save_data;
                L2_mask = save_mask;
                
                next_state = STATE_idle;
            end
            else if (~buff_L1_full) begin
                //L1 CACHE
                L1_wr_enable = 1'b1;
                L1_wr_address = save_address;
                L1_wr_data = save_data;
                L1_mask = save_mask;

                next_state = STATE_fullB2;
            end
            else if (~buff_L2_full) begin
                //L2 CACHE
                L2_wr_enable = 1'b1;
                L2_wr_address = save_address;
                L2_wr_data = save_data;
                L2_mask = save_mask;

                next_state = STATE_fullB1;
            end
        end
        STATE_fullB1MEM: begin
            if ((~buff_L1_full) & (~buff_mem_full))begin
                //L1 CACHE
                L1_wr_enable = 1'b1;
                L1_wr_address = save_address;
                L1_wr_data = save_data;
                L1_mask = save_mask;
                
                //L2 CACHE
                MEM_wr_enable = 1'b1;
                MEM_wr_address = save_address;
                MEM_wr_data = save_data;
                MEM_mask = save_mask;
                
                next_state = STATE_idle;
            end
            else if (~buff_L1_full) begin
                //L1 CACHE
                L1_wr_enable = 1'b1;
                L1_wr_address = save_address;
                L1_wr_data = save_data;
                L1_mask = save_mask;

                next_state = STATE_fullMEM;
            end
            else if (~buff_mem_full) begin
                //MEM
                MEM_wr_enable = 1'b1;
                MEM_wr_address = save_address;
                MEM_wr_data = save_data;
                MEM_mask = save_mask;

                next_state = STATE_fullB1;
            end
        end
        STATE_fullB2MEM: begin
            if ((~buff_L2_full) & (~buff_mem_full))begin
                //L2 CACHE
                L2_wr_enable = 1'b1;
                L2_wr_address = save_address;
                L2_wr_data = save_data;
                L2_mask = save_mask;
                
                //MEM
                MEM_wr_enable = 1'b1;
                MEM_wr_address = save_address;
                MEM_wr_data = save_data;
                MEM_mask = save_mask;
                
                next_state = STATE_idle;
            end
            else if (~buff_L2_full) begin
                //L2 CACHE
                L2_wr_enable = 1'b1;
                L2_wr_address = save_address;
                L2_wr_data = save_data;
                L2_mask = save_mask;

                next_state = STATE_fullMEM;
            end
            else if (~buff_mem_full) begin
                //MEM
                MEM_wr_enable = 1'b1;
                MEM_wr_address = save_address;
                MEM_wr_data = save_data;
                MEM_mask = save_mask;

                next_state = STATE_fullB2;
            end
        end
        STATE_fullALL: begin
            if ((~buff_L1_full) & (~buff_L2_full) & (~buff_mem_full)) begin
                //L1 CACHE
                L1_wr_enable = 1'b1;
                L1_wr_address = save_address;
                L1_wr_data = save_data;
                
                //L2 CACHE
                L2_wr_enable = 1'b1;
                L2_wr_address = save_address;
                L2_wr_data = save_data;
                
                //MEM
                MEM_wr_enable = 1'b1;
                MEM_wr_address = save_address;
                MEM_wr_data = save_data;
                
                //masks
                L1_mask = save_mask;
                L2_mask = save_mask;
                MEM_mask = save_mask;

                next_state = STATE_fullALL;
            end
            else if (buff_L1_full & buff_L2_full) begin
                //MEM
                MEM_wr_enable = 1'b1;
                MEM_wr_address = save_address;
                MEM_wr_data = save_data;
                //masks
                
                MEM_mask = save_mask;
                
                next_state = STATE_fullB1B2;                  
            end
            else if (buff_L1_full & buff_mem_full)begin
                //L2 CACHE
                L2_wr_enable = 1'b1;
                L2_wr_address = save_address;
                L2_wr_data = save_data;

                L2_mask = save_mask;

                next_state = STATE_fullB1MEM;

            end
            else if (buff_L2_full & buff_mem_full) begin
                //L1 CACHE
                L1_wr_enable = 1'b1;
                L1_wr_address = save_address;
                L1_wr_data = save_data;

                L1_mask = save_mask;
                
                next_state = STATE_fullB2MEM;

            end
            else if (buff_L1_full) begin
                //L2 CACHE
                L2_wr_enable = 1'b1;
                L2_wr_address = save_address;
                L2_wr_data = save_data;
                //MEM
                MEM_wr_enable = 1'b1;
                MEM_wr_address = save_address;
                MEM_wr_data = save_data;
                //masks
                
                L2_mask =  save_mask;
                MEM_mask = save_mask;

                next_state = STATE_fullB1;
            end
            else if (buff_L2_full) begin
                //L1 CACHE
                L1_wr_enable = 1'b1;
                L1_wr_address = save_address;
                L1_wr_data = save_data;
                //MEM
                MEM_wr_enable = 1'b1;
                MEM_wr_address = save_address;
                MEM_wr_data = save_data;
                //masks
                
                L1_mask = save_mask;
                MEM_mask = save_mask;

                next_state = STATE_fullB2;
            end
            else if (buff_mem_full) begin
                //L1 CACHE
                L1_wr_enable = 1'b1;
                L1_wr_address = save_address;
                L1_wr_data = save_data;
                
                //L2 CACHE
                L2_wr_enable = 1'b1;
                L2_wr_address = save_address;
                L2_wr_data = save_data;

                
                L1_mask = save_mask;
                L2_mask = save_mask;

                next_state = STATE_fullMEM;
            end
        end

    endcase

end



endmodule