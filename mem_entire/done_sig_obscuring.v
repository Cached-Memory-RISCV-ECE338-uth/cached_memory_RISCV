`timescale 1ns/10ps


module done_sig_obscuring
#(
    parameter BLOCK_WIDTH = 32
)
(

    clk, rst,
    fsm_enabling,
    real_enable,
    real_done, real_data,
    fsm_virtual_done, fsm_virtual_data,
    buff_full
    

);

input clk, rst;
input real_done;
input [BLOCK_WIDTH-1:0] real_data;
input buff_full;
input fsm_enabling;

output reg real_enable;
output reg [BLOCK_WIDTH-1:0] fsm_virtual_data;
output reg fsm_virtual_done;

reg [BLOCK_WIDTH-1:0] data_hold;


reg current_state;
reg next_state;

parameter   NO_INTERFERENCE = 1'b0,
            BUFF_FULL = 1'b1;

always @(posedge clk) begin

    if (rst) begin
        current_state <= NO_INTERFERENCE;
        data_hold <= {BLOCK_WIDTH{1'b0}};
    end
    else begin
        current_state <= next_state;
        if ((~current_state) & real_done & buff_full) begin
            data_hold <= real_data;

        end
    end

end




always @(*) begin

    next_state = current_state;

    fsm_virtual_done = 1'b0;
    fsm_virtual_data = {BLOCK_WIDTH{1'b0}};
    real_enable = fsm_enabling;
    

    case (current_state)
        NO_INTERFERENCE: begin
            if (real_done & (~buff_full))begin
                fsm_virtual_done = real_done;
                fsm_virtual_data = real_data;
            end
            else if (real_done & buff_full) begin
    
                real_enable = 1'b0;
                next_state = BUFF_FULL;
            end 
        end
        BUFF_FULL : begin
            if (~buff_full) begin
                real_enable = 1'b0;
                fsm_virtual_done = 1'b1;
                fsm_virtual_data = data_hold;

                next_state = NO_INTERFERENCE;
            end
            else begin
                real_enable = 1'b0;
    

            end
        

        end
    endcase


end






endmodule

/*
module (clk, rst, get_data, data_got);

    input clk, rst;
    input [31:0] get_data;
    output reg [31:0] data_got;


    always @(posedge clk) begin
        data_got <= get_data;

    end

endmodule*/