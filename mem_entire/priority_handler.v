`timescale 1ns/10ps

module priority_handler 
#(  parameter DATA_WIDTH = 32,
    parameter ADDRESS_WIDTH = 32,
    parameter BLOCK_WIDTH = 32
)

(
    clk, rst, 
    read_en_instr, read_en_data, 
    read_done,
    read_done_virtual_data, read_done_virtual_instr,
    addr_data, addr_instr, 
    memory_block, memory_address, mem_enable,
    block_return
);

input clk, rst;
input read_en_instr, read_en_data;
input [ADDRESS_WIDTH-1:0] addr_data, addr_instr;
input [BLOCK_WIDTH-1:0] memory_block;
input read_done;

output reg [ADDRESS_WIDTH-1:0] memory_address;
output mem_enable;
output reg   [BLOCK_WIDTH-1:0] block_return;
output reg read_done_virtual_data, read_done_virtual_instr;

parameter   STATE_idle = 2'b00,
            STATE_INSTR_READING = 2'b01,
            STATE_DATA_READING = 2'b10;


reg [1:0] current_state;
reg [1:0] next_state;

always @(posedge clk) begin

    if (rst) begin
        current_state <= STATE_idle;
    end
    else begin
        current_state <= next_state;
    end
end

assign mem_enable = read_en_data | read_en_instr;

/*read_en_data, read_en_instr, read_done, current_state*/
always @(*) begin
    
    next_state = current_state;

    memory_address = {ADDRESS_WIDTH{1'b0}};
    read_done_virtual_data = 1'b0;  read_done_virtual_instr = 1'b0;
    block_return = {BLOCK_WIDTH{1'b0}};

    case (current_state)
        STATE_idle: begin
            if (read_en_instr) begin
                memory_address = addr_instr;

                next_state = STATE_INSTR_READING;
            end
            else if (read_en_data) begin
                memory_address = addr_data;

                next_state = STATE_DATA_READING;

            end
        end
        STATE_DATA_READING: begin
            if (read_done) begin
                
                if (read_en_instr) begin
                    memory_address = addr_instr;
                    next_state = STATE_INSTR_READING;
                    block_return = memory_block;
                    read_done_virtual_data = 1'b1;
                end
                else if (read_en_data) begin
                    memory_address = addr_data;
                    block_return = memory_block;
                    read_done_virtual_data = 1'b1;
                    
                end
                else begin
                    next_state = STATE_idle;
                    block_return = memory_block;
                    read_done_virtual_data = 1'b1;
                end

            end
            else begin
                memory_address = addr_data;

            end    

        end
        STATE_INSTR_READING: begin
            if (read_done) begin
                
                
                if (read_en_instr) begin
                    memory_address = addr_instr;
                    next_state = STATE_INSTR_READING;
                    read_done_virtual_instr = 1'b1;
                    block_return = memory_block;
                end
                else if (read_en_data) begin
                    memory_address = addr_data;
                    next_state = STATE_DATA_READING;
                    read_done_virtual_instr = 1'b1;
                    block_return = memory_block;
                end
                else begin
                    next_state = STATE_idle;
                    read_done_virtual_instr = 1'b1;
                    block_return = memory_block;

                end

            end
            else begin
                memory_address = addr_instr;
            end
        end
    endcase

end






endmodule