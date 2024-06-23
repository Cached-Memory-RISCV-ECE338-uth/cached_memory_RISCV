`timescale 1ns/10ps

module write_buffer #(
    parameter ADDRESSIZE = 32,  // Size in bits
    parameter DATASIZE = 16*4,     // Size in bytes
    parameter BUFFER_NUMBER_LOG = 2,
    parameter BUFFER_NUMBER = 4,
    parameter DATA_WIDTH = 32
) (
    input clk,
    input reset,
    input enable,
    input done,
    input [ADDRESSIZE-1:0] address_in,
    input [8*DATASIZE-1:0] data_in,
    input [8*DATASIZE-1:0] data_mask,
    input check_exist_in,
    
    output buffer_full,
    output reg[8*DATASIZE-1:0] data_out,
    output reg [ADDRESSIZE-1:0] address_out,
    output reg [8*DATASIZE-1:0] mask_out,
    output write_enable,
    output reg check_exist_out,

    //address checking 
    input [ADDRESSIZE-1:0] address_to_check,

    output address_exist,
    output [DATASIZE*8-1:0] data_found
);

    reg [BUFFER_NUMBER_LOG-1:0] output_pointer;
    reg [BUFFER_NUMBER_LOG-1:0] input_pointer;

    reg [DATASIZE * 8 - 1:0] data_buffer [2**BUFFER_NUMBER_LOG-1:0];
    reg [ADDRESSIZE - 1:0] address_buffer [2**BUFFER_NUMBER_LOG-1:0];
    reg [DATASIZE - 1:0] mask_buffer [2**BUFFER_NUMBER_LOG-1:0];
    reg [2**BUFFER_NUMBER_LOG-1:0] check_exist_buffer;

    reg [BUFFER_NUMBER_LOG-1:0]data_amount;
    wire [1:0] amount_change;

    assign address_exist = 1'b0;
    assign data_found = {DATASIZE*8{1'b0}};


    reg write_enable_demand;

    parameter   INCREASE = 2'b00,
                DECREASE = 2'b01,
                STEADY = 2'b10;


    integer i;
    //registers storing part
    always @(posedge clk) begin
        if (reset) begin
            for (i =0; i < 2**BUFFER_NUMBER_LOG; i = i+1)begin
                data_buffer[i] <= {DATASIZE*8{1'b0}};
                mask_buffer[i] <=  {DATASIZE*8{1'b0}};
                address_buffer[i] <= {ADDRESSIZE{1'b0}};
                check_exist_buffer[i] <= 1'b0;
            end
            data_amount <= {BUFFER_NUMBER_LOG{1'b0}};
            output_pointer <= {BUFFER_NUMBER_LOG{1'b0}};
            input_pointer <= {BUFFER_NUMBER_LOG{1'b0}};

        end
        else begin

            if (amount_change == INCREASE) begin
                data_amount <= data_amount + {{BUFFER_NUMBER_LOG-1{1'b0}}, 1'b1};
            end
            else if (amount_change == DECREASE)begin
                data_amount <= data_amount - {{BUFFER_NUMBER_LOG-1{1'b0}}, 1'b1};

            end

            if (done) begin
                output_pointer <= output_pointer + {{BUFFER_NUMBER_LOG-1{1'b0}}, 1'b1};
            end    
            
            if (enable) begin
                check_exist_buffer[input_pointer] <= check_exist_in;
                data_buffer[input_pointer]      <= data_in;
                mask_buffer[input_pointer]      <= data_mask;
                address_buffer[input_pointer]   <= address_in;
                
                
                input_pointer <= input_pointer + {{BUFFER_NUMBER_LOG-1{1'b0}}, 1'b1};

            end
        end

    end

    assign write_enable =       ((data_amount == {BUFFER_NUMBER_LOG{1'b0}}) & enable)       | 
                                ((data_amount == {{BUFFER_NUMBER_LOG-1{1'b0}}, 1'b1}) & done & enable)  |
                                ((data_amount > {{BUFFER_NUMBER_LOG-1{1'b0}}, 1'b1}) & done);

    assign buffer_full =    ((data_amount == {BUFFER_NUMBER_LOG{1'b1}}) & (~done)) ;

    assign amount_change =  (enable & done )        ?       STEADY:
                            (enable & (~done))      ?       INCREASE:
                            ((~enable) & done)      ?        DECREASE:
                            2'b11;


    //writing circuit
    always @(*) begin

        
        if ((data_amount == {BUFFER_NUMBER_LOG{1'b0}}) & enable) begin

            check_exist_out = check_exist_in;
            address_out = address_in;
            data_out = data_in;
            mask_out = data_mask;
            
            

        end
        else begin
        
            if (done) begin
             
                if (data_amount == {{BUFFER_NUMBER_LOG-1{1'b0}}, 1'b1}) begin
                    check_exist_out = check_exist_in;
                    address_out = address_in;
                    data_out = data_in;
                    mask_out = data_mask;
                    

                end
                else begin
                    check_exist_out = check_exist_buffer[output_pointer + 1'b1];
                    data_out = data_buffer[output_pointer + 1'b1];
                    address_out = address_buffer[output_pointer + 1'b1];
                    mask_out = mask_buffer[output_pointer + 1'b1];
                    
                end
                
            end
            else begin
                
                check_exist_out = check_exist_buffer[output_pointer];
                data_out = data_buffer[output_pointer];
                address_out = address_buffer[output_pointer];
                mask_out = mask_buffer[output_pointer];
                
            end
        end


    end


endmodule
    