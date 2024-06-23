//vector of RISC V is inputted and offset for controller are outputed

module convert_vector_to_offset
#(

    parameter OFFSET_SIZE = 2,
    parameter VECTOR_SIZE = 4
)
(
    input [VECTOR_SIZE-1:0]     vector,
    output [OFFSET_SIZE-1:0]    begin_offset,
    output [OFFSET_SIZE-1:0]    end_offset
);

    assign {begin_offset, end_offset} =
                vector == 4'b0001   ?       {2'b00, 2'b00}      :
                vector == 4'b0011   ?       {2'b00, 2'b01}      :
                vector == 4'b0111   ?       {2'b00, 2'b10}      :
                vector == 4'b1111   ?       {2'b00, 2'b11}      :
                vector == 4'b1000   ?       {2'b11, 2'b11}      :
                vector == 4'b1100   ?       {2'b10, 2'b11}      :
                vector == 4'b1110   ?       {2'b01, 2'b11}      :
                vector == 4'b0110   ?       {2'b01, 2'b10}      :
                vector == 4'b0100   ?       {2'b10, 2'b10}      :
                vector == 4'b0010   ?       {2'b01, 2'b01}      :
                {2'b00, 2'b00};

    
   
    


endmodule