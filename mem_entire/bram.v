`timescale 1ns/10ps


module bram #(
    parameter ADDR_SIZE = 7,           // Address width
    parameter OFFSET_BITS=6,           //
    parameter DATA_SIZE = 2**(OFFSET_BITS+3),           // Data width
    parameter ROWS = 2**(ADDR_SIZE),      // Total number of rows
    parameter stall_bram = 1000
)(
    input clk,
    input reset,
    input [ADDR_SIZE+OFFSET_BITS-1:0] addr_write,
    input [ADDR_SIZE+OFFSET_BITS-1:0] addr_read,
    input bram_enable,
    input read_enable,
    input write_enable,
    input [DATA_SIZE-1:0] data_input,
    input [DATA_SIZE-1:0] data_mask,
    output reg [DATA_SIZE-1:0] data_out,
    output reg data_ready
);

    integer i;
    
    // Block RAM arrays for each bank
    reg [DATA_SIZE-1:0] memory [0:ROWS-1];
    reg valid_array [0:ROWS-1];
    reg [DATA_SIZE-1:0] data_buffer;
    reg [DATA_SIZE-1:0] data_buffer2;
    
    reg enable_reg;
    reg read_enable_reg;
    reg write_enable_reg;
    reg [DATA_SIZE-1:0]data_in_reg;
    reg [DATA_SIZE-1:0] data_mask_reg;
    reg [DATA_SIZE-1:0] data_out_reg;
    reg data_ready_reg;
    
    reg[ADDR_SIZE-1:0] address_read;
    reg [ADDR_SIZE-1:0] address_write;
    
    `ifdef DATA_HEX
    initial 
    begin
            for (i = 0; i < ROWS; i = i + 1) begin
                    memory[i] = 0;
                    valid_array[i] = 1;
            end
        
        memory[0] = 512'hdeadbeef_cafebabe_B00000B5_babebabe_facccccc_babe2fac_b17C3E55_aaaaaaff_EA7D1CCC_22222222_33333333_44444444_55555555_66666666_77777777_88888888;
        memory[16] = 512'h00000000_00000000_00000000_00000000_00000000_00000000_00000000_00000000_00000000_00000000_0001a083_0030a023_01000193_0001a103_01000193_06400093;
        
            //`ifdef DATA_HEX
            // $readmemh(`DATA_HEX, memory);
            //`endif
        end
    `endif
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < ROWS; i = i + 1) begin
                    // memory[i] <= 0;
                    valid_array[i] <= 1;
            end
            address_read<=0;
            address_write=0;
            data_out <= 0;
            enable_reg<=0;
            write_enable_reg <= 0;
            data_in_reg <= 0;
            data_mask_reg <=0;
        end 
        else begin
            address_read    <= addr_read[ADDR_SIZE+OFFSET_BITS-1:OFFSET_BITS];
            address_write   <= addr_write[ADDR_SIZE+OFFSET_BITS-1:OFFSET_BITS];
            enable_reg       <= bram_enable;
            read_enable_reg  <= read_enable;
            write_enable_reg <= write_enable;
            data_in_reg      <=data_input;
            data_mask_reg    <=data_mask;
            data_ready       <=data_ready_reg;
            data_out         <=data_out_reg;
        end
    end
    // Read and write operations
    always @(*) begin
        data_ready_reg=0;
        data_out_reg = 0;

        if(read_enable|write_enable)
        begin
            if(enable_reg) begin
                if(write_enable_reg) begin
                    $display("Writing memory on %h, data:%h",address_write,data_in_reg);
    //                if(!valid_array[address_write]) begin
                    data_buffer = memory[address_write];
                    data_buffer = data_buffer & (~data_mask); //clearing the data inside
                    data_buffer = data_buffer | (data_in_reg & data_mask);
                    memory[address_write] = data_buffer;
                    valid_array[address_write] = 1;
    //                end 
                end
                if (read_enable) begin
                    if(valid_array[address_read]) begin
                        #60
                        if(read_enable)
                        begin
                        data_out_reg = memory[address_read];
                        data_ready_reg = 1;
                        end
                    end
                    else begin
                        data_out_reg = 0;
                        data_ready_reg = 0;
                    end
                end
                else
                begin
                    data_out_reg = 0;
                    data_ready_reg = 0;
                end
            end
        end
    end
endmodule

/*
reg [block_size-1:0] size_of_block [rows];

*/