`timescale 1ns/10ps

module hash_param #(
    parameter OUTPUT_BITS = 2
)(
    input [31:0] seed,
    output [(OUTPUT_BITS-1):0] hash_out
);
    // Intermediate signals
    wire [31:0] folded;
    integer i;

    // XOR folding process
    assign folded = seed[31:16] ^ seed[15:0];

    // Generate the hash by XORing bits together to reduce to the required number of output bits
    reg [(OUTPUT_BITS-1):0] hash_reg;
    always @(*) begin
        hash_reg = folded[(OUTPUT_BITS-1):0]; // Initialize with lower bits of the folded result
        for (i = OUTPUT_BITS; i < 32; i = i + OUTPUT_BITS) begin
            hash_reg = hash_reg ^ folded[i +: OUTPUT_BITS];
        end
    end

    // Output the final hash value
    assign hash_out = hash_reg;
endmodule