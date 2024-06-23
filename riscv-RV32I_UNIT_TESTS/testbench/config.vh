// RISC-V SiMPLE SV -- common configuration for testbench
// BSD 3-Clause License
// (c) 2017-2021, Arthur Matos, Marcus Vinicius Lamar, Universidade de Brasília,
//                Marek Materzok, University of Wrocław

`ifndef RV_CONFIG
`define RV_CONFIG

// Select ISA extensions
// `define M_MODULE    // multiplication and division

//////////////////////////////////////////
//              Memory config           //
//////////////////////////////////////////

// Program counter initial value
//`define INITIAL_PC      32'h0000400
`define INITIAL_PC      32'd1024

// Instruction memory
`define TEXT_BEGIN      `INITIAL_PC
`define TEXT_BITS       8
`define TEXT_WIDTH      2**`TEXT_BITS
`define TEXT_END        `TEXT_BEGIN + `TEXT_WIDTH - 1

// Data memory
`define DATA_BEGIN      32'h8000_0000
`define DATA_BITS       16
`define DATA_WIDTH      2**`DATA_BITS
`define DATA_END        `DATA_BEGIN + `DATA_WIDTH - 1

`define TEXT_HEX  "instr.hex"
`define DATA_HEX  "data.hex"

`endif