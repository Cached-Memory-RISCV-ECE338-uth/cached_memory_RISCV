// RISC-V SiMPLE SV -- testbench
// BSD 3-Clause License
// (c) 2019, Marek Materzok, University of Wrocław

#include "Vtoplevel.h"
#include "verilated.h"
#include "verilated_vcd_c.h"
#include <iostream>
#include <iomanip>
#include <memory>
#include <string>

#define GREEN "\033[1;32m"
#define RED "\033[1;31m"
#define RESET "\033[1;0m"
#define YELLOW "\033[1;33m"

double sc_time_stamp() { return 0; }

int main(int argc, const char **argv, const char **env)
{
    Verilated::commandArgs(argc, argv);

    // instatiate top level module
    std::unique_ptr<Vtoplevel> top(new Vtoplevel);

    // dump waveforms to simx.vcd
    #if TRACE==1
    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    top->trace(tfp, 99);  // Trace 99 levels of hierarchy
    tfp->open("simx.vcd");
    #endif

    top->reset = 1;

    // define clock and reset signals (reset is active low)
    for (int time = 0; time < 100000; time++) {
        if (time > 9)
            top->reset = 1;
        else if (time > 4)
            top->reset = 0;
        top->clock = time & 1;
        top->eval();

        #if TRACE==1
        tfp->dump(time);
        #endif

        // print debugging messages
        #if VERBOSE==1
        if (top->clock && time > 8) {
            std::cout << std::hex << std::setfill('0')
                      << "pc=" << std::setw(8) << top->toplevel__DOT__cpu__DOT__PC << " "
                      << "inst=" << std::setw(8) << top->toplevel__DOT__cpu__DOT__IFID_instr << " "
                      << "addr=" << std::setw(8) << top->toplevel__DOT__cpu__DOT__EXMEM_ALUOut << " "
                      << "out=" << std::setw(8) << top->toplevel__DOT__cpu__DOT__EXMEM_MemWriteData << " "
                      << "alu out=" << std::setw(8) << top->toplevel__DOT__cpu__DOT__EXMEM_ALUOut << " "                      
                      << (top->toplevel__DOT__cpu__DOT__EXMEM_MemWrite ? "1" : "0") << " \n";
        }
        #endif

        if (top->toplevel__DOT__cpu__DOT__EXMEM_MemWrite && top->toplevel__DOT__cpu__DOT__EXMEM_ALUOut == 0xfffffff0) {
            if (top->toplevel__DOT__cpu__DOT__EXMEM_MemWriteData) {
                printf("%sPASS%s\n", GREEN, RESET);
                #if TRACE==1
                tfp->close();
                #endif
                return 0;
            } else {
                printf("%sFAIL%s\n", RED, RESET);
                #if TRACE==1
                tfp->close();
                #endif
                return -1;
            }
        }
        
    }
    

    #if TRACE==1
    tfp->close();
    #endif

    // need timeout because if something is wrong the processor
    // will never write to the correct memory address and 
    // simulation will never finish
    printf("%sTIMEOUT%s\n", YELLOW, RESET);

    return -1;
}