
 $$$$$$\                      $$\                       $$\      $$\                                                       
$$  __$$\                     $$ |                      $$$\    $$$ |                                                      
$$ /  \__| $$$$$$\   $$$$$$$\ $$$$$$$\   $$$$$$\        $$$$\  $$$$ | $$$$$$\  $$$$$$\$$$$\   $$$$$$\   $$$$$$\  $$\   $$\ 
$$ |       \____$$\ $$  _____|$$  __$$\ $$  __$$\       $$\$$\$$ $$ |$$  __$$\ $$  _$$  _$$\ $$  __$$\ $$  __$$\ $$ |  $$ |
$$ |       $$$$$$$ |$$ /      $$ |  $$ |$$$$$$$$ |      $$ \$$$  $$ |$$$$$$$$ |$$ / $$ / $$ |$$ /  $$ |$$ |  \__|$$ |  $$ |
$$ |  $$\ $$  __$$ |$$ |      $$ |  $$ |$$   ____|      $$ |\$  /$$ |$$   ____|$$ | $$ | $$ |$$ |  $$ |$$ |      $$ |  $$ |
\$$$$$$  |\$$$$$$$ |\$$$$$$$\ $$ |  $$ |\$$$$$$$\       $$ | \_/ $$ |\$$$$$$$\ $$ | $$ | $$ |\$$$$$$  |$$ |      \$$$$$$$ |
 \______/  \_______| \_______|\__|  \__| \_______|      \__|     \__| \_______|\__| \__| \__| \______/ \__|       \____$$ |
                                                                                                                 $$\   $$ |
                                                                                                                 \$$$$$$  |
                                                                                                                  \______/ 

To load this into vivado Add directories:
./meme_entire/
./riscv-RV31I_UNIT_TESTS/src/
./riscv-RV31I_UNIT_TESTS/testbench/


Add files:
/riscv-RV31I_UNIT_TESTS/src/data.hex	

The data hex need to be added as @0x00->0x09 (will be changed later on config.vh) and instructions as @0x10->inf. 
Both Data and Instructions need to be in blocks (16 words) or else the remaining block data will be 00000000.

You can create your own data.hex by going into ./riscv-RV31I_UNIT_TESTS/testbench/Assembler/

There there are 4 files:
instructions.txt -> here you put your assembly hex instructions
data.txt -> here you put your hex data
comments.txt -> here you can put any comments you would like to have added in the data.hex like the assembly.

Then run $: python makehex.py

It should generate a data.hex which you can either import straight to vivado or paste on the /riscv-RV31I_UNIT_TESTS/src/data.hex	

Enjoy!


NOT YET WORKING/EXIST :
-> L2 CACHE
-> DELAYS IN WRITE OPERATIONS
