VERILATOR = verilator
VERILATOR_FLAGS = -cc --exe -Wall --trace
VERILATOR_INPUT = ../../src/pipelined/toplevel.v main.cpp
VERILATOR_INCLUDES = -I../../src/pipelined
TESTDIR=../../tests
TESTS=$(notdir $(patsubst %.S,%,$(wildcard $(TESTDIR)/*.S)))
TEST = addi

verilate:
	@echo 
	@echo "----- VERILATE -----"
	$(VERILATOR) $(VERILATOR_FLAGS) $(VERILATOR_INCLUDES) $(VERILATOR_INPUT)

build:
	@echo 
	@echo "----- BUILD -----"
	$(MAKE) -j -C obj_dir -f Vtoplevel.mk

run:
	@echo
	@echo "----- RUN INDIVIDUAL TEST -----"
	obj_dir/Vtoplevel +trace +text_file=../tests/$(TEST).text.vh +data_file=../tests/$(TEST).data.vh

check: 
	@echo 
	@echo "----- RUN ALL TESTS-----"
	$(addsuffix .run,$(TESTS))

%.run: ./obj_dir/Vtoplevel
	./obj_dir/Vtoplevel +text_file=$(TESTDIR)/$(@:.run=).text.vh +data_file=$(TESTDIR)/$(@:.run=).data.vh

clean:
	rm -rf obj_dir   
