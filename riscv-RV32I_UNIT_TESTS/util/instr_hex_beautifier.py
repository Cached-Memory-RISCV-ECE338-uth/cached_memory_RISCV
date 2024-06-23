import sys
import re

R_OPCODES = "0110011"
I_OPCODES_1 = "0010011"
I_OPCODES_2 = "0000011"
I_OPCODES_3 = "1110011"
I_OPCODES_4 = "1100111"
S_OPCODES = "0100011"
B_OPCODES = "1100011"
J_OPCODES = "1101111"
U_OPCODES_1 = "0110111"
U_OPCODES_2 = "0010111"

# R-Type 
ADD_FUNCT3 = "000"
XOR_FUNCT3 = "100"
OR_FUNCT3 = "110"
AND_FUNCT3 = "111"
SLL_FUNCT3 = "001"
SRL_FUNCT3 = "101"
SLT_FUNCT3 = "010"
SLTU_FUNCT3 = "011"

# I-Type
ADDI_FUNCT3	 = "000"
XORI_FUNCT3 = "100"
ORI_FUNCT3 = "110"
ANDI_FUNCT3 =  "111"
SLLI_FUNCT3 = "001"
SRLI_SRAI_FUNCT3 = "101"
SLTI_FUNCT3 = "010"
SLTIU_FUNCT3 = "011"

LB_FUNCT3 = "000"
LH_FUNCT3 = "001"
LW_FUNCT3 = "010"
LBU_FUNCT3 = "100"
LHU_FUNCT3 = "101"
ECALL_EBREAK_FUNCT3 = "000"
JALR_FUNCT3 = "000"

SRLI_IMM = "000000"
SRAI_IMM = "100000"

ECALL_IMM = "0"
EBREAK_IMM = "1"

# S-Type
SB_FUNCT3 = "000"
SH_FUNCT3 = "001"
SW_FUNCT3 = "010"

# B-Type
BEQ_FUNCT3 = "000"
BNE_FUNCT3 = "001"
BLT_FUNCT3 = "100"
BGE_FUNCT3 = "110"
BLTU_FUNCT3 = "110"
BGEU_FUNCT3 = "111"

# Created the sign extended hex of an initially binary immediate
def bin_to_hex(bin_str):
	bin_sign_extended = (32 - len(bin_str)) * bin_str[0] + bin_str
	hex_str = "0x"
	for i in range(0, 32, 4):
		num = hex(int(bin_sign_extended[i:(i+4)], 2))[2:]
		hex_str += num
	return(hex_str)

def reg_to_string(reg):
	reg_str_array = [
		"zero", "ra", "sp", "gp", "tp",
		"t0", "t1", "t2",
		"s0", "s1",
		"a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7",
		"s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "10", "11",
		"t3", "t4", "t5", "t6" ]
	return reg_str_array[int(reg, 2)]

# Dissasembles RISC-V instructions
def hexToString(instruction_str):
	
	instruction = bin(int(instruction_str, 16))[2:].zfill(32)
	# print(instruction)
	instruction = instruction[::-1]
	
	opcode = instruction[0:7][::-1]
	funct3 = instruction[12:15][::-1]
	funct7 = instruction[25:32][::-1]
	rs1 = instruction[15:20][::-1]
	rs2 = instruction[20:25][::-1]
	rd = instruction[7:12][::-1]

	imm_i = instruction[20:32][::-1]
	imm_b = instruction[31] + instruction[7] + instruction[25:31][::-1] + instruction[8:12][::-1] + "0"
	imm_s = instruction[25:32][::-1] + instruction[7:12][::-1]
	imm_u = instruction[12:32][::-1] + 3*"0000"
	imm_j = instruction[31] + instruction[12:20][::-1] + instruction[20] + instruction[21:31][::-1] + "0"

	# print(imm_b)
	# print(int(imm_b, 2))

	inst_str = ""
	if opcode == R_OPCODES:
		if funct3 == ADD_FUNCT3:
			if funct7 == "100000":
				inst_str = "sub"
			else:
				inst_str = "add"
		elif funct3 == XOR_FUNCT3:
			inst_str = "xor"
		elif funct3 == OR_FUNCT3:
			inst_str = "or"
		elif funct3 == AND_FUNCT3:
			inst_str = "and"
		elif funct3 == SRL_FUNCT3:
			if funct7 == "100000":
				inst_str = "sra"
			else:
				inst_str = "srl"
		elif funct3 == SLL_FUNCT3:
			inst_str = "sll"
		elif funct3 == SLT_FUNCT3:
			inst_str = "slt"
		elif funct3 == SLTU_FUNCT3:
			inst_str = "sltu"
		inst_str = inst_str + " " + reg_to_string(rd) + ", " + reg_to_string(rs1) + ", " + reg_to_string(rs2)
	elif opcode == I_OPCODES_1:
		if funct3 == ADDI_FUNCT3:
			inst_str = "addi"
		elif funct3 == XORI_FUNCT3:
			inst_str = "xori"
		elif funct3 == ORI_FUNCT3:
			inst_str = "ori"
		elif funct3 == ADDI_FUNCT3:
			inst_str = "addi"
		elif funct3 == SLLI_FUNCT3:
			inst_str = "slli"
		elif funct3 == SLLI_FUNCT3:
			inst_str = "slli"
		elif funct3 == SRLI_SRAI_FUNCT3:
			if imm_i[5:11] == SRLI_IMM:
				inst_str = "srli"
			elif imm_i[5:11] == SRAI_IMM:
				inst_str = "srai"
			else:
				inst_str = "unknown"
		elif funct3 == SLTI_FUNCT3:
			inst_str = "slti"
		elif funct3 == SLTIU_FUNCT3:
			inst_str = "sltiu"
		inst_str = inst_str + " " + reg_to_string(rd) + ", " + reg_to_string(rs1) + ", " + bin_to_hex(imm_i)
	elif opcode == I_OPCODES_2:
		if funct3 == LB_FUNCT3:
			inst_str = "lb"
		elif funct3 == LH_FUNCT3:
			inst_str = "lh"
		elif funct3 == LW_FUNCT3:
			inst_str = "lw"
		elif funct3 == LBU_FUNCT3:
			inst_str = "lbu"
		elif funct3 == LHU_FUNCT3:
			inst_str = "lhu"
		imm = int(imm_i, 2) - (int(imm_i[0], 2) << len(imm_i))
		inst_str += " " + reg_to_string(rd) + ", " + str(imm) + "(" + reg_to_string(rs1) + ") "
	elif opcode == I_OPCODES_3:
		if funct3 == ECALL_EBREAK_FUNCT3:
			if imm_i[0] == ECALL_IMM:
				inst_str = "ecall"
			elif imm_i[0] == EBREAK_IMM:
				inst_str = "ebreak"
		else:
			inst_str = "unknown"
		imm = int(imm_i, 2) - (int(imm_i[0], 2) << len(imm_i))
		# inst_str += 
	elif opcode == I_OPCODES_4:
		if funct3 == JALR_FUNCT3:
			inst_str = "jalr"
		else:
			inst_str = "unknown"
		imm = int(imm_i, 2) - (int(imm_i[0], 2) << len(imm_i))
		inst_str = inst_str + " " + reg_to_string(rd) + ", " + reg_to_string(rs1) + ", " + reg_to_string(rs2)
	elif opcode == S_OPCODES:
		if funct3 == SB_FUNCT3:
			inst_str = "sb"
		elif funct3 == SH_FUNCT3:
			inst_str = "sh"
		elif funct3 == SW_FUNCT3:
			inst_str = "sw"
		else:
			inst_str = "unknown"
		imm = int(imm_s, 2) - (int(imm_s[0], 2) << len(imm_s))
		inst_str += " " + reg_to_string(rs2) + ", " + str(imm) + "(" + reg_to_string(rs1) + ") "
	elif opcode == B_OPCODES:
		if funct3 == BEQ_FUNCT3:
			inst_str = "beq"
		elif funct3 == BNE_FUNCT3:
			inst_str = "bne"
		elif funct3 == BLT_FUNCT3:
			inst_str = "blt"
		elif funct3 == BGE_FUNCT3:
			inst_str = "bge"
		elif funct3 == BLTU_FUNCT3:
			inst_str = "bltu"
		elif funct3 == BGEU_FUNCT3:
			inst_str = "bgeu"
		else:
			inst_str = "unknown"
		imm = int(imm_b, 2) - (int(imm_b[0], 2) << len(imm_b))
		inst_str += " " + reg_to_string(rs1) + ", " + reg_to_string(rs2) + ", " + str(imm)
	elif opcode == J_OPCODES:
		imm = int(imm_j, 2) - (int(imm_j[0], 2) << len(imm_j))
		inst_str = "jal" + " " + reg_to_string(rd) + ", " + str(imm)
	elif opcode == U_OPCODES_1:
		# imm = int(imm_u, 2) - (int(imm_u[0], 2) << len(imm_u))
		inst_str = "lui" + " " + reg_to_string(rd) + ", " + bin_to_hex(imm_u)
	elif opcode == U_OPCODES_2:
		imm = int(imm_u, 2) - (int(imm_u[0], 2) << len(imm_u))
		inst_str = "auipc" + " " + reg_to_string(rd) + ", " + str(imm)

	return inst_str

# Generates a string 
def getInstructionLine(addr, hex):
	return "@" + addr + " " + hex + " # " + hexToString((hex))

filename = sys.argv[1] if len(sys.argv) > 1 else print("Please give filename as argument.")
filenameBody = ".".join(filename.split(".")[0:-1])

new_filename = filenameBody.replace("/tests/", "/beautified_tests/") + "beautified.S"


print(filenameBody)

with open(filename, "r") as f:
	with open(new_filename, "w") as new_f:
		new_f.writelines(["\\* This is an autogenerated file that beautifies a RISC-V text memory file in hexadecimal format. *\\"])
		for l in f:
			if re.search("^/\*.*\*/$", l):
				continue
			
			line = l.split()
			addr = 4*int(line[0].split("@")[1], 16)
			instructions = line[1:]

			for instruction in instructions:
				hex_addr = '{:x}'.format(addr)
				formatted_addr = '{:0>{width}}'.format(hex_addr, width=5)
				
				instrLine = getInstructionLine(formatted_addr, instruction)
				addr += 4
				
				# new_f.writeline(instrLine)
				new_f.writelines([instrLine + "\n"])

# beq $9, $1, -5
# print(hexToString("11111110000101001000011011100011"))
# print(hexToString("00000000010000011000000100110011"))
# imm 