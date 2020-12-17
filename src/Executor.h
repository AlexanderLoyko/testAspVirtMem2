
#ifndef RISCV_SIM_EXECUTOR_H
#define RISCV_SIM_EXECUTOR_H

#include "Instruction.h"

class Executor
{
public:
    void Execute(InstructionPtr& instr, Word ip)
    {
        // Add your code here.

        switch (instr->_type) {
            case IType::Csrr : {
                instr->_data = instr->_csrVal;
                instr->_nextIp = ip + 4;
            }
                break;
            case IType::Csrw : {
                instr->_data = instr->_src1Val;
                instr->_nextIp = ip + 4;
            }
                break;
            case IType::St : {
                instr->_data = instr->_src2Val;
                instr->_addr = perform_alu(instr);
                instr->_nextIp = ip + 4;
            }
                break;
            case IType::J : {
                instr->_data = ip + 4;
                if (branch_condition(instr))
                    instr->_nextIp = ip + *instr->_imm;
                else
                    instr->_nextIp = ip + 4;
            }
                break;
            case IType::Jr : {
                instr->_data = ip + 4;
                if (branch_condition(instr))
                    instr->_nextIp = instr->_src1Val + *instr->_imm;
                else
                    instr->_nextIp = ip + 4;
            }
                break;
            case IType::Auipc : {
                instr->_data = ip + *instr->_imm;
                instr->_nextIp = ip + 4;
            }
                break;
            case IType::Alu : {
                instr->_data = perform_alu(instr);
                instr->_nextIp = ip + 4;
            }
                break;
            case IType::Br : {
                if (branch_condition(instr))
                    instr->_nextIp = ip + *instr->_imm;
                else
                    instr->_nextIp = ip + 4;
            }
                break;
            case IType::Ld : {
                instr->_addr = perform_alu(instr);
                instr->_nextIp = ip + 4;
            }
        }

    }

private:
    // Add helper functions here

    Word perform_alu(InstructionPtr& instr) {
        Word a, b;
        if (!instr->_src1)
            return 0;

        a = instr->_src1Val;
        if (!(instr->_imm || instr->_src2))
            return 0;

        b = instr->_imm ? *instr->_imm : instr->_src2Val;
        switch (instr->_aluFunc) {
            case AluFunc::Add :
                return a + b;
            case AluFunc::Sub :
                return a - b;
            case AluFunc::And :
                return a & b;
            case AluFunc::Or :
                return a | b;
            case AluFunc::Xor :
                return a ^ b;
            case AluFunc::Slt : {
                int sext_a = a, sext_b = b;
                return sext_a < sext_b;
            }
            case AluFunc::Sltu :
                return a < b;
            case AluFunc::Sll :
                return a << (b % 32);
            case AluFunc::Srl :
                return a >> (b % 32);
            case AluFunc::Sra : {
                int sext_a = a;
                sext_a = sext_a >> (b % 32);
                return Word(sext_a);
            }
        }
    }

    bool branch_condition(InstructionPtr& instr) {
        Word a, b;
        if (instr->_src1)
            a = instr->_src1Val;
        if (instr->_src2)
            b = instr->_src2Val;

        switch (instr->_brFunc) {
            case BrFunc::Eq :
                return a == b;
            case BrFunc::Neq :
                return a != b;
            case BrFunc::Lt : {
                int sext_a = a, sext_b = b;
                return sext_a < sext_b;
            }
            case BrFunc::Ltu : {
                return a < b;
            }
            case BrFunc::Ge : {
                int sext_a = a, sext_b = b;
                return sext_a >= sext_b;
            }
            case BrFunc::Geu :
                return a >= b;
            case BrFunc::AT :
                return true;
            case BrFunc::NT :
                return false;
        }
    }
};

#endif // RISCV_SIM_EXECUTOR_H
