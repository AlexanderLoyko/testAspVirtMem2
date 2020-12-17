
#ifndef RISCV_SIM_CPU_H
#define RISCV_SIM_CPU_H

#include "Memory.h"
#include "Decoder.h"
#include "RegisterFile.h"
#include "CsrFile.h"
#include "Executor.h"

class Cpu
{
public:
    /*Cpu(CachedMem& mem)
        : _mem(mem)
    {
        _status = Status::Ready;
    }*/

    Cpu(UncachedMem& mem)
            : _mem(mem)
    {
        _status = Status::Ready;
    }

    void Clock()
    {
        _csrf.Clock();
        // Add your code here

        if (_status == Status::Fetching)
            goto fetching;
        if(_status == Status::Executing)
            goto executing;

        _mem.Request(_ip);
fetching:
        _instr_word = _mem.Response();
        if (_instr_word == std::optional<Word>()) {         // if instruction is not fetched yet - return and set status to "Fetching"
            _status = Status::Fetching;
            return;
        }
        _instruction = _decoder.Decode(*_instr_word);
        _rf.Read(_instruction);
        _csrf.Read(_instruction);
        _exe.Execute(_instruction, _ip);
        _mem.Request(_instruction);
executing:
        if (!_mem.Response(_instruction)) {             // if instruction is not executed yet - return and set status to "Executing"
            _status = Status::Executing;
            return;
        }
        _rf.Write(_instruction);
        _csrf.Write(_instruction);
        _csrf.InstructionExecuted();
        _ip = _instruction->_nextIp;
        _status = Status::Ready;                          // set status to "Ready" so the next clock will start normally
    }

    void Reset(Word ip)
    {
        _csrf.Reset();
        _ip = ip;
    }

    std::optional<CpuToHostData> GetMessage()
    {
        return _csrf.GetMessage();
    }

private:
    Reg32 _ip;
    Decoder _decoder;
    RegisterFile _rf;
    CsrFile _csrf;
    Executor _exe;
    //CachedMem& _mem;
    UncachedMem& _mem;
    // Add your code here, if needed

    InstructionPtr _instruction;
    std::optional<Word> _instr_word;
    enum class Status{
        Ready,
        Fetching,
        Executing
    };
    Status _status;
};


#endif //RISCV_SIM_CPU_H
