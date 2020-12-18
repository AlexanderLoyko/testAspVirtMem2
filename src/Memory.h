
#ifndef RISCV_SIM_DATAMEMORY_H
#define RISCV_SIM_DATAMEMORY_H

#include "Instruction.h"
#include <iostream>
#include <fstream>
//#include <elf.h>
#include "/usr/local/include/elf.h"
#include <cstring>
#include <vector>
#include <cassert>
#include <map>
#include <queue>
#include <time.h>
#include <algorithm>
#include <array>
#include <list>

using namespace std;

//static constexpr size_t memSize = 4*1024*1024; // memory size in 4-byte words
static constexpr size_t lineSizeBytes = 128;
static constexpr size_t lineSizeWords = lineSizeBytes / sizeof(Word);
using Line = std::array<Word, lineSizeWords>;
using TagClockPair = std::pair<size_t , clock_t>;
static constexpr size_t dataCacheBytes = 2048; // data cache size in bytes
static constexpr size_t codeCacheBytes = 1024; // instructions cache size in bytes
static Word ToLineAddr(Word addr) { return addr & ~(lineSizeBytes - 1); }

static constexpr size_t mainMemorySizeW = 1024*1024; // memory size in 4-byte words
static constexpr size_t virtualMemorySizeW = 1024*1024*1024;
static constexpr size_t pageSizeW = 1024;
static constexpr size_t pageSizeB = pageSizeW * 4;

static Word ToWordAddr(Word addr) {
    return addr >> 2u;
}

static Word ToPageAddr(Word addr) {
    return (addr & ~(pageSizeB - 1)) / pageSizeB;
}
static Word ToPageOffset(Word addr) {
    return ToWordAddr(addr) & (pageSizeW - 1);
}

static Word ToPageAddr2(Word addr) {
    return addr / pageSizeW;;
}

static Word ToPageOffset2(Word addr) {
    return addr - ToPageAddr2(addr) * pageSizeW;
}

class FifoAlg {
private:
    vector<Word> queueRecords;
    Word countFilledPagesInMainMemory;
    bool isMemoryFull;

public:
    FifoAlg() {
        queueRecords.resize(mainMemorySizeW / pageSizeW);
        countFilledPagesInMainMemory = 0;
        isMemoryFull = false;
    }

    void NewRecordInMainMemory(Word pageNumVirtual) {
        if (countFilledPagesInMainMemory < queueRecords.size()) {
            queueRecords[countFilledPagesInMainMemory] = pageNumVirtual;
            countFilledPagesInMainMemory++;

            if (countFilledPagesInMainMemory == queueRecords.size()) {
                isMemoryFull = true;
            }
        }
        else {
            queueRecords.erase(queueRecords.begin());
            queueRecords.push_back(pageNumVirtual);
        }
    }

    Word getOldestPageNumVirtualFromMainMemory() {
        return queueRecords[0];
    }

    bool getIsMemoryFull() {
        return isMemoryFull;
    }

    Word getCountFilledPagesInMainMemory() {
        return countFilledPagesInMainMemory;
    }
};

class MemoryStorage {
public:
    MemoryStorage() {
        _mem.resize(mainMemorySizeW);
        virtual_memory.resize(virtualMemorySizeW);

        // [0] - pageNumInVirtualMemory, [1] = pageNumInMainMemory, [2] = validBit
        for (Word i = 0; i < (virtualMemorySizeW / pageSizeW); i++) {
            pageTable[i].push_back(i);
            pageTable[i].push_back(-1);
            pageTable[i].push_back(0);
        }
    }

    bool LoadElf(const std::string &elf_filename) {
        std::ifstream elffile;
        elffile.open(elf_filename, std::ios::in | std::ios::binary);

        if (!elffile.is_open()) {
            std::cerr << "ERROR: load_elf: failed opening file \"" << elf_filename << "\"" << std::endl;
            return false;
        }

        elffile.seekg(0, elffile.end);
        size_t buf_sz = elffile.tellg();
        elffile.seekg(0, elffile.beg);

        // Read the entire file. If it doesn't fit in host memory, it won't fit in the risc-v processor
        std::vector<char> buf(buf_sz);
        elffile.read(buf.data(), buf_sz);

        if (!elffile) {
            std::cerr << "ERROR: load_elf: failed reading elf header" << std::endl;
            return false;
        }

        if (buf_sz < sizeof(Elf32_Ehdr)) {
            std::cerr << "ERROR: load_elf: file too small to be a valid elf file" << std::endl;
            return false;
        }

        // make sure the header matches elf32 or elf64
        Elf32_Ehdr *ehdr = (Elf32_Ehdr *) buf.data();
        unsigned char* e_ident = ehdr->e_ident;
        if (e_ident[EI_MAG0] != ELFMAG0
            || e_ident[EI_MAG1] != ELFMAG1
            || e_ident[EI_MAG2] != ELFMAG2
            || e_ident[EI_MAG3] != ELFMAG3) {
            std::cerr << "ERROR: load_elf: file is not an elf file" << std::endl;
            return false;
        }

        if (e_ident[EI_CLASS] == ELFCLASS32) {
            // 32-bit ELF
            return this->LoadElfSpecific<Elf32_Ehdr, Elf32_Phdr>(buf.data(), buf_sz);
        } else if (e_ident[EI_CLASS] == ELFCLASS64) {
            // 64-bit ELF
            return this->LoadElfSpecific<Elf64_Ehdr, Elf64_Phdr>(buf.data(), buf_sz);
        } else {
            std::cerr << "ERROR: load_elf: file is neither 32-bit nor 64-bit" << std::endl;
            return false;
        }
    }

    Word Read(Word ip) {
        //return _mem[ToWordAddr(ip)];
        return _mem[GetPhysicalLocation(ip)];
    }

    void Write(Word ip, Word data)
    {
        //_mem[ToWordAddr(ip)] = data;
        _mem[GetPhysicalLocation(ip)] = data;
    }

    void RecordPageFromMainMemoryToVirtualMemory(Word pageNumVirtualForDelete) {
        copy(_mem.begin() + pageTable[pageNumVirtualForDelete][1] * pageSizeW,
             (_mem.begin() + pageTable[pageNumVirtualForDelete][1] * pageSizeW) + pageSizeW,
             virtual_memory.begin() + pageNumVirtualForDelete * pageSizeW);
    }

    void RecordPageFromVirtualMemoryToMainMemory(Word pageNumVirtual) {
        copy(virtual_memory.begin() + pageNumVirtual * pageSizeW,
             (virtual_memory.begin() + pageNumVirtual * pageSizeW) + pageSizeW,
             _mem.begin() + pageTable[pageNumVirtual][1] * pageSizeW);
    }

    void LoadPageIntoMemory(Word pageNumVirtual) {
        if (fifoAlg.getIsMemoryFull()) {
            Word pageNumVirtualForDelete = fifoAlg.getOldestPageNumVirtualFromMainMemory();
            Word pageNumPhysicalForRewrite = pageTable[pageNumVirtualForDelete][1];
            pageTable[pageNumVirtualForDelete][2]  = false;
            fifoAlg.NewRecordInMainMemory(pageNumVirtual);
            pageTable[pageNumVirtual][1] = pageNumPhysicalForRewrite;
            pageTable[pageNumVirtual][2] = true;
            RecordPageFromMainMemoryToVirtualMemory(pageNumVirtualForDelete);
        }
        else {
            pageTable[pageNumVirtual][1] = fifoAlg.getCountFilledPagesInMainMemory();
            pageTable[pageNumVirtual][2] = true;
            fifoAlg.NewRecordInMainMemory(pageNumVirtual);
        }

        RecordPageFromVirtualMemoryToMainMemory(pageNumVirtual);
    }

    Word GetPhysicalLocation(Word virtualLocation) {
        Word pageNumVirtual = ToPageAddr(virtualLocation);
        Word offsetInPage = ToPageOffset(virtualLocation);
        //Word pageNumVirtual = ToPageAddr2(ToWordAddr(virtualLocation));
        //Word offsetInPage = ToPageOffset2(ToWordAddr(virtualLocation));

        if (!pageTable[pageNumVirtual][2]) {
            LoadPageIntoMemory(pageNumVirtual);
        }

        return pageTable[pageNumVirtual][1] * pageSizeW + offsetInPage;
    }

private:
    template <typename Elf_Ehdr, typename Elf_Phdr>
    bool LoadElfSpecific(char *buf, size_t buf_sz) {
        // 64-bit ELF
        Elf_Ehdr *ehdr = (Elf_Ehdr*) buf;
        Elf_Phdr *phdr = (Elf_Phdr*) (buf + ehdr->e_phoff);
        if (buf_sz < ehdr->e_phoff + ehdr->e_phnum * sizeof(Elf_Phdr)) {
            std::cerr << "ERROR: load_elf: file too small for expected number of program header tables" << std::endl;
            return false;
        }
        auto memptr = reinterpret_cast<char*>(_mem.data());
        // loop through program header tables
        for (int i = 0 ; i < ehdr->e_phnum ; i++) {
            if ((phdr[i].p_type == PT_LOAD) && (phdr[i].p_memsz > 0)) {
                if (phdr[i].p_memsz < phdr[i].p_filesz) {
                    std::cerr << "ERROR: load_elf: file size is larger than memory size" << std::endl;
                    return false;
                }
                if (phdr[i].p_filesz > 0) {
                    if (phdr[i].p_offset + phdr[i].p_filesz > buf_sz) {
                        std::cerr << "ERROR: load_elf: file section overflow" << std::endl;
                        return false;
                    }

                    // start of file section: buf + phdr[i].p_offset
                    // end of file section: buf + phdr[i].p_offset + phdr[i].p_filesz
                    // start of memory: phdr[i].p_paddr
                    std::memcpy(memptr + phdr[i].p_paddr, buf + phdr[i].p_offset, phdr[i].p_filesz);
                }
                if (phdr[i].p_memsz > phdr[i].p_filesz) {
                    // copy 0's to fill up remaining memory
                    size_t zeros_sz = phdr[i].p_memsz - phdr[i].p_filesz;
                    std::memset(memptr + phdr[i].p_paddr + phdr[i].p_filesz, 0, zeros_sz);
                }
            }
        }
        return true;
    }

    vector<Word> _mem;
    vector<Word> virtual_memory;
    vector<Word> pageTable[virtualMemorySizeW / pageSizeW]; // [0] - pageNumVirtual, [1] = pageNumPhysical, [2] = validBit
    FifoAlg fifoAlg;
};

class IMem
{
public:
    IMem() = default;
    virtual ~IMem() = default;
    IMem(const IMem &) = delete;
    IMem(IMem &&) = delete;

    IMem& operator=(const IMem&) = delete;
    IMem& operator=(IMem&&) = delete;

    virtual void Request(Word ip) = 0;
    virtual std::optional<Word> Response() = 0;
    virtual void Request(InstructionPtr &instr) = 0;
    virtual bool Response(InstructionPtr &instr) = 0;
    virtual void Clock() = 0;
};

class UncachedMem : public IMem
{
public:
    explicit UncachedMem(MemoryStorage& amem)
        : _mem(amem)
    {

    }

    void Request(Word ip)
    {
        _requestedIp = ip;
        _waitCycles = latency;
    }

    std::optional<Word> Response()
    {
        if (_waitCycles > 0)
            return std::optional<Word>();
        return _mem.Read(_requestedIp);
    }

    void Request(InstructionPtr &instr)
    {
        if (instr->_type != IType::Ld && instr->_type != IType::St)
            return;

        Request(instr->_addr);
    }

    bool Response(InstructionPtr &instr)
    {
        if (instr->_type != IType::Ld && instr->_type != IType::St)
            return true;

        if (_waitCycles != 0)
            return false;

        if (instr->_type == IType::Ld)
            instr->_data = _mem.Read(instr->_addr);
        else if (instr->_type == IType::St)
            _mem.Write(instr->_addr, instr->_data);

        return true;
    }

    void Clock()
    {
        if (_waitCycles > 0)
            --_waitCycles;
    }


private:
    static constexpr size_t latency = 120;
    Word _requestedIp = 0;
    size_t _waitCycles = 0;
    MemoryStorage& _mem;
};

/*class CachedMem : public IMem
{
public:
    explicit CachedMem(MemoryStorage& amem)
            : _mem(amem)
    {

    }

    void Request(Word ip)
    {
        _tag = ToLineAddr(ip) / lineSizeBytes;
        _cached = false;
        _waitCycles = _latency;
        for (auto & iter : _code_cache) {
            if (iter.first == _tag) {
                _line = iter.second;
                _cached = true;
                _waitCycles = 0;
                break;
            }
        }
        _requestedIp = ip;
    }

    std::optional<Word> Response()
    {
        if (_waitCycles > 0)
            return std::optional<Word>();

        if(_cached) {
            size_t offset = ToLineOffset(_requestedIp);
            _code_queue[_tag] = clock();
            return _line[offset];
        }
        std::optional<Word> response = _mem.Read(_requestedIp);
        Line new_line = Line();
        _requestedIp = ToLineAddr(_requestedIp);
        for (size_t i = 0; i < lineSizeWords; i++) {
            new_line[i] = _mem.Read(_requestedIp);
            _requestedIp += 4;
        }
        auto new_record = std::make_pair(_tag, new_line);
        if (_code_cache.size() >= codeCacheBytes / lineSizeBytes)
        {
            auto min = *std::min_element(_code_queue.begin(), _code_queue.end(), CompareSecond());
            _data_cache.erase(min.first);
            _code_queue.erase(min.first);
        }
        _code_cache.push_back(new_record);
        _code_queue.insert({_tag, clock()});
        return response;
    }


    void Request(InstructionPtr &instr)
    {
        if (instr->_type != IType::Ld && instr->_type != IType::St)
            return;

        _tag = ToLineAddr(instr->_addr) / lineSizeBytes;
        _cached = false;
        _waitCycles = _latency;
        if (_data_cache.find(_tag) != _data_cache.end()) {
            _line = _data_cache[_tag].first;
            _cached = true;
            _waitCycles = 3;
        }
        _requestedIp = instr->_addr;
    }

    bool Response(InstructionPtr &instr)
    {
        if (instr->_type != IType::Ld && instr->_type != IType::St)
            return true;

        if (_waitCycles != 0)
            return false;

        if(!_cached) {
            Line new_line = Line();
            Word line_begin = ToLineAddr(_requestedIp);
            for (size_t i = 0; i < lineSizeWords; i++) {
                new_line[i] = _mem.Read(line_begin);
                line_begin += 4;
            }
            std::pair<Line, bool> new_record = std::make_pair(new_line, true);
            if (_data_cache.size() >= dataCacheBytes / lineSizeBytes) {
                auto min = std::min_element(_data_queue.begin(), _data_queue.end(), CompareSecond());
                auto tag_of_min = min->first;

                if (!_data_cache[tag_of_min].second) {
                    size_t ip = tag_of_min * lineSizeBytes;
                    Line line = _data_cache[tag_of_min].first;
                    for (auto iter = line.begin(); iter != line.end(); iter++) {
                        _mem.Write(ip, *iter);
                        ip += 4;
                    }
                }
                _data_cache.erase(tag_of_min);
                _data_queue.erase(tag_of_min);
            }
            _data_cache.insert({_tag, new_record});
            _data_queue.insert({_tag, clock()});
        }

        _data_queue[_tag] = clock();
        if(instr->_type == IType::Ld)
            instr->_data = _data_cache[_tag].first[ToLineOffset(_requestedIp)];
        else {
            _data_cache[_tag].first[ToLineOffset(_requestedIp)] = instr->_data;
            _data_cache[_tag].second = false;
        }
        return true;
    }

    void Clock()
    {
        if (_waitCycles > 0)
            --_waitCycles;
    }


private:
    struct CompareSecond
    {
        bool operator()(const TagClockPair & left, const TagClockPair & right) const
        {
            return left.second < right.second;
        }
    };
    std::map<size_t, clock_t> _code_queue;
    std::map<size_t, clock_t> _data_queue;
    static constexpr size_t _latency = 152;
    Word _requestedIp = 0;
    size_t _waitCycles = 0;

    size_t _tag;
    Line _line;
    MemoryStorage& _mem;
    std::vector<std::pair<size_t, Line>> _code_cache;
    std::map<size_t, std::pair<Line, bool>> _data_cache;
    bool _cached = false;
};*/

#endif //RISCV_SIM_DATAMEMORY_H

