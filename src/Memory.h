
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
static constexpr size_t mainMemorySizeW = 1024*1024; // memory size in 4-byte words
static constexpr size_t virtualMemorySizeW = 1024*1024*4;
static constexpr size_t pageSizeW = 1024;
static constexpr size_t pageSizeB = pageSizeW * 4;

static constexpr size_t lineSizeBytes = 128;
static constexpr size_t lineSizeWords = lineSizeBytes / sizeof(Word);
using Line = std::array<Word, lineSizeWords>;
using TagClockPair = std::pair<size_t , clock_t>;
static constexpr size_t dataCacheBytes = 2048; // data cache size in bytes
static constexpr size_t codeCacheBytes = 1024; // instructions cache size in bytes
static Word ToWordAddr(Word addr) { return addr >> 2u; }
static Word ToLineAddr(Word addr) { return addr & ~(lineSizeBytes - 1); }
static Word ToLineOffset(Word addr) { return ToWordAddr(addr) & (lineSizeWords - 1); }

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
    vector<Word> queueOfRecordsInMainMemory;
    Word countFilledPagesInMainMemory;
    bool isMemoryFull;

public:
    FifoAlg() {
        queueOfRecordsInMainMemory.resize(mainMemorySizeW / pageSizeW);
        countFilledPagesInMainMemory = 0;
        isMemoryFull = false;
    }

    void NewRecordInMainMemory(Word pageNumVirtual) {
        if (countFilledPagesInMainMemory < queueOfRecordsInMainMemory.size()) {
            queueOfRecordsInMainMemory[countFilledPagesInMainMemory] = pageNumVirtual;
            countFilledPagesInMainMemory++;

            if (countFilledPagesInMainMemory == queueOfRecordsInMainMemory.size()) {
                isMemoryFull = true;
            }
        }
        else {
            queueOfRecordsInMainMemory.erase(queueOfRecordsInMainMemory.begin());
            queueOfRecordsInMainMemory.push_back(pageNumVirtual);
        }
    }

    Word getOldestPageNumVirtualFromMainMemory() {
        return queueOfRecordsInMainMemory[0];
    }

    bool getIsMemoryFull() {
        return isMemoryFull;
    }

    Word IndexEmptyPageInMainMemory() {
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
            pageTable[i].push_back(0);
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

    void RecordPageFromMainMemoryToVirtualMemory(Word pageNumVirtualForRewrite) {
        copy(_mem.begin() + pageTable[pageNumVirtualForRewrite][1] * pageSizeW,
             (_mem.begin() + pageTable[pageNumVirtualForRewrite][1] * pageSizeW) + pageSizeW,
             virtual_memory.begin() + pageNumVirtualForRewrite * pageSizeW);
    }

    void RecordPageFromVirtualMemoryToMainMemory(Word pageNumVirtual) {
        copy(virtual_memory.begin() + pageNumVirtual * pageSizeW,
             (virtual_memory.begin() + pageNumVirtual * pageSizeW) + pageSizeW,
             _mem.begin() + pageTable[pageNumVirtual][1] * pageSizeW);
    }

    void LoadPageIntoMemory(Word pageNumInVirtualMemory) {
        pageTable[pageNumInVirtualMemory][2] = true;

        if (fifoAlg.getIsMemoryFull()) {
            Word pageNumVirtualForRewrite = fifoAlg.getOldestPageNumVirtualFromMainMemory();
            Word pageNumPhysicalForRewrite = pageTable[pageNumVirtualForRewrite][1];

            pageTable[pageNumVirtualForRewrite][2]  = false;

            fifoAlg.NewRecordInMainMemory(pageNumInVirtualMemory);

            pageTable[pageNumInVirtualMemory][1] = pageNumPhysicalForRewrite;

            RecordPageFromMainMemoryToVirtualMemory(pageNumVirtualForRewrite);
        }
        else {
            pageTable[pageNumInVirtualMemory][1] = fifoAlg.IndexEmptyPageInMainMemory();
            fifoAlg.NewRecordInMainMemory(pageNumInVirtualMemory);
        }

        RecordPageFromVirtualMemoryToMainMemory(pageNumInVirtualMemory);
    }

    Word GetPhysicalLocation(Word virtualLocation) {
        Word pageNumInVirtualMemory = ToPageAddr(virtualLocation);
        Word offsetInPage = ToPageOffset(virtualLocation);
        //Word pageNumInVirtualMemory = ToPageAddr2(ToWordAddr(virtualLocation));
        //Word offsetInPage = ToPageOffset2(ToWordAddr(virtualLocation));

        if (pageTable[pageNumInVirtualMemory][2] == false) {
            LoadPageIntoMemory(pageNumInVirtualMemory);
        }

        return pageTable[pageNumInVirtualMemory][1] * pageSizeW + offsetInPage;
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
    vector<Word> pageTable[virtualMemorySizeW / pageSizeW]; // [0] - pageNumInVirtualMemory, [1] = pageNumInMainMemory, [2] = validBit
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

    void Request(Word ip)                                   // sending request to fetch word
    {
        _requestedIp = ip;
        _waitCycles = latency;
    }

    std::optional<Word> Response()                          // fetching word from requested ip - used for fetching instructions
    {
        if (_waitCycles > 0)
            return std::optional<Word>();
        return _mem.Read(_requestedIp);
    }

    void Request(InstructionPtr &instr)                     // if instruction is load/store type - set requested_ip to instr->addr from instruction
    {
        if (instr->_type != IType::Ld && instr->_type != IType::St)
            return;

        Request(instr->_addr);
    }

    bool Response(InstructionPtr &instr)                    // checks whether the instruction is executed yet or not
    {
        if (instr->_type != IType::Ld && instr->_type != IType::St)  // if instruction type is not load or store - it's executed immediately, thus instruction is executed already
            return true;

        if (_waitCycles != 0)                               // if there are cycles to wait for - instruction is not executed yet
            return false;

        if (instr->_type == IType::Ld)                      // perform necessary operations and return true
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
        _tag = ToLineAddr(ip) / lineSizeBytes;                      // evaluating _tag value
        _cached = false;
        _waitCycles = _latency;
        for (auto & iter : _code_cache) {
            if (iter.first == _tag) {                               // if corresponding tag is found
                _line = iter.second;                                // get whole line from cache
                _cached = true;                                     // set cached to true
                _waitCycles = 0;                                    // instructions are fetched from cache immediately
                break;
            }
        }
        _requestedIp = ip;
    }

    std::optional<Word> Response()                                  // fetching instruction
    {
        if (_waitCycles > 0)                                        // if not ready yet - return empty instruction
            return std::optional<Word>();

        if(_cached) {                                               // if instr is cached - return requested value from cache
            size_t offset = ToLineOffset(_requestedIp);
            _code_queue[_tag] = clock();
            return _line[offset];
        }
        std::optional<Word> response = _mem.Read(_requestedIp);     // if instr is not cached - return from memory like usual but
        Line new_line = Line();                                     // put the whole line in which instruction lives in cache
        _requestedIp = ToLineAddr(_requestedIp);                    // set requested ip to line beginning
        for (size_t i = 0; i < lineSizeWords; i++) {                // fill the line with 32 instructions
            new_line[i] = _mem.Read(_requestedIp);
            _requestedIp += 4;
        }
        auto new_record = std::make_pair(_tag, new_line);           // create new cache record assigning _tag value to its tag
        if (_code_cache.size() >= codeCacheBytes / lineSizeBytes)   // if cache is full
        {
            auto min = *std::min_element(_code_queue.begin(), _code_queue.end(), CompareSecond()); // find line with less usage in queue containing tag-clock pair
            _data_cache.erase(min.first);                           // after data is saved to memory from cache it can be removed from cache
            _code_queue.erase(min.first);
        }
        _code_cache.push_back(new_record);                          // push new record in cache and to queue
        _code_queue.insert({_tag, clock()});
        return response;
    }


    void Request(InstructionPtr &instr)                             // if instruction is load/store type - set requested_ip to instr->addr from instruction
    {
        if (instr->_type != IType::Ld && instr->_type != IType::St)
            return;

        _tag = ToLineAddr(instr->_addr) / lineSizeBytes;            // evaluating _tag value
        _cached = false;
        _waitCycles = _latency;
        if (_data_cache.find(_tag) != _data_cache.end()) {          // if record tagged with _tag exists
            _line = _data_cache[_tag].first;                        // get line of data from cache
            _cached = true;                                         // set cached to true
            _waitCycles = 3;                                        // set latency
        }
        _requestedIp = instr->_addr;                                // set requested ip to data addr in instruction
    }

    bool Response(InstructionPtr &instr)                            // checks whether the instruction is executed yet or not
    {
        if (instr->_type != IType::Ld && instr->_type != IType::St) // if instruction type is not load or store - it's executed immediately, thus instruction is executed already
            return true;

        if (_waitCycles != 0)                                       // if there are cycles to wait for - instruction is not executed yet
            return false;

        if(!_cached) {
            Line new_line = Line();
            Word line_begin = ToLineAddr(_requestedIp);
            for (size_t i = 0; i < lineSizeWords; i++) {            // fill the line with 32 words from memory
                new_line[i] = _mem.Read(line_begin);
                line_begin += 4;
            }
            std::pair<Line, bool> new_record = std::make_pair(new_line, true);
            if (_data_cache.size() >= dataCacheBytes / lineSizeBytes) {
                auto min = std::min_element(_data_queue.begin(), _data_queue.end(), CompareSecond()); // find line with less usage in queue2 containing tag-clock pair
                auto tag_of_min = min->first;

                if (!_data_cache[tag_of_min].second) {
                    size_t ip = tag_of_min * lineSizeBytes;
                    Line line = _data_cache[tag_of_min].first;
                    for (auto iter = line.begin(); iter != line.end(); iter++) {            // iterate over line and write 32 words to memory
                        _mem.Write(ip, *iter);
                        ip += 4;
                    }
                }
                _data_cache.erase(tag_of_min);                                              // after data is saved to memory from cache it can be removed from cache
                _data_queue.erase(tag_of_min);
            }
            _data_cache.insert({_tag, new_record});
            _data_queue.insert({_tag, clock()});
        }

        _data_queue[_tag] = clock();
        if(instr->_type == IType::Ld)
            instr->_data = _data_cache[_tag].first[ToLineOffset(_requestedIp)];             // get data from cache
        else {
            _data_cache[_tag].first[ToLineOffset(_requestedIp)] = instr->_data;             // alter data in cache
            _data_cache[_tag].second = false;                                               // data is not fresh anymore(ew)
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
    std::map<size_t, clock_t> _code_queue;                           // key - _tag, value - last time accessed
    std::map<size_t, clock_t> _data_queue;                           // key - _tag, value - last time accessed
    static constexpr size_t _latency = 152;
    Word _requestedIp = 0;
    size_t _waitCycles = 0;

    size_t _tag;
    Line _line;
    MemoryStorage& _mem;
    std::vector<std::pair<size_t, Line>> _code_cache;               // pair of tag and line
    std::map<size_t, std::pair<Line, bool>> _data_cache;            // tag, line and flag indicating whether thr record is fresh or not
    bool _cached = false;
};*/

#endif //RISCV_SIM_DATAMEMORY_H

