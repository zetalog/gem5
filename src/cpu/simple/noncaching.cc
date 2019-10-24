/*
 * Copyright (c) 2012, 2018 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Andreas Sandberg
 */

#include "cpu/simple/noncaching.hh"
#include "base/mem_page.hh"

using namespace TheISA;

NonCachingSimpleCPU::NonCachingSimpleCPU(NonCachingSimpleCPUParams *p)
    : AtomicSimpleCPU(p)
{
}

void
NonCachingSimpleCPU::verifyMemoryMode() const
{
    if (!(system->isAtomicMode() && system->bypassCaches())) {
        fatal("The direct CPU requires the memory system to be in the "
              "'atomic_noncaching' mode.\n");
    }
}

Tick
NonCachingSimpleCPU::sendPacket(MasterPort &port, const PacketPtr &pkt)
{
    if (system->isMemAddr(pkt->getAddr())) {
        system->getPhysMem().access(pkt);
        return 0;
    } else {
        return port.sendAtomic(pkt);
    }
}

NonCachingSimpleCPU *
NonCachingSimpleCPUParams::create()
{
    numThreads = 1;
    if (!FullSystem && workload.size() != 1)
        fatal("only one workload allowed");
    return new NonCachingSimpleCPU(this);
}

void
NonCachingSimpleCPU::dumpSimulatedPages()
{
    std::set<MemPage> page_set;
    Addr page_addr;
    Addr page_offset;
    uint8_t *page_data_buf = NULL;
    std::set<MemPage>::iterator page_it;
    std::pair<std::set<MemPage>::iterator, bool> ret;

    /* Merge memory reads and writes into a page set.
       If one position in page is read more than once, then the value of the
       first read is recored. */
    for (auto &item : reads) {
        page_addr = item.addr & MEM_PAGE_MASK;
        page_offset = item.addr & (~MEM_PAGE_MASK);
        MemPage curr_page(page_addr);
        ret = page_set.insert(curr_page);
        page_data_buf = (uint8_t *)(*ret.first).data;
        for (int i = 0; i < item.size; i++) {
            uint8_t byte_value = *((uint8_t *)&item.value + i);
            if (page_data_buf[page_offset + i] == 0)
                page_data_buf[page_offset + i] = byte_value;
        }
    }
    for (auto &item : writes) {
        page_addr = item.addr & MEM_PAGE_MASK;
        MemPage curr_page(page_addr);
        page_set.insert(curr_page);
    }

    /* Dump pages in the order of address growth. */
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;
    std::set<MemPage>::iterator it;
    thread->getIsaPtr()->dumpMemPagePrefix(this, page_set.size());
    for (it = page_set.begin(); it != page_set.end(); ++it) {
        page_addr = (*it).addr;
        page_data_buf = (uint8_t *)(*it).data;
        thread->getIsaPtr()->dumpMemPageBegin(this, page_addr);
        int i = 0;
        int zero_byte_cnt = 0;
        do {
            if (page_data_buf[i] != 0) {
                zero_byte_cnt = 0;
                thread->getIsaPtr()->dumpMemOneByte(this, page_data_buf[i]);
                i++;
                continue;
            }
            zero_byte_cnt = 1;
            while (i + zero_byte_cnt < MEM_PAGE_SIZE) {
                if (page_data_buf[i + zero_byte_cnt] == 0)
                    zero_byte_cnt++;
                else
                    break;
            }
            thread->getIsaPtr()->dumpMemZeroBytes(this, zero_byte_cnt);
            i += zero_byte_cnt;
        } while (i < MEM_PAGE_SIZE);
        thread->getIsaPtr()->dumpMemPageEnd(this, page_addr);
    }

    /* Dump one fucntion for generating page table entry for all pages. */
    Addr phys_offset = 0;
    thread->getIsaPtr()->dumpPteGenBegin(this, page_set.size());
    for (it = page_set.begin(); it != page_set.end(); ++it) {
        page_addr = (*it).addr;
        thread->getIsaPtr()->dumpPteGen(this, page_addr, phys_offset);
                phys_offset += MEM_PAGE_SIZE;
    }
    thread->getIsaPtr()->dumpPteGenEnd(this);

    /* Dump one function for restoring heap memory pages. */
    thread->getIsaPtr()->dumpMemBegin(this);
    for (it = page_set.begin(); it != page_set.end(); ++it) {
        page_addr = (*it).addr;
                for (Addr offset = 0; offset < MEM_PAGE_SIZE; offset += 8) {
            uint64_t value = *(uint64_t*)((uint8_t *)(*it).data + offset);
            if (value == 0) {
                continue;
            }
            thread->getIsaPtr()->dumpMemU64(this, page_addr + offset, value);
        }
    }
    thread->getIsaPtr()->dumpMemEnd(this);
}
void
NonCachingSimpleCPU::dumpSimulatedContexts()
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    std::cout << "Dumping contexts..." << std::endl;
    thread->getIsaPtr()->dumpContextRegsLate(this, thread);
    thread->getIsaPtr()->dumpSimpointStart(this);
}

void
NonCachingSimpleCPU::dumpSimulatedSymbols()
{
    if (!debugSymbolTable || !simpoint_asm.is_open())
        return;

    if (curThread != 0)
        fatal("Execution disassembly currently does not support SMT");

    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;
    TheISA::Decoder *decoder = &(thread->decoder);
    SymbolTable *symtab = debugSymbolTable;

    int symbol_cnt = 0;
    int i;
    std::string sym_str;
    std::string lab_str;
    Addr funcStart, funcEnd, addr, target;
    StaticInstPtr instPtr;
    std::string disassembly;
    TheISA::PCState pc;
    bool doDumpSymbols = false;
    bool doDumpNormal = false;

realDump:
    i = 0;
    symbol_cnt = symbols.size();
    while (i < symbol_cnt) {
        if (!symtab->findNearestSymbol(symbols[i], sym_str,
                                       funcStart, funcEnd))
            return;
        if (funcStart != symbols[i])
            return;

        if (doDumpSymbols) {
            if (sym_str == "exit")
                doDumpNormal = false;
            else
                doDumpNormal = true;
        }
        if (doDumpNormal)
            simpoint_asm << std::endl << sym_str << ":" << std::endl;
        addr = funcStart;
        pc = thread->pcState();
        pc.set(addr);
        thread->pcState(pc);
        t_info.fetchOffset = 0;

        while (addr < funcEnd) {
            if (doDumpSymbols && simpoint_entry == addr) {
                simpoint_asm << "simpoint_start:" << std::endl;
                if (!doDumpNormal)
                    thread->getIsaPtr()->dumpSimpointStop(this);
            }
            if (doDumpNormal && symtab->findTarget(addr, lab_str))
                simpoint_asm << lab_str << ":" << std::endl;

            Fault fault = NoFault;
            ifetch_req->taskId(taskId());
            setupFetchRequest(ifetch_req);
            fault = thread->itb->translateAtomic(ifetch_req,
                                                 thread->getTC(),
                                                 BaseTLB::Execute);
            if (fault == NoFault) {
                Packet ifetch_pkt = Packet(ifetch_req, MemCmd::ReadReq);
                ifetch_pkt.dataStatic(&inst);
                sendPacket(icachePort, &ifetch_pkt);
                gtoh(inst);

                Addr fetchPC = (addr & PCMask) + t_info.fetchOffset;
                decoder->moreBytes(pc, fetchPC, inst);
                instPtr = decoder->decode(pc);
                if (instPtr) {
                    t_info.stayAtPC = false;
                    thread->pcState(pc);
                } else {
                    fatal("Fetching MicroOP?");
                    t_info.stayAtPC = true;
                    t_info.fetchOffset += sizeof(MachInst);
                }

                if (doDumpNormal) {
                    disassembly = instPtr->disassemble(addr, symtab, true);
                    simpoint_asm << disassembly << std::endl;
                } else {
                    bool ret1 = instPtr->markTarget(addr, target, symtab);
                    bool ret2 = symtab->findLabel(target, sym_str);
                    if (ret1 && ret2)
                        markBranched(target);
                }
            }
            if (fault != NoFault || !t_info.stayAtPC)
                advancePC(fault);
            pc = thread->pcState();
            addr = pc.instAddr();
        }
        i++;
    }
    if (!doDumpSymbols) {
        doDumpSymbols = true;
        dumpSimulatedContexts();
        goto realDump;
    }
    simpoint_asm << std::endl;

    thread->getIsaPtr()->dumpSimpointStop(this);

    simpoint_asm << "/* Branch targets not executed */" << std::endl;
    for (auto b : branches) {
        if (!symtab->findNearestSymbol(b, sym_str, funcStart, funcEnd))
            continue;
        if (funcStart != b)
            continue;
        for (auto s : symbols) {
            if (s == b) {
                simpoint_asm << "// ";
                break;
            }
        }
        if (sym_str == "exit")
            continue;
        simpoint_asm << sym_str << ":" << std::endl;
    }
    simpoint_asm << std::endl;
    thread->getIsaPtr()->dumpSimpointExit(this);
    dumpSimulatedPages();
}

static bool
readMem(BaseCPU *cpu, Addr addr, uint8_t *data,
        unsigned size, Request::Flags flags)
{
    Fault fault = NoFault;
    NonCachingSimpleCPU *this_cpu = dynamic_cast<NonCachingSimpleCPU *>(cpu);

    if (!this_cpu)
        return false;
    fault = this_cpu->readMem(addr, data, size, flags);
    return fault == NoFault ? true : false;
}

void
NonCachingSimpleCPU::dumpSimulatedRegisters()
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    std::cout << "Dumping registers..." << std::endl;
    thread->getIsaPtr()->dumpContextRegsEarly(this, thread);
    thread->getIsaPtr()->dumpStackStore(this, thread, ::readMem);
    thread->getIsaPtr()->dumpSimpointInit(this);
    thread->getIsaPtr()->dumpStackLoad(this, thread);
}
