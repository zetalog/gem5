/*
 * Copyright (c) 2009 The Regents of The University of Michigan
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2014 Sven Karlsson
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
 * All rights reserved.
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
 * Authors: Gabe Black
 *          Timothy M. Jones
 *          Sven Karlsson
 *          Alec Roelke
 */

#ifndef __ARCH_RISCV_ISA_HH__
#define __ARCH_RISCV_ISA_HH__

#include <map>
#include <string>

#include "arch/riscv/registers.hh"
#include "arch/riscv/tlb.hh"
#include "arch/riscv/types.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "cpu/reg_class.hh"
#include "sim/sim_object.hh"

struct RiscvISAParams;
class ThreadContext;
class BaseCPU;
class Checkpoint;
class EventManager;

namespace RiscvISA
{

enum PrivilegeMode {
    PRV_U = 0,
    PRV_S = 1,
    PRV_M = 3
};

class ISA : public SimObject
{
  protected:
    std::vector<RegVal> miscRegFile;

    bool hpmCounterEnabled(int counter) const;

  public:
    typedef RiscvISAParams Params;

    void clear();

    RegVal readMiscRegNoEffect(int misc_reg) const;
    RegVal readMiscReg(int misc_reg, ThreadContext *tc);
    void setMiscRegNoEffect(int misc_reg, RegVal val);
    void setMiscReg(int misc_reg, RegVal val, ThreadContext *tc);

    RegId flattenRegId(const RegId &regId) const { return regId; }
    int flattenIntIndex(int reg) const { return reg; }
    int flattenFloatIndex(int reg) const { return reg; }
    int flattenVecIndex(int reg) const { return reg; }
    int flattenVecElemIndex(int reg) const { return reg; }
    int flattenVecPredIndex(int reg) const { return reg; }
    int flattenCCIndex(int reg) const { return reg; }
    int flattenMiscIndex(int reg) const { return reg; }

    void startup(ThreadContext *tc) {}

    // Dump register context
    void dumpGenRegStore(BaseCPU *cpu, ThreadContext *tc) {}
    void dumpGenRegLoad(BaseCPU *cpu, ThreadContext *tc) {}
    void dumpMiscRegStore(BaseCPU *cpu, ThreadContext *tc) {}
    void dumpMiscRegLoad(BaseCPU *cpu, ThreadContext *tc) {}
    void dumpContextRegsEarly(BaseCPU *cpu, ThreadContext *tc) {}
    void dumpContextRegsLate(BaseCPU *cpu, ThreadContext *tc) {}
    void dumpStackStore(BaseCPU *cpu, ThreadContext *tc,
        bool (*__readMem)(BaseCPU *cpu, Addr, uint8_t *, unsigned,
                          Request::Flags)) {}
    void dumpStackLoad(BaseCPU *cpu, ThreadContext *tc) {}
    void dumpMemPagePrefix(BaseCPU *cpu, int page_cnt) {}
    void dumpMemPageBegin(BaseCPU *cpu, Addr addr) {}
    void dumpMemPageEnd(BaseCPU *cpu, Addr addr) {}
    void dumpMemZeroBytes(BaseCPU *cpu, Addr size) {}
    void dumpMemOneByte(BaseCPU *cpu, uint8_t data) {}
    void dumpPteGenBegin(BaseCPU *cpu, int page_cnt) {}
    void dumpPteGen(BaseCPU *cpu, Addr virt_addr, Addr phys_offset) {}
    void dumpPteGenEnd(BaseCPU *cpu) {}
    void dumpMemBegin(BaseCPU *cpu) {}
    void dumpMemU64(BaseCPU *cpu, Addr addr, uint64_t value) {}
    void dumpMemEnd(BaseCPU *cpu) {}
    // Dump init of simpoint
    void dumpSimpointInit(BaseCPU *cpu) {}
    // Dump exit of simpoint
    void dumpSimpointExit(BaseCPU *cpu) {}
    // Dump start of simpoint
    void dumpSimpointStart(BaseCPU *cpu) {}
    // Dump stop of simpoint
    void dumpSimpointStop(BaseCPU *cpu) {}

    /// Explicitly import the otherwise hidden startup
    using SimObject::startup;

    const Params *params() const;

    ISA(Params *p);
};

} // namespace RiscvISA

#endif // __ARCH_RISCV_ISA_HH__
