/*
 * Copyright (c) 2011-2013,2018 ARM Limited
 * All rights reserved
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
 * Authors: Gabe Black
 */

#include "arch/arm/insts/mem64.hh"

#include "arch/arm/tlb.hh"
#include "base/loader/symtab.hh"
#include "mem/request.hh"

using namespace std;

namespace ArmISA
{

std::string
SysDC64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, ", ");
    printIntReg(ss, base);
    return ss.str();
}



void
Memory64::startDisassembly(std::ostream &os) const
{
    printMnemonic(os, "", false);
    if (isDataPrefetch()||isInstPrefetch()){
        printPFflags(os, dest);
    }else{
        printIntReg(os, dest);
    }
    ccprintf(os, ", [");
    printIntReg(os, base);
}

void
Memory64::startDisassembly(std::ostream &os, int rd_width) const
{
    printMnemonic(os, "", false);
    if (isDataPrefetch()||isInstPrefetch()){
        printPFflags(os, dest);
    }else{
        printIntReg(os, dest, rd_width);
    }
    ccprintf(os, ", [");
    printIntReg(os, base, 64);
}

void
Memory64::setExcAcRel(bool exclusive, bool acrel)
{
    if (exclusive)
        memAccessFlags |= Request::LLSC;
    else
        memAccessFlags |= ArmISA::TLB::AllowUnaligned;
    if (acrel) {
        flags[IsMemBarrier] = true;
        flags[IsWriteBarrier] = true;
        flags[IsReadBarrier] = true;
    }
}

std::string
MemoryImm64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    uint32_t size = bits(machInst, 31, 30);
    uint32_t opc = bits(machInst, 23, 22);
    int rd_width = (size == 0x3 || opc == 0x2) ? 64 : 32;
    startDisassembly(ss, rd_width);
    if (imm)
        ccprintf(ss, ", #%d", imm);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryDImm64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, dest);
    ccprintf(ss, ", ");
    printIntReg(ss, dest2);
    ccprintf(ss, ", [");
    printIntReg(ss, base);
    if (imm)
        ccprintf(ss, ", #%d", imm);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryDImmEx64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, result);
    ccprintf(ss, ", ");
    printIntReg(ss, dest);
    ccprintf(ss, ", ");
    printIntReg(ss, dest2);
    ccprintf(ss, ", [");
    printIntReg(ss, base);
    if (imm)
        ccprintf(ss, ", #%d", imm);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryPreIndex64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    uint32_t size = bits(machInst, 31, 30);
    uint32_t opc = bits(machInst, 23, 22);
    int rd_width = (size == 0x3 || opc == 0x2) ? 64 : 32;
    startDisassembly(ss, rd_width);
    ccprintf(ss, ", #%d]!", imm);
    return ss.str();
}

std::string
MemoryPostIndex64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    uint32_t size = bits(machInst, 31, 30);
    uint32_t opc = bits(machInst, 23, 22);
    int rd_width = (size == 0x3 || opc == 0x2) ? 64 : 32;
    startDisassembly(ss, rd_width);
    ccprintf(ss, "], #%d", imm);
    return ss.str();
}

void
MemoryReg64::printExtendOperand(bool firstOperand, std::ostream &os,
                                IntRegIndex rm, ArmExtendType type,
                                int64_t shiftAmt, int rm_width) const
{
    if (!firstOperand)
        ccprintf(os, ", ");
    printIntReg(os, rm, rm_width);
    if (type == UXTX && shiftAmt == 0)
        return;
    switch (type) {
      case UXTB: ccprintf(os, ", UXTB");
        break;
      case UXTH: ccprintf(os, ", UXTH");
        break;
      case UXTW: ccprintf(os, ", UXTW");
        break;
      case UXTX: ccprintf(os, ", LSL");
        break;
      case SXTB: ccprintf(os, ", SXTB");
        break;
      case SXTH: ccprintf(os, ", SXTH");
        break;
      case SXTW: ccprintf(os, ", SXTW");
        break;
      case SXTX: ccprintf(os, ", SXTX");
        break;
    }
    if (type == UXTX || shiftAmt)
        ccprintf(os, " #%d", shiftAmt);
}

std::string
MemoryReg64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    uint32_t size = bits(machInst, 31, 30);
    uint32_t opc = bits(machInst, 23, 22);
    uint32_t option = bits(machInst, 15, 13);
    int rd_width = (size == 0x3 || opc == 0x2) ? 64 : 32;
    int rm_width = (option & 0x1) ? 64 : 32;
    startDisassembly(ss, rd_width);
    printExtendOperand(false, ss, offset, type, shiftAmt, rm_width);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryRaw64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    startDisassembly(ss);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryEx64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printIntReg(ss, dest);
    ccprintf(ss, ", ");
    printIntReg(ss, result);
    ccprintf(ss, ", [");
    printIntReg(ss, base);
    ccprintf(ss, "]");
    return ss.str();
}

std::string
MemoryLiteral64::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    uint32_t opc = bits(machInst, 31, 30);
    int rd_width = (opc == 0x0) ? 32 : 64;
    printMnemonic(ss, "", false);
    if (isDataPrefetch()||isInstPrefetch()){
        printPFflags(ss, dest);
    }else{
        printIntReg(ss, dest, rd_width);
    }
    ccprintf(ss, ", #%d", pc + imm);
    return ss.str();
}
}
