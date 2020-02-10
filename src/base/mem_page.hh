#ifndef __CPU_MEM_PAGE_HH__
#define __CPU_MEM_PAGE_HH__

#include <memory>
#include <set>

#include "arch/types.hh"
#include "base/types.hh"

#define MEM_PAGE_SHIFT (12)
#define MEM_PAGE_SIZE (((Addr)1) << MEM_PAGE_SHIFT)
#define MEM_PAGE_MASK (~(MEM_PAGE_SIZE-1))

class MemPage
{
  public:
    uint8_t data[MEM_PAGE_SIZE];
    Addr addr;

    MemPage(Addr _addr) : addr(_addr)
    {
        memset(data, 0, sizeof(data));
    }

    bool operator< (const MemPage &page) const
    {
        return (this->addr < page.addr);
    }
};

#endif /* __CPU_MEM_PAGE_HH__ */
