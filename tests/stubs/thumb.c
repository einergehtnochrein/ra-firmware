
#include "lpclib.h"

uint32_t __RBIT (uint32_t x)
{
    uint32_t rbit = 0;

    for (int i = 0; i < 32; i++) {
        if (x & (1u << i)) {
            rbit |= (1u << (31 - i));
        }
    }

    return rbit;
}
