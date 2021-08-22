#include "com_helper.h"

static unsigned int ElfHash(const unsigned char *s, unsigned int len)
{
    unsigned int hash = 0, high;
    unsigned int i = 0;
    for (i = 0; i < len; i++)
    {
        hash = (hash << 4) + (*s++);
        if ((high = hash & 0xF0000000) != 0)
        {
            hash ^= (high >> 24);
        }
        hash &= ~high;
    }
    return hash;
}