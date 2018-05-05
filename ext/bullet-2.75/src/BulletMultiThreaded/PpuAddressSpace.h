#ifndef __PPU_ADDRESS_SPACE_H
#define __PPU_ADDRESS_SPACE_H


#ifdef WIN32
//stop those casting warnings until we have a better solution for D_ppu_address_t / void* / uint64 conversions
#pragma warning (disable: 4311)
#pragma warning (disable: 4312)
#endif //WIN32

#ifdef USE_ADDR64
typedef D_uint64_t D_ppu_address_t;
#else
typedef D_uint32_t D_ppu_address_t;
#endif

#endif

