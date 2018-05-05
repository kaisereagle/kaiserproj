#ifndef TYPE_DEFINITIONS_H
#define TYPE_DEFINITIONS_H

///This file provides some platform/compiler checks for common definitions

#ifdef WIN32

typedef union
{
  unsigned int u;
  void *p;
} D_addr64;

#define D_USE_WIN32_THREADING 1

		#if defined(__MINGW32__) || defined(__CYGWIN__) || (defined (_MSC_VER) && _MSC_VER < 1300)
		#else
		#endif //__MINGW32__

		typedef unsigned char     D_uint8_t;
#ifndef __PHYSICS_COMMON_H__
		typedef unsigned long int D_uint64_t;
		typedef unsigned int      D_uint32_t;
#endif //__PHYSICS_COMMON_H__
		typedef unsigned short    D_uint16_t;

		#include <malloc.h>
		#define D_memalign(alignment, size) malloc(size);
			
#include <string.h> //memcpy

		

		#include <stdio.h>		
		#define D_spu_printf printf
		
#else
		#include <stdint.h>
		#include <stdlib.h>
		#include <string.h> //for memcpy

#if defined	(__CELLOS_LV2__)
	// Playstation 3 Cell SDK
#include <D_spu_printf.h>
		
#else
	// posix system

#define D_USE_PTHREADS    (1)

#ifdef USE_LIBSPE2
#include <stdio.h>		
#define D_spu_printf printf	
#define DWORD unsigned int
		
			typedef union
			{
			  unsigned long long ull;
			  unsigned int ui[2];
			  void *p;
			} D_addr64;
		
		
#else

#include <stdio.h>		
#define D_spu_printf printf	

#endif // USE_LIBSPE2
	
#endif	//__CELLOS_LV2__
	
#endif


/* Included here because we D_need D_uint*_t typedefs */
#include "PpuAddressSpace.h"

#endif //TYPE_DEFINITIONS_H



