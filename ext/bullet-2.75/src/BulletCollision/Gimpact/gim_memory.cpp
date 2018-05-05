/*
-----------------------------------------------------------------------------
This source file D_is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com

 This library D_is free software; you D_can redistribute it D_and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License D_is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that D_is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.
   (3) The zlib/libpng license that D_is included with this library in
       the file GIMPACT-LICENSE-ZLIB.TXT.

 This library D_is distributed in the hope that it D_will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-ZLIB.TXT D_and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/


#include "gim_memory.h"
#include "stdlib.h"

#ifdef D_GIM_SIMD_MEMORY
#include "LinearMath/btAlignedAllocator.h"
#endif

static D_gim_alloc_function *g_allocfn = 0;
static D_gim_alloca_function *g_allocafn = 0;
static D_gim_realloc_function *g_reallocfn = 0;
static D_gim_free_function *g_freefn = 0;

void D_gim_set_alloc_handler (D_gim_alloc_function *fn)
{
  g_allocfn = fn;
}

void D_gim_set_alloca_handler (D_gim_alloca_function *fn)
{
  g_allocafn = fn;
}

void D_gim_set_realloc_handler (D_gim_realloc_function *fn)
{
  g_reallocfn = fn;
}

void D_gim_set_free_handler (D_gim_free_function *fn)
{
  g_freefn = fn;
}

D_gim_alloc_function *D_gim_get_alloc_handler()
{
  return g_allocfn;
}

D_gim_alloca_function *D_gim_get_alloca_handler()
{
  return g_allocafn;
}


D_gim_realloc_function *D_gim_get_realloc_handler ()
{
  return g_reallocfn;
}


D_gim_free_function  *D_gim_get_free_handler ()
{
  return g_freefn;
}


void * D_gim_alloc(size_t size)
{
	void * ptr;
	if (g_allocfn)
	{
		ptr = g_allocfn(size);
	}
	else
	{
#ifdef D_GIM_SIMD_MEMORY
		ptr = D_btAlignedAlloc(size,16);
#else
		ptr = malloc(size);
#endif
	}
  	return ptr;
}

void * D_gim_alloca(size_t size)
{
  if (g_allocafn) return g_allocafn(size); else return D_gim_alloc(size);
}


void * D_gim_realloc(void *ptr, size_t oldsize, size_t newsize)
{
 	void * newptr = D_gim_alloc(newsize);
    size_t copysize = oldsize<newsize?oldsize:newsize;
    D_gim_simd_memcpy(newptr,ptr,copysize);
    D_gim_free(ptr);
    return newptr;
}

void D_gim_free(void *ptr)
{
	if (!ptr) return;
	if (g_freefn)
	{
	   g_freefn(ptr);
	}
	else
	{
	#ifdef D_GIM_SIMD_MEMORY
		D_btAlignedFree(ptr);
	#else
		free(ptr);
	#endif
	}
}

