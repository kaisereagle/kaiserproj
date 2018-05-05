#ifndef D_GIM_MEMORY_H_INCLUDED
#define D_GIM_MEMORY_H_INCLUDED
/*! \file D_gim_memory.h
\author Francisco Len Nßjera
*/
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


#include "gim_math.h"
#include <string.h>

#ifdef PREFETCH
#include <xmmintrin.h>	// for prefetch
#define D_pfval	64
#define D_pfval2	128
//! Prefetch 64
#define D_pf(_x,_i)	_mm_prefetch((void *)(_x + _i + D_pfval), 0)
//! Prefetch 128
#define D_pf2(_x,_i)	_mm_prefetch((void *)(_x + _i + D_pfval2), 0)
#else
//! Prefetch 64
#define D_pf(_x,_i)
//! Prefetch 128
#define D_pf2(_x,_i)
#endif


///Functions for manip packed arrays of numbers
#define D_GIM_COPY_ARRAYS(dest_array,source_array,element_count)\
{\
    for (D_GUINT _i_=0;_i_<element_count ;++_i_)\
    {\
    	dest_array[_i_] = source_array[_i_];\
    }\
}\

#define D_GIM_COPY_ARRAYS_1(dest_array,source_array,element_count,copy_macro)\
{\
    for (D_GUINT _i_=0;_i_<element_count ;++_i_)\
    {\
    	copy_macro(dest_array[_i_],source_array[_i_]);\
    }\
}\


#define D_GIM_ZERO_ARRAY(array,element_count)\
{\
    for (D_GUINT _i_=0;_i_<element_count ;++_i_)\
    {\
    	array[_i_] = 0;\
    }\
}\

#define D_GIM_CONSTANT_ARRAY(array,element_count,constant)\
{\
    for (D_GUINT _i_=0;_i_<element_count ;++_i_)\
    {\
    	array[_i_] = constant;\
    }\
}\


///Function prototypes D_to allocate D_and free memory.
typedef void * D_gim_alloc_function (size_t size);
typedef void * D_gim_alloca_function (size_t size);//Allocs on the heap
typedef void * D_gim_realloc_function (void *ptr, size_t oldsize, size_t newsize);
typedef void D_gim_free_function (void *ptr);


///Memory Function Handlers
///set new memory management functions. if fn D_is 0, the default handlers D_are used.
void D_gim_set_alloc_handler (D_gim_alloc_function *fn);
void D_gim_set_alloca_handler (D_gim_alloca_function *fn);
void D_gim_set_realloc_handler (D_gim_realloc_function *fn);
void D_gim_set_free_handler (D_gim_free_function *fn);


///get current memory management functions.
D_gim_alloc_function *D_gim_get_alloc_handler (void);
D_gim_alloca_function *D_gim_get_alloca_handler(void);
D_gim_realloc_function *D_gim_get_realloc_handler (void);
D_gim_free_function  *D_gim_get_free_handler (void);


///Standar Memory functions
void * D_gim_alloc(size_t size);
void * D_gim_alloca(size_t size);
void * D_gim_realloc(void *ptr, size_t oldsize, size_t newsize);
void D_gim_free(void *ptr);



#if defined (WIN32) && !defined(__MINGW32__) && !defined(__CYGWIN__)
    #define D_GIM_SIMD_MEMORY 1
#endif

//! SIMD POINTER INTEGER
#define D_SIMD_T D_GUINT64
//! SIMD INTEGER SIZE
#define D_SIMD_T_SIZE sizeof(D_SIMD_T)


inline void D_gim_simd_memcpy(void * dst, const void * src, size_t copysize)
{
#ifdef D_GIM_SIMD_MEMORY
/*
//'long long int' D_is incompatible with visual studio 6...
    //copy words
    D_SIMD_T * ui_src_ptr = (D_SIMD_T *)src;
    D_SIMD_T * ui_dst_ptr = (D_SIMD_T *)dst;
    while(copysize>=D_SIMD_T_SIZE)
    {
        *(ui_dst_ptr++) = *(ui_src_ptr++);
        copysize-=D_SIMD_T_SIZE;
    }
    if(copysize==0) return;
*/

    char * c_src_ptr = (char *)src;
    char * c_dst_ptr = (char *)dst;
    while(copysize>0)
    {
        *(c_dst_ptr++) = *(c_src_ptr++);
        copysize--;
    }
    return;
#else
    memcpy(dst,src,copysize);
#endif
}



template<class D_T>
inline void D_gim_swap_elements(D_T* _array,size_t _i,size_t _j)
{
	D_T _e_tmp_ = _array[_i];
	_array[_i] = _array[_j];
	_array[_j] = _e_tmp_;
}


template<class D_T>
inline void D_gim_swap_elements_memcpy(D_T* _array,size_t _i,size_t _j)
{
	char _e_tmp_[sizeof(D_T)];
	D_gim_simd_memcpy(_e_tmp_,&_array[_i],sizeof(D_T));
	D_gim_simd_memcpy(&_array[_i],&_array[_j],sizeof(D_T));
	D_gim_simd_memcpy(&_array[_j],_e_tmp_,sizeof(D_T));
}

template <int SIZE>
inline void D_gim_swap_elements_ptr(char * _array,size_t _i,size_t _j)
{
	char _e_tmp_[SIZE];
	_i*=SIZE;
	_j*=SIZE;
	D_gim_simd_memcpy(_e_tmp_,_array+_i,SIZE);
	D_gim_simd_memcpy(_array+_i,_array+_j,SIZE);
	D_gim_simd_memcpy(_array+_j,_e_tmp_,SIZE);
}

#endif // D_GIM_MEMORY_H_INCLUDED
