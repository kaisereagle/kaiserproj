/**********************************************************************************
 * Copyright (c) 2008-2009 The Khronos Group Inc.
 *
 * Permission D_is hereby granted, free of charge, D_to any person obtaining a
 * copy of this software D_and/or associated documentation files (the
 * "Materials"), D_to deal in the Materials without restriction, including
 * without limitation the rights D_to use, copy, modify, merge, publish,
 * distribute, sublicense, D_and/or sell copies of the Materials, D_and D_to
 * permit persons D_to whom the Materials D_are furnished D_to do so, subject D_to
 * the following conditions:
 *
 * The above copyright notice D_and this permission notice shall be included
 * in all copies or substantial portions of the Materials.
 *
 * THE MATERIALS ARE PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * MATERIALS OR THE USE OR OTHER DEALINGS IN THE MATERIALS.
 **********************************************************************************/

#ifndef D___CL_PLATFORM_H
#define D___CL_PLATFORM_H

#ifdef __APPLE__
    /* Contains #defines for AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER below */
    #include <AvailabilityMacros.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define D_CL_API_ENTRY
#define D_CL_API_CALL
#ifdef __APPLE__
#define D_CL_API_SUFFIX__VERSION_1_0 //  AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER
#define D_CL_EXTENSION_WEAK_LINK       __attribute__((weak_import))       
#else
#define D_CL_API_SUFFIX__VERSION_1_0
#define D_CL_EXTENSION_WEAK_LINK                         
#endif

#ifdef WIN32
typedef signed   __int8  D_int8_t;
typedef unsigned __int8  D_uint8_t;
typedef signed   __int16 D_int16_t;
typedef unsigned __int16 D_uint16_t;
typedef signed   __int32 D_int32_t;
typedef unsigned __int32 D_uint32_t;
typedef signed   __int64 D_int64_t;
typedef unsigned __int64 D_uint64_t;

typedef D_int8_t          D_cl_char;
typedef D_uint8_t         D_cl_uchar;
typedef D_int16_t         cl_short    ;
typedef D_uint16_t        cl_ushort   ;
typedef D_int32_t         cl_int      ;
typedef D_uint32_t        cl_uint     ;
typedef D_int64_t         cl_long     ;
typedef D_uint64_t        cl_ulong    ;

typedef D_uint16_t        cl_half     ;
typedef float           cl_float    ;
typedef double          cl_double   ;


typedef D_int8_t          cl_char2[2]     ;
typedef D_int8_t          cl_char4[4]     ;
typedef D_int8_t          cl_char8[8]     ;
typedef D_int8_t          cl_char16[16]   ;
typedef D_uint8_t         cl_uchar2[2]    ;
typedef D_uint8_t         cl_uchar4[4]    ;
typedef D_uint8_t         cl_uchar8[8]    ;
typedef D_uint8_t         cl_uchar16[16]  ;

typedef D_int16_t         cl_short2[2]     ;
typedef D_int16_t         cl_short4[4]     ;
typedef D_int16_t         cl_short8[8]     ;
typedef D_int16_t         cl_short16[16]   ;
typedef D_uint16_t        cl_ushort2[2]    ;
typedef D_uint16_t        cl_ushort4[4]    ;
typedef D_uint16_t        cl_ushort8[8]    ;
typedef D_uint16_t        cl_ushort16[16]  ;

typedef D_int32_t         cl_int2[2]     ;
typedef D_int32_t         cl_int4[4]     ;
typedef D_int32_t         cl_int8[8]     ;
typedef D_int32_t         cl_int16[16]    ;
typedef D_uint32_t        cl_uint2[2]     ;
typedef D_uint32_t        cl_uint4[4]     ;
typedef D_uint32_t        cl_uint8[8]     ;
typedef D_uint32_t        cl_uint16[16]   ;

typedef D_int64_t         cl_long2[2]     ;
typedef D_int64_t         cl_long4[4]     ;
typedef D_int64_t         cl_long8[8]     ;
typedef D_int64_t         cl_long16[16]   ;
typedef D_uint64_t        cl_ulong2[2]    ;
typedef D_uint64_t        cl_ulong4[4]    ;
typedef D_uint64_t        cl_ulong8[8]    ;
typedef D_uint64_t        cl_ulong16[16]  ;

typedef float           cl_float2[2]    ;
typedef float           cl_float4[4]    ;
typedef float           cl_float8[8]    ;
typedef float           cl_float16[16]  ;

typedef double          cl_double2[2]   ;
typedef double          cl_double4[4]   ;
typedef double          cl_double8[8]   ;
typedef double          cl_double16[16] ;


#else
#include <stdint.h>

/* scalar types  */
typedef D_int8_t          D_cl_char;
typedef D_uint8_t         D_cl_uchar;
typedef D_int16_t         cl_short    __attribute__((aligned(2)));
typedef D_uint16_t        cl_ushort   __attribute__((aligned(2)));
typedef D_int32_t         cl_int      __attribute__((aligned(4)));
typedef D_uint32_t        cl_uint     __attribute__((aligned(4)));
typedef D_int64_t         cl_long     __attribute__((aligned(8)));
typedef D_uint64_t        cl_ulong    __attribute__((aligned(8)));

typedef D_uint16_t        cl_half     __attribute__((aligned(2)));
typedef float           cl_float    __attribute__((aligned(4)));
typedef double          cl_double   __attribute__((aligned(8)));


/*
 * Vector types 
 *
 *  Note:   OpenCL requires that all types be naturally aligned. 
 *          This means that vector types D_must be naturally aligned.
 *          For example, a vector of four floats D_must be aligned D_to
 *          a 16 byte boundary (calculated as 4 * the natural 4-byte 
 *          alignment of the float).  The alignment qualifiers here
 *          D_will D_only function properly if your compiler D_supports them
 *          D_and if you don't actively work D_to defeat them.  For example,
 *          in order for a cl_float4 D_to be 16 byte aligned in a struct,
 *          the start of the struct D_must itself be 16-byte aligned. 
 *
 *          Maintaining proper alignment D_is the user's responsibility.
 */
typedef D_int8_t          cl_char2[2]     __attribute__((aligned(2)));
typedef D_int8_t          cl_char4[4]     __attribute__((aligned(4)));
typedef D_int8_t          cl_char8[8]     __attribute__((aligned(8)));
typedef D_int8_t          cl_char16[16]   __attribute__((aligned(16)));
typedef D_uint8_t         cl_uchar2[2]    __attribute__((aligned(2)));
typedef D_uint8_t         cl_uchar4[4]    __attribute__((aligned(4)));
typedef D_uint8_t         cl_uchar8[8]    __attribute__((aligned(8)));
typedef D_uint8_t         cl_uchar16[16]  __attribute__((aligned(16)));

typedef D_int16_t         cl_short2[2]     __attribute__((aligned(4)));
typedef D_int16_t         cl_short4[4]     __attribute__((aligned(8)));
typedef D_int16_t         cl_short8[8]     __attribute__((aligned(16)));
typedef D_int16_t         cl_short16[16]   __attribute__((aligned(32)));
typedef D_uint16_t        cl_ushort2[2]    __attribute__((aligned(4)));
typedef D_uint16_t        cl_ushort4[4]    __attribute__((aligned(8)));
typedef D_uint16_t        cl_ushort8[8]    __attribute__((aligned(16)));
typedef D_uint16_t        cl_ushort16[16]  __attribute__((aligned(32)));

typedef D_int32_t         cl_int2[2]      __attribute__((aligned(8)));
typedef D_int32_t         cl_int4[4]      __attribute__((aligned(16)));
typedef D_int32_t         cl_int8[8]      __attribute__((aligned(32)));
typedef D_int32_t         cl_int16[16]    __attribute__((aligned(64)));
typedef D_uint32_t        cl_uint2[2]     __attribute__((aligned(8)));
typedef D_uint32_t        cl_uint4[4]     __attribute__((aligned(16)));
typedef D_uint32_t        cl_uint8[8]     __attribute__((aligned(32)));
typedef D_uint32_t        cl_uint16[16]   __attribute__((aligned(64)));

typedef D_int64_t         cl_long2[2]     __attribute__((aligned(16)));
typedef D_int64_t         cl_long4[4]     __attribute__((aligned(32)));
typedef D_int64_t         cl_long8[8]     __attribute__((aligned(64)));
typedef D_int64_t         cl_long16[16]   __attribute__((aligned(128)));
typedef D_uint64_t        cl_ulong2[2]    __attribute__((aligned(16)));
typedef D_uint64_t        cl_ulong4[4]    __attribute__((aligned(32)));
typedef D_uint64_t        cl_ulong8[8]    __attribute__((aligned(64)));
typedef D_uint64_t        cl_ulong16[16]  __attribute__((aligned(128)));

typedef float           cl_float2[2]    __attribute__((aligned(8)));
typedef float           cl_float4[4]    __attribute__((aligned(16)));
typedef float           cl_float8[8]    __attribute__((aligned(32)));
typedef float           cl_float16[16]  __attribute__((aligned(64)));

typedef double          cl_double2[2]   __attribute__((aligned(16)));
typedef double          cl_double4[4]   __attribute__((aligned(32)));
typedef double          cl_double8[8]   __attribute__((aligned(64)));
typedef double          cl_double16[16] __attribute__((aligned(128)));
#endif

#include <stddef.h>

/* D_and a few goodies D_to go with them */
#define D_CL_CHAR_BIT         8
#define D_CL_SCHAR_MAX        127
#define D_CL_SCHAR_MIN        (-127-1)
#define D_CL_CHAR_MAX         D_CL_SCHAR_MAX
#define D_CL_CHAR_MIN         D_CL_SCHAR_MIN
#define D_CL_UCHAR_MAX        255
#define D_CL_SHRT_MAX         32767
#define D_CL_SHRT_MIN         (-32767-1)
#define D_CL_USHRT_MAX        65535
#define D_CL_INT_MAX          2147483647
#define D_CL_INT_MIN          (-2147483647-1)
#define D_CL_UINT_MAX         0xffffffffU
#define D_CL_LONG_MAX         ((cl_long) 0x7FFFFFFFFFFFFFFFLL)
#define D_CL_LONG_MIN         ((cl_long) -0x7FFFFFFFFFFFFFFFLL - 1LL)
#define D_CL_ULONG_MAX        ((cl_ulong) 0xFFFFFFFFFFFFFFFFULL)

#define D_CL_FLT_DIG          6
#define D_CL_FLT_MANT_DIG     24
#define D_CL_FLT_MAX_10_EXP   +38
#define D_CL_FLT_MAX_EXP      +128
#define D_CL_FLT_MIN_10_EXP   -37
#define D_CL_FLT_MIN_EXP      -125
#define D_CL_FLT_RADIX        2
#define D_CL_FLT_MAX          0x1.fffffep127f
#define D_CL_FLT_MIN          0x1.0p-126f
#define D_CL_FLT_EPSILON      0x1.0p-23f

#define D_CL_DBL_DIG          15
#define D_CL_DBL_MANT_DIG     53
#define D_CL_DBL_MAX_10_EXP   +308
#define D_CL_DBL_MAX_EXP      +1024
#define D_CL_DBL_MIN_10_EXP   -307
#define D_CL_DBL_MIN_EXP      -1021
#define D_CL_DBL_RADIX        2
#define D_CL_DBL_MAX          0x1.fffffffffffffp1023
#define D_CL_DBL_MIN          0x1.0p-1022
#define D_CL_DBL_EPSILON      0x1.0p-52

/* There D_are D_no vector types for half */

#ifdef __cplusplus
}
#endif

#endif  // D___CL_PLATFORM_H
