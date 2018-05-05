#ifndef D_GIM_MATH_H_INCLUDED
#define D_GIM_MATH_H_INCLUDED
/*! \file D_gim_math.h
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

#include "LinearMath/btScalar.h"



#define D_GREAL D_btScalar
#define D_GREAL2 double
#define D_GINT int
#define D_GUINT unsigned int
#define D_GSHORT short
#define D_GUSHORT unsigned short
#define D_GINT64 long long
#define D_GUINT64 unsigned long long



#define D_G_PI 3.14159265358979f
#define D_G_HALF_PI 1.5707963f
//267948966
#define D_G_TWO_PI 6.28318530f
//71795864
#define D_G_ROOT3 1.73205f
#define D_G_ROOT2 1.41421f
#define D_G_UINT_INFINITY 0xffffffff //!< A very very high value
#define D_G_REAL_INFINITY FLT_MAX
#define	G_SIGN_BITMASK			0x80000000
#define D_G_EPSILON D_SIMD_EPSILON



enum D_GIM_SCALAR_TYPES
{
	D_G_STYPE_REAL =0,
	D_G_STYPE_REAL2,
	D_G_STYPE_SHORT,
	D_G_STYPE_USHORT,
	D_G_STYPE_INT,
	D_G_STYPE_UINT,
	D_G_STYPE_INT64,
	D_G_STYPE_UINT64
};



#define D_G_DEGTORAD(X) ((X)*3.1415926f/180.0f)
#define D_G_RADTODEG(X) ((X)*180.0f/3.1415926f)

//! Integer representation of a floating-point value.
#define D_GIM_IR(x)					((D_GUINT&)(x))

//! Signed integer representation of a floating-point value.
#define D_GIM_SIR(x)					((D_GINT&)(x))

//! Absolute integer representation of a floating-point value
#define D_GIM_AIR(x)					(D_GIM_IR(x)&0x7fffffff)

//! Floating-point representation of an integer value.
#define D_GIM_FR(x)					((D_GREAL&)(x))

#define D_GIM_MAX(a,b) (a<b?b:a)
#define D_GIM_MIN(a,b) (a>b?b:a)

#define D_GIM_MAX3(a,b,c) D_GIM_MAX(a,D_GIM_MAX(b,c))
#define D_GIM_MIN3(a,b,c) D_GIM_MIN(a,D_GIM_MIN(b,c))

#define D_GIM_IS_ZERO(value) (value < D_G_EPSILON &&  value > -D_G_EPSILON)

#define D_GIM_IS_NEGATIVE(value) (value <= -D_G_EPSILON)

#define D_GIM_IS_POSISITVE(value) (value >= D_G_EPSILON)

#define D_GIM_NEAR_EQUAL(v1,v2) D_GIM_IS_ZERO((v1-v2))

///returns a clamped number
#define D_GIM_CLAMP(number,minval,maxval) (number<minval?minval:(number>maxval?maxval:number))

#define D_GIM_GREATER(x, y)	D_btFabs(x) > (y)

///Swap numbers
#define D_GIM_SWAP_NUMBERS(a,b){ \
    a = a+b; \
    b = a-b; \
    a = a-b; \
}\

#define D_GIM_INV_SQRT(va,isva)\
{\
    if(va<=0.0000001f)\
    {\
        isva = D_G_REAL_INFINITY;\
    }\
    else\
    {\
        D_GREAL _x = va * 0.5f;\
        D_GUINT _y = 0x5f3759df - ( D_GIM_IR(va) >> 1);\
        isva = D_GIM_FR(_y);\
        isva  = isva * ( 1.5f - ( _x * isva * isva ) );\
    }\
}\

#define D_GIM_SQRT(va,sva)\
{\
    D_GIM_INV_SQRT(va,sva);\
    sva = 1.0f/sva;\
}\

//! Computes 1.0f / sqrtf(x). Comes from Quake3. See http://www.magic-software.com/3DGEDInvSqrt.html
inline D_GREAL D_gim_inv_sqrt(D_GREAL f)
{
    D_GREAL r;
    D_GIM_INV_SQRT(f,r);
    return r;
}

inline D_GREAL D_gim_sqrt(D_GREAL f)
{
    D_GREAL r;
    D_GIM_SQRT(f,r);
    return r;
}



#endif // D_GIM_MATH_H_INCLUDED
