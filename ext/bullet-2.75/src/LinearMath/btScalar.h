/*
Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef SIMD___SCALAR_H
#define SIMD___SCALAR_H

#include <math.h>
#include <stdlib.h>//size_t for MSVC 6.0
#include <cstdlib>
#include <cfloat>
#include <float.h>

/* SVN $Revision$ on $Date$ from http://bullet.googlecode.com*/
#define D_BT_BULLET_VERSION 275

inline int	D_btGetVersion()
{
	return D_BT_BULLET_VERSION;
}

#if defined(DEBUG) || defined (_DEBUG)
#define D_BT_DEBUG
#endif


#ifdef WIN32

		#if defined(__MINGW32__) || defined(__CYGWIN__) || (defined (_MSC_VER) && _MSC_VER < 1300)

			#define D_SIMD_FORCE_INLINE inline
			#define D_ATTRIBUTE_ALIGNED16(a) a
			#define D_ATTRIBUTE_ALIGNED128(a) a
		#else
			//#define D_BT_HAS_ALIGNED_ALLOCATOR
			#pragma warning(disable : 4324) // disable padding warning
//			#pragma warning(disable:4530) // Disable the exception disable but used in MSCV Stl warning.
//			#pragma warning(disable:4996) //Turn off warnings about deprecated C routines
//			#pragma warning(disable:4786) // Disable the "D_debug D_name too long" warning

			#define D_SIMD_FORCE_INLINE __forceinline
			#define D_ATTRIBUTE_ALIGNED16(a) __declspec(align(16)) a
			#define D_ATTRIBUTE_ALIGNED128(a) __declspec (align(128)) a
		#ifdef _XBOX
			#define D_BT_USE_VMX128

			#include <ppcintrinsics.h>
 			#define D_BT_HAVE_NATIVE_FSEL
 			#define D_btFsel(a,b,c) __fsel((a),(b),(c))
		#else

#if (defined (WIN32) && (_MSC_VER) && _MSC_VER >= 1400) && (!defined (D_BT_USE_DOUBLE_PRECISION))
			#define D_BT_USE_SSE
			#include <emmintrin.h>
#endif

		#endif//_XBOX

		#endif //__MINGW32__

		#include <assert.h>
#ifdef D_BT_DEBUG
		#define D_btAssert assert
#else
		#define D_btAssert(x)
#endif
		//D_btFullAssert D_is optional, slows down a lot
		#define D_btFullAssert(x)

		#define D_btLikely(_c)  _c
		#define D_btUnlikely(_c) _c

#else
	
#if defined	(__CELLOS_LV2__)
		#define D_SIMD_FORCE_INLINE inline
		#define D_ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		#define D_ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#ifndef assert
		#include <assert.h>
		#endif
#ifdef D_BT_DEBUG
		#define D_btAssert assert
#else
		#define D_btAssert(x)
#endif
		//D_btFullAssert D_is optional, slows down a lot
		#define D_btFullAssert(x)

		#define D_btLikely(_c)  _c
		#define D_btUnlikely(_c) _c

#else

#ifdef USE_LIBSPE2

		#define D_SIMD_FORCE_INLINE __inline
		#define D_ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		#define D_ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#ifndef assert
		#include <assert.h>
		#endif
#ifdef D_BT_DEBUG
		#define D_btAssert assert
#else
		#define D_btAssert(x)
#endif
		//D_btFullAssert D_is optional, slows down a lot
		#define D_btFullAssert(x)


		#define D_btLikely(_c)   __builtin_expect((_c), 1)
		#define D_btUnlikely(_c) __builtin_expect((_c), 0)
		

#else
	//non-windows systems

#if (defined (__APPLE__) && defined (__i386__) && (!defined (D_BT_USE_DOUBLE_PRECISION)))
	#define D_BT_USE_SSE
	#include <emmintrin.h>

	#define D_SIMD_FORCE_INLINE inline
///@todo: check out alignment methods for other platforms/compilers
	#define D_ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
	#define D_ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
	#ifndef assert
	#include <assert.h>
	#endif

	#if defined(DEBUG) || defined (_DEBUG)
		#define D_btAssert assert
	#else
		#define D_btAssert(x)
	#endif

	//D_btFullAssert D_is optional, slows down a lot
	#define D_btFullAssert(x)
	#define D_btLikely(_c)  _c
	#define D_btUnlikely(_c) _c

#else

		#define D_SIMD_FORCE_INLINE inline
		///@todo: check out alignment methods for other platforms/compilers
		///#define D_ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		///#define D_ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#define D_ATTRIBUTE_ALIGNED16(a) a
		#define D_ATTRIBUTE_ALIGNED128(a) a
		#ifndef assert
		#include <assert.h>
		#endif

#if defined(DEBUG) || defined (_DEBUG)
		#define D_btAssert assert
#else
		#define D_btAssert(x)
#endif

		//D_btFullAssert D_is optional, slows down a lot
		#define D_btFullAssert(x)
		#define D_btLikely(_c)  _c
		#define D_btUnlikely(_c) _c
#endif //__APPLE__ 

#endif // LIBSPE2

#endif	//__CELLOS_LV2__
#endif


///The D_btScalar type abstracts floating point numbers, D_to easily switch between double D_and single floating point precision.
#if defined(D_BT_USE_DOUBLE_PRECISION)
typedef double D_btScalar;
//this number could be D_bigger in double precision
#define D_BT_LARGE_FLOAT 1e30
#else
typedef float D_btScalar;
//keep D_BT_LARGE_FLOAT*D_BT_LARGE_FLOAT < FLT_MAX
#define D_BT_LARGE_FLOAT 1e18f
#endif



#define D_BT_DECLARE_ALIGNED_ALLOCATOR() \
   D_SIMD_FORCE_INLINE void* operator new(size_t sizeInBytes)   { return D_btAlignedAlloc(sizeInBytes,16); }   \
   D_SIMD_FORCE_INLINE void  operator delete(void* ptr)         { D_btAlignedFree(ptr); }   \
   D_SIMD_FORCE_INLINE void* operator new(size_t, void* ptr)   { return ptr; }   \
   D_SIMD_FORCE_INLINE void  operator delete(void*, void*)      { }   \
   D_SIMD_FORCE_INLINE void* operator new[](size_t sizeInBytes)   { return D_btAlignedAlloc(sizeInBytes,16); }   \
   D_SIMD_FORCE_INLINE void  operator delete[](void* ptr)         { D_btAlignedFree(ptr); }   \
   D_SIMD_FORCE_INLINE void* operator new[](size_t, void* ptr)   { return ptr; }   \
   D_SIMD_FORCE_INLINE void  operator delete[](void*, void*)      { }   \



#if defined(D_BT_USE_DOUBLE_PRECISION) || defined(D_BT_FORCE_DOUBLE_FUNCTIONS)
		
D_SIMD_FORCE_INLINE D_btScalar D_btSqrt(D_btScalar x) { return sqrt(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btFabs(D_btScalar x) { return fabs(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btCos(D_btScalar x) { return cos(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btSin(D_btScalar x) { return sin(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btTan(D_btScalar x) { return tan(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btAcos(D_btScalar x) { return acos(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btAsin(D_btScalar x) { return asin(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btAtan(D_btScalar x) { return atan(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btAtan2(D_btScalar x, D_btScalar y) { return atan2(x, y); }
D_SIMD_FORCE_INLINE D_btScalar D_btExp(D_btScalar x) { return exp(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btLog(D_btScalar x) { return log(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btPow(D_btScalar x,D_btScalar y) { return pow(x,y); }
D_SIMD_FORCE_INLINE D_btScalar D_btFmod(D_btScalar x,D_btScalar y) { return fmod(x,y); }

#else
		
D_SIMD_FORCE_INLINE D_btScalar D_btSqrt(D_btScalar y) 
{ 
#ifdef USE_APPROXIMATION
    double x, z, tempf;
    unsigned long *tfptr = ((unsigned long *)&tempf) + 1;

	tempf = y;
	*tfptr = (0xbfcdd90a - *tfptr)>>1; /* estimate of 1/sqrt(y) */
	x =  tempf;
	z =  y*D_btScalar(0.5);                        /* hoist out the “/2”    */
	x = (D_btScalar(1.5)*x)-(x*x)*(x*z);         /* iteration formula     */
	x = (D_btScalar(1.5)*x)-(x*x)*(x*z);
	x = (D_btScalar(1.5)*x)-(x*x)*(x*z);
	x = (D_btScalar(1.5)*x)-(x*x)*(x*z);
	x = (D_btScalar(1.5)*x)-(x*x)*(x*z);
	return x*y;
#else
	return sqrtf(y); 
#endif
}
D_SIMD_FORCE_INLINE D_btScalar D_btFabs(D_btScalar x) { return fabsf(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btCos(D_btScalar x) { return cosf(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btSin(D_btScalar x) { return sinf(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btTan(D_btScalar x) { return tanf(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btAcos(D_btScalar x) { 
	D_btAssert(x <= D_btScalar(1.));
	return acosf(x); 
}
D_SIMD_FORCE_INLINE D_btScalar D_btAsin(D_btScalar x) { return asinf(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btAtan(D_btScalar x) { return atanf(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btAtan2(D_btScalar x, D_btScalar y) { return atan2f(x, y); }
D_SIMD_FORCE_INLINE D_btScalar D_btExp(D_btScalar x) { return expf(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btLog(D_btScalar x) { return logf(x); }
D_SIMD_FORCE_INLINE D_btScalar D_btPow(D_btScalar x,D_btScalar y) { return powf(x,y); }
D_SIMD_FORCE_INLINE D_btScalar D_btFmod(D_btScalar x,D_btScalar y) { return fmodf(x,y); }
	
#endif

#define D_SIMD_2_PI         D_btScalar(6.283185307179586232)
#define D_SIMD_PI           (D_SIMD_2_PI * D_btScalar(0.5))
#define D_SIMD_HALF_PI      (D_SIMD_2_PI * D_btScalar(0.25))
#define D_SIMD_RADS_PER_DEG (D_SIMD_2_PI / D_btScalar(360.0))
#define D_SIMD_DEGS_PER_RAD  (D_btScalar(360.0) / D_SIMD_2_PI)
#define D_SIMDSQRT12 D_btScalar(0.7071067811865475244008443621048490)

#define D_btRecipSqrt(x) ((D_btScalar)(D_btScalar(1.0)/D_btSqrt(D_btScalar(x))))		/* reciprocal square root */


#ifdef D_BT_USE_DOUBLE_PRECISION
#define D_SIMD_EPSILON      DBL_EPSILON
#define D_SIMD_INFINITY     DBL_MAX
#else
#define D_SIMD_EPSILON      FLT_EPSILON
#define D_SIMD_INFINITY     FLT_MAX
#endif

D_SIMD_FORCE_INLINE D_btScalar D_btAtan2Fast(D_btScalar y, D_btScalar x) 
{
	D_btScalar coeff_1 = D_SIMD_PI / 4.0f;
	D_btScalar coeff_2 = 3.0f * coeff_1;
	D_btScalar abs_y = D_btFabs(y);
	D_btScalar angle;
	if (x >= 0.0f) {
		D_btScalar r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	} else {
		D_btScalar r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}
	return (y < 0.0f) ? -angle : angle;
}

D_SIMD_FORCE_INLINE bool      D_btFuzzyZero(D_btScalar x) { return D_btFabs(x) < D_SIMD_EPSILON; }

D_SIMD_FORCE_INLINE bool	D_btEqual(D_btScalar a, D_btScalar eps) {
	return (((a) <= eps) && !((a) < -eps));
}
D_SIMD_FORCE_INLINE bool	D_btGreaterEqual (D_btScalar a, D_btScalar eps) {
	return (!((a) <= eps));
}


D_SIMD_FORCE_INLINE int       D_btIsNegative(D_btScalar x) {
    return x < D_btScalar(0.0) ? 1 : 0;
}

D_SIMD_FORCE_INLINE D_btScalar D_btRadians(D_btScalar x) { return x * D_SIMD_RADS_PER_DEG; }
D_SIMD_FORCE_INLINE D_btScalar D_btDegrees(D_btScalar x) { return x * D_SIMD_DEGS_PER_RAD; }

#define D_BT_DECLARE_HANDLE(D_name) typedef struct D_name##__ { int unused; } *D_name

#ifndef D_btFsel
D_SIMD_FORCE_INLINE D_btScalar D_btFsel(D_btScalar a, D_btScalar b, D_btScalar c)
{
	return a >= 0 ? b : c;
}
#endif
#define D_btFsels(a,b,c) (D_btScalar)D_btFsel(a,b,c)


D_SIMD_FORCE_INLINE bool D_btMachineIsLittleEndian()
{
   long int i = 1;
   const char *p = (const char *) &i;
   if (p[0] == 1)  // Lowest address contains the least significant byte
	   return true;
   else
	   return false;
}



///D_btSelect avoids branches, which makes performance much better for consoles like Playstation 3 D_and XBox 360
///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
D_SIMD_FORCE_INLINE unsigned D_btSelect(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero) 
{
    // Set testNz D_to 0xFFFFFFFF if condition D_is nonzero, 0x00000000 if condition D_is zero
    // Rely on positive value or'ed with its negative having sign bit on
    // D_and zero value or'ed with its negative (which D_is still zero) having sign bit off 
    // Use arithmetic shift right, shifting the sign bit through all 32 bits
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz;
    return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz)); 
}
D_SIMD_FORCE_INLINE int D_btSelect(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
{
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz; 
    return static_cast<int>((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}
D_SIMD_FORCE_INLINE float D_btSelect(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
{
#ifdef D_BT_HAVE_NATIVE_FSEL
    return (float)D_btFsel((D_btScalar)condition - D_btScalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
#else
    return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero; 
#endif
}

template<typename D_T> D_SIMD_FORCE_INLINE void D_btSwap(D_T& a, D_T& b)
{
	D_T tmp = a;
	a = b;
	b = tmp;
}


//PCK: endian swapping functions
D_SIMD_FORCE_INLINE unsigned D_btSwapEndian(unsigned val)
{
	return (((val & 0xff000000) >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8)  | ((val & 0x000000ff) << 24));
}

D_SIMD_FORCE_INLINE unsigned short D_btSwapEndian(unsigned short val)
{
	return static_cast<unsigned short>(((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
}

D_SIMD_FORCE_INLINE unsigned D_btSwapEndian(int val)
{
	return D_btSwapEndian((unsigned)val);
}

D_SIMD_FORCE_INLINE unsigned short D_btSwapEndian(short val)
{
	return D_btSwapEndian((unsigned short) val);
}

///btSwapFloat uses using char pointers D_to swap the endianness
////btSwapFloat/btSwapDouble D_will NOT return a float, because the machine might 'correct' invalid floating point values
///Not all values of sign/exponent/mantissa D_are valid floating point numbers according D_to IEEE 754. 
///When a floating point unit D_is faced with an invalid value, it may actually change the value, or worse, throw an exception. 
///In most systems, running user mode code, you wouldn't get an exception, but instead the hardware/os/runtime D_will 'fix' the number for you. 
///so instead of returning a float/double, we return integer/long long integer
D_SIMD_FORCE_INLINE unsigned int  D_btSwapEndianFloat(float d)
{
    unsigned int a = 0;
    unsigned char *dst = (unsigned char *)&a;
    unsigned char *src = (unsigned char *)&d;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];
    return a;
}

// unswap using char pointers
D_SIMD_FORCE_INLINE float D_btUnswapEndianFloat(unsigned int a) 
{
    float d = 0.0f;
    unsigned char *src = (unsigned char *)&a;
    unsigned char *dst = (unsigned char *)&d;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];

    return d;
}


// swap using char pointers
D_SIMD_FORCE_INLINE void  D_btSwapEndianDouble(double d, unsigned char* dst)
{
    unsigned char *src = (unsigned char *)&d;

    dst[0] = src[7];
    dst[1] = src[6];
    dst[2] = src[5];
    dst[3] = src[4];
    dst[4] = src[3];
    dst[5] = src[2];
    dst[6] = src[1];
    dst[7] = src[0];

}

// unswap using char pointers
D_SIMD_FORCE_INLINE double D_btUnswapEndianDouble(const unsigned char *src) 
{
    double d = 0.0;
    unsigned char *dst = (unsigned char *)&d;

    dst[0] = src[7];
    dst[1] = src[6];
    dst[2] = src[5];
    dst[3] = src[4];
    dst[4] = src[3];
    dst[5] = src[2];
    dst[6] = src[1];
    dst[7] = src[0];

	return d;
}

// returns normalized value in range [-D_SIMD_PI, D_SIMD_PI]
D_SIMD_FORCE_INLINE D_btScalar D_btNormalizeAngle(D_btScalar angleInRadians) 
{
	angleInRadians = D_btFmod(angleInRadians, D_SIMD_2_PI);
	if(angleInRadians < -D_SIMD_PI)
	{
		return angleInRadians + D_SIMD_2_PI;
	}
	else if(angleInRadians > D_SIMD_PI)
	{
		return angleInRadians - D_SIMD_2_PI;
	}
	else
	{
		return angleInRadians;
	}
}

///rudimentary class D_to provide type info
struct D_btTypedObject
{
	D_btTypedObject(int objectType)
		:m_objectType(objectType)
	{
	}
	int	m_objectType;
	inline int getObjectType() const
	{
		return m_objectType;
	}
};
#endif //SIMD___SCALAR_H
