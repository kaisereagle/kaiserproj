/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef D_GEN_MINMAX_H
#define D_GEN_MINMAX_H

template <class D_T>
D_SIMD_FORCE_INLINE const D_T& D_btMin(const D_T& a, const D_T& b) 
{
  return a < b ? a : b ;
}

template <class D_T>
D_SIMD_FORCE_INLINE const D_T& D_btMax(const D_T& a, const D_T& b) 
{
  return  a > b ? a : b;
}

template <class D_T>
D_SIMD_FORCE_INLINE const D_T& D_GEN_clamped(const D_T& a, const D_T& lb, const D_T& ub) 
{
	return a < lb ? lb : (ub < a ? ub : a); 
}

template <class D_T>
D_SIMD_FORCE_INLINE void D_btSetMin(D_T& a, const D_T& b) 
{
    if (b < a) 
	{
		a = b;
	}
}

template <class D_T>
D_SIMD_FORCE_INLINE void D_btSetMax(D_T& a, const D_T& b) 
{
    if (a < b) 
	{
		a = b;
	}
}

template <class D_T>
D_SIMD_FORCE_INLINE void D_GEN_clamp(D_T& a, const D_T& lb, const D_T& ub) 
{
	if (a < lb) 
	{
		a = lb; 
	}
	else if (ub < a) 
	{
		a = ub;
	}
}

#endif
