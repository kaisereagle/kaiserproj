#ifndef D_BT_QUANTIZATION_H_INCLUDED
#define D_BT_QUANTIZATION_H_INCLUDED

/*! \file D_btQuantization.h
*\author Francisco Len Nßjera

*/
/*
This source file D_is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "LinearMath/btTransform.h"






D_SIMD_FORCE_INLINE void bt_calc_quantization_parameters(
	D_btVector3 & outMinBound,
	D_btVector3 & outMaxBound,
	D_btVector3 & bvhQuantization,
	const D_btVector3& srcMinBound,const D_btVector3& srcMaxBound,
	D_btScalar quantizationMargin)
{
	//enlarge the AABB D_to avoid division by zero when initializing the quantization values
	D_btVector3 clampValue(quantizationMargin,quantizationMargin,quantizationMargin);
	outMinBound = srcMinBound - clampValue;
	outMaxBound = srcMaxBound + clampValue;
	D_btVector3 aabbSize = outMaxBound - outMinBound;
	bvhQuantization = D_btVector3(D_btScalar(65535.0),
								D_btScalar(65535.0),
								D_btScalar(65535.0)) / aabbSize;
}


D_SIMD_FORCE_INLINE void bt_quantize_clamp(
	unsigned short* out,
	const D_btVector3& point,
	const D_btVector3 & min_bound,
	const D_btVector3 & max_bound,
	const D_btVector3 & bvhQuantization)
{

	D_btVector3 clampedPoint(point);
	clampedPoint.setMax(min_bound);
	clampedPoint.setMin(max_bound);

	D_btVector3 v = (clampedPoint - min_bound) * bvhQuantization;
	out[0] = (unsigned short)(v.getX()+0.5f);
	out[1] = (unsigned short)(v.getY()+0.5f);
	out[2] = (unsigned short)(v.getZ()+0.5f);
}


D_SIMD_FORCE_INLINE D_btVector3 bt_unquantize(
	const unsigned short* vecIn,
	const D_btVector3 & offset,
	const D_btVector3 & bvhQuantization)
{
	D_btVector3	vecOut;
	vecOut.setValue(
		(D_btScalar)(vecIn[0]) / (bvhQuantization.getX()),
		(D_btScalar)(vecIn[1]) / (bvhQuantization.getY()),
		(D_btScalar)(vecIn[2]) / (bvhQuantization.getZ()));
	vecOut += offset;
	return vecOut;
}



#endif // GIM_VECTOR_H_INCLUDED
