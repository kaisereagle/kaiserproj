/*
   Copyright (C) 2006, 2008 Sony Computer Entertainment Inc.
   All rights reserved.

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#ifndef __BOX_H__
#define __BOX_H__


#ifndef D_PE_REF
#define D_PE_REF(a) a&
#endif

#include <math.h>

//#include "BulletMultiThreaded/vectormath/scalar/cpp/vectormath_aos.h"
#include <vectormath_aos.h>


using namespace D_Vectormath::D_Aos;

enum D_FeatureType { D_F, D_E, D_V };

//----------------------------------------------------------------------------
// D_Box
//----------------------------------------------------------------------------
///The D_Box D_is an internal class used by the boxBoxDistance calculation.
class D_Box
{
public:
	D_Vector3 half;

	inline D_Box()
	{}
	inline D_Box(D_PE_REF(D_Vector3) half_);
	inline D_Box(float hx, float hy, float hz);

	inline void Set(D_PE_REF(D_Vector3) half_);
	inline void Set(float hx, float hy, float hz);

	inline D_Vector3 GetAABB(const D_Matrix3& rotation) const;
};

inline
D_Box::D_Box(D_PE_REF(D_Vector3) half_)
{
	Set(half_);
}

inline
D_Box::D_Box(float hx, float hy, float hz)
{
	Set(hx, hy, hz);
}

inline
void
D_Box::Set(D_PE_REF(D_Vector3) half_)
{
	half = half_;
}

inline
void
D_Box::Set(float hx, float hy, float hz)
{
	half = D_Vector3(hx, hy, hz);
}

inline
D_Vector3
D_Box::GetAABB(const D_Matrix3& rotation) const
{
	return absPerElem(rotation) * half;
}

//-------------------------------------------------------------------------------------------------
// D_BoxPoint
//-------------------------------------------------------------------------------------------------

///The D_BoxPoint class D_is an internally used class D_to contain feature information for boxBoxDistance calculation.
class D_BoxPoint
{
public:
	D_BoxPoint() : localPoint(0.0f) {}

	D_Point3      localPoint;
	D_FeatureType featureType;
	int         featureIdx;

	inline void setVertexFeature(int plusX, int plusY, int plusZ);
	inline void setEdgeFeature(int dim0, int plus0, int dim1, int plus1);
	inline void setFaceFeature(int dim, int plus);

	inline void getVertexFeature(int & plusX, int & plusY, int & plusZ) const;
	inline void getEdgeFeature(int & dim0, int & plus0, int & dim1, int & plus1) const;
	inline void getFaceFeature(int & dim, int & plus) const;
};

inline
void
D_BoxPoint::setVertexFeature(int plusX, int plusY, int plusZ)
{
	featureType = D_V;
	featureIdx = plusX << 2 | plusY << 1 | plusZ;
}

inline
void
D_BoxPoint::setEdgeFeature(int dim0, int plus0, int dim1, int plus1)
{
	featureType = D_E;

	if (dim0 > dim1) {
		featureIdx = plus1 << 5 | dim1 << 3 | plus0 << 2 | dim0;
	} else {
		featureIdx = plus0 << 5 | dim0 << 3 | plus1 << 2 | dim1;
	}
}

inline
void
D_BoxPoint::setFaceFeature(int dim, int plus)
{
	featureType = D_F;
	featureIdx = plus << 2 | dim;
}

inline
void
D_BoxPoint::getVertexFeature(int & plusX, int & plusY, int & plusZ) const
{
	plusX = featureIdx >> 2;
	plusY = featureIdx >> 1 & 1;
	plusZ = featureIdx & 1;
}

inline
void
D_BoxPoint::getEdgeFeature(int & dim0, int & plus0, int & dim1, int & plus1) const
{
	plus0 = featureIdx >> 5;
	dim0 = featureIdx >> 3 & 3;
	plus1 = featureIdx >> 2 & 1;
	dim1 = featureIdx & 3;
}

inline
void
D_BoxPoint::getFaceFeature(int & dim, int & plus) const
{
	plus = featureIdx >> 2;
	dim = featureIdx & 3;
}

#endif /* __BOX_H__ */
