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



#ifndef AABB_UTIL2
#define AABB_UTIL2

#include "btTransform.h"
#include "btVector3.h"
#include "btMinMax.h"



D_SIMD_FORCE_INLINE void AabbExpand (D_btVector3& aabbMin,
								   D_btVector3& aabbMax,
								   const D_btVector3& expansionMin,
								   const D_btVector3& expansionMax)
{
	aabbMin = aabbMin + expansionMin;
	aabbMax = aabbMax + expansionMax;
}

/// conservative test for overlap between two aabbs
D_SIMD_FORCE_INLINE bool TestPointAgainstAabb2(const D_btVector3 &aabbMin1, const D_btVector3 &aabbMax1,
								const D_btVector3 &point)
{
	bool overlap = true;
	overlap = (aabbMin1.getX() > point.getX() || aabbMax1.getX() < point.getX()) ? false : overlap;
	overlap = (aabbMin1.getZ() > point.getZ() || aabbMax1.getZ() < point.getZ()) ? false : overlap;
	overlap = (aabbMin1.getY() > point.getY() || aabbMax1.getY() < point.getY()) ? false : overlap;
	return overlap;
}


/// conservative test for overlap between two aabbs
D_SIMD_FORCE_INLINE bool TestAabbAgainstAabb2(const D_btVector3 &aabbMin1, const D_btVector3 &aabbMax1,
								const D_btVector3 &aabbMin2, const D_btVector3 &aabbMax2)
{
	bool overlap = true;
	overlap = (aabbMin1.getX() > aabbMax2.getX() || aabbMax1.getX() < aabbMin2.getX()) ? false : overlap;
	overlap = (aabbMin1.getZ() > aabbMax2.getZ() || aabbMax1.getZ() < aabbMin2.getZ()) ? false : overlap;
	overlap = (aabbMin1.getY() > aabbMax2.getY() || aabbMax1.getY() < aabbMin2.getY()) ? false : overlap;
	return overlap;
}

/// conservative test for overlap between triangle D_and aabb
D_SIMD_FORCE_INLINE bool TestTriangleAgainstAabb2(const D_btVector3 *vertices,
									const D_btVector3 &aabbMin, const D_btVector3 &aabbMax)
{
	const D_btVector3 &p1 = vertices[0];
	const D_btVector3 &p2 = vertices[1];
	const D_btVector3 &p3 = vertices[2];

	if (D_btMin(D_btMin(p1[0], p2[0]), p3[0]) > aabbMax[0]) return false;
	if (D_btMax(D_btMax(p1[0], p2[0]), p3[0]) < aabbMin[0]) return false;

	if (D_btMin(D_btMin(p1[2], p2[2]), p3[2]) > aabbMax[2]) return false;
	if (D_btMax(D_btMax(p1[2], p2[2]), p3[2]) < aabbMin[2]) return false;
  
	if (D_btMin(D_btMin(p1[1], p2[1]), p3[1]) > aabbMax[1]) return false;
	if (D_btMax(D_btMax(p1[1], p2[1]), p3[1]) < aabbMin[1]) return false;
	return true;
}


D_SIMD_FORCE_INLINE int	D_btOutcode(const D_btVector3& p,const D_btVector3& halfExtent) 
{
	return (p.getX()  < -halfExtent.getX() ? 0x01 : 0x0) |    
		   (p.getX() >  halfExtent.getX() ? 0x08 : 0x0) |
		   (p.getY() < -halfExtent.getY() ? 0x02 : 0x0) |    
		   (p.getY() >  halfExtent.getY() ? 0x10 : 0x0) |
		   (p.getZ() < -halfExtent.getZ() ? 0x4 : 0x0) |    
		   (p.getZ() >  halfExtent.getZ() ? 0x20 : 0x0);
}



D_SIMD_FORCE_INLINE bool D_btRayAabb2(const D_btVector3& rayFrom,
								  const D_btVector3& rayInvDirection,
								  const unsigned int raySign[3],
								  const D_btVector3 bounds[2],
								  D_btScalar& tmin,
								  D_btScalar lambda_min,
								  D_btScalar lambda_max)
{
	D_btScalar tmax, tymin, tymax, tzmin, tzmax;
	tmin = (bounds[raySign[0]].getX() - rayFrom.getX()) * rayInvDirection.getX();
	tmax = (bounds[1-raySign[0]].getX() - rayFrom.getX()) * rayInvDirection.getX();
	tymin = (bounds[raySign[1]].getY() - rayFrom.getY()) * rayInvDirection.getY();
	tymax = (bounds[1-raySign[1]].getY() - rayFrom.getY()) * rayInvDirection.getY();

	if ( (tmin > tymax) || (tymin > tmax) )
		return false;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	tzmin = (bounds[raySign[2]].getZ() - rayFrom.getZ()) * rayInvDirection.getZ();
	tzmax = (bounds[1-raySign[2]].getZ() - rayFrom.getZ()) * rayInvDirection.getZ();

	if ( (tmin > tzmax) || (tzmin > tmax) )
		return false;
	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;
	return ( (tmin < lambda_max) && (tmax > lambda_min) );
}

D_SIMD_FORCE_INLINE bool D_btRayAabb(const D_btVector3& rayFrom, 
								 const D_btVector3& rayTo, 
								 const D_btVector3& aabbMin, 
								 const D_btVector3& aabbMax,
					  D_btScalar& param, D_btVector3& normal) 
{
	D_btVector3 aabbHalfExtent = (aabbMax-aabbMin)* D_btScalar(0.5);
	D_btVector3 aabbCenter = (aabbMax+aabbMin)* D_btScalar(0.5);
	D_btVector3	source = rayFrom - aabbCenter;
	D_btVector3	target = rayTo - aabbCenter;
	int	sourceOutcode = D_btOutcode(source,aabbHalfExtent);
	int targetOutcode = D_btOutcode(target,aabbHalfExtent);
	if ((sourceOutcode & targetOutcode) == 0x0)
	{
		D_btScalar lambda_enter = D_btScalar(0.0);
		D_btScalar lambda_exit  = param;
		D_btVector3 r = target - source;
		int i;
		D_btScalar	normSign = 1;
		D_btVector3	hitNormal(0,0,0);
		int bit=1;

		for (int j=0;j<2;j++)
		{
			for (i = 0; i != 3; ++i)
			{
				if (sourceOutcode & bit)
				{
					D_btScalar lambda = (-source[i] - aabbHalfExtent[i]*normSign) / r[i];
					if (lambda_enter <= lambda)
					{
						lambda_enter = lambda;
						hitNormal.setValue(0,0,0);
						hitNormal[i] = normSign;
					}
				}
				else if (targetOutcode & bit) 
				{
					D_btScalar lambda = (-source[i] - aabbHalfExtent[i]*normSign) / r[i];
					D_btSetMin(lambda_exit, lambda);
				}
				bit<<=1;
			}
			normSign = D_btScalar(-1.);
		}
		if (lambda_enter <= lambda_exit)
		{
			param = lambda_enter;
			normal = hitNormal;
			return true;
		}
	}
	return false;
}



D_SIMD_FORCE_INLINE	void D_btTransformAabb(const D_btVector3& halfExtents, D_btScalar margin,const D_btTransform& t,D_btVector3& aabbMinOut,D_btVector3& aabbMaxOut)
{
	D_btVector3 halfExtentsWithMargin = halfExtents+D_btVector3(margin,margin,margin);
	D_btMatrix3x3 abs_b = t.getBasis().absolute();  
	D_btVector3 center = t.getOrigin();
	D_btVector3 extent = D_btVector3(abs_b[0].dot(halfExtentsWithMargin),
		   abs_b[1].dot(halfExtentsWithMargin),
		  abs_b[2].dot(halfExtentsWithMargin));
	aabbMinOut = center - extent;
	aabbMaxOut = center + extent;
}


D_SIMD_FORCE_INLINE	void D_btTransformAabb(const D_btVector3& localAabbMin,const D_btVector3& localAabbMax, D_btScalar margin,const D_btTransform& trans,D_btVector3& aabbMinOut,D_btVector3& aabbMaxOut)
{
		D_btAssert(localAabbMin.getX() <= localAabbMax.getX());
		D_btAssert(localAabbMin.getY() <= localAabbMax.getY());
		D_btAssert(localAabbMin.getZ() <= localAabbMax.getZ());
		D_btVector3 localHalfExtents = D_btScalar(0.5)*(localAabbMax-localAabbMin);
		localHalfExtents+=D_btVector3(margin,margin,margin);

		D_btVector3 localCenter = D_btScalar(0.5)*(localAabbMax+localAabbMin);
		D_btMatrix3x3 abs_b = trans.getBasis().absolute();  
		D_btVector3 center = trans(localCenter);
		D_btVector3 extent = D_btVector3(abs_b[0].dot(localHalfExtents),
			   abs_b[1].dot(localHalfExtents),
			  abs_b[2].dot(localHalfExtents));
		aabbMinOut = center-extent;
		aabbMaxOut = center+extent;
}

#define D_USE_BANCHLESS 1
#ifdef D_USE_BANCHLESS
	//This block replaces the block below D_and uses D_no branches, D_and replaces the 8 bit return with a 32 bit return for improved performance (~3x on XBox 360)
	D_SIMD_FORCE_INLINE unsigned testQuantizedAabbAgainstQuantizedAabb(const unsigned short int* aabbMin1,const unsigned short int* aabbMax1,const unsigned short int* aabbMin2,const unsigned short int* aabbMax2)
	{		
		return static_cast<unsigned int>(D_btSelect((unsigned)((aabbMin1[0] <= aabbMax2[0]) & (aabbMax1[0] >= aabbMin2[0])
			& (aabbMin1[2] <= aabbMax2[2]) & (aabbMax1[2] >= aabbMin2[2])
			& (aabbMin1[1] <= aabbMax2[1]) & (aabbMax1[1] >= aabbMin2[1])),
			1, 0));
	}
#else
	D_SIMD_FORCE_INLINE bool testQuantizedAabbAgainstQuantizedAabb(const unsigned short int* aabbMin1,const unsigned short int* aabbMax1,const unsigned short int* aabbMin2,const unsigned short int* aabbMax2)
	{
		bool overlap = true;
		overlap = (aabbMin1[0] > aabbMax2[0] || aabbMax1[0] < aabbMin2[0]) ? false : overlap;
		overlap = (aabbMin1[2] > aabbMax2[2] || aabbMax1[2] < aabbMin2[2]) ? false : overlap;
		overlap = (aabbMin1[1] > aabbMax2[1] || aabbMax1[1] < aabbMin2[1]) ? false : overlap;
		return overlap;
	}
#endif //D_USE_BANCHLESS

#endif


