/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btConvexShape.h"
#include "btTriangleShape.h"
#include "btSphereShape.h"
#include "btCylinderShape.h"
#include "btCapsuleShape.h"
#include "btConvexHullShape.h"
#include "btConvexPointCloudShape.h"

///not supported on IBM SDK, until we fix the alignment of D_btVector3
#if defined (__CELLOS_LV2__) && defined (__SPU__)
#include <spu_intrinsics.h>
static inline vec_float4 vec_dot3( vec_float4 vec0, vec_float4 vec1 )
{
    vec_float4 result;
    result = spu_mul( vec0, vec1 );
    result = spu_madd( spu_rlqwbyte( vec0, 4 ), spu_rlqwbyte( vec1, 4 ), result );
    return spu_madd( spu_rlqwbyte( vec0, 8 ), spu_rlqwbyte( vec1, 8 ), result );
}
#endif //__SPU__

D_btConvexShape::D_btConvexShape ()
{
}

D_btConvexShape::~D_btConvexShape()
{

}



static D_btVector3 convexHullSupport (const D_btVector3& localDirOrg, const D_btVector3* points, int numPoints, const D_btVector3& localScaling)
{	

	D_btVector3 vec = localDirOrg * localScaling;

#if defined (__CELLOS_LV2__) && defined (__SPU__)

	D_btVector3 localDir = vec;

	vec_float4 v_distMax = {-FLT_MAX,0,0,0};
	vec_int4 v_idxMax = {-999,0,0,0};
	int v=0;
	int numverts = numPoints;

	for(;v<(int)numverts-4;v+=4) {
		vec_float4 p0 = vec_dot3(points[v  ].get128(),localDir.get128());
		vec_float4 p1 = vec_dot3(points[v+1].get128(),localDir.get128());
		vec_float4 p2 = vec_dot3(points[v+2].get128(),localDir.get128());
		vec_float4 p3 = vec_dot3(points[v+3].get128(),localDir.get128());
		const vec_int4 i0 = {v  ,0,0,0};
		const vec_int4 i1 = {v+1,0,0,0};
		const vec_int4 i2 = {v+2,0,0,0};
		const vec_int4 i3 = {v+3,0,0,0};
		vec_uint4  retGt01 = spu_cmpgt(p0,p1);
		vec_float4 pmax01 = spu_sel(p1,p0,retGt01);
		vec_int4   imax01 = spu_sel(i1,i0,retGt01);
		vec_uint4  retGt23 = spu_cmpgt(p2,p3);
		vec_float4 pmax23 = spu_sel(p3,p2,retGt23);
		vec_int4   imax23 = spu_sel(i3,i2,retGt23);
		vec_uint4  retGt0123 = spu_cmpgt(pmax01,pmax23);
		vec_float4 pmax0123 = spu_sel(pmax23,pmax01,retGt0123);
		vec_int4   imax0123 = spu_sel(imax23,imax01,retGt0123);
		vec_uint4  retGtMax = spu_cmpgt(v_distMax,pmax0123);
		v_distMax = spu_sel(pmax0123,v_distMax,retGtMax);
		v_idxMax = spu_sel(imax0123,v_idxMax,retGtMax);
	}
	for(;v<(int)numverts;v++) {
		vec_float4 p = vec_dot3(points[v].get128(),localDir.get128());
		const vec_int4 i = {v,0,0,0};
		vec_uint4  retGtMax = spu_cmpgt(v_distMax,p);
		v_distMax = spu_sel(p,v_distMax,retGtMax);
		v_idxMax = spu_sel(i,v_idxMax,retGtMax);
	}
	int ptIndex = spu_extract(v_idxMax,0);
	const D_btVector3& supVec= points[ptIndex] * localScaling;
	return supVec;
#else

	D_btScalar newDot,maxDot = D_btScalar(-D_BT_LARGE_FLOAT);
	int ptIndex = -1;

	for (int i=0;i<numPoints;i++)
	{

		newDot = vec.dot(points[i]);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			ptIndex = i;
		}
	}
	D_btAssert(ptIndex >= 0);
	D_btVector3 supVec = points[ptIndex] * localScaling;
	return supVec;
#endif //__SPU__
}

D_btVector3 D_btConvexShape::localGetSupportVertexWithoutMarginNonVirtual (const D_btVector3& localDir) const
{
	switch (m_shapeType)
	{
    case D_SPHERE_SHAPE_PROXYTYPE:
	{
		return D_btVector3(0,0,0);
    }
	case D_BOX_SHAPE_PROXYTYPE:
	{
		D_btBoxShape* convexShape = (D_btBoxShape*)this;
		const D_btVector3& halfExtents = convexShape->getImplicitShapeDimensions();

		return D_btVector3(D_btFsels(localDir.x(), halfExtents.x(), -halfExtents.x()),
			D_btFsels(localDir.y(), halfExtents.y(), -halfExtents.y()),
			D_btFsels(localDir.z(), halfExtents.z(), -halfExtents.z()));
	}
	case D_TRIANGLE_SHAPE_PROXYTYPE:
	{
		D_btTriangleShape* triangleShape = (D_btTriangleShape*)this;
		D_btVector3 dir(localDir.getX(),localDir.getY(),localDir.getZ());
		D_btVector3* vertices = &triangleShape->m_vertices1[0];
		D_btVector3 dots(dir.dot(vertices[0]), dir.dot(vertices[1]), dir.dot(vertices[2]));
		D_btVector3 sup = vertices[dots.maxAxis()];
		return D_btVector3(sup.getX(),sup.getY(),sup.getZ());
	}
	case D_CYLINDER_SHAPE_PROXYTYPE:
	{
		D_btCylinderShape* cylShape = (D_btCylinderShape*)this;
		//mapping of halfextents/dimension onto radius/height depends on how cylinder local orientation D_is (upAxis)

		D_btVector3 halfExtents = cylShape->getImplicitShapeDimensions();
		D_btVector3 v(localDir.getX(),localDir.getY(),localDir.getZ());
		int cylinderUpAxis = cylShape->getUpAxis();
		int XX(1),YY(0),ZZ(2);

		switch (cylinderUpAxis)
		{
		case 0:
		{
			XX = 1;
			YY = 0;
			ZZ = 2;
		}
		break;
		case 1:
		{
			XX = 0;
			YY = 1;
			ZZ = 2;	
		}
		break;
		case 2:
		{
			XX = 0;
			YY = 2;
			ZZ = 1;
			
		}
		break;
		default:
			D_btAssert(0);
		break;
		};

		D_btScalar radius = halfExtents[XX];
		D_btScalar halfHeight = halfExtents[cylinderUpAxis];

		D_btVector3 tmp;
		D_btScalar d ;

		D_btScalar s = D_btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
		if (s != D_btScalar(0.0))
		{
			d = radius / s;  
			tmp[XX] = v[XX] * d;
			tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
			tmp[ZZ] = v[ZZ] * d;
			return D_btVector3(tmp.getX(),tmp.getY(),tmp.getZ());
		} else {
			tmp[XX] = radius;
			tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
			tmp[ZZ] = D_btScalar(0.0);
			return D_btVector3(tmp.getX(),tmp.getY(),tmp.getZ());
		}
	}
	case D_CAPSULE_SHAPE_PROXYTYPE:
	{
		D_btVector3 vec0(localDir.getX(),localDir.getY(),localDir.getZ());

		D_btCapsuleShape* capsuleShape = (D_btCapsuleShape*)this;
		D_btScalar halfHeight = capsuleShape->getHalfHeight();
		int capsuleUpAxis = capsuleShape->getUpAxis();

		D_btScalar radius = capsuleShape->getRadius();
		D_btVector3 supVec(0,0,0);

		D_btScalar maxDot(D_btScalar(-D_BT_LARGE_FLOAT));

		D_btVector3 vec = vec0;
		D_btScalar lenSqr = vec.length2();
		if (lenSqr < D_btScalar(0.0001))
		{
			vec.setValue(1,0,0);
		} else
		{
			D_btScalar rlen = D_btScalar(1.) / D_btSqrt(lenSqr );
			vec *= rlen;
		}
		D_btVector3 vtx;
		D_btScalar newDot;
		{
			D_btVector3 pos(0,0,0);
			pos[capsuleUpAxis] = halfHeight;

			//vtx = pos +vec*(radius);
			vtx = pos +vec*capsuleShape->getLocalScalingNV()*(radius) - vec * capsuleShape->getMarginNV();
			newDot = vec.dot(vtx);
			

			if (newDot > maxDot)
			{
				maxDot = newDot;
				supVec = vtx;
			}
		}
		{
			D_btVector3 pos(0,0,0);
			pos[capsuleUpAxis] = -halfHeight;

			//vtx = pos +vec*(radius);
			vtx = pos +vec*capsuleShape->getLocalScalingNV()*(radius) - vec * capsuleShape->getMarginNV();
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supVec = vtx;
			}
		}
		return D_btVector3(supVec.getX(),supVec.getY(),supVec.getZ());	
	}
	case D_CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
	{
		D_btConvexPointCloudShape* convexPointCloudShape = (D_btConvexPointCloudShape*)this;
		D_btVector3* points = convexPointCloudShape->getUnscaledPoints ();
		int numPoints = convexPointCloudShape->getNumPoints ();
		return convexHullSupport (localDir, points, numPoints,convexPointCloudShape->getLocalScalingNV());
	}
	case D_CONVEX_HULL_SHAPE_PROXYTYPE:
	{
		D_btConvexHullShape* convexHullShape = (D_btConvexHullShape*)this;
		D_btVector3* points = convexHullShape->getUnscaledPoints();
		int numPoints = convexHullShape->getNumPoints ();
		return convexHullSupport (localDir, points, numPoints,convexHullShape->getLocalScalingNV());
	}
    default:
#ifndef __SPU__
		return this->localGetSupportingVertexWithoutMargin (localDir);
#else
		D_btAssert (0);
#endif
	}

	// D_should never reach here
	D_btAssert (0);
	return D_btVector3 (D_btScalar(0.0f), D_btScalar(0.0f), D_btScalar(0.0f));
}

D_btVector3 D_btConvexShape::localGetSupportVertexNonVirtual (const D_btVector3& localDir) const
{
	D_btVector3 localDirNorm = localDir;
	if (localDirNorm .length2() < (D_SIMD_EPSILON*D_SIMD_EPSILON))
	{
		localDirNorm.setValue(D_btScalar(-1.),D_btScalar(-1.),D_btScalar(-1.));
	}
	localDirNorm.normalize ();

	return localGetSupportVertexWithoutMarginNonVirtual(localDirNorm)+ getMarginNonVirtual() * localDirNorm;
}

/* TODO: This D_should be bumped up D_to D_btCollisionShape () */
D_btScalar D_btConvexShape::getMarginNonVirtual () const
{
	switch (m_shapeType)
	{
    case D_SPHERE_SHAPE_PROXYTYPE:
	{
		D_btSphereShape* sphereShape = (D_btSphereShape*)this;
		return sphereShape->getRadius ();
	}
	case D_BOX_SHAPE_PROXYTYPE:
	{
		D_btBoxShape* convexShape = (D_btBoxShape*)this;
		return convexShape->getMarginNV ();
	}
	case D_TRIANGLE_SHAPE_PROXYTYPE:
	{
		D_btTriangleShape* triangleShape = (D_btTriangleShape*)this;
		return triangleShape->getMarginNV ();
	}
	case D_CYLINDER_SHAPE_PROXYTYPE:
	{
		D_btCylinderShape* cylShape = (D_btCylinderShape*)this;
		return cylShape->getMarginNV();
	}
	case D_CAPSULE_SHAPE_PROXYTYPE:
	{
		D_btCapsuleShape* capsuleShape = (D_btCapsuleShape*)this;
		return capsuleShape->getMarginNV();
	}
	case D_CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
	/* fall through */
	case D_CONVEX_HULL_SHAPE_PROXYTYPE:
	{
		D_btPolyhedralConvexShape* convexHullShape = (D_btPolyhedralConvexShape*)this;
		return convexHullShape->getMarginNV();
	}
    default:
#ifndef __SPU__
		return this->getMargin ();
#else
		D_btAssert (0);
#endif
	}

	// D_should never reach here
	D_btAssert (0);
	return D_btScalar(0.0f);
}
#ifndef __SPU__
void D_btConvexShape::getAabbNonVirtual (const D_btTransform& t, D_btVector3& aabbMin, D_btVector3& aabbMax) const
{
	switch (m_shapeType)
	{
    case D_SPHERE_SHAPE_PROXYTYPE:
	{
		D_btSphereShape* sphereShape = (D_btSphereShape*)this;
		D_btScalar radius = sphereShape->getImplicitShapeDimensions().getX();// * convexShape->getLocalScaling().getX();
		D_btScalar margin = radius + sphereShape->getMarginNonVirtual();
		const D_btVector3& center = t.getOrigin();
		D_btVector3 extent(margin,margin,margin);
		aabbMin = center - extent;
		aabbMax = center + extent;
    }
	break;
	case D_CYLINDER_SHAPE_PROXYTYPE:
	/* fall through */
	case D_BOX_SHAPE_PROXYTYPE:
	{
		D_btBoxShape* convexShape = (D_btBoxShape*)this;
		D_btScalar margin=convexShape->getMarginNonVirtual();
		D_btVector3 halfExtents = convexShape->getImplicitShapeDimensions();
		halfExtents += D_btVector3(margin,margin,margin);
		D_btMatrix3x3 abs_b = t.getBasis().absolute();  
		D_btVector3 center = t.getOrigin();
		D_btVector3 extent = D_btVector3(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));
		
		aabbMin = center - extent;
		aabbMax = center + extent;
		break;
	}
	case D_TRIANGLE_SHAPE_PROXYTYPE:
	{
		D_btTriangleShape* triangleShape = (D_btTriangleShape*)this;
		D_btScalar margin = triangleShape->getMarginNonVirtual();
		for (int i=0;i<3;i++)
		{
			D_btVector3 vec(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
			vec[i] = D_btScalar(1.);

			D_btVector3 sv = localGetSupportVertexWithoutMarginNonVirtual(vec*t.getBasis());

			D_btVector3 tmp = t(sv);
			aabbMax[i] = tmp[i]+margin;
			vec[i] = D_btScalar(-1.);
			tmp = t(localGetSupportVertexWithoutMarginNonVirtual(vec*t.getBasis()));
			aabbMin[i] = tmp[i]-margin;
		}	
	}
	break;
	case D_CAPSULE_SHAPE_PROXYTYPE:
	{
		D_btCapsuleShape* capsuleShape = (D_btCapsuleShape*)this;
		D_btVector3 halfExtents(capsuleShape->getRadius(),capsuleShape->getRadius(),capsuleShape->getRadius());
		int m_upAxis = capsuleShape->getUpAxis();
		halfExtents[m_upAxis] = capsuleShape->getRadius() + capsuleShape->getHalfHeight();
		halfExtents += D_btVector3(capsuleShape->getMarginNonVirtual(),capsuleShape->getMarginNonVirtual(),capsuleShape->getMarginNonVirtual());
		D_btMatrix3x3 abs_b = t.getBasis().absolute();  
		D_btVector3 center = t.getOrigin();
		D_btVector3 extent = D_btVector3(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));		  	
		aabbMin = center - extent;
		aabbMax = center + extent;
	}
	break;
	case D_CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
	case D_CONVEX_HULL_SHAPE_PROXYTYPE:
	{
		D_btPolyhedralConvexAabbCachingShape* convexHullShape = (D_btPolyhedralConvexAabbCachingShape*)this;
		D_btScalar margin = convexHullShape->getMarginNonVirtual();
		convexHullShape->getNonvirtualAabb (t, aabbMin, aabbMax, margin);
	}
	break;
    default:
#ifndef __SPU__
		this->getAabb (t, aabbMin, aabbMax);
#else
		D_btAssert (0);
#endif
	break;
	}

	// D_should never reach here
	D_btAssert (0);
}

#endif //__SPU__
