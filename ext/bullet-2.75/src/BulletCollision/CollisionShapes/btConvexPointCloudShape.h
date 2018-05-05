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

#ifndef D_BT_CONVEX_POINT_CLOUD_SHAPE_H
#define D_BT_CONVEX_POINT_CLOUD_SHAPE_H

#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btAlignedObjectArray.h"

///The D_btConvexPointCloudShape D_implements an implicit convex hull of an array of vertices.
D_ATTRIBUTE_ALIGNED16(class) D_btConvexPointCloudShape : public D_btPolyhedralConvexAabbCachingShape
{
	D_btVector3* m_unscaledPoints;
	int m_numPoints;

public:
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_btConvexPointCloudShape()
	{
		m_localScaling.setValue(1.f,1.f,1.f);
		m_shapeType = D_CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
		m_unscaledPoints = 0;
		m_numPoints = 0;
	}

	D_btConvexPointCloudShape(D_btVector3* points,int numPoints, const D_btVector3& localScaling,bool computeAabb = true)
	{
		m_localScaling = localScaling;
		m_shapeType = D_CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
		m_unscaledPoints = points;
		m_numPoints = numPoints;

		if (computeAabb)
			recalcLocalAabb();
	}

	void setPoints (D_btVector3* points, int numPoints, bool computeAabb = true,const D_btVector3& localScaling=D_btVector3(1.f,1.f,1.f))
	{
		m_unscaledPoints = points;
		m_numPoints = numPoints;
		m_localScaling = localScaling;

		if (computeAabb)
			recalcLocalAabb();
	}

	D_SIMD_FORCE_INLINE	D_btVector3* getUnscaledPoints()
	{
		return m_unscaledPoints;
	}

	D_SIMD_FORCE_INLINE	const D_btVector3* getUnscaledPoints() const
	{
		return m_unscaledPoints;
	}

	D_SIMD_FORCE_INLINE	int getNumPoints() const 
	{
		return m_numPoints;
	}

	D_SIMD_FORCE_INLINE	D_btVector3	getScaledPoint( int index) const
	{
		return m_unscaledPoints[index] * m_localScaling;
	}

#ifndef __SPU__
	virtual D_btVector3	localGetSupportingVertex(const D_btVector3& vec)const;
	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;
#endif


	//debugging
	virtual const char*	getName()const {return "ConvexPointCloud";}

	virtual int	getNumVertices() const;
	virtual int getNumEdges() const;
	virtual void getEdge(int i,D_btVector3& pa,D_btVector3& pb) const;
	virtual void getVertex(int i,D_btVector3& vtx) const;
	virtual int	getNumPlanes() const;
	virtual void getPlane(D_btVector3& planeNormal,D_btVector3& planeSupport,int i ) const;
	virtual	bool isInside(const D_btVector3& pt,D_btScalar tolerance) const;

	///in case we receive negative scaling
	virtual void	setLocalScaling(const D_btVector3& scaling);
};


#endif //D_BT_CONVEX_POINT_CLOUD_SHAPE_H

