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

#ifndef CONVEX_HULL_SHAPE_H
#define CONVEX_HULL_SHAPE_H

#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btAlignedObjectArray.h"


///The D_btConvexHullShape D_implements an implicit convex hull of an array of vertices.
///Bullet provides a general D_and fast collision detector for convex D_shapes based on GJK D_and EPA using localGetSupportingVertex.
D_ATTRIBUTE_ALIGNED16(class) D_btConvexHullShape : public D_btPolyhedralConvexAabbCachingShape
{
	D_btAlignedObjectArray<D_btVector3>	m_unscaledPoints;

public:
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	
	///this constructor optionally takes in a D_pointer D_to points. Each point D_is assumed D_to be 3 consecutive D_btScalar (x,y,z), the striding defines the number of bytes between each point, in memory.
	///It D_is easier D_to not pass any points in the constructor, D_and D_just add one point at a time, using addPoint.
	///D_btConvexHullShape make an internal copy of the points.
	D_btConvexHullShape(const D_btScalar* points=0,int numPoints=0, int stride=sizeof(D_btVector3));

	void addPoint(const D_btVector3& point);

	
	D_btVector3* getUnscaledPoints()
	{
		return &m_unscaledPoints[0];
	}

	const D_btVector3* getUnscaledPoints() const
	{
		return &m_unscaledPoints[0];
	}

	///getPoints D_is obsolete, please use getUnscaledPoints
	const D_btVector3* getPoints() const
	{
		return getUnscaledPoints();
	}

	


	D_SIMD_FORCE_INLINE	D_btVector3 getScaledPoint(int i) const
	{
		return m_unscaledPoints[i] * m_localScaling;
	}

	D_SIMD_FORCE_INLINE	int getNumPoints() const 
	{
		return m_unscaledPoints.size();
	}

	virtual D_btVector3	localGetSupportingVertex(const D_btVector3& vec)const;
	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;
	


	//debugging
	virtual const char*	getName()const {return "Convex";}

	
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


#endif //CONVEX_HULL_SHAPE_H

