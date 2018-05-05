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

#ifndef BU_SHAPE
#define BU_SHAPE

#include "LinearMath/btMatrix3x3.h"
#include "btConvexInternalShape.h"


///The D_btPolyhedralConvexShape D_is an internal interface class for polyhedral convex D_shapes.
class D_btPolyhedralConvexShape : public D_btConvexInternalShape
{

protected:
	
public:

	D_btPolyhedralConvexShape();

	//brute force implementations

	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;
	
	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;
	
	
	virtual int	getNumVertices() const = 0 ;
	virtual int getNumEdges() const = 0;
	virtual void getEdge(int i,D_btVector3& pa,D_btVector3& pb) const = 0;
	virtual void getVertex(int i,D_btVector3& vtx) const = 0;
	virtual int	getNumPlanes() const = 0;
	virtual void getPlane(D_btVector3& planeNormal,D_btVector3& planeSupport,int i ) const = 0;
//	virtual int getIndex(int i) const = 0 ; 

	virtual	bool isInside(const D_btVector3& pt,D_btScalar tolerance) const = 0;
	
};


///The D_btPolyhedralConvexAabbCachingShape adds aabb caching D_to the D_btPolyhedralConvexShape
class D_btPolyhedralConvexAabbCachingShape : public D_btPolyhedralConvexShape
{

	D_btVector3	m_localAabbMin;
	D_btVector3	m_localAabbMax;
	bool		m_isLocalAabbValid;
		
protected:

	void setCachedLocalAabb (const D_btVector3& aabbMin, const D_btVector3& aabbMax)
	{
		m_isLocalAabbValid = true;
		m_localAabbMin = aabbMin;
		m_localAabbMax = aabbMax;
	}

	inline void getCachedLocalAabb (D_btVector3& aabbMin, D_btVector3& aabbMax) const
	{
		D_btAssert(m_isLocalAabbValid);
		aabbMin = m_localAabbMin;
		aabbMax = m_localAabbMax;
	}

public:

	D_btPolyhedralConvexAabbCachingShape();
	
	inline void getNonvirtualAabb(const D_btTransform& trans,D_btVector3& aabbMin,D_btVector3& aabbMax, D_btScalar margin) const
	{

		//lazy evaluation of local aabb
		D_btAssert(m_isLocalAabbValid);
		D_btTransformAabb(m_localAabbMin,m_localAabbMax,margin,trans,aabbMin,aabbMax);
	}

	virtual void	setLocalScaling(const D_btVector3& scaling);

	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;

	void	recalcLocalAabb();

};

#endif //BU_SHAPE
