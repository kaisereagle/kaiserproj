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


#include "btScaledBvhTriangleMeshShape.h"

D_btScaledBvhTriangleMeshShape::D_btScaledBvhTriangleMeshShape(D_btBvhTriangleMeshShape* childShape,const D_btVector3& localScaling)
:m_localScaling(localScaling),m_bvhTriMeshShape(childShape)
{
	m_shapeType = D_SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;
}

D_btScaledBvhTriangleMeshShape::~D_btScaledBvhTriangleMeshShape()
{
}


class D_btScaledTriangleCallback : public D_btTriangleCallback
{
	D_btTriangleCallback* m_originalCallback;

	D_btVector3	m_localScaling;

public:

	D_btScaledTriangleCallback(D_btTriangleCallback* originalCallback,const D_btVector3& localScaling)
		:m_originalCallback(originalCallback),
		m_localScaling(localScaling)
	{
	}

	virtual void processTriangle(D_btVector3* triangle, int partId, int triangleIndex)
	{
		D_btVector3 newTriangle[3];
		newTriangle[0] = triangle[0]*m_localScaling;
		newTriangle[1] = triangle[1]*m_localScaling;
		newTriangle[2] = triangle[2]*m_localScaling;
		m_originalCallback->processTriangle(&newTriangle[0],partId,triangleIndex);
	}
};

void	D_btScaledBvhTriangleMeshShape::processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{
	D_btScaledTriangleCallback scaledCallback(callback,m_localScaling);
	
	D_btVector3 invLocalScaling(1.f/m_localScaling.getX(),1.f/m_localScaling.getY(),1.f/m_localScaling.getZ());
	D_btVector3 scaledAabbMin,scaledAabbMax;

	///support negative scaling
	scaledAabbMin[0] = m_localScaling.getX() >= 0. ? aabbMin[0] * invLocalScaling[0] : aabbMax[0] * invLocalScaling[0];
	scaledAabbMin[1] = m_localScaling.getY() >= 0. ? aabbMin[1] * invLocalScaling[1] : aabbMax[1] * invLocalScaling[1];
	scaledAabbMin[2] = m_localScaling.getZ() >= 0. ? aabbMin[2] * invLocalScaling[2] : aabbMax[2] * invLocalScaling[2];
	
	scaledAabbMax[0] = m_localScaling.getX() <= 0. ? aabbMin[0] * invLocalScaling[0] : aabbMax[0] * invLocalScaling[0];
	scaledAabbMax[1] = m_localScaling.getY() <= 0. ? aabbMin[1] * invLocalScaling[1] : aabbMax[1] * invLocalScaling[1];
	scaledAabbMax[2] = m_localScaling.getZ() <= 0. ? aabbMin[2] * invLocalScaling[2] : aabbMax[2] * invLocalScaling[2];
	
	
	m_bvhTriMeshShape->processAllTriangles(&scaledCallback,scaledAabbMin,scaledAabbMax);
}


void	D_btScaledBvhTriangleMeshShape::getAabb(const D_btTransform& trans,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	D_btVector3 localAabbMin = m_bvhTriMeshShape->getLocalAabbMin();
	D_btVector3 localAabbMax = m_bvhTriMeshShape->getLocalAabbMax();

	D_btVector3 tmpLocalAabbMin = localAabbMin * m_localScaling;
	D_btVector3 tmpLocalAabbMax = localAabbMax * m_localScaling;

	localAabbMin[0] = (m_localScaling.getX() >= 0.) ? tmpLocalAabbMin[0] : tmpLocalAabbMax[0];
	localAabbMin[1] = (m_localScaling.getY() >= 0.) ? tmpLocalAabbMin[1] : tmpLocalAabbMax[1];
	localAabbMin[2] = (m_localScaling.getZ() >= 0.) ? tmpLocalAabbMin[2] : tmpLocalAabbMax[2];
	localAabbMax[0] = (m_localScaling.getX() <= 0.) ? tmpLocalAabbMin[0] : tmpLocalAabbMax[0];
	localAabbMax[1] = (m_localScaling.getY() <= 0.) ? tmpLocalAabbMin[1] : tmpLocalAabbMax[1];
	localAabbMax[2] = (m_localScaling.getZ() <= 0.) ? tmpLocalAabbMin[2] : tmpLocalAabbMax[2];

	D_btVector3 localHalfExtents = D_btScalar(0.5)*(localAabbMax-localAabbMin);
	D_btScalar margin = m_bvhTriMeshShape->getMargin();
	localHalfExtents += D_btVector3(margin,margin,margin);
	D_btVector3 localCenter = D_btScalar(0.5)*(localAabbMax+localAabbMin);
	
	D_btMatrix3x3 abs_b = trans.getBasis().absolute();  

	D_btVector3 center = trans(localCenter);

	D_btVector3 extent = D_btVector3(abs_b[0].dot(localHalfExtents),
		   abs_b[1].dot(localHalfExtents),
		  abs_b[2].dot(localHalfExtents));
	aabbMin = center - extent;
	aabbMax = center + extent;

}

void	D_btScaledBvhTriangleMeshShape::setLocalScaling(const D_btVector3& scaling)
{
	m_localScaling = scaling;
}

const D_btVector3& D_btScaledBvhTriangleMeshShape::getLocalScaling() const
{
	return m_localScaling;
}

void	D_btScaledBvhTriangleMeshShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	///don't make this a movable object!
//	D_btAssert(0);
}
