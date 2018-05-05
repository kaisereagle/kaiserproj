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

#include "btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "LinearMath/btQuaternion.h"


D_btConvexHullShape ::D_btConvexHullShape (const D_btScalar* points,int numPoints,int stride) : D_btPolyhedralConvexAabbCachingShape ()
{
	m_shapeType = D_CONVEX_HULL_SHAPE_PROXYTYPE;
	m_unscaledPoints.resize(numPoints);

	unsigned char* pointsAddress = (unsigned char*)points;

	for (int i=0;i<numPoints;i++)
	{
		D_btScalar* point = (D_btScalar*)pointsAddress;
		m_unscaledPoints[i] = D_btVector3(point[0], point[1], point[2]);
		pointsAddress += stride;
	}

	recalcLocalAabb();

}



void D_btConvexHullShape::setLocalScaling(const D_btVector3& scaling)
{
	m_localScaling = scaling;
	recalcLocalAabb();
}

void D_btConvexHullShape::addPoint(const D_btVector3& point)
{
	m_unscaledPoints.push_back(point);
	recalcLocalAabb();

}

D_btVector3	D_btConvexHullShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec0)const
{
	D_btVector3 supVec(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	D_btScalar newDot,maxDot = D_btScalar(-D_BT_LARGE_FLOAT);

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


	for (int i=0;i<m_unscaledPoints.size();i++)
	{
		D_btVector3 vtx = m_unscaledPoints[i] * m_localScaling;

		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}
	return supVec;
}

void	D_btConvexHullShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
	D_btScalar newDot;
	//use 'w' component of supportVerticesOut?
	{
		for (int i=0;i<numVectors;i++)
		{
			supportVerticesOut[i][3] = D_btScalar(-D_BT_LARGE_FLOAT);
		}
	}
	for (int i=0;i<m_unscaledPoints.size();i++)
	{
		D_btVector3 vtx = getScaledPoint(i);

		for (int j=0;j<numVectors;j++)
		{
			const D_btVector3& vec = vectors[j];
			
			newDot = vec.dot(vtx);
			if (newDot > supportVerticesOut[j][3])
			{
				//WARNING: don't swap next lines, the w component would get overwritten!
				supportVerticesOut[j] = vtx;
				supportVerticesOut[j][3] = newDot;
			}
		}
	}



}
	


D_btVector3	D_btConvexHullShape::localGetSupportingVertex(const D_btVector3& vec)const
{
	D_btVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);

	if ( getMargin()!=D_btScalar(0.) )
	{
		D_btVector3 vecnorm = vec;
		if (vecnorm .length2() < (D_SIMD_EPSILON*D_SIMD_EPSILON))
		{
			vecnorm.setValue(D_btScalar(-1.),D_btScalar(-1.),D_btScalar(-1.));
		} 
		vecnorm.normalize();
		supVertex+= getMargin() * vecnorm;
	}
	return supVertex;
}









//currently D_just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you D_can D_debug-draw D_btConvexHullShape with the Raytracer Demo
int	D_btConvexHullShape::getNumVertices() const
{
	return m_unscaledPoints.size();
}

int D_btConvexHullShape::getNumEdges() const
{
	return m_unscaledPoints.size();
}

void D_btConvexHullShape::getEdge(int i,D_btVector3& pa,D_btVector3& pb) const
{

	int index0 = i%m_unscaledPoints.size();
	int index1 = (i+1)%m_unscaledPoints.size();
	pa = getScaledPoint(index0);
	pb = getScaledPoint(index1);
}

void D_btConvexHullShape::getVertex(int i,D_btVector3& vtx) const
{
	vtx = getScaledPoint(i);
}

int	D_btConvexHullShape::getNumPlanes() const
{
	return 0;
}

void D_btConvexHullShape::getPlane(D_btVector3& ,D_btVector3& ,int ) const
{

	D_btAssert(0);
}

//not yet
bool D_btConvexHullShape::isInside(const D_btVector3& ,D_btScalar ) const
{
	D_btAssert(0);
	return false;
}

