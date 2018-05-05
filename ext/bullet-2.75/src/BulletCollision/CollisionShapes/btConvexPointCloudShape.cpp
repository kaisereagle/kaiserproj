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

#include "btConvexPointCloudShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "LinearMath/btQuaternion.h"

void D_btConvexPointCloudShape::setLocalScaling(const D_btVector3& scaling)
{
	m_localScaling = scaling;
	recalcLocalAabb();
}

#ifndef __SPU__
D_btVector3	D_btConvexPointCloudShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec0)const
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


	for (int i=0;i<m_numPoints;i++)
	{
		D_btVector3 vtx = getScaledPoint(i);

		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}
	return supVec;
}

void	D_btConvexPointCloudShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
	D_btScalar newDot;
	//use 'w' component of supportVerticesOut?
	{
		for (int i=0;i<numVectors;i++)
		{
			supportVerticesOut[i][3] = D_btScalar(-D_BT_LARGE_FLOAT);
		}
	}
	for (int i=0;i<m_numPoints;i++)
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
	


D_btVector3	D_btConvexPointCloudShape::localGetSupportingVertex(const D_btVector3& vec)const
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


#endif






//currently D_just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you D_can D_debug-draw D_btConvexHullShape with the Raytracer Demo
int	D_btConvexPointCloudShape::getNumVertices() const
{
	return m_numPoints;
}

int D_btConvexPointCloudShape::getNumEdges() const
{
	return 0;
}

void D_btConvexPointCloudShape::getEdge(int i,D_btVector3& pa,D_btVector3& pb) const
{
	D_btAssert (0);
}

void D_btConvexPointCloudShape::getVertex(int i,D_btVector3& vtx) const
{
	vtx = m_unscaledPoints[i]*m_localScaling;
}

int	D_btConvexPointCloudShape::getNumPlanes() const
{
	return 0;
}

void D_btConvexPointCloudShape::getPlane(D_btVector3& ,D_btVector3& ,int ) const
{

	D_btAssert(0);
}

//not yet
bool D_btConvexPointCloudShape::isInside(const D_btVector3& ,D_btScalar ) const
{
	D_btAssert(0);
	return false;
}

