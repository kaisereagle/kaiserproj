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



#include "btMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "LinearMath/btQuaternion.h"

D_btMultiSphereShape::D_btMultiSphereShape (const D_btVector3* positions,const D_btScalar* radi,int numSpheres)
:D_btConvexInternalAabbCachingShape ()
{
	m_shapeType = D_MULTI_SPHERE_SHAPE_PROXYTYPE;
	//D_btScalar startMargin = D_btScalar(D_BT_LARGE_FLOAT);

	m_localPositionArray.resize(numSpheres);
	m_radiArray.resize(numSpheres);
	for (int i=0;i<numSpheres;i++)
	{
		m_localPositionArray[i] = positions[i];
		m_radiArray[i] = radi[i];
		
	}

	recalcLocalAabb();

}

 
 D_btVector3	D_btMultiSphereShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec0)const
{
	int i;
	D_btVector3 supVec(0,0,0);

	D_btScalar maxDot(D_btScalar(-D_BT_LARGE_FLOAT));


	D_btVector3 vec = vec0;
	D_btScalar lenSqr = vec.length2();
	if (lenSqr < (D_SIMD_EPSILON*D_SIMD_EPSILON))
	{
		vec.setValue(1,0,0);
	} else
	{
		D_btScalar rlen = D_btScalar(1.) / D_btSqrt(lenSqr );
		vec *= rlen;
	}

	D_btVector3 vtx;
	D_btScalar newDot;

	const D_btVector3* pos = &m_localPositionArray[0];
	const D_btScalar* rad = &m_radiArray[0];
	int numSpheres = m_localPositionArray.size();

	for (i=0;i<numSpheres;i++)
	{
		vtx = (*pos) +vec*m_localScaling*(*rad) - vec * getMargin();
		pos++;
		rad++;
		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}

	return supVec;

}

 void	D_btMultiSphereShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{

	for (int j=0;j<numVectors;j++)
	{
		D_btScalar maxDot(D_btScalar(-D_BT_LARGE_FLOAT));

		const D_btVector3& vec = vectors[j];

		D_btVector3 vtx;
		D_btScalar newDot;

		const D_btVector3* pos = &m_localPositionArray[0];
		const D_btScalar* rad = &m_radiArray[0];
		int numSpheres = m_localPositionArray.size();
		for (int i=0;i<numSpheres;i++)
		{
			vtx = (*pos) +vec*m_localScaling*(*rad) - vec * getMargin();
			pos++;
			rad++;
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supportVerticesOut[j] = vtx;
			}
		}
	}
}








void	D_btMultiSphereShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	//as an approximation, take the inertia of the box that bounds the spheres

	D_btVector3 localAabbMin,localAabbMax;
	getCachedLocalAabb(localAabbMin,localAabbMax);
	D_btVector3 halfExtents = (localAabbMax-localAabbMin)*D_btScalar(0.5);

	D_btScalar lx=D_btScalar(2.)*(halfExtents.x());
	D_btScalar ly=D_btScalar(2.)*(halfExtents.y());
	D_btScalar lz=D_btScalar(2.)*(halfExtents.z());

	inertia.setValue(mass/(D_btScalar(12.0)) * (ly*ly + lz*lz),
					mass/(D_btScalar(12.0)) * (lx*lx + lz*lz),
					mass/(D_btScalar(12.0)) * (lx*lx + ly*ly));

}


