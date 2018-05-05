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


#include "btCapsuleShape.h"

#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "LinearMath/btQuaternion.h"

D_btCapsuleShape::D_btCapsuleShape(D_btScalar radius, D_btScalar height) : D_btConvexInternalShape ()
{
	m_shapeType = D_CAPSULE_SHAPE_PROXYTYPE;
	m_upAxis = 1;
	m_implicitShapeDimensions.setValue(radius,0.5f*height,radius);
}

 
 D_btVector3	D_btCapsuleShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec0)const
{

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
	
	D_btScalar radius = getRadius();


	{
		D_btVector3 pos(0,0,0);
		pos[getUpAxis()] = getHalfHeight();

		vtx = pos +vec*m_localScaling*(radius) - vec * getMargin();
		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}
	{
		D_btVector3 pos(0,0,0);
		pos[getUpAxis()] = -getHalfHeight();

		vtx = pos +vec*m_localScaling*(radius) - vec * getMargin();
		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}

	return supVec;

}

 void	D_btCapsuleShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{

	
	D_btScalar radius = getRadius();

	for (int j=0;j<numVectors;j++)
	{
		D_btScalar maxDot(D_btScalar(-D_BT_LARGE_FLOAT));
		const D_btVector3& vec = vectors[j];

		D_btVector3 vtx;
		D_btScalar newDot;
		{
			D_btVector3 pos(0,0,0);
			pos[getUpAxis()] = getHalfHeight();
			vtx = pos +vec*m_localScaling*(radius) - vec * getMargin();
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supportVerticesOut[j] = vtx;
			}
		}
		{
			D_btVector3 pos(0,0,0);
			pos[getUpAxis()] = -getHalfHeight();
			vtx = pos +vec*m_localScaling*(radius) - vec * getMargin();
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supportVerticesOut[j] = vtx;
			}
		}
		
	}
}


void	D_btCapsuleShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	//as an approximation, take the inertia of the box that bounds the spheres

	D_btTransform ident;
	ident.setIdentity();

	
	D_btScalar radius = getRadius();

	D_btVector3 halfExtents(radius,radius,radius);
	halfExtents[getUpAxis()]+=getHalfHeight();

	D_btScalar margin = D_CONVEX_DISTANCE_MARGIN;

	D_btScalar lx=D_btScalar(2.)*(halfExtents[0]+margin);
	D_btScalar ly=D_btScalar(2.)*(halfExtents[1]+margin);
	D_btScalar lz=D_btScalar(2.)*(halfExtents[2]+margin);
	const D_btScalar x2 = lx*lx;
	const D_btScalar y2 = ly*ly;
	const D_btScalar z2 = lz*lz;
	const D_btScalar scaledmass = mass * D_btScalar(.08333333);

	inertia[0] = scaledmass * (y2+z2);
	inertia[1] = scaledmass * (x2+z2);
	inertia[2] = scaledmass * (x2+y2);

}

D_btCapsuleShapeX::D_btCapsuleShapeX(D_btScalar radius,D_btScalar height)
{
	m_upAxis = 0;
	m_implicitShapeDimensions.setValue(0.5f*height, radius,radius);
}






D_btCapsuleShapeZ::D_btCapsuleShapeZ(D_btScalar radius,D_btScalar height)
{
	m_upAxis = 2;
	m_implicitShapeDimensions.setValue(radius,radius,0.5f*height);
}




