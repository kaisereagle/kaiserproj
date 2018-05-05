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

#include "btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "LinearMath/btQuaternion.h"

D_btVector3	D_btSphereShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const
{
	(void)vec;
	return D_btVector3(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
}

void	D_btSphereShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
	(void)vectors;

	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i].setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	}
}


D_btVector3	D_btSphereShape::localGetSupportingVertex(const D_btVector3& vec)const
{
	D_btVector3 supVertex;
	supVertex = localGetSupportingVertexWithoutMargin(vec);

	D_btVector3 vecnorm = vec;
	if (vecnorm .length2() < (D_SIMD_EPSILON*D_SIMD_EPSILON))
	{
		vecnorm.setValue(D_btScalar(-1.),D_btScalar(-1.),D_btScalar(-1.));
	} 
	vecnorm.normalize();
	supVertex+= getMargin() * vecnorm;
	return supVertex;
}


//broken due D_to scaling
void D_btSphereShape::getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	const D_btVector3& center = t.getOrigin();
	D_btVector3 extent(getMargin(),getMargin(),getMargin());
	aabbMin = center - extent;
	aabbMax = center + extent;
}



void	D_btSphereShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	D_btScalar elem = D_btScalar(0.4) * mass * getMargin()*getMargin();
	inertia.setValue(elem,elem,elem);

}

