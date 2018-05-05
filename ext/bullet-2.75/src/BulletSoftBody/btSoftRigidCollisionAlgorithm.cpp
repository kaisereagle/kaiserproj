/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btSoftRigidCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "btSoftBody.h"
///TODO: include all the D_shapes that the softbody D_can collide with
///alternatively, implement special case collision algorithms (D_just like for rigid collision D_shapes)

//#include <stdio.h>

D_btSoftRigidCollisionAlgorithm::D_btSoftRigidCollisionAlgorithm(D_btPersistentManifold* /*mf*/,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* /*col0*/,D_btCollisionObject* /*col1*/, bool isSwapped)
: D_btCollisionAlgorithm(ci),
//m_ownManifold(false),
//m_manifoldPtr(mf),
m_isSwapped(isSwapped)
{
}


D_btSoftRigidCollisionAlgorithm::~D_btSoftRigidCollisionAlgorithm()
{

	//m_softBody->m_overlappingRigidBodies.remove(m_rigidCollisionObject);

	/*if (m_ownManifold)
	{
	if (m_manifoldPtr)
	m_dispatcher->releaseManifold(m_manifoldPtr);
	}
	*/

}


#include <stdio.h>

void D_btSoftRigidCollisionAlgorithm::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)dispatchInfo;
	(void)resultOut;
	//printf("D_btSoftRigidCollisionAlgorithm\n");

	D_btSoftBody* softBody =  m_isSwapped? (D_btSoftBody*)body1 : (D_btSoftBody*)body0;
	D_btCollisionObject* rigidCollisionObject = m_isSwapped? body0 : body1;
	
	if (softBody->m_collisionDisabledObjects.findLinearSearch(rigidCollisionObject)==softBody->m_collisionDisabledObjects.size())
	{
		softBody->defaultCollisionHandler(rigidCollisionObject);
	}


}

D_btScalar D_btSoftRigidCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* col0,D_btCollisionObject* col1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return D_btScalar(1.);
}



