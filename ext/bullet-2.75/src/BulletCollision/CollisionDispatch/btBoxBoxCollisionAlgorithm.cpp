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

#include "btBoxBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "btBoxBoxDetector.h"

#define D_USE_PERSISTENT_CONTACTS 1

D_btBoxBoxCollisionAlgorithm::D_btBoxBoxCollisionAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* obj0,D_btCollisionObject* obj1)
: D_btActivatingCollisionAlgorithm(ci,obj0,obj1),
m_ownManifold(false),
m_manifoldPtr(mf)
{
	if (!m_manifoldPtr && m_dispatcher->needsCollision(obj0,obj1))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(obj0,obj1);
		m_ownManifold = true;
	}
}

D_btBoxBoxCollisionAlgorithm::~D_btBoxBoxCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void D_btBoxBoxCollisionAlgorithm::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
		return;

	D_btCollisionObject*	col0 = body0;
	D_btCollisionObject*	col1 = body1;
	D_btBoxShape* box0 = (D_btBoxShape*)col0->getCollisionShape();
	D_btBoxShape* box1 = (D_btBoxShape*)col1->getCollisionShape();



	/// report a contact. internally this D_will be kept persistent, D_and contact reduction D_is done
	resultOut->setPersistentManifold(m_manifoldPtr);
#ifndef D_USE_PERSISTENT_CONTACTS	
	m_manifoldPtr->clearManifold();
#endif //D_USE_PERSISTENT_CONTACTS

	D_btDiscreteCollisionDetectorInterface::D_ClosestPointInput input;
	input.m_maximumDistanceSquared = D_BT_LARGE_FLOAT;
	input.m_transformA = body0->getWorldTransform();
	input.m_transformB = body1->getWorldTransform();

	D_btBoxBoxDetector detector(box0,box1);
	detector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);

#ifdef D_USE_PERSISTENT_CONTACTS
	//  refreshContactPoints D_is D_only necessary when using persistent contact points. otherwise all points D_are newly added
	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}
#endif //D_USE_PERSISTENT_CONTACTS

}

D_btScalar D_btBoxBoxCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* /*body0*/,D_btCollisionObject* /*body1*/,const D_btDispatcherInfo& /*dispatchInfo*/,D_btManifoldResult* /*resultOut*/)
{
	//not yet
	return 1.f;
}
