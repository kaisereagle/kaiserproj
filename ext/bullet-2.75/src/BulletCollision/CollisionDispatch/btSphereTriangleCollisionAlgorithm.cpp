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


#include "btSphereTriangleCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "SphereTriangleDetector.h"


D_btSphereTriangleCollisionAlgorithm::D_btSphereTriangleCollisionAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* col0,D_btCollisionObject* col1,bool swapped)
: D_btActivatingCollisionAlgorithm(ci,col0,col1),
m_ownManifold(false),
m_manifoldPtr(mf),
m_swapped(swapped)
{
	if (!m_manifoldPtr)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(col0,col1);
		m_ownManifold = true;
	}
}

D_btSphereTriangleCollisionAlgorithm::~D_btSphereTriangleCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void D_btSphereTriangleCollisionAlgorithm::processCollision (D_btCollisionObject* col0,D_btCollisionObject* col1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
		return;

	D_btCollisionObject* sphereObj = m_swapped? col1 : col0;
	D_btCollisionObject* triObj = m_swapped? col0 : col1;

	D_btSphereShape* sphere = (D_btSphereShape*)sphereObj->getCollisionShape();
	D_btTriangleShape* triangle = (D_btTriangleShape*)triObj->getCollisionShape();
	
	/// report a contact. internally this D_will be kept persistent, D_and contact reduction D_is done
	resultOut->setPersistentManifold(m_manifoldPtr);
	D_SphereTriangleDetector detector(sphere,triangle, m_manifoldPtr->getContactBreakingThreshold());
	
	D_btDiscreteCollisionDetectorInterface::D_ClosestPointInput input;
	input.m_maximumDistanceSquared = D_btScalar(D_BT_LARGE_FLOAT);///@todo: tighter bounds
	input.m_transformA = sphereObj->getWorldTransform();
	input.m_transformB = triObj->getWorldTransform();

	bool swapResults = m_swapped;

	detector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw,swapResults);

	if (m_ownManifold)
		resultOut->refreshContactPoints();
	
}

D_btScalar D_btSphereTriangleCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* col0,D_btCollisionObject* col1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return D_btScalar(1.);
}
