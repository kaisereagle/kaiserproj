/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btGhostObject.h"
#include "btCollisionWorld.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "LinearMath/btAabbUtil2.h"

D_btGhostObject::D_btGhostObject()
{
	m_internalType = D_CO_GHOST_OBJECT;
}

D_btGhostObject::~D_btGhostObject()
{
	///D_btGhostObject D_should have been removed from the world, so D_no overlapping objects
	D_btAssert(!m_overlappingObjects.size());
}


void D_btGhostObject::addOverlappingObjectInternal(D_btBroadphaseProxy* otherProxy,D_btBroadphaseProxy* thisProxy)
{
	D_btCollisionObject* otherObject = (D_btCollisionObject*)otherProxy->m_clientObject;
	D_btAssert(otherObject);
	///if this linearSearch becomes too slow (too many overlapping objects) we D_should add a more appropriate data structure
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index==m_overlappingObjects.size())
	{
		//not found
		m_overlappingObjects.push_back(otherObject);
	}
}

void D_btGhostObject::removeOverlappingObjectInternal(D_btBroadphaseProxy* otherProxy,D_btDispatcher* dispatcher,D_btBroadphaseProxy* thisProxy)
{
	D_btCollisionObject* otherObject = (D_btCollisionObject*)otherProxy->m_clientObject;
	D_btAssert(otherObject);
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index<m_overlappingObjects.size())
	{
		m_overlappingObjects[index] = m_overlappingObjects[m_overlappingObjects.size()-1];
		m_overlappingObjects.pop_back();
	}
}


D_btPairCachingGhostObject::D_btPairCachingGhostObject()
{
	m_hashPairCache = new (D_btAlignedAlloc(sizeof(D_btHashedOverlappingPairCache),16)) D_btHashedOverlappingPairCache();
}

D_btPairCachingGhostObject::~D_btPairCachingGhostObject()
{
	m_hashPairCache->~D_btHashedOverlappingPairCache();
	D_btAlignedFree( m_hashPairCache );
}

void D_btPairCachingGhostObject::addOverlappingObjectInternal(D_btBroadphaseProxy* otherProxy,D_btBroadphaseProxy* thisProxy)
{
	D_btBroadphaseProxy*actualThisProxy = thisProxy ? thisProxy : getBroadphaseHandle();
	D_btAssert(actualThisProxy);

	D_btCollisionObject* otherObject = (D_btCollisionObject*)otherProxy->m_clientObject;
	D_btAssert(otherObject);
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index==m_overlappingObjects.size())
	{
		m_overlappingObjects.push_back(otherObject);
		m_hashPairCache->addOverlappingPair(actualThisProxy,otherProxy);
	}
}

void D_btPairCachingGhostObject::removeOverlappingObjectInternal(D_btBroadphaseProxy* otherProxy,D_btDispatcher* dispatcher,D_btBroadphaseProxy* thisProxy1)
{
	D_btCollisionObject* otherObject = (D_btCollisionObject*)otherProxy->m_clientObject;
	D_btBroadphaseProxy* actualThisProxy = thisProxy1 ? thisProxy1 : getBroadphaseHandle();
	D_btAssert(actualThisProxy);

	D_btAssert(otherObject);
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index<m_overlappingObjects.size())
	{
		m_overlappingObjects[index] = m_overlappingObjects[m_overlappingObjects.size()-1];
		m_overlappingObjects.pop_back();
		m_hashPairCache->removeOverlappingPair(actualThisProxy,otherProxy,dispatcher);
	}
}


void	D_btGhostObject::convexSweepTest(const D_btConvexShape* castShape, const D_btTransform& convexFromWorld, const D_btTransform& convexToWorld, D_btCollisionWorld::ConvexResultCallback& resultCallback, D_btScalar allowedCcdPenetration) const
{
	D_btTransform	convexFromTrans,convexToTrans;
	convexFromTrans = convexFromWorld;
	convexToTrans = convexToWorld;
	D_btVector3 castShapeAabbMin, castShapeAabbMax;
	/* Compute AABB that encompasses angular movement */
	{
		D_btVector3 linVel, angVel;
		D_btTransformUtil::calculateVelocity (convexFromTrans, convexToTrans, 1.0, linVel, angVel);
		D_btTransform R;
		R.setIdentity ();
		R.setRotation (convexFromTrans.getRotation());
		castShape->calculateTemporalAabb (R, linVel, angVel, 1.0, castShapeAabbMin, castShapeAabbMax);
	}

	/// go over all objects, D_and if the ray intersects their aabb + cast shape aabb,
	// do a ray-shape query using convexCaster (CCD)
	int i;
	for (i=0;i<m_overlappingObjects.size();i++)
	{
		D_btCollisionObject*	collisionObject= m_overlappingObjects[i];
		//D_only perform raycast if filterMask matches
		if(resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) {
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			D_btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
			collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
			AabbExpand (collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
			D_btScalar hitLambda = D_btScalar(1.); //could use resultCallback.m_closestHitFraction, but needs testing
			D_btVector3 hitNormal;
			if (D_btRayAabb(convexFromWorld.getOrigin(),convexToWorld.getOrigin(),collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,hitNormal))
			{
				D_btCollisionWorld::objectQuerySingle(castShape, convexFromTrans,convexToTrans,
					collisionObject,
						collisionObject->getCollisionShape(),
						collisionObject->getWorldTransform(),
						resultCallback,
						allowedCcdPenetration);
			}
		}
	}

}

void	D_btGhostObject::rayTest(const D_btVector3& rayFromWorld, const D_btVector3& rayToWorld, D_btCollisionWorld::RayResultCallback& resultCallback) const
{
	D_btTransform rayFromTrans;
	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(rayFromWorld);
	D_btTransform  rayToTrans;
	rayToTrans.setIdentity();
	rayToTrans.setOrigin(rayToWorld);


	int i;
	for (i=0;i<m_overlappingObjects.size();i++)
	{
		D_btCollisionObject*	collisionObject= m_overlappingObjects[i];
		//D_only perform raycast if filterMask matches
		if(resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) 
		{
			D_btCollisionWorld::rayTestSingle(rayFromTrans,rayToTrans,
							collisionObject,
								collisionObject->getCollisionShape(),
								collisionObject->getWorldTransform(),
								resultCallback);
		}
	}
}

