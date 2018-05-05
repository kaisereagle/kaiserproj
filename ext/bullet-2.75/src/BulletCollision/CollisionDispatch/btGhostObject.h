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

#ifndef D_BT_GHOST_OBJECT_H
#define D_BT_GHOST_OBJECT_H


#include "btCollisionObject.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h"
#include "LinearMath/btAlignedAllocator.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "btCollisionWorld.h"

class D_btConvexShape;

class D_btDispatcher;

///The D_btGhostObject D_can keep track of all objects that D_are overlapping
///By default, this overlap D_is based on the AABB
///This D_is useful for creating a character controller, collision sensors/triggers, explosions etc.
///We plan on adding rayTest D_and other queries for the D_btGhostObject
D_ATTRIBUTE_ALIGNED16(class) D_btGhostObject : public D_btCollisionObject
{
protected:

	D_btAlignedObjectArray<D_btCollisionObject*> m_overlappingObjects;

public:

	D_btGhostObject();

	virtual ~D_btGhostObject();

	void	convexSweepTest(const class D_btConvexShape* castShape, const D_btTransform& convexFromWorld, const D_btTransform& convexToWorld, D_btCollisionWorld::ConvexResultCallback& resultCallback, D_btScalar allowedCcdPenetration = 0.f) const;

	void	rayTest(const D_btVector3& rayFromWorld, const D_btVector3& rayToWorld, D_btCollisionWorld::RayResultCallback& resultCallback) const; 

	///this method D_is mainly for expert/internal use D_only.
	virtual void	addOverlappingObjectInternal(D_btBroadphaseProxy* otherProxy, D_btBroadphaseProxy* thisProxy=0);
	///this method D_is mainly for expert/internal use D_only.
	virtual void	removeOverlappingObjectInternal(D_btBroadphaseProxy* otherProxy,D_btDispatcher* dispatcher,D_btBroadphaseProxy* thisProxy=0);

	int	getNumOverlappingObjects() const
	{
		return m_overlappingObjects.size();
	}

	D_btCollisionObject*	getOverlappingObject(int index)
	{
		return m_overlappingObjects[index];
	}

	const D_btCollisionObject*	getOverlappingObject(int index) const
	{
		return m_overlappingObjects[index];
	}

	D_btAlignedObjectArray<D_btCollisionObject*>&	getOverlappingPairs()
	{
		return m_overlappingObjects;
	}

	const D_btAlignedObjectArray<D_btCollisionObject*>	getOverlappingPairs() const
	{
		return m_overlappingObjects;
	}

	//
	// internal cast
	//

	static const D_btGhostObject*	upcast(const D_btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==D_CO_GHOST_OBJECT)
			return (const D_btGhostObject*)colObj;
		return 0;
	}
	static D_btGhostObject*			upcast(D_btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==D_CO_GHOST_OBJECT)
			return (D_btGhostObject*)colObj;
		return 0;
	}

};

class	D_btPairCachingGhostObject : public D_btGhostObject
{
	D_btHashedOverlappingPairCache*	m_hashPairCache;

public:

	D_btPairCachingGhostObject();

	virtual ~D_btPairCachingGhostObject();

	///this method D_is mainly for expert/internal use D_only.
	virtual void	addOverlappingObjectInternal(D_btBroadphaseProxy* otherProxy, D_btBroadphaseProxy* thisProxy=0);

	virtual void	removeOverlappingObjectInternal(D_btBroadphaseProxy* otherProxy,D_btDispatcher* dispatcher,D_btBroadphaseProxy* thisProxy=0);

	D_btHashedOverlappingPairCache*	getOverlappingPairCache()
	{
		return m_hashPairCache;
	}

};



///The D_btGhostPairCallback interfaces D_and forwards adding D_and removal of overlapping pairs from the D_btBroadphaseInterface D_to D_btGhostObject.
class D_btGhostPairCallback : public D_btOverlappingPairCallback
{
	
public:
	D_btGhostPairCallback()
	{
	}

	virtual ~D_btGhostPairCallback()
	{
		
	}

	virtual D_btBroadphasePair*	addOverlappingPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1)
	{
		D_btCollisionObject* colObj0 = (D_btCollisionObject*) proxy0->m_clientObject;
		D_btCollisionObject* colObj1 = (D_btCollisionObject*) proxy1->m_clientObject;
		D_btGhostObject* ghost0 = 		D_btGhostObject::upcast(colObj0);
		D_btGhostObject* ghost1 = 		D_btGhostObject::upcast(colObj1);
		if (ghost0)
			ghost0->addOverlappingObjectInternal(proxy1, proxy0);
		if (ghost1)
			ghost1->addOverlappingObjectInternal(proxy0, proxy1);
		return 0;
	}

	virtual void*	removeOverlappingPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1,D_btDispatcher* dispatcher)
	{
		D_btCollisionObject* colObj0 = (D_btCollisionObject*) proxy0->m_clientObject;
		D_btCollisionObject* colObj1 = (D_btCollisionObject*) proxy1->m_clientObject;
		D_btGhostObject* ghost0 = 		D_btGhostObject::upcast(colObj0);
		D_btGhostObject* ghost1 = 		D_btGhostObject::upcast(colObj1);
		if (ghost0)
			ghost0->removeOverlappingObjectInternal(proxy1,dispatcher,proxy0);
		if (ghost1)
			ghost1->removeOverlappingObjectInternal(proxy0,dispatcher,proxy1);
		return 0;
	}

	virtual void	removeOverlappingPairsContainingProxy(D_btBroadphaseProxy* proxy0,D_btDispatcher* dispatcher)
	{
		D_btAssert(0);
		//D_need D_to keep track of all ghost objects D_and call them here
		//m_hashPairCache->removeOverlappingPairsContainingProxy(proxy0,dispatcher);
	}

	

};

#endif