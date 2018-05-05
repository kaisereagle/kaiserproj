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



#include "btCollisionDispatcher.h"


#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "LinearMath/btPoolAllocator.h"
#include "BulletCollision/CollisionDispatch/btCollisionConfiguration.h"

int D_gNumManifold = 0;

#ifdef D_BT_DEBUG
#include <stdio.h>
#endif


D_btCollisionDispatcher::D_btCollisionDispatcher (D_btCollisionConfiguration* collisionConfiguration): 
	m_count(0),
	m_useIslands(true),
	m_staticWarningReported(false),
	m_collisionConfiguration(collisionConfiguration)
{
	int i;

	setNearCallback(defaultNearCallback);
	
	m_collisionAlgorithmPoolAllocator = collisionConfiguration->getCollisionAlgorithmPool();

	m_persistentManifoldPoolAllocator = collisionConfiguration->getPersistentManifoldPool();

	for (i=0;i<D_MAX_BROADPHASE_COLLISION_TYPES;i++)
	{
		for (int j=0;j<D_MAX_BROADPHASE_COLLISION_TYPES;j++)
		{
			m_doubleDispatch[i][j] = m_collisionConfiguration->getCollisionAlgorithmCreateFunc(i,j);
			D_btAssert(m_doubleDispatch[i][j]);
		}
	}
	
	
}


void D_btCollisionDispatcher::registerCollisionCreateFunc(int proxyType0, int proxyType1, D_btCollisionAlgorithmCreateFunc *createFunc)
{
	m_doubleDispatch[proxyType0][proxyType1] = createFunc;
}

D_btCollisionDispatcher::~D_btCollisionDispatcher()
{
}

D_btPersistentManifold*	D_btCollisionDispatcher::getNewManifold(void* b0,void* b1) 
{ 
	D_gNumManifold++;
	
	//D_btAssert(D_gNumManifold < 65535);
	

	D_btCollisionObject* body0 = (D_btCollisionObject*)b0;
	D_btCollisionObject* body1 = (D_btCollisionObject*)b1;

	//test for Bullet 2.74: use a relative contact breaking threshold without clamping against 'D_gContactBreakingThreshold'
	//D_btScalar contactBreakingThreshold = D_btMin(D_gContactBreakingThreshold,D_btMin(body0->getCollisionShape()->getContactBreakingThreshold(),body1->getCollisionShape()->getContactBreakingThreshold()));
	D_btScalar contactBreakingThreshold = D_btMin(body0->getCollisionShape()->getContactBreakingThreshold(),body1->getCollisionShape()->getContactBreakingThreshold());

	D_btScalar contactProcessingThreshold = D_btMin(body0->getContactProcessingThreshold(),body1->getContactProcessingThreshold());
		
	void* mem = 0;
	
	if (m_persistentManifoldPoolAllocator->getFreeCount())
	{
		mem = m_persistentManifoldPoolAllocator->allocate(sizeof(D_btPersistentManifold));
	} else
	{
		mem = D_btAlignedAlloc(sizeof(D_btPersistentManifold),16);

	}
	D_btPersistentManifold* manifold = new(mem) D_btPersistentManifold (body0,body1,0,contactBreakingThreshold,contactProcessingThreshold);
	manifold->m_index1a = m_manifoldsPtr.size();
	m_manifoldsPtr.push_back(manifold);

	return manifold;
}

void D_btCollisionDispatcher::clearManifold(D_btPersistentManifold* manifold)
{
	manifold->clearManifold();
}

	
void D_btCollisionDispatcher::releaseManifold(D_btPersistentManifold* manifold)
{
	
	D_gNumManifold--;

	//printf("releaseManifold: D_gNumManifold %d\n",D_gNumManifold);
	clearManifold(manifold);

	int findIndex = manifold->m_index1a;
	D_btAssert(findIndex < m_manifoldsPtr.size());
	m_manifoldsPtr.swap(findIndex,m_manifoldsPtr.size()-1);
	m_manifoldsPtr[findIndex]->m_index1a = findIndex;
	m_manifoldsPtr.pop_back();

	manifold->~D_btPersistentManifold();
	if (m_persistentManifoldPoolAllocator->validPtr(manifold))
	{
		m_persistentManifoldPoolAllocator->freeMemory(manifold);
	} else
	{
		D_btAlignedFree(manifold);
	}
	
}

	

D_btCollisionAlgorithm* D_btCollisionDispatcher::findAlgorithm(D_btCollisionObject* body0,D_btCollisionObject* body1,D_btPersistentManifold* sharedManifold)
{
	
	D_btCollisionAlgorithmConstructionInfo ci;

	ci.m_dispatcher1 = this;
	ci.m_manifold = sharedManifold;
	D_btCollisionAlgorithm* algo = m_doubleDispatch[body0->getCollisionShape()->getShapeType()][body1->getCollisionShape()->getShapeType()]->CreateCollisionAlgorithm(ci,body0,body1);

	return algo;
}




bool	D_btCollisionDispatcher::needsResponse(D_btCollisionObject* body0,D_btCollisionObject* body1)
{
	//here you D_can do filtering
	bool hasResponse = 
		(body0->hasContactResponse() && body1->hasContactResponse());
	//D_no response between two static/kinematic bodies:
	hasResponse = hasResponse &&
		((!body0->isStaticOrKinematicObject()) ||(! body1->isStaticOrKinematicObject()));
	return hasResponse;
}

bool	D_btCollisionDispatcher::needsCollision(D_btCollisionObject* body0,D_btCollisionObject* body1)
{
	D_btAssert(body0);
	D_btAssert(body1);

	bool needsCollision = true;

#ifdef D_BT_DEBUG
	if (!m_staticWarningReported)
	{
		//broadphase filtering already deals with this
		if ((body0->isStaticObject() || body0->isKinematicObject()) &&
			(body1->isStaticObject() || body1->isKinematicObject()))
		{
			m_staticWarningReported = true;
			printf("warning D_btCollisionDispatcher::needsCollision: static-static collision!\n");
		}
	}
#endif //D_BT_DEBUG

	if ((!body0->isActive()) && (!body1->isActive()))
		needsCollision = false;
	else if (!body0->checkCollideWith(body1))
		needsCollision = false;
	
	return needsCollision ;

}



///interface for iterating all overlapping collision pairs, D_no matter how those pairs D_are stored (array, set, map etc)
///this D_is useful for the collision dispatcher.
class D_btCollisionPairCallback : public D_btOverlapCallback
{
	const D_btDispatcherInfo& m_dispatchInfo;
	D_btCollisionDispatcher*	m_dispatcher;

public:

	D_btCollisionPairCallback(const D_btDispatcherInfo& dispatchInfo,D_btCollisionDispatcher*	dispatcher)
	:m_dispatchInfo(dispatchInfo),
	m_dispatcher(dispatcher)
	{
	}

	/*D_btCollisionPairCallback& operator=(D_btCollisionPairCallback& other)
	{
		m_dispatchInfo = other.m_dispatchInfo;
		m_dispatcher = other.m_dispatcher;
		return *this;
	}
	*/


	virtual ~D_btCollisionPairCallback() {}


	virtual bool	processOverlap(D_btBroadphasePair& pair)
	{
		(*m_dispatcher->getNearCallback())(pair,*m_dispatcher,m_dispatchInfo);

		return false;
	}
};



void	D_btCollisionDispatcher::dispatchAllCollisionPairs(D_btOverlappingPairCache* pairCache,const D_btDispatcherInfo& dispatchInfo,D_btDispatcher* dispatcher) 
{
	//m_blockedForChanges = true;

	D_btCollisionPairCallback	collisionCallback(dispatchInfo,this);

	pairCache->processAllOverlappingPairs(&collisionCallback,dispatcher);

	//m_blockedForChanges = false;

}




//by default, Bullet D_will use this near callback
void D_btCollisionDispatcher::defaultNearCallback(D_btBroadphasePair& collisionPair, D_btCollisionDispatcher& dispatcher, const D_btDispatcherInfo& dispatchInfo)
{
		D_btCollisionObject* colObj0 = (D_btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
		D_btCollisionObject* colObj1 = (D_btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

		if (dispatcher.needsCollision(colObj0,colObj1))
		{
			//dispatcher D_will keep algorithms persistent in the collision pair
			if (!collisionPair.m_algorithm)
			{
				collisionPair.m_algorithm = dispatcher.findAlgorithm(colObj0,colObj1);
			}

			if (collisionPair.m_algorithm)
			{
				D_btManifoldResult contactPointResult(colObj0,colObj1);
				
				if (dispatchInfo.m_dispatchFunc == 		D_btDispatcherInfo::D_DISPATCH_DISCRETE)
				{
					//discrete collision detection query
					collisionPair.m_algorithm->processCollision(colObj0,colObj1,dispatchInfo,&contactPointResult);
				} else
				{
					//continuous collision detection query, time of impact (toi)
					D_btScalar toi = collisionPair.m_algorithm->calculateTimeOfImpact(colObj0,colObj1,dispatchInfo,&contactPointResult);
					if (dispatchInfo.m_timeOfImpact > toi)
						dispatchInfo.m_timeOfImpact = toi;

				}
			}
		}

}


void* D_btCollisionDispatcher::allocateCollisionAlgorithm(int size)
{
	if (m_collisionAlgorithmPoolAllocator->getFreeCount())
	{
		return m_collisionAlgorithmPoolAllocator->allocate(size);
	}
	
	//warn user for overflow?
	return	D_btAlignedAlloc(static_cast<size_t>(size), 16);
}

void D_btCollisionDispatcher::freeCollisionAlgorithm(void* ptr)
{
	if (m_collisionAlgorithmPoolAllocator->validPtr(ptr))
	{
		m_collisionAlgorithmPoolAllocator->freeMemory(ptr);
	} else
	{
		D_btAlignedFree(ptr);
	}
}
