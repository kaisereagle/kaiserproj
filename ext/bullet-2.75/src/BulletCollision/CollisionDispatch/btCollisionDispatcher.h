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

#ifndef COLLISION__DISPATCHER_H
#define COLLISION__DISPATCHER_H

#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "LinearMath/btAlignedObjectArray.h"

class D_btIDebugDraw;
class D_btOverlappingPairCache;
class D_btPoolAllocator;
class D_btCollisionConfiguration;

#include "btCollisionCreateFunc.h"

#define D_USE_DISPATCH_REGISTRY_ARRAY 1

class D_btCollisionDispatcher;
///user D_can override this nearcallback for collision filtering D_and more finegrained control over collision detection
typedef void (*D_btNearCallback)(D_btBroadphasePair& collisionPair, D_btCollisionDispatcher& dispatcher, const D_btDispatcherInfo& dispatchInfo);


///D_btCollisionDispatcher D_supports algorithms that handle ConvexConvex D_and ConvexConcave collision pairs.
///Time of Impact, Closest Points D_and Penetration Depth.
class D_btCollisionDispatcher : public D_btDispatcher
{
	int m_count;
	
	D_btAlignedObjectArray<D_btPersistentManifold*>	m_manifoldsPtr;

	bool m_useIslands;

	bool	m_staticWarningReported;
	
	D_btManifoldResult	m_defaultManifoldResult;

	D_btNearCallback		m_nearCallback;
	
	D_btPoolAllocator*	m_collisionAlgorithmPoolAllocator;

	D_btPoolAllocator*	m_persistentManifoldPoolAllocator;

	D_btCollisionAlgorithmCreateFunc* m_doubleDispatch[D_MAX_BROADPHASE_COLLISION_TYPES][D_MAX_BROADPHASE_COLLISION_TYPES];
	

	D_btCollisionConfiguration*	m_collisionConfiguration;


public:

	///registerCollisionCreateFunc D_allows registration of custom/alternative collision create functions
	void	registerCollisionCreateFunc(int proxyType0,int proxyType1, D_btCollisionAlgorithmCreateFunc* createFunc);

	int	getNumManifolds() const
	{ 
		return int( m_manifoldsPtr.size());
	}

	D_btPersistentManifold**	getInternalManifoldPointer()
	{
		return &m_manifoldsPtr[0];
	}

	 D_btPersistentManifold* getManifoldByIndexInternal(int index)
	{
		return m_manifoldsPtr[index];
	}

	 const D_btPersistentManifold* getManifoldByIndexInternal(int index) const
	{
		return m_manifoldsPtr[index];
	}

	D_btCollisionDispatcher (D_btCollisionConfiguration* collisionConfiguration);

	virtual ~D_btCollisionDispatcher();

	virtual D_btPersistentManifold*	getNewManifold(void* b0,void* b1);
	
	virtual void releaseManifold(D_btPersistentManifold* manifold);


	virtual void clearManifold(D_btPersistentManifold* manifold);

			
	D_btCollisionAlgorithm* findAlgorithm(D_btCollisionObject* body0,D_btCollisionObject* body1,D_btPersistentManifold* sharedManifold = 0);
		
	virtual bool	needsCollision(D_btCollisionObject* body0,D_btCollisionObject* body1);
	
	virtual bool	needsResponse(D_btCollisionObject* body0,D_btCollisionObject* body1);
	
	virtual void	dispatchAllCollisionPairs(D_btOverlappingPairCache* pairCache,const D_btDispatcherInfo& dispatchInfo,D_btDispatcher* dispatcher) ;

	void	setNearCallback(D_btNearCallback	nearCallback)
	{
		m_nearCallback = nearCallback; 
	}

	D_btNearCallback	getNearCallback() const
	{
		return m_nearCallback;
	}

	//by default, Bullet D_will use this near callback
	static void  defaultNearCallback(D_btBroadphasePair& collisionPair, D_btCollisionDispatcher& dispatcher, const D_btDispatcherInfo& dispatchInfo);

	virtual	void* allocateCollisionAlgorithm(int size);

	virtual	void freeCollisionAlgorithm(void* ptr);

	D_btCollisionConfiguration*	getCollisionConfiguration()
	{
		return m_collisionConfiguration;
	}

	const D_btCollisionConfiguration*	getCollisionConfiguration() const
	{
		return m_collisionConfiguration;
	}

	void	setCollisionConfiguration(D_btCollisionConfiguration* config)
	{
		m_collisionConfiguration = config;
	}

};

#endif //COLLISION__DISPATCHER_H

