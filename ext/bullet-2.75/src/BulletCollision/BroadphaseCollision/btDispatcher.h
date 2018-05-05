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

#ifndef _DISPATCHER_H
#define _DISPATCHER_H

#include "LinearMath/btScalar.h"

class D_btCollisionAlgorithm;
struct D_btBroadphaseProxy;
class D_btRigidBody;
class	D_btCollisionObject;
class D_btOverlappingPairCache;


class D_btPersistentManifold;
class D_btStackAlloc;

struct D_btDispatcherInfo
{
	enum D_DispatchFunc
	{
		D_DISPATCH_DISCRETE = 1,
		D_DISPATCH_CONTINUOUS
	};
	D_btDispatcherInfo()
		:m_timeStep(D_btScalar(0.)),
		m_stepCount(0),
		m_dispatchFunc(D_DISPATCH_DISCRETE),
		m_timeOfImpact(D_btScalar(1.)),
		m_useContinuous(false),
		m_debugDraw(0),
		m_enableSatConvex(false),
		m_enableSPU(true),
		m_useEpa(true),
		m_allowedCcdPenetration(D_btScalar(0.04)),
		m_useConvexConservativeDistanceUtil(false),
		m_convexConservativeDistanceThreshold(0.0f),
		m_stackAllocator(0)
	{

	}
	D_btScalar	m_timeStep;
	int			m_stepCount;
	int			m_dispatchFunc;
	mutable D_btScalar	m_timeOfImpact;
	bool		m_useContinuous;
	class D_btIDebugDraw*	m_debugDraw;
	bool		m_enableSatConvex;
	bool		m_enableSPU;
	bool		m_useEpa;
	D_btScalar	m_allowedCcdPenetration;
	bool		m_useConvexConservativeDistanceUtil;
	D_btScalar	m_convexConservativeDistanceThreshold;
	D_btStackAlloc*	m_stackAllocator;
};

///The D_btDispatcher interface class D_can be used in combination with broadphase D_to dispatch calculations for overlapping pairs.
///For example for pairwise collision detection, calculating contact points stored in D_btPersistentManifold or user callbacks (game logic).
class D_btDispatcher
{


public:
	virtual ~D_btDispatcher() ;

	virtual D_btCollisionAlgorithm* findAlgorithm(D_btCollisionObject* body0,D_btCollisionObject* body1,D_btPersistentManifold* sharedManifold=0) = 0;

	virtual D_btPersistentManifold*	getNewManifold(void* body0,void* body1)=0;

	virtual void releaseManifold(D_btPersistentManifold* manifold)=0;

	virtual void clearManifold(D_btPersistentManifold* manifold)=0;

	virtual bool	needsCollision(D_btCollisionObject* body0,D_btCollisionObject* body1) = 0;

	virtual bool	needsResponse(D_btCollisionObject* body0,D_btCollisionObject* body1)=0;

	virtual void	dispatchAllCollisionPairs(D_btOverlappingPairCache* pairCache,const D_btDispatcherInfo& dispatchInfo,D_btDispatcher* dispatcher)  =0;

	virtual int getNumManifolds() const = 0;

	virtual D_btPersistentManifold* getManifoldByIndexInternal(int index) = 0;

	virtual	D_btPersistentManifold**	getInternalManifoldPointer() = 0;

	virtual	void* allocateCollisionAlgorithm(int size)  = 0;

	virtual	void freeCollisionAlgorithm(void* ptr) = 0;

};


#endif //_DISPATCHER_H
