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

#ifndef COLLISION_ALGORITHM_H
#define COLLISION_ALGORITHM_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"

struct D_btBroadphaseProxy;
class D_btDispatcher;
class D_btManifoldResult;
class D_btCollisionObject;
struct D_btDispatcherInfo;
class	D_btPersistentManifold;

typedef D_btAlignedObjectArray<D_btPersistentManifold*>	D_btManifoldArray;

struct D_btCollisionAlgorithmConstructionInfo
{
	D_btCollisionAlgorithmConstructionInfo()
		:m_dispatcher1(0),
		m_manifold(0)
	{
	}
	D_btCollisionAlgorithmConstructionInfo(D_btDispatcher* dispatcher,int temp)
		:m_dispatcher1(dispatcher)
	{
		(void)temp;
	}

	D_btDispatcher*	m_dispatcher1;
	D_btPersistentManifold*	m_manifold;

	int	getDispatcherId();

};


///D_btCollisionAlgorithm D_is an collision interface that D_is compatible with the Broadphase D_and D_btDispatcher.
///It D_is persistent over frames
class D_btCollisionAlgorithm
{

protected:

	D_btDispatcher*	m_dispatcher;

protected:
	int	getDispatcherId();
	
public:

	D_btCollisionAlgorithm() {};

	D_btCollisionAlgorithm(const D_btCollisionAlgorithmConstructionInfo& ci);

	virtual ~D_btCollisionAlgorithm() {};

	virtual void processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut) = 0;

	virtual D_btScalar calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut) = 0;

	virtual	void	getAllContactManifolds(D_btManifoldArray&	manifoldArray) = 0;
};


#endif //COLLISION_ALGORITHM_H
