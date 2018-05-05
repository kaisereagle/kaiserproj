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

#ifndef COMPOUND_COLLISION_ALGORITHM_H
#define COMPOUND_COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"

#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class D_btDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "btCollisionCreateFunc.h"
#include "LinearMath/btAlignedObjectArray.h"
class D_btDispatcher;
class D_btCollisionObject;

/// D_btCompoundCollisionAlgorithm  D_supports collision between CompoundCollisionShapes D_and other collision D_shapes
class D_btCompoundCollisionAlgorithm  : public D_btActivatingCollisionAlgorithm
{
	D_btAlignedObjectArray<D_btCollisionAlgorithm*> m_childCollisionAlgorithms;
	bool m_isSwapped;

	class D_btPersistentManifold*	m_sharedManifold;
	bool					m_ownsManifold;

	int	m_compoundShapeRevision;//D_to keep track of changes, so that childAlgorithm array D_can be updated
	
	void	removeChildAlgorithms();
	
	void	preallocateChildAlgorithms(D_btCollisionObject* body0,D_btCollisionObject* body1);

public:

	D_btCompoundCollisionAlgorithm( const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1,bool isSwapped);

	virtual ~D_btCompoundCollisionAlgorithm();

	virtual void processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	D_btScalar	calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(D_btManifoldArray&	manifoldArray)
	{
		int i;
		for (i=0;i<m_childCollisionAlgorithms.size();i++)
		{
			if (m_childCollisionAlgorithms[i])
				m_childCollisionAlgorithms[i]->getAllContactManifolds(manifoldArray);
		}
	}

	struct D_CreateFunc :public 	D_btCollisionAlgorithmCreateFunc
	{
		virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(D_btCompoundCollisionAlgorithm));
			return new(mem) D_btCompoundCollisionAlgorithm(ci,body0,body1,false);
		}
	};

	struct D_SwappedCreateFunc :public 	D_btCollisionAlgorithmCreateFunc
	{
		virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(D_btCompoundCollisionAlgorithm));
			return new(mem) D_btCompoundCollisionAlgorithm(ci,body0,body1,true);
		}
	};

};

#endif //COMPOUND_COLLISION_ALGORITHM_H
