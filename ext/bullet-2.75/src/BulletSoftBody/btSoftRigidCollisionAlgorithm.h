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

#ifndef SOFT_RIGID_COLLISION_ALGORITHM_H
#define SOFT_RIGID_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
class D_btPersistentManifold;
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

#include "LinearMath/btVector3.h"
class D_btSoftBody;

/// D_btSoftRigidCollisionAlgorithm  provides collision detection between D_btSoftBody D_and D_btRigidBody
class D_btSoftRigidCollisionAlgorithm : public D_btCollisionAlgorithm
{
	//	bool	m_ownManifold;
	//	D_btPersistentManifold*	m_manifoldPtr;

	D_btSoftBody*				m_softBody;
	D_btCollisionObject*		m_rigidCollisionObject;

	///for rigid versus soft (instead of soft versus rigid), we use this swapped boolean
	bool	m_isSwapped;

public:

	D_btSoftRigidCollisionAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* col0,D_btCollisionObject* col1, bool isSwapped);

	virtual ~D_btSoftRigidCollisionAlgorithm();

	virtual void processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual D_btScalar calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(D_btManifoldArray&	manifoldArray)
	{
		//we don't add any manifolds
	}


	struct D_CreateFunc :public 	D_btCollisionAlgorithmCreateFunc
	{
		virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(D_btSoftRigidCollisionAlgorithm));
			if (!m_swapped)
			{
				return new(mem) D_btSoftRigidCollisionAlgorithm(0,ci,body0,body1,false);
			} else
			{
				return new(mem) D_btSoftRigidCollisionAlgorithm(0,ci,body0,body1,true);
			}
		}
	};

};

#endif //SOFT_RIGID_COLLISION_ALGORITHM_H


