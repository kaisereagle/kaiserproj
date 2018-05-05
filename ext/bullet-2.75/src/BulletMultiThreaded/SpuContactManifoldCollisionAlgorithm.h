/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef SPU_CONTACTMANIFOLD_COLLISION_ALGORITHM_H
#define SPU_CONTACTMANIFOLD_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "LinearMath/btTransformUtil.h"

class D_btPersistentManifold;

//#define USE_SEPDISTANCE_UTIL 1

/// SpuContactManifoldCollisionAlgorithm  provides contact manifold D_and D_should be processed on SPU.
D_ATTRIBUTE_ALIGNED16(class) SpuContactManifoldCollisionAlgorithm : public D_btCollisionAlgorithm
{
	D_btVector3	m_shapeDimensions0;
	D_btVector3	m_shapeDimensions1;
	D_btPersistentManifold*	m_manifoldPtr;
	int		m_shapeType0;
	int		m_shapeType1;
	float	m_collisionMargin0;
	float	m_collisionMargin1;

	D_btCollisionObject*	m_collisionObject0;
	D_btCollisionObject*	m_collisionObject1;
	
	

	
public:
	
	virtual void processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual D_btScalar calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	
	SpuContactManifoldCollisionAlgorithm(const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1);
#ifdef USE_SEPDISTANCE_UTIL
	D_btConvexSeparatingDistanceUtil	m_sepDistance;
#endif //USE_SEPDISTANCE_UTIL

	virtual ~SpuContactManifoldCollisionAlgorithm();

	virtual	void	getAllContactManifolds(D_btManifoldArray&	manifoldArray)
	{
		if (m_manifoldPtr)
			manifoldArray.push_back(m_manifoldPtr);
	}

	D_btPersistentManifold*	getContactManifoldPtr()
	{
		return m_manifoldPtr;
	}

	D_btCollisionObject*	getCollisionObject0()
	{
		return m_collisionObject0;
	}
	
	D_btCollisionObject*	getCollisionObject1()
	{
		return m_collisionObject1;
	}

	int		getShapeType0() const
	{
		return m_shapeType0;
	}

	int		getShapeType1() const
	{
		return m_shapeType1;
	}
	float	getCollisionMargin0() const
	{
		return m_collisionMargin0;
	}
	float	getCollisionMargin1() const
	{
		return m_collisionMargin1;
	}

	const D_btVector3&	getShapeDimensions0() const
	{
		return m_shapeDimensions0;
	}

	const D_btVector3&	getShapeDimensions1() const
	{
		return m_shapeDimensions1;
	}

	struct D_CreateFunc :public 	D_btCollisionAlgorithmCreateFunc
	{
		virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(SpuContactManifoldCollisionAlgorithm));
			return new(mem) SpuContactManifoldCollisionAlgorithm(ci,body0,body1);
		}
	};

};

#endif //SPU_CONTACTMANIFOLD_COLLISION_ALGORITHM_H
