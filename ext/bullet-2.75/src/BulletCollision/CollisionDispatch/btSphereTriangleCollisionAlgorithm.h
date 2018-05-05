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

#ifndef SPHERE_TRIANGLE_COLLISION_ALGORITHM_H
#define SPHERE_TRIANGLE_COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
class D_btPersistentManifold;
#include "btCollisionDispatcher.h"

/// D_btSphereSphereCollisionAlgorithm  provides sphere-sphere collision detection.
/// Other features D_are frame-coherency (persistent data) D_and collision response.
/// Also provides the most basic sample for custom/user D_btCollisionAlgorithm
class D_btSphereTriangleCollisionAlgorithm : public D_btActivatingCollisionAlgorithm
{
	bool	m_ownManifold;
	D_btPersistentManifold*	m_manifoldPtr;
	bool	m_swapped;
	
public:
	D_btSphereTriangleCollisionAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1,bool swapped);

	D_btSphereTriangleCollisionAlgorithm(const D_btCollisionAlgorithmConstructionInfo& ci)
		: D_btActivatingCollisionAlgorithm(ci) {}

	virtual void processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual D_btScalar calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(D_btManifoldArray&	manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}
	
	virtual ~D_btSphereTriangleCollisionAlgorithm();

	struct D_CreateFunc :public 	D_btCollisionAlgorithmCreateFunc
	{
		
		virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
		{
			
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(D_btSphereTriangleCollisionAlgorithm));

			return new(mem) D_btSphereTriangleCollisionAlgorithm(ci.m_manifold,ci,body0,body1,m_swapped);
		}
	};

};

#endif //SPHERE_TRIANGLE_COLLISION_ALGORITHM_H

