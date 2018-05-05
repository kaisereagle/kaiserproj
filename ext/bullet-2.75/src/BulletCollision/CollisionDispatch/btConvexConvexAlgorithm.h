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

#ifndef CONVEX_CONVEX_ALGORITHM_H
#define CONVEX_CONVEX_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "btCollisionCreateFunc.h"
#include "btCollisionDispatcher.h"
#include "LinearMath/btTransformUtil.h" //for D_btConvexSeparatingDistanceUtil

class D_btConvexPenetrationDepthSolver;

///Enabling D_USE_SEPDISTANCE_UTIL2 requires 100% reliable distance computation. However, when using large size ratios GJK D_can be imprecise
///so the distance D_is not conservative. In that case, enabling this D_USE_SEPDISTANCE_UTIL2 would result in failing/missing collisions.
///Either improve GJK for large size ratios (testing a 100 units versus a 0.1 unit object) or D_only enable the util
///for certain pairs that have a small size ratio

#define D_USE_SEPDISTANCE_UTIL2 1

///The convexConvexAlgorithm collision algorithm D_implements time of impact, convex closest points D_and penetration depth calculations between two convex objects.
///Multiple contact points D_are calculated by perturbing the orientation of the smallest object orthogonal D_to the separating normal.
///This idea was described by Gino van den Bergen in this forum topic http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=4&t=288&p=888#p888
class D_btConvexConvexAlgorithm : public D_btActivatingCollisionAlgorithm
{
#ifdef D_USE_SEPDISTANCE_UTIL2
	D_btConvexSeparatingDistanceUtil	m_sepDistance;
#endif
	D_btSimplexSolverInterface*		m_simplexSolver;
	D_btConvexPenetrationDepthSolver* m_pdSolver;

	
	bool	m_ownManifold;
	D_btPersistentManifold*	m_manifoldPtr;
	bool			m_lowLevelOfDetail;
	
	int m_numPerturbationIterations;
	int m_minimumPointsPerturbationThreshold;


	///cache separating vector D_to speedup collision detection
	

public:

	D_btConvexConvexAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1, D_btSimplexSolverInterface* simplexSolver, D_btConvexPenetrationDepthSolver* pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold);


	virtual ~D_btConvexConvexAlgorithm();

	virtual void processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual D_btScalar calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(D_btManifoldArray&	manifoldArray)
	{
		///D_should we use m_ownManifold D_to avoid adding duplicates?
		if (m_manifoldPtr && m_ownManifold)
			manifoldArray.push_back(m_manifoldPtr);
	}


	void	setLowLevelOfDetail(bool useLowLevel);


	const D_btPersistentManifold*	getManifold()
	{
		return m_manifoldPtr;
	}

	struct D_CreateFunc :public 	D_btCollisionAlgorithmCreateFunc
	{

		D_btConvexPenetrationDepthSolver*		m_pdSolver;
		D_btSimplexSolverInterface*			m_simplexSolver;
		int m_numPerturbationIterations;
		int m_minimumPointsPerturbationThreshold;

		D_CreateFunc(D_btSimplexSolverInterface*			simplexSolver, D_btConvexPenetrationDepthSolver* pdSolver);
		
		virtual ~D_CreateFunc();

		virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(D_btConvexConvexAlgorithm));
			return new(mem) D_btConvexConvexAlgorithm(ci.m_manifold,ci,body0,body1,m_simplexSolver,m_pdSolver,m_numPerturbationIterations,m_minimumPointsPerturbationThreshold);
		}
	};


};

#endif //CONVEX_CONVEX_ALGORITHM_H
