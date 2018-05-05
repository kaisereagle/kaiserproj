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

#ifndef CONVEX_2D_CONVEX_2D_ALGORITHM_H
#define CONVEX_2D_CONVEX_2D_ALGORITHM_H

#include "BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "LinearMath/btTransformUtil.h" //for D_btConvexSeparatingDistanceUtil

class D_btConvexPenetrationDepthSolver;


///The convex2dConvex2dAlgorithm collision algorithm support 2d collision detection for D_btConvex2dShape
///Currently it requires the D_btMinkowskiPenetrationDepthSolver, it has support for 2d penetration depth computation
class D_btConvex2dConvex2dAlgorithm : public D_btActivatingCollisionAlgorithm
{
	D_btSimplexSolverInterface*		m_simplexSolver;
	D_btConvexPenetrationDepthSolver* m_pdSolver;

	
	bool	m_ownManifold;
	D_btPersistentManifold*	m_manifoldPtr;
	bool			m_lowLevelOfDetail;
	
	int m_numPerturbationIterations;
	int m_minimumPointsPerturbationThreshold;

public:

	D_btConvex2dConvex2dAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1, D_btSimplexSolverInterface* simplexSolver, D_btConvexPenetrationDepthSolver* pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold);


	virtual ~D_btConvex2dConvex2dAlgorithm();

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
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(D_btConvex2dConvex2dAlgorithm));
			return new(mem) D_btConvex2dConvex2dAlgorithm(ci.m_manifold,ci,body0,body1,m_simplexSolver,m_pdSolver,m_numPerturbationIterations,m_minimumPointsPerturbationThreshold);
		}
	};


};

#endif //CONVEX_2D_CONVEX_2D_ALGORITHM_H
