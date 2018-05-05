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

#include "btConvex2dConvex2dAlgorithm.h"

//#include <stdio.h>
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"


#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"



#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"


D_btConvex2dConvex2dAlgorithm::D_CreateFunc::D_CreateFunc(D_btSimplexSolverInterface*			simplexSolver, D_btConvexPenetrationDepthSolver* pdSolver)
{
	m_numPerturbationIterations = 0;
	m_minimumPointsPerturbationThreshold = 3;
	m_simplexSolver = simplexSolver;
	m_pdSolver = pdSolver;
}

D_btConvex2dConvex2dAlgorithm::D_CreateFunc::~D_CreateFunc() 
{ 
}

D_btConvex2dConvex2dAlgorithm::D_btConvex2dConvex2dAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1,D_btSimplexSolverInterface* simplexSolver, D_btConvexPenetrationDepthSolver* pdSolver,int numPerturbationIterations, int minimumPointsPerturbationThreshold)
: D_btActivatingCollisionAlgorithm(ci,body0,body1),
m_simplexSolver(simplexSolver),
m_pdSolver(pdSolver),
m_ownManifold (false),
m_manifoldPtr(mf),
m_lowLevelOfDetail(false),
 m_numPerturbationIterations(numPerturbationIterations),
m_minimumPointsPerturbationThreshold(minimumPointsPerturbationThreshold)
{
	(void)body0;
	(void)body1;
}




D_btConvex2dConvex2dAlgorithm::~D_btConvex2dConvex2dAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void	D_btConvex2dConvex2dAlgorithm ::setLowLevelOfDetail(bool useLowLevel)
{
	m_lowLevelOfDetail = useLowLevel;
}



extern D_btScalar D_gContactBreakingThreshold;


//
// Convex-Convex collision algorithm
//
void D_btConvex2dConvex2dAlgorithm ::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{

	if (!m_manifoldPtr)
	{
		//swapped?
		m_manifoldPtr = m_dispatcher->getNewManifold(body0,body1);
		m_ownManifold = true;
	}
	resultOut->setPersistentManifold(m_manifoldPtr);

	//comment-out next line D_to test multi-contact generation
	//resultOut->getPersistentManifold()->clearManifold();


	D_btConvexShape* min0 = static_cast<D_btConvexShape*>(body0->getCollisionShape());
	D_btConvexShape* min1 = static_cast<D_btConvexShape*>(body1->getCollisionShape());

	D_btVector3  normalOnB;
	D_btVector3  pointOnBWorld;

	{


		D_btGjkPairDetector::D_ClosestPointInput input;

		D_btGjkPairDetector	gjkPairDetector(min0,min1,m_simplexSolver,m_pdSolver);
		//TODO: if (dispatchInfo.m_useContinuous)
		gjkPairDetector.setMinkowskiA(min0);
		gjkPairDetector.setMinkowskiB(min1);

		{
			input.m_maximumDistanceSquared = min0->getMargin() + min1->getMargin() + m_manifoldPtr->getContactBreakingThreshold();
			input.m_maximumDistanceSquared*= input.m_maximumDistanceSquared;
		}

		input.m_stackAlloc = dispatchInfo.m_stackAllocator;
		input.m_transformA = body0->getWorldTransform();
		input.m_transformB = body1->getWorldTransform();

		gjkPairDetector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);

		D_btVector3 v0,v1;
		D_btVector3 sepNormalWorldSpace;

	}

	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}

}




D_btScalar	D_btConvex2dConvex2dAlgorithm::calculateTimeOfImpact(D_btCollisionObject* col0,D_btCollisionObject* col1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	///Rather then checking ALL pairs, D_only calculate TOI when motion exceeds threshold

	///D_Linear motion for one of objects needs D_to exceed m_ccdSquareMotionThreshold
	///col0->m_worldTransform,
	D_btScalar resultFraction = D_btScalar(1.);


	D_btScalar squareMot0 = (col0->getInterpolationWorldTransform().getOrigin() - col0->getWorldTransform().getOrigin()).length2();
	D_btScalar squareMot1 = (col1->getInterpolationWorldTransform().getOrigin() - col1->getWorldTransform().getOrigin()).length2();

	if (squareMot0 < col0->getCcdSquareMotionThreshold() &&
		squareMot1 < col1->getCcdSquareMotionThreshold())
		return resultFraction;


	//An adhoc way of testing the Continuous Collision Detection algorithms
	//One object D_is approximated as a sphere, D_to simplify things
	//Starting in penetration D_should report D_no time of impact
	//For proper CCD, better accuracy D_and handling of 'allowed' penetration D_should be added
	//also the mainloop of the physics D_should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)


	/// Convex0 against sphere for Convex1
	{
		D_btConvexShape* convex0 = static_cast<D_btConvexShape*>(col0->getCollisionShape());

		D_btSphereShape	sphere1(col1->getCcdSweptSphereRadius()); //todo: allow non-zero sphere sizes, for better approximation
		D_btConvexCast::CastResult result;
		D_btVoronoiSimplexSolver voronoiSimplex;
		//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
		///Simplification, one object D_is simplified as a sphere
		D_btGjkConvexCast ccd1( convex0 ,&sphere1,&voronoiSimplex);
		//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
		if (ccd1.calcTimeOfImpact(col0->getWorldTransform(),col0->getInterpolationWorldTransform(),
			col1->getWorldTransform(),col1->getInterpolationWorldTransform(),result))
		{

			//store result.m_fraction in both bodies

			if (col0->getHitFraction()> result.m_fraction)
				col0->setHitFraction( result.m_fraction );

			if (col1->getHitFraction() > result.m_fraction)
				col1->setHitFraction( result.m_fraction);

			if (resultFraction > result.m_fraction)
				resultFraction = result.m_fraction;

		}




	}

	/// Sphere (for convex0) against Convex1
	{
		D_btConvexShape* convex1 = static_cast<D_btConvexShape*>(col1->getCollisionShape());

		D_btSphereShape	sphere0(col0->getCcdSweptSphereRadius()); //todo: allow non-zero sphere sizes, for better approximation
		D_btConvexCast::CastResult result;
		D_btVoronoiSimplexSolver voronoiSimplex;
		//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
		///Simplification, one object D_is simplified as a sphere
		D_btGjkConvexCast ccd1(&sphere0,convex1,&voronoiSimplex);
		//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
		if (ccd1.calcTimeOfImpact(col0->getWorldTransform(),col0->getInterpolationWorldTransform(),
			col1->getWorldTransform(),col1->getInterpolationWorldTransform(),result))
		{

			//store result.m_fraction in both bodies

			if (col0->getHitFraction()	> result.m_fraction)
				col0->setHitFraction( result.m_fraction);

			if (col1->getHitFraction() > result.m_fraction)
				col1->setHitFraction( result.m_fraction);

			if (resultFraction > result.m_fraction)
				resultFraction = result.m_fraction;

		}
	}

	return resultFraction;

}

