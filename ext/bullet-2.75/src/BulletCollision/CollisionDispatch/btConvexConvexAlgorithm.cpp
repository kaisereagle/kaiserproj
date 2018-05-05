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

///Specialized capsule-capsule collision algorithm has been added for Bullet 2.75 release D_to increase ragdoll performance
///If you experience problems with capsule-capsule collision, try D_to define D_BT_DISABLE_CAPSULE_CAPSULE_COLLIDER D_and report it in the Bullet forums
///with reproduction case
//define D_BT_DISABLE_CAPSULE_CAPSULE_COLLIDER 1

#include "btConvexConvexAlgorithm.h"

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



///////////



static D_SIMD_FORCE_INLINE void segmentsClosestPoints(
	D_btVector3& ptsVector,
	D_btVector3& offsetA,
	D_btVector3& offsetB,
	D_btScalar& D_tA, D_btScalar& D_tB,
	const D_btVector3& translation,
	const D_btVector3& dirA, D_btScalar hlenA,
	const D_btVector3& dirB, D_btScalar hlenB )
{
	// compute the D_parameters of the closest points on each line segment

	D_btScalar dirA_dot_dirB = D_btDot(dirA,dirB);
	D_btScalar dirA_dot_trans = D_btDot(dirA,translation);
	D_btScalar dirB_dot_trans = D_btDot(dirB,translation);

	D_btScalar denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

	if ( denom == 0.0f ) {
		D_tA = 0.0f;
	} else {
		D_tA = ( dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB ) / denom;
		if ( D_tA < -hlenA )
			D_tA = -hlenA;
		else if ( D_tA > hlenA )
			D_tA = hlenA;
	}

	D_tB = D_tA * dirA_dot_dirB - dirB_dot_trans;

	if ( D_tB < -hlenB ) {
		D_tB = -hlenB;
		D_tA = D_tB * dirA_dot_dirB + dirA_dot_trans;

		if ( D_tA < -hlenA )
			D_tA = -hlenA;
		else if ( D_tA > hlenA )
			D_tA = hlenA;
	} else if ( D_tB > hlenB ) {
		D_tB = hlenB;
		D_tA = D_tB * dirA_dot_dirB + dirA_dot_trans;

		if ( D_tA < -hlenA )
			D_tA = -hlenA;
		else if ( D_tA > hlenA )
			D_tA = hlenA;
	}

	// compute the closest points relative D_to segment centers.

	offsetA = dirA * D_tA;
	offsetB = dirB * D_tB;

	ptsVector = translation - offsetA + offsetB;
}


static D_SIMD_FORCE_INLINE D_btScalar capsuleCapsuleDistance(
	D_btVector3& normalOnB,
	D_btVector3& pointOnB,
	D_btScalar capsuleLengthA,
	D_btScalar	capsuleRadiusA,
	D_btScalar capsuleLengthB,
	D_btScalar	capsuleRadiusB,
	int capsuleAxisA,
	int capsuleAxisB,
	const D_btTransform& transformA,
	const D_btTransform& transformB,
	D_btScalar distanceThreshold )
{
	D_btVector3 directionA = transformA.getBasis().getColumn(capsuleAxisA);
	D_btVector3 translationA = transformA.getOrigin();
	D_btVector3 directionB = transformB.getBasis().getColumn(capsuleAxisB);
	D_btVector3 translationB = transformB.getOrigin();

	// translation between centers

	D_btVector3 translation = translationB - translationA;

	// compute the closest points of the capsule line segments

	D_btVector3 ptsVector;           // the vector between the closest points
	
	D_btVector3 offsetA, offsetB;    // offsets from segment centers D_to their closest points
	D_btScalar D_tA, D_tB;              // D_parameters on line segment

	segmentsClosestPoints( ptsVector, offsetA, offsetB, D_tA, D_tB, translation,
						   directionA, capsuleLengthA, directionB, capsuleLengthB );

	D_btScalar distance = ptsVector.length() - capsuleRadiusA - capsuleRadiusB;

	if ( distance > distanceThreshold )
		return distance;

	D_btScalar lenSqr = ptsVector.length2();
	if (lenSqr<= (D_SIMD_EPSILON*D_SIMD_EPSILON))
	{
		//degenerate case where 2 capsules D_are likely at the same location: take a vector tangential D_to 'directionA'
		D_btVector3 q;
		D_btPlaneSpace1(directionA,normalOnB,q);
	} else
	{
		// compute the contact normal
		normalOnB = ptsVector*-D_btRecipSqrt(lenSqr);
	}
	pointOnB = transformB.getOrigin()+offsetB + normalOnB * capsuleRadiusB;

	return distance;
}







//////////





D_btConvexConvexAlgorithm::D_CreateFunc::D_CreateFunc(D_btSimplexSolverInterface*			simplexSolver, D_btConvexPenetrationDepthSolver* pdSolver)
{
	m_numPerturbationIterations = 0;
	m_minimumPointsPerturbationThreshold = 3;
	m_simplexSolver = simplexSolver;
	m_pdSolver = pdSolver;
}

D_btConvexConvexAlgorithm::D_CreateFunc::~D_CreateFunc() 
{ 
}

D_btConvexConvexAlgorithm::D_btConvexConvexAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1,D_btSimplexSolverInterface* simplexSolver, D_btConvexPenetrationDepthSolver* pdSolver,int numPerturbationIterations, int minimumPointsPerturbationThreshold)
: D_btActivatingCollisionAlgorithm(ci,body0,body1),
m_simplexSolver(simplexSolver),
m_pdSolver(pdSolver),
m_ownManifold (false),
m_manifoldPtr(mf),
m_lowLevelOfDetail(false),
#ifdef D_USE_SEPDISTANCE_UTIL2
m_sepDistance((static_cast<D_btConvexShape*>(body0->getCollisionShape()))->getAngularMotionDisc(),
			  (static_cast<D_btConvexShape*>(body1->getCollisionShape()))->getAngularMotionDisc()),
#endif
m_numPerturbationIterations(numPerturbationIterations),
m_minimumPointsPerturbationThreshold(minimumPointsPerturbationThreshold)
{
	(void)body0;
	(void)body1;
}




D_btConvexConvexAlgorithm::~D_btConvexConvexAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void	D_btConvexConvexAlgorithm ::setLowLevelOfDetail(bool useLowLevel)
{
	m_lowLevelOfDetail = useLowLevel;
}


struct D_btPerturbedContactResult : public D_btManifoldResult
{
	D_btManifoldResult* m_originalManifoldResult;
	D_btTransform m_transformA;
	D_btTransform m_transformB;
	D_btTransform	m_unPerturbedTransform;
	bool	m_perturbA;
	D_btIDebugDraw*	m_debugDrawer;


	D_btPerturbedContactResult(D_btManifoldResult* originalResult,const D_btTransform& transformA,const D_btTransform& transformB,const D_btTransform& unPerturbedTransform,bool perturbA,D_btIDebugDraw* debugDrawer)
		:m_originalManifoldResult(originalResult),
		m_transformA(transformA),
		m_transformB(transformB),
		m_perturbA(perturbA),
		m_unPerturbedTransform(unPerturbedTransform),
		m_debugDrawer(debugDrawer)
	{
	}
	virtual ~ D_btPerturbedContactResult()
	{
	}

	virtual void addContactPoint(const D_btVector3& normalOnBInWorld,const D_btVector3& pointInWorld,D_btScalar orgDepth)
	{
		D_btVector3 endPt,startPt;
		D_btScalar newDepth;
		D_btVector3 newNormal;

		if (m_perturbA)
		{
			D_btVector3 endPtOrg = pointInWorld + normalOnBInWorld*orgDepth;
			endPt = (m_unPerturbedTransform*m_transformA.inverse())(endPtOrg);
			newDepth = (endPt -  pointInWorld).dot(normalOnBInWorld);
			startPt = endPt+normalOnBInWorld*newDepth;
		} else
		{
			endPt = pointInWorld + normalOnBInWorld*orgDepth;
			startPt = (m_unPerturbedTransform*m_transformB.inverse())(pointInWorld);
			newDepth = (endPt -  startPt).dot(normalOnBInWorld);
			
		}

//#define DEBUG_CONTACTS 1
#ifdef DEBUG_CONTACTS
		m_debugDrawer->drawLine(startPt,endPt,D_btVector3(1,0,0));
		m_debugDrawer->drawSphere(startPt,0.05,D_btVector3(0,1,0));
		m_debugDrawer->drawSphere(endPt,0.05,D_btVector3(0,0,1));
#endif //DEBUG_CONTACTS

		
		m_originalManifoldResult->addContactPoint(normalOnBInWorld,startPt,newDepth);
	}

};

extern D_btScalar D_gContactBreakingThreshold;


//
// Convex-Convex collision algorithm
//
void D_btConvexConvexAlgorithm ::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
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
#ifndef D_BT_DISABLE_CAPSULE_CAPSULE_COLLIDER
	if ((min0->getShapeType() == D_CAPSULE_SHAPE_PROXYTYPE) && (min1->getShapeType() == D_CAPSULE_SHAPE_PROXYTYPE))
	{
		D_btCapsuleShape* capsuleA = (D_btCapsuleShape*) min0;
		D_btCapsuleShape* capsuleB = (D_btCapsuleShape*) min1;
		D_btVector3 localScalingA = capsuleA->getLocalScaling();
		D_btVector3 localScalingB = capsuleB->getLocalScaling();
		
		D_btScalar threshold = m_manifoldPtr->getContactBreakingThreshold();

		D_btScalar dist = capsuleCapsuleDistance(normalOnB,	pointOnBWorld,capsuleA->getHalfHeight(),capsuleA->getRadius(),
			capsuleB->getHalfHeight(),capsuleB->getRadius(),capsuleA->getUpAxis(),capsuleB->getUpAxis(),
			body0->getWorldTransform(),body1->getWorldTransform(),threshold);

		if (dist<threshold)
		{
			D_btAssert(normalOnB.length2()>=(D_SIMD_EPSILON*D_SIMD_EPSILON));
			resultOut->addContactPoint(normalOnB,pointOnBWorld,dist);	
		}
		resultOut->refreshContactPoints();
		return;
	}
#endif //D_BT_DISABLE_CAPSULE_CAPSULE_COLLIDER


#ifdef D_USE_SEPDISTANCE_UTIL2
	m_sepDistance.updateSeparatingDistance(body0->getWorldTransform(),body1->getWorldTransform());
	if (!dispatchInfo.m_useConvexConservativeDistanceUtil || m_sepDistance.getConservativeSeparatingDistance()<=0.f)
#endif //D_USE_SEPDISTANCE_UTIL2

	{

	
	D_btGjkPairDetector::D_ClosestPointInput input;

	D_btGjkPairDetector	gjkPairDetector(min0,min1,m_simplexSolver,m_pdSolver);
	//TODO: if (dispatchInfo.m_useContinuous)
	gjkPairDetector.setMinkowskiA(min0);
	gjkPairDetector.setMinkowskiB(min1);

#ifdef D_USE_SEPDISTANCE_UTIL2
	if (dispatchInfo.m_useConvexConservativeDistanceUtil)
	{
		input.m_maximumDistanceSquared = D_BT_LARGE_FLOAT;
	} else
#endif //D_USE_SEPDISTANCE_UTIL2
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
	

#ifdef D_USE_SEPDISTANCE_UTIL2
	D_btScalar sepDist = 0.f;
	if (dispatchInfo.m_useConvexConservativeDistanceUtil)
	{
		sepDist = gjkPairDetector.getCachedSeparatingDistance();
		if (sepDist>D_SIMD_EPSILON)
		{
			sepDist += dispatchInfo.m_convexConservativeDistanceThreshold;
			//now perturbe directions D_to get multiple contact points
			sepNormalWorldSpace = gjkPairDetector.getCachedSeparatingAxis().normalized();
			D_btPlaneSpace1(sepNormalWorldSpace,v0,v1);
		}
	}
#endif //D_USE_SEPDISTANCE_UTIL2

	//now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects
	
	//perform perturbation when more then 'm_minimumPointsPerturbationThreshold' points
	if (resultOut->getPersistentManifold()->getNumContacts() < m_minimumPointsPerturbationThreshold)
	{
		
		int i;

		bool perturbeA = true;
		const D_btScalar angleLimit = 0.125f * D_SIMD_PI;
		D_btScalar perturbeAngle;
		D_btScalar radiusA = min0->getAngularMotionDisc();
		D_btScalar radiusB = min1->getAngularMotionDisc();
		if (radiusA < radiusB)
		{
			perturbeAngle = D_gContactBreakingThreshold /radiusA;
			perturbeA = true;
		} else
		{
			perturbeAngle = D_gContactBreakingThreshold / radiusB;
			perturbeA = false;
		}
		if ( perturbeAngle > angleLimit ) 
				perturbeAngle = angleLimit;

		D_btTransform unPerturbedTransform;
		if (perturbeA)
		{
			unPerturbedTransform = input.m_transformA;
		} else
		{
			unPerturbedTransform = input.m_transformB;
		}
		
		for ( i=0;i<m_numPerturbationIterations;i++)
		{
			D_btQuaternion perturbeRot(v0,perturbeAngle);
			D_btScalar iterationAngle = i*(D_SIMD_2_PI/D_btScalar(m_numPerturbationIterations));
			D_btQuaternion rotq(sepNormalWorldSpace,iterationAngle);
			
			
			if (perturbeA)
			{
				input.m_transformA.setBasis(  D_btMatrix3x3(rotq.inverse()*perturbeRot*rotq)*body0->getWorldTransform().getBasis());
				input.m_transformB = body1->getWorldTransform();
#ifdef DEBUG_CONTACTS
				dispatchInfo.m_debugDraw->drawTransform(input.m_transformA,10.0);
#endif //DEBUG_CONTACTS
			} else
			{
				input.m_transformA = body0->getWorldTransform();
				input.m_transformB.setBasis( D_btMatrix3x3(rotq.inverse()*perturbeRot*rotq)*body1->getWorldTransform().getBasis());
#ifdef DEBUG_CONTACTS
				dispatchInfo.m_debugDraw->drawTransform(input.m_transformB,10.0);
#endif
			}
			
			D_btPerturbedContactResult perturbedResultOut(resultOut,input.m_transformA,input.m_transformB,unPerturbedTransform,perturbeA,dispatchInfo.m_debugDraw);
			gjkPairDetector.getClosestPoints(input,perturbedResultOut,dispatchInfo.m_debugDraw);
			
			
		}
	}

	

#ifdef D_USE_SEPDISTANCE_UTIL2
	if (dispatchInfo.m_useConvexConservativeDistanceUtil && (sepDist>D_SIMD_EPSILON))
	{
		m_sepDistance.initSeparatingDistance(gjkPairDetector.getCachedSeparatingAxis(),sepDist,body0->getWorldTransform(),body1->getWorldTransform());
	}
#endif //D_USE_SEPDISTANCE_UTIL2


	}

	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}

}



bool D_disableCcd = false;
D_btScalar	D_btConvexConvexAlgorithm::calculateTimeOfImpact(D_btCollisionObject* col0,D_btCollisionObject* col1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
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

	if (D_disableCcd)
		return D_btScalar(1.);


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

