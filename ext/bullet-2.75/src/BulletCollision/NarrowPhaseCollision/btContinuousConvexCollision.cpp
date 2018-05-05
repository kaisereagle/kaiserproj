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


#include "btContinuousConvexCollision.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "LinearMath/btTransformUtil.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "btGjkPairDetector.h"
#include "btPointCollector.h"



D_btContinuousConvexCollision::D_btContinuousConvexCollision ( const D_btConvexShape*	convexA,const D_btConvexShape*	convexB,D_btSimplexSolverInterface* simplexSolver, D_btConvexPenetrationDepthSolver* penetrationDepthSolver)
:m_simplexSolver(simplexSolver),
m_penetrationDepthSolver(penetrationDepthSolver),
m_convexA(convexA),m_convexB(convexB)
{
}

/// This maximum D_should not be necessary. It D_allows for untested/degenerate cases in production code.
/// You don't want your game ever D_to lock-up.
#define D_MAX_ITERATIONS 64

bool	D_btContinuousConvexCollision::calcTimeOfImpact(
				const D_btTransform& fromA,
				const D_btTransform& toA,
				const D_btTransform& fromB,
				const D_btTransform& toB,
				CastResult& result)
{

	m_simplexSolver->reset();

	/// compute linear D_and angular velocity for this interval, D_to interpolate
	D_btVector3 linVelA,angVelA,linVelB,angVelB;
	D_btTransformUtil::calculateVelocity(fromA,toA,D_btScalar(1.),linVelA,angVelA);
	D_btTransformUtil::calculateVelocity(fromB,toB,D_btScalar(1.),linVelB,angVelB);


	D_btScalar boundingRadiusA = m_convexA->getAngularMotionDisc();
	D_btScalar boundingRadiusB = m_convexB->getAngularMotionDisc();

	D_btScalar maxAngularProjectedVelocity = angVelA.length() * boundingRadiusA + angVelB.length() * boundingRadiusB;
	D_btVector3 relLinVel = (linVelB-linVelA);

	D_btScalar relLinVelocLength = (linVelB-linVelA).length();
	
	if ((relLinVelocLength+maxAngularProjectedVelocity) == 0.f)
		return false;


	D_btScalar radius = D_btScalar(0.001);

	D_btScalar lambda = D_btScalar(0.);
	D_btVector3 v(1,0,0);

	int maxIter = D_MAX_ITERATIONS;

	D_btVector3 n;
	n.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	bool hasResult = false;
	D_btVector3 c;

	D_btScalar lastLambda = lambda;
	//D_btScalar epsilon = D_btScalar(0.001);

	int numIter = 0;
	//first solution, using GJK


	D_btTransform identityTrans;
	identityTrans.setIdentity();

	D_btSphereShape	raySphere(D_btScalar(0.0));
	raySphere.setMargin(D_btScalar(0.));


//	result.drawCoordSystem(sphereTr);

	D_btPointCollector	pointCollector1;

	{
		
		D_btGjkPairDetector gjk(m_convexA,m_convexB,m_convexA->getShapeType(),m_convexB->getShapeType(),m_convexA->getMargin(),m_convexB->getMargin(),m_simplexSolver,m_penetrationDepthSolver);		
		D_btGjkPairDetector::D_ClosestPointInput input;
	
		//we don't use margins during CCD
	//	gjk.setIgnoreMargin(true);

		input.m_transformA = fromA;
		input.m_transformB = fromB;
		gjk.getClosestPoints(input,pointCollector1,0);

		hasResult = pointCollector1.m_hasResult;
		c = pointCollector1.m_pointInWorld;
	}

	if (hasResult)
	{
		D_btScalar dist;
		dist = pointCollector1.m_distance;
		n = pointCollector1.m_normalOnBInWorld;

		D_btScalar projectedLinearVelocity = relLinVel.dot(n);
		
		//not close enough
		while (dist > radius)
		{
			if (result.m_debugDrawer)
			{
				result.m_debugDrawer->drawSphere(c,0.2f,D_btVector3(1,1,1));
			}
			numIter++;
			if (numIter > maxIter)
			{
				return false; //todo: report a failure
			}
			D_btScalar D_dLambda = D_btScalar(0.);

			projectedLinearVelocity = relLinVel.dot(n);

			//calculate safe moving fraction from distance / (linear+rotational velocity)
			
			//D_btScalar clippedDist  = GEN_min(angularConservativeRadius,dist);
			//D_btScalar clippedDist  = dist;
			
			//don't report time of impact for motion away from the contact normal (or causes minor penetration)
			if ((projectedLinearVelocity+ maxAngularProjectedVelocity)<=D_SIMD_EPSILON)
				return false;
			
			D_dLambda = dist / (projectedLinearVelocity+ maxAngularProjectedVelocity);

			
			
			lambda = lambda + D_dLambda;

			if (lambda > D_btScalar(1.))
				return false;

			if (lambda < D_btScalar(0.))
				return false;


			//todo: next check with relative epsilon
			if (lambda <= lastLambda)
			{
				return false;
				//n.setValue(0,0,0);
				break;
			}
			lastLambda = lambda;

			

			//interpolate D_to next lambda
			D_btTransform interpolatedTransA,interpolatedTransB,relativeTrans;

			D_btTransformUtil::integrateTransform(fromA,linVelA,angVelA,lambda,interpolatedTransA);
			D_btTransformUtil::integrateTransform(fromB,linVelB,angVelB,lambda,interpolatedTransB);
			relativeTrans = interpolatedTransB.inverseTimes(interpolatedTransA);

			if (result.m_debugDrawer)
			{
				result.m_debugDrawer->drawSphere(interpolatedTransA.getOrigin(),0.2f,D_btVector3(1,0,0));
			}

			result.D_DebugDraw( lambda );

			D_btPointCollector	pointCollector;
			D_btGjkPairDetector gjk(m_convexA,m_convexB,m_simplexSolver,m_penetrationDepthSolver);
			D_btGjkPairDetector::D_ClosestPointInput input;
			input.m_transformA = interpolatedTransA;
			input.m_transformB = interpolatedTransB;
			gjk.getClosestPoints(input,pointCollector,0);
			if (pointCollector.m_hasResult)
			{
				if (pointCollector.m_distance < D_btScalar(0.))
				{
					//degenerate ?!
					result.m_fraction = lastLambda;
					n = pointCollector.m_normalOnBInWorld;
					result.m_normal=n;//.setValue(1,1,1);// = n;
					result.m_hitPoint = pointCollector.m_pointInWorld;
					return true;
				}
				c = pointCollector.m_pointInWorld;		
				n = pointCollector.m_normalOnBInWorld;
				dist = pointCollector.m_distance;
			} else
			{
				//??
				return false;
			}
			

		}
	
		if ((projectedLinearVelocity+ maxAngularProjectedVelocity)<=result.m_allowedPenetration)//D_SIMD_EPSILON)
			return false;
			
		result.m_fraction = lambda;
		result.m_normal = n;
		result.m_hitPoint = c;
		return true;
	}

	return false;

/*
//todo:
	//if movement away from normal, discard result
	D_btVector3 move = transBLocalTo.getOrigin() - transBLocalFrom.getOrigin();
	if (result.m_fraction < D_btScalar(1.))
	{
		if (move.dot(result.m_normal) <= D_btScalar(0.))
		{
		}
	}
*/

}
