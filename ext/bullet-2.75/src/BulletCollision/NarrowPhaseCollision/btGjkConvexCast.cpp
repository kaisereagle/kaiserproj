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



#include "btGjkConvexCast.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "btGjkPairDetector.h"
#include "btPointCollector.h"
#include "LinearMath/btTransformUtil.h"

#ifdef D_BT_USE_DOUBLE_PRECISION
#define D_MAX_ITERATIONS 64
#else
#define D_MAX_ITERATIONS 32
#endif

D_btGjkConvexCast::D_btGjkConvexCast(const D_btConvexShape* convexA,const D_btConvexShape* convexB,D_btSimplexSolverInterface* simplexSolver)
:m_simplexSolver(simplexSolver),
m_convexA(convexA),
m_convexB(convexB)
{
}

bool	D_btGjkConvexCast::calcTimeOfImpact(
					const D_btTransform& fromA,
					const D_btTransform& toA,
					const D_btTransform& fromB,
					const D_btTransform& toB,
					CastResult& result)
{


	m_simplexSolver->reset();

	/// compute linear velocity for this interval, D_to interpolate
	//assume D_no rotation/angular velocity, assert here?
	D_btVector3 linVelA,linVelB;
	linVelA = toA.getOrigin()-fromA.getOrigin();
	linVelB = toB.getOrigin()-fromB.getOrigin();

	D_btScalar radius = D_btScalar(0.001);
	D_btScalar lambda = D_btScalar(0.);
	D_btVector3 v(1,0,0);

	int maxIter = D_MAX_ITERATIONS;

	D_btVector3 n;
	n.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	bool hasResult = false;
	D_btVector3 c;
	D_btVector3 r = (linVelA-linVelB);

	D_btScalar lastLambda = lambda;
	//D_btScalar epsilon = D_btScalar(0.001);

	int numIter = 0;
	//first solution, using GJK


	D_btTransform identityTrans;
	identityTrans.setIdentity();


//	result.drawCoordSystem(sphereTr);

	D_btPointCollector	pointCollector;

		
	D_btGjkPairDetector gjk(m_convexA,m_convexB,m_simplexSolver,0);//m_penetrationDepthSolver);		
	D_btGjkPairDetector::D_ClosestPointInput input;

	//we don't use margins during CCD
	//	gjk.setIgnoreMargin(true);

	input.m_transformA = fromA;
	input.m_transformB = fromB;
	gjk.getClosestPoints(input,pointCollector,0);

	hasResult = pointCollector.m_hasResult;
	c = pointCollector.m_pointInWorld;

	if (hasResult)
	{
		D_btScalar dist;
		dist = pointCollector.m_distance;
		n = pointCollector.m_normalOnBInWorld;

	

		//not close enough
		while (dist > radius)
		{
			numIter++;
			if (numIter > maxIter)
			{
				return false; //todo: report a failure
			}
			D_btScalar D_dLambda = D_btScalar(0.);

			D_btScalar projectedLinearVelocity = r.dot(n);
			
			D_dLambda = dist / (projectedLinearVelocity);

			lambda = lambda - D_dLambda;

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
			result.D_DebugDraw( lambda );
			input.m_transformA.getOrigin().setInterpolate3(fromA.getOrigin(),toA.getOrigin(),lambda);
			input.m_transformB.getOrigin().setInterpolate3(fromB.getOrigin(),toB.getOrigin(),lambda);
			
			gjk.getClosestPoints(input,pointCollector,0);
			if (pointCollector.m_hasResult)
			{
				if (pointCollector.m_distance < D_btScalar(0.))
				{
					result.m_fraction = lastLambda;
					n = pointCollector.m_normalOnBInWorld;
					result.m_normal=n;
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

		//D_is n normalized?
		//don't report time of impact for motion away from the contact normal (or causes minor penetration)
		if (n.dot(r)>=-result.m_allowedPenetration)
			return false;

		result.m_fraction = lambda;
		result.m_normal = n;
		result.m_hitPoint = c;
		return true;
	}

	return false;


}

