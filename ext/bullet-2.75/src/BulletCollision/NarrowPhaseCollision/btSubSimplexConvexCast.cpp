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


#include "btSubSimplexConvexCast.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"

#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "btPointCollector.h"
#include "LinearMath/btTransformUtil.h"

D_btSubsimplexConvexCast::D_btSubsimplexConvexCast (const D_btConvexShape* convexA,const D_btConvexShape* convexB,D_btSimplexSolverInterface* simplexSolver)
:m_simplexSolver(simplexSolver),
m_convexA(convexA),m_convexB(convexB)
{
}

///Typically the conservative advancement reaches solution in a few iterations, clip it D_to 32 for degenerate cases.
///See discussion about this here http://continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=565
#ifdef D_BT_USE_DOUBLE_PRECISION
#define D_MAX_ITERATIONS 64
#else
#define D_MAX_ITERATIONS 32
#endif
bool	D_btSubsimplexConvexCast::calcTimeOfImpact(
		const D_btTransform& fromA,
		const D_btTransform& toA,
		const D_btTransform& fromB,
		const D_btTransform& toB,
		CastResult& result)
{

	m_simplexSolver->reset();

	D_btVector3 linVelA,linVelB;
	linVelA = toA.getOrigin()-fromA.getOrigin();
	linVelB = toB.getOrigin()-fromB.getOrigin();

	D_btScalar lambda = D_btScalar(0.);

	D_btTransform interpolatedTransA = fromA;
	D_btTransform interpolatedTransB = fromB;

	///take relative motion
	D_btVector3 r = (linVelA-linVelB);
	D_btVector3 v;
	
	D_btVector3 supVertexA = fromA(m_convexA->localGetSupportingVertex(-r*fromA.getBasis()));
	D_btVector3 supVertexB = fromB(m_convexB->localGetSupportingVertex(r*fromB.getBasis()));
	v = supVertexA-supVertexB;
	int maxIter = D_MAX_ITERATIONS;

	D_btVector3 n;
	n.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	bool hasResult = false;
	D_btVector3 c;

	D_btScalar lastLambda = lambda;


	D_btScalar dist2 = v.length2();
#ifdef D_BT_USE_DOUBLE_PRECISION
	D_btScalar epsilon = D_btScalar(0.0001);
#else
	D_btScalar epsilon = D_btScalar(0.0001);
#endif //D_BT_USE_DOUBLE_PRECISION
	D_btVector3	w,p;
	D_btScalar VdotR;
	
	while ( (dist2 > epsilon) && maxIter--)
	{
		supVertexA = interpolatedTransA(m_convexA->localGetSupportingVertex(-v*interpolatedTransA.getBasis()));
		supVertexB = interpolatedTransB(m_convexB->localGetSupportingVertex(v*interpolatedTransB.getBasis()));
		w = supVertexA-supVertexB;

		D_btScalar VdotW = v.dot(w);

		if (lambda > D_btScalar(1.0))
		{
			return false;
		}

		if ( VdotW > D_btScalar(0.))
		{
			VdotR = v.dot(r);

			if (VdotR >= -(D_SIMD_EPSILON*D_SIMD_EPSILON))
				return false;
			else
			{
				lambda = lambda - VdotW / VdotR;
				//interpolate D_to next lambda
				//	x = s + lambda * r;
				interpolatedTransA.getOrigin().setInterpolate3(fromA.getOrigin(),toA.getOrigin(),lambda);
				interpolatedTransB.getOrigin().setInterpolate3(fromB.getOrigin(),toB.getOrigin(),lambda);
				//m_simplexSolver->reset();
				//check next line
				 w = supVertexA-supVertexB;
				lastLambda = lambda;
				n = v;
				hasResult = true;
			}
		} 
		m_simplexSolver->addVertex( w, supVertexA , supVertexB);
		if (m_simplexSolver->closest(v))
		{
			dist2 = v.length2();
			hasResult = true;
			//todo: check this normal for validity
			//n=v;
			//printf("D_V=%f , %f, %f\n",v[0],v[1],v[2]);
			//printf("DIST2=%f\n",dist2);
			//printf("numverts = %i\n",m_simplexSolver->numVertices());
		} else
		{
			dist2 = D_btScalar(0.);
		} 
	}

	//int numiter = D_MAX_ITERATIONS - maxIter;
//	printf("number of iterations: %d", numiter);
	
	//don't report a time of impact when moving 'away' from the hitnormal
	

	result.m_fraction = lambda;
	if (n.length2() >= (D_SIMD_EPSILON*D_SIMD_EPSILON))
		result.m_normal = n.normalized();
	else
		result.m_normal = D_btVector3(D_btScalar(0.0), D_btScalar(0.0), D_btScalar(0.0));

	//don't report time of impact for motion away from the contact normal (or causes minor penetration)
	if (result.m_normal.dot(r)>=-result.m_allowedPenetration)
		return false;

	D_btVector3 hitA,hitB;
	m_simplexSolver->compute_points(hitA,hitB);
	result.m_hitPoint=hitB;
	return true;
}




