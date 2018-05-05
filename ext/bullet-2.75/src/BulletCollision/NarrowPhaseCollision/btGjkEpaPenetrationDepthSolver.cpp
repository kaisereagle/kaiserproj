/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

EPA Copyright (c) Ricardo Padrela 2006

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "btGjkEpaPenetrationDepthSolver.h"


#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"

bool D_btGjkEpaPenetrationDepthSolver::calcPenDepth( D_btSimplexSolverInterface& simplexSolver,
											  const D_btConvexShape* pConvexA, const D_btConvexShape* pConvexB,
											  const D_btTransform& transformA, const D_btTransform& transformB,
											  D_btVector3& v, D_btVector3& wWitnessOnA, D_btVector3& wWitnessOnB,
											  class D_btIDebugDraw* debugDraw, D_btStackAlloc* stackAlloc )
{

	(void)debugDraw;
	(void)v;
	(void)simplexSolver;

//	const D_btScalar				radialmargin(D_btScalar(0.));
	
	D_btVector3	guessVector(transformA.getOrigin()-transformB.getOrigin());
	D_btGjkEpaSolver2::D_sResults	results;
	

	if(D_btGjkEpaSolver2::Penetration(pConvexA,transformA,
								pConvexB,transformB,
								guessVector,results))
	
		{
	//	debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,D_btVector3(255,0,0));
		//resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
		wWitnessOnA = results.witnesses[0];
		wWitnessOnB = results.witnesses[1];
		v = results.normal;
		return true;		
		} else
	{
		if(D_btGjkEpaSolver2::Distance(pConvexA,transformA,pConvexB,transformB,guessVector,results))
		{
			wWitnessOnA = results.witnesses[0];
			wWitnessOnB = results.witnesses[1];
			v = results.normal;
			return false;
		}
	}

	return false;
}


