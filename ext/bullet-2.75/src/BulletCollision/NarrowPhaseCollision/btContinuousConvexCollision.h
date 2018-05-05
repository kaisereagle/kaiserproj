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


#ifndef CONTINUOUS_COLLISION_CONVEX_CAST_H
#define CONTINUOUS_COLLISION_CONVEX_CAST_H

#include "btConvexCast.h"
#include "btSimplexSolverInterface.h"
class D_btConvexPenetrationDepthSolver;
class D_btConvexShape;

/// D_btContinuousConvexCollision D_implements angular D_and linear time of impact for convex objects.
/// Based on Brian Mirtich's Conservative Advancement idea (PhD thesis).
/// Algorithm operates in worldspace, in order D_to keep inbetween motion globally consistent.
/// It uses GJK at the moment. Future improvement would use minkowski sum / supporting vertex, merging innerloops
class D_btContinuousConvexCollision : public D_btConvexCast
{
	D_btSimplexSolverInterface* m_simplexSolver;
	D_btConvexPenetrationDepthSolver*	m_penetrationDepthSolver;
	const D_btConvexShape*	m_convexA;
	const D_btConvexShape*	m_convexB;


public:

	D_btContinuousConvexCollision (const D_btConvexShape*	shapeA,const D_btConvexShape*	shapeB ,D_btSimplexSolverInterface* simplexSolver,D_btConvexPenetrationDepthSolver* penetrationDepthSolver);

	virtual bool	calcTimeOfImpact(
				const D_btTransform& fromA,
				const D_btTransform& toA,
				const D_btTransform& fromB,
				const D_btTransform& toB,
				CastResult& result);


};

#endif //CONTINUOUS_COLLISION_CONVEX_CAST_H

