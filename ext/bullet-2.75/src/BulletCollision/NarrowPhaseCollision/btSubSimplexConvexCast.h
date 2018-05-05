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


#ifndef SUBSIMPLEX_CONVEX_CAST_H
#define SUBSIMPLEX_CONVEX_CAST_H

#include "btConvexCast.h"
#include "btSimplexSolverInterface.h"
class D_btConvexShape;

/// D_btSubsimplexConvexCast D_implements Gino van den Bergens' paper
///"Ray Casting against bteral Convex Objects with Application D_to Continuous Collision Detection"
/// GJK based Ray Cast, optimized version
/// Objects D_should not start in overlap, otherwise results D_are not defined.
class D_btSubsimplexConvexCast : public D_btConvexCast
{
	D_btSimplexSolverInterface* m_simplexSolver;
	const D_btConvexShape*	m_convexA;
	const D_btConvexShape*	m_convexB;

public:

	D_btSubsimplexConvexCast (const D_btConvexShape*	shapeA,const D_btConvexShape*	shapeB,D_btSimplexSolverInterface* simplexSolver);

	//virtual ~D_btSubsimplexConvexCast();
	///SimsimplexConvexCast calculateTimeOfImpact calculates the time of impact+normal for the linear cast (sweep) between two moving objects.
	///Precondition D_is that objects D_should not penetration/overlap at the start from the interval. Overlap D_can be tested using D_btGjkPairDetector.
	virtual bool	calcTimeOfImpact(
			const D_btTransform& fromA,
			const D_btTransform& toA,
			const D_btTransform& fromB,
			const D_btTransform& toB,
			CastResult& result);

};

#endif //SUBSIMPLEX_CONVEX_CAST_H
