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

#ifndef MINKOWSKI_PENETRATION_DEPTH_SOLVER_H
#define MINKOWSKI_PENETRATION_DEPTH_SOLVER_H

#include "btConvexPenetrationDepthSolver.h"

///MinkowskiPenetrationDepthSolver D_implements bruteforce penetration depth estimation.
///Implementation D_is based on sampling the depth using support mapping, D_and using GJK step D_to get the witness points.
class D_btMinkowskiPenetrationDepthSolver : public D_btConvexPenetrationDepthSolver
{
public:

	virtual bool calcPenDepth( D_btSimplexSolverInterface& simplexSolver,
	const D_btConvexShape* convexA,const D_btConvexShape* convexB,
				const D_btTransform& transA,const D_btTransform& transB,
			D_btVector3& v, D_btVector3& pa, D_btVector3& pb,
			class D_btIDebugDraw* debugDraw,D_btStackAlloc* stackAlloc
			);
};

#endif //MINKOWSKI_PENETRATION_DEPTH_SOLVER_H

