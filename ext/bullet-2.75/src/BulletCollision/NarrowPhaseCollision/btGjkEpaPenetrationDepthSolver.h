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
#ifndef D_BT_GJP_EPA_PENETRATION_DEPTH_H
#define D_BT_GJP_EPA_PENETRATION_DEPTH_H

#include "btConvexPenetrationDepthSolver.h"

///EpaPenetrationDepthSolver uses the Expanding Polytope Algorithm D_to
///calculate the penetration depth between two convex D_shapes.
class D_btGjkEpaPenetrationDepthSolver : public D_btConvexPenetrationDepthSolver
{
	public :

		D_btGjkEpaPenetrationDepthSolver()
		{
		}

		bool			calcPenDepth( D_btSimplexSolverInterface& simplexSolver,
									  const D_btConvexShape* pConvexA, const D_btConvexShape* pConvexB,
									  const D_btTransform& transformA, const D_btTransform& transformB,
									  D_btVector3& v, D_btVector3& wWitnessOnA, D_btVector3& wWitnessOnB,
									  class D_btIDebugDraw* debugDraw,D_btStackAlloc* stackAlloc );

	private :

};

#endif	// D_BT_GJP_EPA_PENETRATION_DEPTH_H

