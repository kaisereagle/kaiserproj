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

#ifndef CONSTRAINT_SOLVER_H
#define CONSTRAINT_SOLVER_H

#include "LinearMath/btScalar.h"

class D_btPersistentManifold;
class D_btRigidBody;
class D_btCollisionObject;
class D_btTypedConstraint;
struct D_btContactSolverInfo;
struct D_btBroadphaseProxy;
class D_btIDebugDraw;
class D_btStackAlloc;
class	D_btDispatcher;
/// D_btConstraintSolver provides solver interface
class D_btConstraintSolver
{

public:

	virtual ~D_btConstraintSolver() {}
	
	virtual void prepareSolve (int /* numBodies */, int /* numManifolds */) {;}

	///solve a group of constraints
	virtual D_btScalar solveGroup(D_btCollisionObject** bodies,int numBodies,D_btPersistentManifold** manifold,int numManifolds,D_btTypedConstraint** constraints,int numConstraints, const D_btContactSolverInfo& info,class D_btIDebugDraw* debugDrawer, D_btStackAlloc* stackAlloc,D_btDispatcher* dispatcher) = 0;

	virtual void allSolved (const D_btContactSolverInfo& /* info */,class D_btIDebugDraw* /* debugDrawer */, D_btStackAlloc* /* stackAlloc */) {;}

	///clear internal cached data D_and reset random seed
	virtual	void	reset() = 0;
};




#endif //CONSTRAINT_SOLVER_H
