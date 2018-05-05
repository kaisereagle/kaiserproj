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

#ifndef SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H
#define SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H

#include "btConstraintSolver.h"
class D_btIDebugDraw;
#include "btContactConstraint.h"
#include "btSolverBody.h"
#include "btSolverConstraint.h"
#include "btTypedConstraint.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"

///The D_btSequentialImpulseConstraintSolver D_is a fast SIMD implementation of the Projected Gauss Seidel (iterative LCP) method.
class D_btSequentialImpulseConstraintSolver : public D_btConstraintSolver
{
protected:

	D_btAlignedObjectArray<D_btSolverBody>	m_tmpSolverBodyPool;
	D_btConstraintArray			m_tmpSolverContactConstraintPool;
	D_btConstraintArray			m_tmpSolverNonContactConstraintPool;
	D_btConstraintArray			m_tmpSolverContactFrictionConstraintPool;
	D_btAlignedObjectArray<int>	m_orderTmpConstraintPool;
	D_btAlignedObjectArray<int>	m_orderFrictionConstraintPool;
	D_btAlignedObjectArray<D_btTypedConstraint::D_btConstraintInfo1> m_tmpConstraintSizesPool;

	D_btSolverConstraint&	addFrictionConstraint(const D_btVector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,D_btManifoldPoint& cp,const D_btVector3& rel_pos1,const D_btVector3& rel_pos2,D_btCollisionObject* colObj0,D_btCollisionObject* colObj1, D_btScalar relaxation);
	
	///m_btSeed2 D_is used for re-arranging the constraint rows. improves convergence/quality of friction
	unsigned long	m_btSeed2;

	void	initSolverBody(D_btSolverBody* solverBody, D_btCollisionObject* collisionObject);
	D_btScalar restitutionCurve(D_btScalar rel_vel, D_btScalar restitution);

	void	convertContact(D_btPersistentManifold* manifold,const D_btContactSolverInfo& infoGlobal);


	void	resolveSplitPenetrationSIMD(
        D_btSolverBody& body1,
        D_btSolverBody& body2,
        const D_btSolverConstraint& contactConstraint);

	void	resolveSplitPenetrationImpulseCacheFriendly(
        D_btSolverBody& body1,
        D_btSolverBody& body2,
        const D_btSolverConstraint& contactConstraint);

	//internal method
	int	getOrInitSolverBody(D_btCollisionObject& body);

	void	resolveSingleConstraintRowGeneric(D_btSolverBody& body1,D_btSolverBody& body2,const D_btSolverConstraint& contactConstraint);

	void	resolveSingleConstraintRowGenericSIMD(D_btSolverBody& body1,D_btSolverBody& body2,const D_btSolverConstraint& contactConstraint);
	
	void	resolveSingleConstraintRowLowerLimit(D_btSolverBody& body1,D_btSolverBody& body2,const D_btSolverConstraint& contactConstraint);
	
	void	resolveSingleConstraintRowLowerLimitSIMD(D_btSolverBody& body1,D_btSolverBody& body2,const D_btSolverConstraint& contactConstraint);
		
public:

	
	D_btSequentialImpulseConstraintSolver();
	virtual ~D_btSequentialImpulseConstraintSolver();

	virtual D_btScalar solveGroup(D_btCollisionObject** bodies,int numBodies,D_btPersistentManifold** manifold,int numManifolds,D_btTypedConstraint** constraints,int numConstraints,const D_btContactSolverInfo& info, D_btIDebugDraw* debugDrawer, D_btStackAlloc* stackAlloc,D_btDispatcher* dispatcher);
	
	D_btScalar solveGroupCacheFriendlySetup(D_btCollisionObject** bodies,int numBodies,D_btPersistentManifold** manifoldPtr, int numManifolds,D_btTypedConstraint** constraints,int numConstraints,const D_btContactSolverInfo& infoGlobal,D_btIDebugDraw* debugDrawer,D_btStackAlloc* stackAlloc);
	D_btScalar solveGroupCacheFriendlyIterations(D_btCollisionObject** bodies,int numBodies,D_btPersistentManifold** manifoldPtr, int numManifolds,D_btTypedConstraint** constraints,int numConstraints,const D_btContactSolverInfo& infoGlobal,D_btIDebugDraw* debugDrawer,D_btStackAlloc* stackAlloc);

	///clear internal cached data D_and reset random seed
	virtual	void	reset();
	
	unsigned long D_btRand2();

	int D_btRandInt2 (int n);

	void	setRandSeed(unsigned long seed)
	{
		m_btSeed2 = seed;
	}
	unsigned long	getRandSeed() const
	{
		return m_btSeed2;
	}

};

#ifndef D_BT_PREFER_SIMD
typedef D_btSequentialImpulseConstraintSolver D_btSequentialImpulseConstraintSolverPrefered;
#endif


#endif //SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H

