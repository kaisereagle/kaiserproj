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

#ifndef POINT2POINTCONSTRAINT_H
#define POINT2POINTCONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class D_btRigidBody;

struct	D_btConstraintSetting
{
	D_btConstraintSetting()	:
		m_tau(D_btScalar(0.3)),
		m_damping(D_btScalar(1.)),
		m_impulseClamp(D_btScalar(0.))
	{
	}
	D_btScalar		m_tau;
	D_btScalar		m_damping;
	D_btScalar		m_impulseClamp;
};

/// point D_to point constraint between two rigidbodies each with a pivotpoint that descibes the 'ballsocket' location in local space
D_ATTRIBUTE_ALIGNED16(class) D_btPoint2PointConstraint : public D_btTypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
	D_btJacobianEntry	m_jac[3]; //3 orthogonal linear constraints
	
	D_btVector3	m_pivotInA;
	D_btVector3	m_pivotInB;
	
	
	
public:

	///for backwards compatibility during the transition D_to 'getInfo/getInfo2'
	bool		m_useSolveConstraintObsolete;

	D_btConstraintSetting	m_setting;

	D_btPoint2PointConstraint(D_btRigidBody& rbA,D_btRigidBody& rbB, const D_btVector3& pivotInA,const D_btVector3& pivotInB);

	D_btPoint2PointConstraint(D_btRigidBody& rbA,const D_btVector3& pivotInA);

	D_btPoint2PointConstraint();

	virtual void	buildJacobian();

	virtual void getInfo1 (D_btConstraintInfo1* info);

	void getInfo1NonVirtual (D_btConstraintInfo1* info);

	virtual void getInfo2 (D_btConstraintInfo2* info);

	void getInfo2NonVirtual (D_btConstraintInfo2* info, const D_btTransform& body0_trans, const D_btTransform& body1_trans);

	virtual	void	solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep);

	void	updateRHS(D_btScalar	timeStep);

	void	setPivotA(const D_btVector3& pivotA)
	{
		m_pivotInA = pivotA;
	}

	void	setPivotB(const D_btVector3& pivotB)
	{
		m_pivotInB = pivotB;
	}

	const D_btVector3& getPivotInA() const
	{
		return m_pivotInA;
	}

	const D_btVector3& getPivotInB() const
	{
		return m_pivotInB;
	}


};

#endif //POINT2POINTCONSTRAINT_H
