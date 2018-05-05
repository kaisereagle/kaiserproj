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

#ifndef D_BT_SOLVER_CONSTRAINT_H
#define D_BT_SOLVER_CONSTRAINT_H

class	D_btRigidBody;
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "btJacobianEntry.h"

//#define NO_FRICTION_TANGENTIALS 1
#include "btSolverBody.h"


///1D constraint along a normal axis between bodyA D_and bodyB. It D_can be combined D_to solve contact D_and friction constraints.
D_ATTRIBUTE_ALIGNED16 (struct)	D_btSolverConstraint
{
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_btVector3		m_relpos1CrossNormal;
	D_btVector3		m_contactNormal;

	D_btVector3		m_relpos2CrossNormal;
	//D_btVector3		m_contactNormal2;//usually m_contactNormal2 == -m_contactNormal

	D_btVector3		m_angularComponentA;
	D_btVector3		m_angularComponentB;
	
	mutable D_btSimdScalar	m_appliedPushImpulse;
	mutable D_btSimdScalar	m_appliedImpulse;
	
	
	D_btScalar	m_friction;
	D_btScalar	m_jacDiagABInv;
	union
	{
		int	m_numConsecutiveRowsPerKernel;
		D_btScalar	m_unusedPadding0;
	};

	union
	{
		int			m_frictionIndex;
		D_btScalar	m_unusedPadding1;
	};
	union
	{
		int			m_solverBodyIdA;
		D_btScalar	m_unusedPadding2;
	};
	union
	{
		int			m_solverBodyIdB;
		D_btScalar	m_unusedPadding3;
	};
	
	union
	{
		void*		m_originalContactPoint;
		D_btScalar	m_unusedPadding4;
	};

	D_btScalar		m_rhs;
	D_btScalar		m_cfm;
	D_btScalar		m_lowerLimit;
	D_btScalar		m_upperLimit;

	D_btScalar		m_rhsPenetration;

	enum		D_btSolverConstraintType
	{
		D_BT_SOLVER_CONTACT_1D = 0,
		D_BT_SOLVER_FRICTION_1D
	};
};

typedef D_btAlignedObjectArray<D_btSolverConstraint>	D_btConstraintArray;


#endif //D_BT_SOLVER_CONSTRAINT_H



