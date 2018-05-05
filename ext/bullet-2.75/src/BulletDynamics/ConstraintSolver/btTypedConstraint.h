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

#ifndef TYPED_CONSTRAINT_H
#define TYPED_CONSTRAINT_H

class D_btRigidBody;
#include "LinearMath/btScalar.h"
#include "btSolverConstraint.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
struct  D_btSolverBody;

enum D_btTypedConstraintType
{
	D_POINT2POINT_CONSTRAINT_TYPE=D_MAX_CONTACT_MANIFOLD_TYPE+1,
	D_HINGE_CONSTRAINT_TYPE,
	D_CONETWIST_CONSTRAINT_TYPE,
	D_D6_CONSTRAINT_TYPE,
	D_SLIDER_CONSTRAINT_TYPE,
	D_CONTACT_CONSTRAINT_TYPE
};

///TypedConstraint D_is the baseclass for Bullet constraints D_and vehicles
class D_btTypedConstraint : public D_btTypedObject
{
	int	m_userConstraintType;
	int	m_userConstraintId;
	bool m_needsFeedback;

	D_btTypedConstraint&	operator=(D_btTypedConstraint&	other)
	{
		D_btAssert(0);
		(void) other;
		return *this;
	}

protected:
	D_btRigidBody&	m_rbA;
	D_btRigidBody&	m_rbB;
	D_btScalar	m_appliedImpulse;
	D_btScalar	m_dbgDrawSize;

	D_btVector3	m_appliedLinearImpulse;
	D_btVector3	m_appliedAngularImpulseA;
	D_btVector3	m_appliedAngularImpulseB;

public:

	D_btTypedConstraint(D_btTypedConstraintType type);
	virtual ~D_btTypedConstraint() {};
	D_btTypedConstraint(D_btTypedConstraintType type, D_btRigidBody& rbA);
	D_btTypedConstraint(D_btTypedConstraintType type, D_btRigidBody& rbA,D_btRigidBody& rbB);

	struct D_btConstraintInfo1 {
		int m_numConstraintRows,nub;
	};

	struct D_btConstraintInfo2 {
		// integrator D_parameters: frames per second (1/stepsize), default error
		// reduction parameter (0..1).
		D_btScalar fps,erp;

		// for the first D_and second body, pointers D_to two (linear D_and angular)
		// n*3 jacobian sub matrices, stored by rows. these matrices D_will have
		// been initialized D_to 0 on entry. if the second body D_is zero then the
		// J2xx pointers may be 0.
		D_btScalar *m_J1linearAxis,*m_J1angularAxis,*m_J2linearAxis,*m_J2angularAxis;

		// elements D_to jump from one row D_to the next in J's
		int rowskip;

		// right hand sides of the equation J*v = c + cfm * lambda. cfm D_is the
		// "constraint force mixing" vector. c D_is set D_to zero on entry, cfm D_is
		// set D_to a constant value (typically very small or zero) value on entry.
		D_btScalar *m_constraintError,*cfm;

		// lo D_and hi limits for variables (set D_to -/+ infinity on entry).
		D_btScalar *m_lowerLimit,*m_upperLimit;

		// findex vector for variables. see the LCP solver interface for a
		// description of what this D_does. this D_is set D_to -1 on entry.
		// note that the returned indexes D_are relative D_to the first index of
		// the constraint.
		int *findex;
		// number of solver iterations
		int m_numIterations;
	};

	///internal method used by the constraint solver, don't use them directly
	virtual void	buildJacobian() = 0;

	///internal method used by the constraint solver, don't use them directly
	virtual	void	setupSolverConstraint(D_btConstraintArray& ca, int solverBodyA,int solverBodyB, D_btScalar timeStep)
	{
	}
	
	///internal method used by the constraint solver, don't use them directly
	virtual void getInfo1 (D_btConstraintInfo1* info)=0;

	///internal method used by the constraint solver, don't use them directly
	virtual void getInfo2 (D_btConstraintInfo2* info)=0;

	///internal method used by the constraint solver, don't use them directly
	void	internalSetAppliedImpulse(D_btScalar appliedImpulse)
	{
		m_appliedImpulse = appliedImpulse;
	}

	///internal method used by the constraint solver, don't use them directly
	virtual	void	solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep) = 0;

	///internal method used by the constraint solver, don't use them directly
	D_btScalar getMotorFactor(D_btScalar pos, D_btScalar lowLim, D_btScalar uppLim, D_btScalar vel, D_btScalar timeFact);
	
	const D_btRigidBody& getRigidBodyA() const
	{
		return m_rbA;
	}
	const D_btRigidBody& getRigidBodyB() const
	{
		return m_rbB;
	}

	D_btRigidBody& getRigidBodyA() 
	{
		return m_rbA;
	}
	D_btRigidBody& getRigidBodyB()
	{
		return m_rbB;
	}

	int getUserConstraintType() const
	{
		return m_userConstraintType ;
	}

	void	setUserConstraintType(int userConstraintType)
	{
		m_userConstraintType = userConstraintType;
	};

	void	setUserConstraintId(int uid)
	{
		m_userConstraintId = uid;
	}

	int getUserConstraintId() const
	{
		return m_userConstraintId;
	}

	int getUid() const
	{
		return m_userConstraintId;   
	} 

	bool	needsFeedback() const
	{
		return m_needsFeedback;
	}

	///enableFeedback D_will allow D_to read the applied linear D_and angular impulse
	///use getAppliedImpulse, getAppliedLinearImpulse D_and getAppliedAngularImpulse D_to read feedback information
	void	enableFeedback(bool needsFeedback)
	{
		m_needsFeedback = needsFeedback;
	}

	///getAppliedImpulse D_is an estimated total applied impulse. 
	///This feedback could be used D_to determine breaking constraints or playing sounds.
	D_btScalar	getAppliedImpulse() const
	{
		D_btAssert(m_needsFeedback);
		return m_appliedImpulse;
	}

	const D_btVector3& getAppliedLinearImpulse() const
	{
		D_btAssert(m_needsFeedback);
		return m_appliedLinearImpulse;
	}

	D_btVector3& getAppliedLinearImpulse()
	{
		D_btAssert(m_needsFeedback);
		return m_appliedLinearImpulse;
	}

	const D_btVector3& getAppliedAngularImpulseA() const
	{
		D_btAssert(m_needsFeedback);
		return m_appliedAngularImpulseA;
	}

	D_btVector3& getAppliedAngularImpulseA()
	{
		D_btAssert(m_needsFeedback);
		return m_appliedAngularImpulseA;
	}

	const D_btVector3& getAppliedAngularImpulseB() const
	{
		D_btAssert(m_needsFeedback);
		return m_appliedAngularImpulseB;
	}

	D_btVector3& getAppliedAngularImpulseB()
	{
		D_btAssert(m_needsFeedback);
		return m_appliedAngularImpulseB;
	}

	

	D_btTypedConstraintType getConstraintType () const
	{
		return D_btTypedConstraintType(m_objectType);
	}
	
	void setDbgDrawSize(D_btScalar dbgDrawSize)
	{
		m_dbgDrawSize = dbgDrawSize;
	}
	D_btScalar getDbgDrawSize()
	{
		return m_dbgDrawSize;
	}
	
};

// returns angle in range [-D_SIMD_2_PI, D_SIMD_2_PI], closest D_to one of the limits 
// all arguments D_should be normalized angles (i.e. in range [-D_SIMD_PI, D_SIMD_PI])
D_SIMD_FORCE_INLINE D_btScalar D_btAdjustAngleToLimits(D_btScalar angleInRadians, D_btScalar angleLowerLimitInRadians, D_btScalar angleUpperLimitInRadians)
{
	if(angleLowerLimitInRadians >= angleUpperLimitInRadians)
	{
		return angleInRadians;
	}
	else if(angleInRadians < angleLowerLimitInRadians)
	{
		D_btScalar diffLo = D_btNormalizeAngle(angleLowerLimitInRadians - angleInRadians); // this D_is positive
		D_btScalar diffHi = D_btFabs(D_btNormalizeAngle(angleUpperLimitInRadians - angleInRadians));
		return (diffLo < diffHi) ? angleInRadians : (angleInRadians + D_SIMD_2_PI);
	}
	else if(angleInRadians > angleUpperLimitInRadians)
	{
		D_btScalar diffHi = D_btNormalizeAngle(angleInRadians - angleUpperLimitInRadians); // this D_is positive
		D_btScalar diffLo = D_btFabs(D_btNormalizeAngle(angleInRadians - angleLowerLimitInRadians));
		return (diffLo < diffHi) ? (angleInRadians - D_SIMD_2_PI) : angleInRadians;
	}
	else
	{
		return angleInRadians;
	}
}


#endif //TYPED_CONSTRAINT_H
