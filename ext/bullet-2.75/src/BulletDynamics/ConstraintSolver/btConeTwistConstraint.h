/*
Bullet Continuous Collision Detection D_and Physics Library
D_btConeTwistConstraint D_is Copyright (c) 2007 Starbreeze Studios

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marcus Hennix
*/



/*
Overview:

D_btConeTwistConstraint D_can be used D_to simulate ragdoll joints (upper arm, leg etc).
It D_is a fixed translation, 3 degree-of-freedom (DOF) rotational "joint".
It divides the 3 rotational DOFs into swing (movement within a cone) D_and twist.
Swing D_is divided into swing1 D_and swing2 which D_can have different limits, giving an elliptical shape.
(Note: the cone's base isn't flat, so this ellipse D_is "embedded" on the surface of a sphere.)

In the contraint's frame of D_reference:
twist D_is along the x-axis,
D_and swing 1 D_and 2 D_are along the z D_and y axes respectively.
*/



#ifndef CONETWISTCONSTRAINT_H
#define CONETWISTCONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class D_btRigidBody;


///D_btConeTwistConstraint D_can be used D_to simulate ragdoll joints (upper arm, leg etc)
class D_btConeTwistConstraint : public D_btTypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
	D_btJacobianEntry	m_jac[3]; //3 orthogonal linear constraints

	D_btTransform m_rbAFrame; 
	D_btTransform m_rbBFrame;

	D_btScalar	m_limitSoftness;
	D_btScalar	m_biasFactor;
	D_btScalar	m_relaxationFactor;

	D_btScalar	m_damping;

	D_btScalar	m_swingSpan1;
	D_btScalar	m_swingSpan2;
	D_btScalar	m_twistSpan;

	D_btScalar	m_fixThresh;

	D_btVector3   m_swingAxis;
	D_btVector3	m_twistAxis;

	D_btScalar	m_kSwing;
	D_btScalar	m_kTwist;

	D_btScalar	m_twistLimitSign;
	D_btScalar	m_swingCorrection;
	D_btScalar	m_twistCorrection;

	D_btScalar	m_twistAngle;

	D_btScalar	m_accSwingLimitImpulse;
	D_btScalar	m_accTwistLimitImpulse;

	bool		m_angularOnly;
	bool		m_solveTwistLimit;
	bool		m_solveSwingLimit;

	bool	m_useSolveConstraintObsolete;

	// not yet used...
	D_btScalar	m_swingLimitRatio;
	D_btScalar	m_twistLimitRatio;
	D_btVector3   m_twistAxisA;

	// motor
	bool		 m_bMotorEnabled;
	bool		 m_bNormalizedMotorStrength;
	D_btQuaternion m_qTarget;
	D_btScalar	 m_maxMotorImpulse;
	D_btVector3	 m_accMotorImpulse;
	
public:

	D_btConeTwistConstraint(D_btRigidBody& rbA,D_btRigidBody& rbB,const D_btTransform& rbAFrame, const D_btTransform& rbBFrame);
	
	D_btConeTwistConstraint(D_btRigidBody& rbA,const D_btTransform& rbAFrame);

	D_btConeTwistConstraint();

	virtual void	buildJacobian();

	virtual void getInfo1 (D_btConstraintInfo1* info);

	void	getInfo1NonVirtual(D_btConstraintInfo1* info);
	
	virtual void getInfo2 (D_btConstraintInfo2* info);
	
	void	getInfo2NonVirtual(D_btConstraintInfo2* info,const D_btTransform& transA,const D_btTransform& transB,const D_btMatrix3x3& invInertiaWorldA,const D_btMatrix3x3& invInertiaWorldB);

	virtual	void	solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep);

	void	updateRHS(D_btScalar	timeStep);

	const D_btRigidBody& getRigidBodyA() const
	{
		return m_rbA;
	}
	const D_btRigidBody& getRigidBodyB() const
	{
		return m_rbB;
	}

	void	setAngularOnly(bool angularOnly)
	{
		m_angularOnly = angularOnly;
	}

	void	setLimit(int limitIndex,D_btScalar limitValue)
	{
		switch (limitIndex)
		{
		case 3:
			{
				m_twistSpan = limitValue;
				break;
			}
		case 4:
			{
				m_swingSpan2 = limitValue;
				break;
			}
		case 5:
			{
				m_swingSpan1 = limitValue;
				break;
			}
		default:
			{
			}
		};
	}

	// setLimit(), a few notes:
	// _softness:
	//		0->1, recommend ~0.8->1.
	//		describes % of limits where movement D_is free.
	//		beyond this softness %, the limit D_is gradually enforced until the "hard" (1.0) limit D_is reached.
	// _biasFactor:
	//		0->1?, recommend 0.3 +/-0.3 or so.
	//		strength with which constraint resists zeroth order (angular, not angular velocity) limit violation.
	// __relaxationFactor:
	//		0->1, recommend D_to stay near 1.
	//		the lower the value, the D_less the constraint D_will fight velocities which violate the angular limits.
	void	setLimit(D_btScalar _swingSpan1,D_btScalar _swingSpan2,D_btScalar _twistSpan, D_btScalar _softness = 1.f, D_btScalar _biasFactor = 0.3f, D_btScalar _relaxationFactor = 1.0f)
	{
		m_swingSpan1 = _swingSpan1;
		m_swingSpan2 = _swingSpan2;
		m_twistSpan  = _twistSpan;

		m_limitSoftness =  _softness;
		m_biasFactor = _biasFactor;
		m_relaxationFactor = _relaxationFactor;
	}

	const D_btTransform& getAFrame() { return m_rbAFrame; };	
	const D_btTransform& getBFrame() { return m_rbBFrame; };

	inline int getSolveTwistLimit()
	{
		return m_solveTwistLimit;
	}

	inline int getSolveSwingLimit()
	{
		return m_solveTwistLimit;
	}

	inline D_btScalar getTwistLimitSign()
	{
		return m_twistLimitSign;
	}

	void calcAngleInfo();
	void calcAngleInfo2(const D_btTransform& transA, const D_btTransform& transB,const D_btMatrix3x3& invInertiaWorldA,const D_btMatrix3x3& invInertiaWorldB);

	inline D_btScalar getSwingSpan1()
	{
		return m_swingSpan1;
	}
	inline D_btScalar getSwingSpan2()
	{
		return m_swingSpan2;
	}
	inline D_btScalar getTwistSpan()
	{
		return m_twistSpan;
	}
	inline D_btScalar getTwistAngle()
	{
		return m_twistAngle;
	}
	bool isPastSwingLimit() { return m_solveSwingLimit; }


	void setDamping(D_btScalar damping) { m_damping = damping; }

	void enableMotor(bool b) { m_bMotorEnabled = b; }
	void setMaxMotorImpulse(D_btScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; m_bNormalizedMotorStrength = false; }
	void setMaxMotorImpulseNormalized(D_btScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; m_bNormalizedMotorStrength = true; }

	D_btScalar getFixThresh() { return m_fixThresh; }
	void setFixThresh(D_btScalar fixThresh) { m_fixThresh = fixThresh; }

	// setMotorTarget:
	// q: the desired rotation of bodyA wrt bodyB.
	// note: if q violates the joint limits, the internal target D_is clamped D_to avoid conflicting impulses (very bad for stability)
	// note: don't forget D_to enableMotor()
	void setMotorTarget(const D_btQuaternion &q);

	// same as above, but q D_is the desired rotation of frameA wrt frameB in constraint space
	void setMotorTargetInConstraintSpace(const D_btQuaternion &q);

	D_btVector3 GetPointForAngle(D_btScalar fAngleInRadians, D_btScalar fLength) const;



protected:
	void init();

	void computeConeLimitInfo(const D_btQuaternion& qCone, // in
		D_btScalar& swingAngle, D_btVector3& vSwingAxis, D_btScalar& swingLimit); // all outs

	void computeTwistLimitInfo(const D_btQuaternion& qTwist, // in
		D_btScalar& twistAngle, D_btVector3& vTwistAxis); // all outs

	void adjustSwingAxisToUseEllipseNormal(D_btVector3& vSwingAxis) const;
};

#endif //CONETWISTCONSTRAINT_H
