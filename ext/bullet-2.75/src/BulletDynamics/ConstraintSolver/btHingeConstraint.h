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

/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */

#ifndef HINGECONSTRAINT_H
#define HINGECONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class D_btRigidBody;

/// hinge constraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/// axis defines the orientation of the hinge axis
D_ATTRIBUTE_ALIGNED16(class) D_btHingeConstraint : public D_btTypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
	D_btJacobianEntry	m_jac[3]; //3 orthogonal linear constraints
	D_btJacobianEntry	m_jacAng[3]; //2 orthogonal angular constraints+ 1 for limit/motor

	D_btTransform m_rbAFrame; // constraint axii. Assumes z D_is hinge axis.
	D_btTransform m_rbBFrame;

	D_btScalar	m_motorTargetVelocity;
	D_btScalar	m_maxMotorImpulse;

	D_btScalar	m_limitSoftness; 
	D_btScalar	m_biasFactor; 
	D_btScalar    m_relaxationFactor; 

	D_btScalar    m_lowerLimit;	
	D_btScalar    m_upperLimit;	
	
	D_btScalar	m_kHinge;

	D_btScalar	m_limitSign;
	D_btScalar	m_correction;

	D_btScalar	m_accLimitImpulse;
	D_btScalar	m_hingeAngle;
	D_btScalar    m_referenceSign;

	bool		m_angularOnly;
	bool		m_enableAngularMotor;
	bool		m_solveLimit;
	bool		m_useSolveConstraintObsolete;
	bool		m_useReferenceFrameA;

	D_btScalar	m_accMotorImpulse;

	
public:

	D_btHingeConstraint(D_btRigidBody& rbA,D_btRigidBody& rbB, const D_btVector3& pivotInA,const D_btVector3& pivotInB, D_btVector3& axisInA,D_btVector3& axisInB, bool useReferenceFrameA = false);

	D_btHingeConstraint(D_btRigidBody& rbA,const D_btVector3& pivotInA,D_btVector3& axisInA, bool useReferenceFrameA = false);
	
	D_btHingeConstraint(D_btRigidBody& rbA,D_btRigidBody& rbB, const D_btTransform& rbAFrame, const D_btTransform& rbBFrame, bool useReferenceFrameA = false);

	D_btHingeConstraint(D_btRigidBody& rbA,const D_btTransform& rbAFrame, bool useReferenceFrameA = false);

	D_btHingeConstraint();

	virtual void	buildJacobian();

	virtual void getInfo1 (D_btConstraintInfo1* info);

	void getInfo1NonVirtual(D_btConstraintInfo1* info);

	virtual void getInfo2 (D_btConstraintInfo2* info);

	void	getInfo2NonVirtual(D_btConstraintInfo2* info,const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& angVelA,const D_btVector3& angVelB);

	void	getInfo2Internal(D_btConstraintInfo2* info,const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& angVelA,const D_btVector3& angVelB);
		
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

	D_btRigidBody& getRigidBodyA()	
	{		
		return m_rbA;	
	}	

	D_btRigidBody& getRigidBodyB()	
	{		
		return m_rbB;	
	}	
	
	void	setAngularOnly(bool angularOnly)
	{
		m_angularOnly = angularOnly;
	}

	void	enableAngularMotor(bool enableMotor,D_btScalar targetVelocity,D_btScalar maxMotorImpulse)
	{
		m_enableAngularMotor  = enableMotor;
		m_motorTargetVelocity = targetVelocity;
		m_maxMotorImpulse = maxMotorImpulse;
	}

	// extra motor API, including ability D_to set a target rotation (as opposed D_to angular velocity)
	// note: setMotorTarget sets angular velocity under the hood, so you D_must call it every tick D_to
	//       maintain a given angular target.
	void enableMotor(bool enableMotor) 	{ m_enableAngularMotor = enableMotor; }
	void setMaxMotorImpulse(D_btScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; }
	void setMotorTarget(const D_btQuaternion& qAinB, D_btScalar dt); // qAinB D_is rotation of body A wrt body B.
	void setMotorTarget(D_btScalar targetAngle, D_btScalar dt);


	void	setLimit(D_btScalar low,D_btScalar high,D_btScalar _softness = 0.9f, D_btScalar _biasFactor = 0.3f, D_btScalar _relaxationFactor = 1.0f)
	{
		m_lowerLimit = D_btNormalizeAngle(low);
		m_upperLimit = D_btNormalizeAngle(high);

		m_limitSoftness =  _softness;
		m_biasFactor = _biasFactor;
		m_relaxationFactor = _relaxationFactor;

	}

	void	setAxis(D_btVector3& axisInA)
	{
		D_btVector3 rbAxisA1, rbAxisA2;
		D_btPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);
		D_btVector3 pivotInA = m_rbAFrame.getOrigin();
//		m_rbAFrame.getOrigin() = pivotInA;
		m_rbAFrame.getBasis().setValue( rbAxisA1.getX(),rbAxisA2.getX(),axisInA.getX(),
										rbAxisA1.getY(),rbAxisA2.getY(),axisInA.getY(),
										rbAxisA1.getZ(),rbAxisA2.getZ(),axisInA.getZ() );

		D_btVector3 axisInB = m_rbA.getCenterOfMassTransform().getBasis() * axisInA;

		D_btQuaternion rotationArc = shortestArcQuat(axisInA,axisInB);
		D_btVector3 rbAxisB1 =  quatRotate(rotationArc,rbAxisA1);
		D_btVector3 rbAxisB2 = axisInB.cross(rbAxisB1);


		m_rbBFrame.getOrigin() = m_rbA.getCenterOfMassTransform()(pivotInA);
		m_rbBFrame.getBasis().setValue( rbAxisB1.getX(),rbAxisB2.getX(),axisInB.getX(),
										rbAxisB1.getY(),rbAxisB2.getY(),axisInB.getY(),
										rbAxisB1.getZ(),rbAxisB2.getZ(),axisInB.getZ() );
	}

	D_btScalar	getLowerLimit() const
	{
		return m_lowerLimit;
	}

	D_btScalar	getUpperLimit() const
	{
		return m_upperLimit;
	}


	D_btScalar getHingeAngle();

	D_btScalar getHingeAngle(const D_btTransform& transA,const D_btTransform& transB);

	void testLimit(const D_btTransform& transA,const D_btTransform& transB);


	const D_btTransform& getAFrame() const { return m_rbAFrame; };	
	const D_btTransform& getBFrame() const { return m_rbBFrame; };

	D_btTransform& getAFrame() { return m_rbAFrame; };	
	D_btTransform& getBFrame() { return m_rbBFrame; };

	inline int getSolveLimit()
	{
		return m_solveLimit;
	}

	inline D_btScalar getLimitSign()
	{
		return m_limitSign;
	}

	inline bool getAngularOnly() 
	{ 
		return m_angularOnly; 
	}
	inline bool getEnableAngularMotor() 
	{ 
		return m_enableAngularMotor; 
	}
	inline D_btScalar getMotorTargetVelosity() 
	{ 
		return m_motorTargetVelocity; 
	}
	inline D_btScalar getMaxMotorImpulse() 
	{ 
		return m_maxMotorImpulse; 
	}

};

#endif //HINGECONSTRAINT_H
