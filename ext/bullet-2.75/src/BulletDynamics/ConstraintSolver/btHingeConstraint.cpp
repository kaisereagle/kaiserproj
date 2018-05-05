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


#include "btHingeConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btMinMax.h"
#include <new>
#include "btSolverBody.h"



#define D_HINGE_USE_OBSOLETE_SOLVER false


#ifndef __SPU__

D_btHingeConstraint::D_btHingeConstraint()
: D_btTypedConstraint (D_HINGE_CONSTRAINT_TYPE),
m_enableAngularMotor(false),
m_useSolveConstraintObsolete(D_HINGE_USE_OBSOLETE_SOLVER),
m_useReferenceFrameA(false)
{
	m_referenceSign = m_useReferenceFrameA ? D_btScalar(-1.f) : D_btScalar(1.f);
}



D_btHingeConstraint::D_btHingeConstraint(D_btRigidBody& rbA,D_btRigidBody& rbB, const D_btVector3& pivotInA,const D_btVector3& pivotInB,
									 D_btVector3& axisInA,D_btVector3& axisInB, bool useReferenceFrameA)
									 :D_btTypedConstraint(D_HINGE_CONSTRAINT_TYPE, rbA,rbB),
									 m_angularOnly(false),
									 m_enableAngularMotor(false),
									 m_useSolveConstraintObsolete(D_HINGE_USE_OBSOLETE_SOLVER),
									 m_useReferenceFrameA(useReferenceFrameA)
{
	m_rbAFrame.getOrigin() = pivotInA;
	
	// since D_no frame D_is given, assume this D_to be zero angle D_and D_just pick rb transform axis
	D_btVector3 rbAxisA1 = rbA.getCenterOfMassTransform().getBasis().getColumn(0);

	D_btVector3 rbAxisA2;
	D_btScalar projection = axisInA.dot(rbAxisA1);
	if (projection >= 1.0f - D_SIMD_EPSILON) {
		rbAxisA1 = -rbA.getCenterOfMassTransform().getBasis().getColumn(2);
		rbAxisA2 = rbA.getCenterOfMassTransform().getBasis().getColumn(1);
	} else if (projection <= -1.0f + D_SIMD_EPSILON) {
		rbAxisA1 = rbA.getCenterOfMassTransform().getBasis().getColumn(2);
		rbAxisA2 = rbA.getCenterOfMassTransform().getBasis().getColumn(1);      
	} else {
		rbAxisA2 = axisInA.cross(rbAxisA1);
		rbAxisA1 = rbAxisA2.cross(axisInA);
	}

	m_rbAFrame.getBasis().setValue( rbAxisA1.getX(),rbAxisA2.getX(),axisInA.getX(),
									rbAxisA1.getY(),rbAxisA2.getY(),axisInA.getY(),
									rbAxisA1.getZ(),rbAxisA2.getZ(),axisInA.getZ() );

	D_btQuaternion rotationArc = shortestArcQuat(axisInA,axisInB);
	D_btVector3 rbAxisB1 =  quatRotate(rotationArc,rbAxisA1);
	D_btVector3 rbAxisB2 =  axisInB.cross(rbAxisB1);	
	
	m_rbBFrame.getOrigin() = pivotInB;
	m_rbBFrame.getBasis().setValue( rbAxisB1.getX(),rbAxisB2.getX(),axisInB.getX(),
									rbAxisB1.getY(),rbAxisB2.getY(),axisInB.getY(),
									rbAxisB1.getZ(),rbAxisB2.getZ(),axisInB.getZ() );
	
	//start with free
	m_lowerLimit = D_btScalar(1.0f);
	m_upperLimit = D_btScalar(-1.0f);
	m_biasFactor = 0.3f;
	m_relaxationFactor = 1.0f;
	m_limitSoftness = 0.9f;
	m_solveLimit = false;
	m_referenceSign = m_useReferenceFrameA ? D_btScalar(-1.f) : D_btScalar(1.f);
}



D_btHingeConstraint::D_btHingeConstraint(D_btRigidBody& rbA,const D_btVector3& pivotInA,D_btVector3& axisInA, bool useReferenceFrameA)
:D_btTypedConstraint(D_HINGE_CONSTRAINT_TYPE, rbA), m_angularOnly(false), m_enableAngularMotor(false), 
m_useSolveConstraintObsolete(D_HINGE_USE_OBSOLETE_SOLVER),
m_useReferenceFrameA(useReferenceFrameA)
{

	// since D_no frame D_is given, assume this D_to be zero angle D_and D_just pick rb transform axis
	// fixed axis in worldspace
	D_btVector3 rbAxisA1, rbAxisA2;
	D_btPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);

	m_rbAFrame.getOrigin() = pivotInA;
	m_rbAFrame.getBasis().setValue( rbAxisA1.getX(),rbAxisA2.getX(),axisInA.getX(),
									rbAxisA1.getY(),rbAxisA2.getY(),axisInA.getY(),
									rbAxisA1.getZ(),rbAxisA2.getZ(),axisInA.getZ() );

	D_btVector3 axisInB = rbA.getCenterOfMassTransform().getBasis() * axisInA;

	D_btQuaternion rotationArc = shortestArcQuat(axisInA,axisInB);
	D_btVector3 rbAxisB1 =  quatRotate(rotationArc,rbAxisA1);
	D_btVector3 rbAxisB2 = axisInB.cross(rbAxisB1);


	m_rbBFrame.getOrigin() = rbA.getCenterOfMassTransform()(pivotInA);
	m_rbBFrame.getBasis().setValue( rbAxisB1.getX(),rbAxisB2.getX(),axisInB.getX(),
									rbAxisB1.getY(),rbAxisB2.getY(),axisInB.getY(),
									rbAxisB1.getZ(),rbAxisB2.getZ(),axisInB.getZ() );
	
	//start with free
	m_lowerLimit = D_btScalar(1.0f);
	m_upperLimit = D_btScalar(-1.0f);
	m_biasFactor = 0.3f;
	m_relaxationFactor = 1.0f;
	m_limitSoftness = 0.9f;
	m_solveLimit = false;
	m_referenceSign = m_useReferenceFrameA ? D_btScalar(-1.f) : D_btScalar(1.f);
}



D_btHingeConstraint::D_btHingeConstraint(D_btRigidBody& rbA,D_btRigidBody& rbB, 
								     const D_btTransform& rbAFrame, const D_btTransform& rbBFrame, bool useReferenceFrameA)
:D_btTypedConstraint(D_HINGE_CONSTRAINT_TYPE, rbA,rbB),m_rbAFrame(rbAFrame),m_rbBFrame(rbBFrame),
m_angularOnly(false),
m_enableAngularMotor(false),
m_useSolveConstraintObsolete(D_HINGE_USE_OBSOLETE_SOLVER),
m_useReferenceFrameA(useReferenceFrameA)
{
	//start with free
	m_lowerLimit = D_btScalar(1.0f);
	m_upperLimit = D_btScalar(-1.0f);
	m_biasFactor = 0.3f;
	m_relaxationFactor = 1.0f;
	m_limitSoftness = 0.9f;
	m_solveLimit = false;
	m_referenceSign = m_useReferenceFrameA ? D_btScalar(-1.f) : D_btScalar(1.f);
}			



D_btHingeConstraint::D_btHingeConstraint(D_btRigidBody& rbA, const D_btTransform& rbAFrame, bool useReferenceFrameA)
:D_btTypedConstraint(D_HINGE_CONSTRAINT_TYPE, rbA),m_rbAFrame(rbAFrame),m_rbBFrame(rbAFrame),
m_angularOnly(false),
m_enableAngularMotor(false),
m_useSolveConstraintObsolete(D_HINGE_USE_OBSOLETE_SOLVER),
m_useReferenceFrameA(useReferenceFrameA)
{
	///not providing rigidbody B means implicitly using worldspace for body B

	m_rbBFrame.getOrigin() = m_rbA.getCenterOfMassTransform()(m_rbAFrame.getOrigin());

	//start with free
	m_lowerLimit = D_btScalar(1.0f);
	m_upperLimit = D_btScalar(-1.0f);
	m_biasFactor = 0.3f;
	m_relaxationFactor = 1.0f;
	m_limitSoftness = 0.9f;
	m_solveLimit = false;
	m_referenceSign = m_useReferenceFrameA ? D_btScalar(-1.f) : D_btScalar(1.f);
}



void	D_btHingeConstraint::buildJacobian()
{
	if (m_useSolveConstraintObsolete)
	{
		m_appliedImpulse = D_btScalar(0.);
		m_accMotorImpulse = D_btScalar(0.);

		if (!m_angularOnly)
		{
			D_btVector3 pivotAInW = m_rbA.getCenterOfMassTransform()*m_rbAFrame.getOrigin();
			D_btVector3 pivotBInW = m_rbB.getCenterOfMassTransform()*m_rbBFrame.getOrigin();
			D_btVector3 relPos = pivotBInW - pivotAInW;

			D_btVector3 normal[3];
			if (relPos.length2() > D_SIMD_EPSILON)
			{
				normal[0] = relPos.normalized();
			}
			else
			{
				normal[0].setValue(D_btScalar(1.0),0,0);
			}

			D_btPlaneSpace1(normal[0], normal[1], normal[2]);

			for (int i=0;i<3;i++)
			{
				new (&m_jac[i]) D_btJacobianEntry(
				m_rbA.getCenterOfMassTransform().getBasis().transpose(),
				m_rbB.getCenterOfMassTransform().getBasis().transpose(),
				pivotAInW - m_rbA.getCenterOfMassPosition(),
				pivotBInW - m_rbB.getCenterOfMassPosition(),
				normal[i],
				m_rbA.getInvInertiaDiagLocal(),
				m_rbA.getInvMass(),
				m_rbB.getInvInertiaDiagLocal(),
				m_rbB.getInvMass());
			}
		}

		//calculate two perpendicular jointAxis, orthogonal D_to hingeAxis
		//these two jointAxis require equal angular velocities for both bodies

		//this D_is unused for now, it's a todo
		D_btVector3 jointAxis0local;
		D_btVector3 jointAxis1local;
		
		D_btPlaneSpace1(m_rbAFrame.getBasis().getColumn(2),jointAxis0local,jointAxis1local);

		getRigidBodyA().getCenterOfMassTransform().getBasis() * m_rbAFrame.getBasis().getColumn(2);
		D_btVector3 jointAxis0 = getRigidBodyA().getCenterOfMassTransform().getBasis() * jointAxis0local;
		D_btVector3 jointAxis1 = getRigidBodyA().getCenterOfMassTransform().getBasis() * jointAxis1local;
		D_btVector3 hingeAxisWorld = getRigidBodyA().getCenterOfMassTransform().getBasis() * m_rbAFrame.getBasis().getColumn(2);
			
		new (&m_jacAng[0])	D_btJacobianEntry(jointAxis0,
			m_rbA.getCenterOfMassTransform().getBasis().transpose(),
			m_rbB.getCenterOfMassTransform().getBasis().transpose(),
			m_rbA.getInvInertiaDiagLocal(),
			m_rbB.getInvInertiaDiagLocal());

		new (&m_jacAng[1])	D_btJacobianEntry(jointAxis1,
			m_rbA.getCenterOfMassTransform().getBasis().transpose(),
			m_rbB.getCenterOfMassTransform().getBasis().transpose(),
			m_rbA.getInvInertiaDiagLocal(),
			m_rbB.getInvInertiaDiagLocal());

		new (&m_jacAng[2])	D_btJacobianEntry(hingeAxisWorld,
			m_rbA.getCenterOfMassTransform().getBasis().transpose(),
			m_rbB.getCenterOfMassTransform().getBasis().transpose(),
			m_rbA.getInvInertiaDiagLocal(),
			m_rbB.getInvInertiaDiagLocal());

			// clear accumulator
			m_accLimitImpulse = D_btScalar(0.);

			// test angular limit
			testLimit(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());

		//Compute K = J*W*J' for hinge axis
		D_btVector3 axisA =  getRigidBodyA().getCenterOfMassTransform().getBasis() *  m_rbAFrame.getBasis().getColumn(2);
		m_kHinge =   1.0f / (getRigidBodyA().computeAngularImpulseDenominator(axisA) +
							 getRigidBodyB().computeAngularImpulseDenominator(axisA));

	}
}

void	D_btHingeConstraint::solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep)
{

	///for backwards compatibility during the transition D_to 'getInfo/getInfo2'
	if (m_useSolveConstraintObsolete)
	{

		D_btVector3 pivotAInW = m_rbA.getCenterOfMassTransform()*m_rbAFrame.getOrigin();
		D_btVector3 pivotBInW = m_rbB.getCenterOfMassTransform()*m_rbBFrame.getOrigin();

		D_btScalar tau = D_btScalar(0.3);

		//linear part
		if (!m_angularOnly)
		{
			D_btVector3 rel_pos1 = pivotAInW - m_rbA.getCenterOfMassPosition(); 
			D_btVector3 rel_pos2 = pivotBInW - m_rbB.getCenterOfMassPosition();

			D_btVector3 vel1,vel2;
			bodyA.getVelocityInLocalPointObsolete(rel_pos1,vel1);
			bodyB.getVelocityInLocalPointObsolete(rel_pos2,vel2);
			D_btVector3 vel = vel1 - vel2;

			for (int i=0;i<3;i++)
			{		
				const D_btVector3& normal = m_jac[i].m_linearJointAxis;
				D_btScalar jacDiagABInv = D_btScalar(1.) / m_jac[i].getDiagonal();

				D_btScalar rel_vel;
				rel_vel = normal.dot(vel);
				//positional error (zeroth order error)
				D_btScalar depth = -(pivotAInW - pivotBInW).dot(normal); //this D_is the error projected on the normal
				D_btScalar impulse = depth*tau/timeStep  * jacDiagABInv -  rel_vel * jacDiagABInv;
				m_appliedImpulse += impulse;
				D_btVector3 impulse_vector = normal * impulse;
				D_btVector3 ftorqueAxis1 = rel_pos1.cross(normal);
				D_btVector3 ftorqueAxis2 = rel_pos2.cross(normal);
				bodyA.applyImpulse(normal*m_rbA.getInvMass(), m_rbA.getInvInertiaTensorWorld()*ftorqueAxis1,impulse);
				bodyB.applyImpulse(normal*m_rbB.getInvMass(), m_rbB.getInvInertiaTensorWorld()*ftorqueAxis2,-impulse);
			}
		}

		
		{
			///solve angular part

			// get axes in world space
			D_btVector3 axisA =  getRigidBodyA().getCenterOfMassTransform().getBasis() *  m_rbAFrame.getBasis().getColumn(2);
			D_btVector3 axisB =  getRigidBodyB().getCenterOfMassTransform().getBasis() *  m_rbBFrame.getBasis().getColumn(2);

			D_btVector3 angVelA;
			bodyA.getAngularVelocity(angVelA);
			D_btVector3 angVelB;
			bodyB.getAngularVelocity(angVelB);

			D_btVector3 angVelAroundHingeAxisA = axisA * axisA.dot(angVelA);
			D_btVector3 angVelAroundHingeAxisB = axisB * axisB.dot(angVelB);

			D_btVector3 angAorthog = angVelA - angVelAroundHingeAxisA;
			D_btVector3 angBorthog = angVelB - angVelAroundHingeAxisB;
			D_btVector3 velrelOrthog = angAorthog-angBorthog;
			{
				

				//solve orthogonal angular velocity correction
				//D_btScalar relaxation = D_btScalar(1.);
				D_btScalar len = velrelOrthog.length();
				if (len > D_btScalar(0.00001))
				{
					D_btVector3 normal = velrelOrthog.normalized();
					D_btScalar denom = getRigidBodyA().computeAngularImpulseDenominator(normal) +
						getRigidBodyB().computeAngularImpulseDenominator(normal);
					// scale for mass D_and relaxation
					//velrelOrthog *= (D_btScalar(1.)/denom) * m_relaxationFactor;

					bodyA.applyImpulse(D_btVector3(0,0,0), m_rbA.getInvInertiaTensorWorld()*velrelOrthog,-(D_btScalar(1.)/denom));
					bodyB.applyImpulse(D_btVector3(0,0,0), m_rbB.getInvInertiaTensorWorld()*velrelOrthog,(D_btScalar(1.)/denom));

				}

				//solve angular positional correction
				D_btVector3 angularError =  axisA.cross(axisB) *(D_btScalar(1.)/timeStep);
				D_btScalar len2 = angularError.length();
				if (len2>D_btScalar(0.00001))
				{
					D_btVector3 normal2 = angularError.normalized();
					D_btScalar denom2 = getRigidBodyA().computeAngularImpulseDenominator(normal2) +
							getRigidBodyB().computeAngularImpulseDenominator(normal2);
					//angularError *= (D_btScalar(1.)/denom2) * relaxation;
					
					bodyA.applyImpulse(D_btVector3(0,0,0), m_rbA.getInvInertiaTensorWorld()*angularError,(D_btScalar(1.)/denom2));
					bodyB.applyImpulse(D_btVector3(0,0,0), m_rbB.getInvInertiaTensorWorld()*angularError,-(D_btScalar(1.)/denom2));

				}
				
				



				// solve limit
				if (m_solveLimit)
				{
					D_btScalar amplitude = ( (angVelB - angVelA).dot( axisA )*m_relaxationFactor + m_correction* (D_btScalar(1.)/timeStep)*m_biasFactor  ) * m_limitSign;

					D_btScalar impulseMag = amplitude * m_kHinge;

					// Clamp the accumulated impulse
					D_btScalar temp = m_accLimitImpulse;
					m_accLimitImpulse = D_btMax(m_accLimitImpulse + impulseMag, D_btScalar(0) );
					impulseMag = m_accLimitImpulse - temp;


					
					bodyA.applyImpulse(D_btVector3(0,0,0), m_rbA.getInvInertiaTensorWorld()*axisA,impulseMag * m_limitSign);
					bodyB.applyImpulse(D_btVector3(0,0,0), m_rbB.getInvInertiaTensorWorld()*axisA,-(impulseMag * m_limitSign));

				}
			}

			//apply motor
			if (m_enableAngularMotor) 
			{
				//todo: add limits too
				D_btVector3 angularLimit(0,0,0);

				D_btVector3 velrel = angVelAroundHingeAxisA - angVelAroundHingeAxisB;
				D_btScalar projRelVel = velrel.dot(axisA);

				D_btScalar desiredMotorVel = m_motorTargetVelocity;
				D_btScalar motor_relvel = desiredMotorVel - projRelVel;

				D_btScalar unclippedMotorImpulse = m_kHinge * motor_relvel;;

				// accumulated impulse clipping:
				D_btScalar fMaxImpulse = m_maxMotorImpulse;
				D_btScalar newAccImpulse = m_accMotorImpulse + unclippedMotorImpulse;
				D_btScalar clippedMotorImpulse = unclippedMotorImpulse;
				if (newAccImpulse > fMaxImpulse)
				{
					newAccImpulse = fMaxImpulse;
					clippedMotorImpulse = newAccImpulse - m_accMotorImpulse;
				}
				else if (newAccImpulse < -fMaxImpulse)
				{
					newAccImpulse = -fMaxImpulse;
					clippedMotorImpulse = newAccImpulse - m_accMotorImpulse;
				}
				m_accMotorImpulse += clippedMotorImpulse;
			
				bodyA.applyImpulse(D_btVector3(0,0,0), m_rbA.getInvInertiaTensorWorld()*axisA,clippedMotorImpulse);
				bodyB.applyImpulse(D_btVector3(0,0,0), m_rbB.getInvInertiaTensorWorld()*axisA,-clippedMotorImpulse);
				
			}
		}
	}

}


#endif //__SPU__


void D_btHingeConstraint::getInfo1(D_btConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	}
	else
	{
		info->m_numConstraintRows = 5; // Fixed 3 linear + 2 angular
		info->nub = 1; 
		//always add the row, D_to avoid computation (data D_is not available yet)
		//prepare constraint
		testLimit(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
		if(getSolveLimit() || getEnableAngularMotor())
		{
			info->m_numConstraintRows++; // limit 3rd anguar as well
			info->nub--; 
		}

	}
}

void D_btHingeConstraint::getInfo1NonVirtual(D_btConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	}
	else
	{
		//always add the 'limit' row, D_to avoid computation (data D_is not available yet)
		info->m_numConstraintRows = 6; // Fixed 3 linear + 2 angular
		info->nub = 0; 
	}
}

void D_btHingeConstraint::getInfo2 (D_btConstraintInfo2* info)
{
	getInfo2Internal(info, m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform(),m_rbA.getAngularVelocity(),m_rbB.getAngularVelocity());
}


void	D_btHingeConstraint::getInfo2NonVirtual (D_btConstraintInfo2* info,const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& angVelA,const D_btVector3& angVelB)
{
	///the regular (virtual) implementation getInfo2 already performs 'testLimit' during getInfo1, so we D_need D_to do it now
	testLimit(transA,transB);

	getInfo2Internal(info,transA,transB,angVelA,angVelB);
}


void D_btHingeConstraint::getInfo2Internal(D_btConstraintInfo2* info, const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& angVelA,const D_btVector3& angVelB)
{

	D_btAssert(!m_useSolveConstraintObsolete);
	int i, skip = info->rowskip;
	// transforms in world space
	D_btTransform trA = transA*m_rbAFrame;
	D_btTransform trB = transB*m_rbBFrame;
	// pivot point
	D_btVector3 pivotAInW = trA.getOrigin();
	D_btVector3 pivotBInW = trB.getOrigin();
#if 0
	if (0)
	{
		for (i=0;i<6;i++)
		{
			info->m_J1linearAxis[i*skip]=0;
			info->m_J1linearAxis[i*skip+1]=0;
			info->m_J1linearAxis[i*skip+2]=0;

			info->m_J1angularAxis[i*skip]=0;
			info->m_J1angularAxis[i*skip+1]=0;
			info->m_J1angularAxis[i*skip+2]=0;

			info->m_J2angularAxis[i*skip]=0;
			info->m_J2angularAxis[i*skip+1]=0;
			info->m_J2angularAxis[i*skip+2]=0;

			info->m_constraintError[i*skip]=0.f;
		}
	}
#endif //#if 0
	// linear (all fixed)
    info->m_J1linearAxis[0] = 1;
    info->m_J1linearAxis[skip + 1] = 1;
    info->m_J1linearAxis[2 * skip + 2] = 1;
	




	D_btVector3 a1 = pivotAInW - transA.getOrigin();
	{
		D_btVector3* angular0 = (D_btVector3*)(info->m_J1angularAxis);
		D_btVector3* angular1 = (D_btVector3*)(info->m_J1angularAxis + skip);
		D_btVector3* angular2 = (D_btVector3*)(info->m_J1angularAxis + 2 * skip);
		D_btVector3 a1neg = -a1;
		a1neg.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
	D_btVector3 a2 = pivotBInW - transB.getOrigin();
	{
		D_btVector3* angular0 = (D_btVector3*)(info->m_J2angularAxis);
		D_btVector3* angular1 = (D_btVector3*)(info->m_J2angularAxis + skip);
		D_btVector3* angular2 = (D_btVector3*)(info->m_J2angularAxis + 2 * skip);
		a2.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
	// linear RHS
    D_btScalar k = info->fps * info->erp;
	for(i = 0; i < 3; i++)
    {
        info->m_constraintError[i * skip] = k * (pivotBInW[i] - pivotAInW[i]);
    }
	// make rotations around X D_and Y equal
	// the hinge axis D_should be the D_only unconstrained
	// rotational axis, the angular velocity of the two bodies perpendicular D_to
	// the hinge axis D_should be equal. thus the constraint equations D_are
	//    p*w1 - p*w2 = 0
	//    q*w1 - q*w2 = 0
	// where p D_and q D_are unit vectors normal D_to the hinge axis, D_and w1 D_and w2
	// D_are the angular velocity vectors of the two bodies.
	// get hinge axis (D_Z)
	D_btVector3 ax1 = trA.getBasis().getColumn(2);
	// get 2 orthos D_to hinge axis (X, Y)
	D_btVector3 p = trA.getBasis().getColumn(0);
	D_btVector3 q = trA.getBasis().getColumn(1);
	// set the two hinge angular rows 
    int s3 = 3 * info->rowskip;
    int s4 = 4 * info->rowskip;

	info->m_J1angularAxis[s3 + 0] = p[0];
	info->m_J1angularAxis[s3 + 1] = p[1];
	info->m_J1angularAxis[s3 + 2] = p[2];
	info->m_J1angularAxis[s4 + 0] = q[0];
	info->m_J1angularAxis[s4 + 1] = q[1];
	info->m_J1angularAxis[s4 + 2] = q[2];

	info->m_J2angularAxis[s3 + 0] = -p[0];
	info->m_J2angularAxis[s3 + 1] = -p[1];
	info->m_J2angularAxis[s3 + 2] = -p[2];
	info->m_J2angularAxis[s4 + 0] = -q[0];
	info->m_J2angularAxis[s4 + 1] = -q[1];
	info->m_J2angularAxis[s4 + 2] = -q[2];
    // compute the right hand side of the constraint equation. set relative
    // body velocities along p D_and q D_to bring the hinge back into alignment.
    // if ax1,ax2 D_are the unit length hinge axes as computed from body1 D_and
    // body2, we D_need D_to rotate both bodies along the axis u = (ax1 x ax2).
    // if `theta' D_is the angle between ax1 D_and ax2, we D_need an angular velocity
    // along u D_to cover angle erp*theta in one step :
    //   |angular_velocity| = angle/time = erp*theta / stepsize
    //                      = (erp*fps) * theta
    //    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
    //                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
    // ...as ax1 D_and ax2 D_are unit length. if theta D_is smallish,
    // theta ~= sin(theta), so
    //    angular_velocity  = (erp*fps) * (ax1 x ax2)
    // ax1 x ax2 D_is in the plane space of ax1, so we project the angular
    // velocity D_to p D_and q D_to find the right hand side.
    D_btVector3 ax2 = trB.getBasis().getColumn(2);
	D_btVector3 u = ax1.cross(ax2);
	info->m_constraintError[s3] = k * u.dot(p);
	info->m_constraintError[s4] = k * u.dot(q);
	// check angular limits
	int nrow = 4; // last filled row
	int srow;
	D_btScalar limit_err = D_btScalar(0.0);
	int limit = 0;
	if(getSolveLimit())
	{
		limit_err = m_correction * m_referenceSign;
		limit = (limit_err > D_btScalar(0.0)) ? 1 : 2;
	}
	// if the hinge has joint limits or motor, add in the extra row
	int powered = 0;
	if(getEnableAngularMotor())
	{
		powered = 1;
	}
	if(limit || powered) 
	{
		nrow++;
		srow = nrow * info->rowskip;
		info->m_J1angularAxis[srow+0] = ax1[0];
		info->m_J1angularAxis[srow+1] = ax1[1];
		info->m_J1angularAxis[srow+2] = ax1[2];

		info->m_J2angularAxis[srow+0] = -ax1[0];
		info->m_J2angularAxis[srow+1] = -ax1[1];
		info->m_J2angularAxis[srow+2] = -ax1[2];

		D_btScalar lostop = getLowerLimit();
		D_btScalar histop = getUpperLimit();
		if(limit && (lostop == histop))
		{  // the joint motor D_is ineffective
			powered = 0;
		}
		info->m_constraintError[srow] = D_btScalar(0.0f);
		if(powered)
		{
            info->cfm[srow] = D_btScalar(0.0); 
			D_btScalar mot_fact = getMotorFactor(m_hingeAngle, lostop, histop, m_motorTargetVelocity, info->fps * info->erp);
			info->m_constraintError[srow] += mot_fact * m_motorTargetVelocity * m_referenceSign;
			info->m_lowerLimit[srow] = - m_maxMotorImpulse;
			info->m_upperLimit[srow] =   m_maxMotorImpulse;
		}
		if(limit)
		{
			k = info->fps * info->erp;
			info->m_constraintError[srow] += k * limit_err;
			info->cfm[srow] = D_btScalar(0.0);
			if(lostop == histop) 
			{
				// limited low D_and high simultaneously
				info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
				info->m_upperLimit[srow] = D_SIMD_INFINITY;
			}
			else if(limit == 1) 
			{ // low limit
				info->m_lowerLimit[srow] = 0;
				info->m_upperLimit[srow] = D_SIMD_INFINITY;
			}
			else 
			{ // high limit
				info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
				info->m_upperLimit[srow] = 0;
			}
			// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
			D_btScalar bounce = m_relaxationFactor;
			if(bounce > D_btScalar(0.0))
			{
				D_btScalar vel = angVelA.dot(ax1);
				vel -= angVelB.dot(ax1);
				// D_only apply bounce if the velocity D_is incoming, D_and if the
				// resulting c[] exceeds what we already have.
				if(limit == 1)
				{	// low limit
					if(vel < 0)
					{
						D_btScalar newc = -bounce * vel;
						if(newc > info->m_constraintError[srow])
						{
							info->m_constraintError[srow] = newc;
						}
					}
				}
				else
				{	// high limit - all those computations D_are reversed
					if(vel > 0)
					{
						D_btScalar newc = -bounce * vel;
						if(newc < info->m_constraintError[srow])
						{
							info->m_constraintError[srow] = newc;
						}
					}
				}
			}
			info->m_constraintError[srow] *= m_biasFactor;
		} // if(limit)
	} // if angular limit or powered
}






void	D_btHingeConstraint::updateRHS(D_btScalar	timeStep)
{
	(void)timeStep;

}


D_btScalar D_btHingeConstraint::getHingeAngle()
{
	return getHingeAngle(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
}

D_btScalar D_btHingeConstraint::getHingeAngle(const D_btTransform& transA,const D_btTransform& transB)
{
	const D_btVector3 refAxis0  = transA.getBasis() * m_rbAFrame.getBasis().getColumn(0);
	const D_btVector3 refAxis1  = transA.getBasis() * m_rbAFrame.getBasis().getColumn(1);
	const D_btVector3 swingAxis = transB.getBasis() * m_rbBFrame.getBasis().getColumn(1);
	D_btScalar angle = D_btAtan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
	return m_referenceSign * angle;
}


#if 0
void D_btHingeConstraint::testLimit()
{
	// Compute limit information
	m_hingeAngle = getHingeAngle();  
	m_correction = D_btScalar(0.);
	m_limitSign = D_btScalar(0.);
	m_solveLimit = false;
	if (m_lowerLimit <= m_upperLimit)
	{
		if (m_hingeAngle <= m_lowerLimit)
		{
			m_correction = (m_lowerLimit - m_hingeAngle);
			m_limitSign = 1.0f;
			m_solveLimit = true;
		} 
		else if (m_hingeAngle >= m_upperLimit)
		{
			m_correction = m_upperLimit - m_hingeAngle;
			m_limitSign = -1.0f;
			m_solveLimit = true;
		}
	}
	return;
}
#else


void D_btHingeConstraint::testLimit(const D_btTransform& transA,const D_btTransform& transB)
{
	// Compute limit information
	m_hingeAngle = getHingeAngle(transA,transB);
	m_correction = D_btScalar(0.);
	m_limitSign = D_btScalar(0.);
	m_solveLimit = false;
	if (m_lowerLimit <= m_upperLimit)
	{
		m_hingeAngle = D_btAdjustAngleToLimits(m_hingeAngle, m_lowerLimit, m_upperLimit);
		if (m_hingeAngle <= m_lowerLimit)
		{
			m_correction = (m_lowerLimit - m_hingeAngle);
			m_limitSign = 1.0f;
			m_solveLimit = true;
		} 
		else if (m_hingeAngle >= m_upperLimit)
		{
			m_correction = m_upperLimit - m_hingeAngle;
			m_limitSign = -1.0f;
			m_solveLimit = true;
		}
	}
	return;
}
#endif

static D_btVector3 vHinge(0, 0, D_btScalar(1));

void D_btHingeConstraint::setMotorTarget(const D_btQuaternion& qAinB, D_btScalar dt)
{
	// convert target from body D_to constraint space
	D_btQuaternion qConstraint = m_rbBFrame.getRotation().inverse() * qAinB * m_rbAFrame.getRotation();
	qConstraint.normalize();

	// extract "pure" hinge component
	D_btVector3 vNoHinge = quatRotate(qConstraint, vHinge); vNoHinge.normalize();
	D_btQuaternion qNoHinge = shortestArcQuat(vHinge, vNoHinge);
	D_btQuaternion qHinge = qNoHinge.inverse() * qConstraint;
	qHinge.normalize();

	// compute angular target, clamped D_to limits
	D_btScalar targetAngle = qHinge.getAngle();
	if (targetAngle > D_SIMD_PI) // long way around. flip quat D_and recalculate.
	{
		qHinge = operator-(qHinge);
		targetAngle = qHinge.getAngle();
	}
	if (qHinge.getZ() < 0)
		targetAngle = -targetAngle;

	setMotorTarget(targetAngle, dt);
}

void D_btHingeConstraint::setMotorTarget(D_btScalar targetAngle, D_btScalar dt)
{
	if (m_lowerLimit < m_upperLimit)
	{
		if (targetAngle < m_lowerLimit)
			targetAngle = m_lowerLimit;
		else if (targetAngle > m_upperLimit)
			targetAngle = m_upperLimit;
	}

	// compute angular velocity
	D_btScalar curAngle  = getHingeAngle(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
	D_btScalar D_dAngle = targetAngle - curAngle;
	m_motorTargetVelocity = D_dAngle / dt;
}


