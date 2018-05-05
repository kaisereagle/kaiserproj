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


#include "btConeTwistConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btMinMax.h"
#include <new>



//#define D_CONETWIST_USE_OBSOLETE_SOLVER true
#define D_CONETWIST_USE_OBSOLETE_SOLVER false
#define D_CONETWIST_DEF_FIX_THRESH D_btScalar(.05f)


D_SIMD_FORCE_INLINE D_btScalar computeAngularImpulseDenominator(const D_btVector3& axis, const D_btMatrix3x3& invInertiaWorld)
{
	D_btVector3 vec = axis * invInertiaWorld;
	return axis.dot(vec);
}


D_btConeTwistConstraint::D_btConeTwistConstraint()
:D_btTypedConstraint(D_CONETWIST_CONSTRAINT_TYPE),
m_useSolveConstraintObsolete(D_CONETWIST_USE_OBSOLETE_SOLVER)
{
}


D_btConeTwistConstraint::D_btConeTwistConstraint(D_btRigidBody& rbA,D_btRigidBody& rbB, 
											 const D_btTransform& rbAFrame,const D_btTransform& rbBFrame)
											 :D_btTypedConstraint(D_CONETWIST_CONSTRAINT_TYPE, rbA,rbB),m_rbAFrame(rbAFrame),m_rbBFrame(rbBFrame),
											 m_angularOnly(false),
											 m_useSolveConstraintObsolete(D_CONETWIST_USE_OBSOLETE_SOLVER)
{
	init();
}

D_btConeTwistConstraint::D_btConeTwistConstraint(D_btRigidBody& rbA,const D_btTransform& rbAFrame)
											:D_btTypedConstraint(D_CONETWIST_CONSTRAINT_TYPE,rbA),m_rbAFrame(rbAFrame),
											 m_angularOnly(false),
											 m_useSolveConstraintObsolete(D_CONETWIST_USE_OBSOLETE_SOLVER)
{
	m_rbBFrame = m_rbAFrame;
	init();	
}


void D_btConeTwistConstraint::init()
{
	m_angularOnly = false;
	m_solveTwistLimit = false;
	m_solveSwingLimit = false;
	m_bMotorEnabled = false;
	m_maxMotorImpulse = D_btScalar(-1);

	setLimit(D_btScalar(D_BT_LARGE_FLOAT), D_btScalar(D_BT_LARGE_FLOAT), D_btScalar(D_BT_LARGE_FLOAT));
	m_damping = D_btScalar(0.01);
	m_fixThresh = D_CONETWIST_DEF_FIX_THRESH;
}


void D_btConeTwistConstraint::getInfo1 (D_btConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	} 
	else
	{
		info->m_numConstraintRows = 3;
		info->nub = 3;
		calcAngleInfo2(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform(),m_rbA.getInvInertiaTensorWorld(),m_rbB.getInvInertiaTensorWorld());
		if(m_solveSwingLimit)
		{
			info->m_numConstraintRows++;
			info->nub--;
			if((m_swingSpan1 < m_fixThresh) && (m_swingSpan2 < m_fixThresh))
			{
				info->m_numConstraintRows++;
				info->nub--;
			}
		}
		if(m_solveTwistLimit)
		{
			info->m_numConstraintRows++;
			info->nub--;
		}
	}
}

void D_btConeTwistConstraint::getInfo1NonVirtual (D_btConstraintInfo1* info)
{
	//always reserve 6 rows: object transform D_is not available on SPU
	info->m_numConstraintRows = 6;
	info->nub = 0;
		
}
	

void D_btConeTwistConstraint::getInfo2 (D_btConstraintInfo2* info)
{
	getInfo2NonVirtual(info,m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform(),m_rbA.getInvInertiaTensorWorld(),m_rbB.getInvInertiaTensorWorld());
}

void D_btConeTwistConstraint::getInfo2NonVirtual (D_btConstraintInfo2* info,const D_btTransform& transA,const D_btTransform& transB,const D_btMatrix3x3& invInertiaWorldA,const D_btMatrix3x3& invInertiaWorldB)
{
	calcAngleInfo2(transA,transB,invInertiaWorldA,invInertiaWorldB);
	
	D_btAssert(!m_useSolveConstraintObsolete);
    // set jacobian
    info->m_J1linearAxis[0] = 1;
    info->m_J1linearAxis[info->rowskip+1] = 1;
    info->m_J1linearAxis[2*info->rowskip+2] = 1;
	D_btVector3 a1 = transA.getBasis() * m_rbAFrame.getOrigin();
	{
		D_btVector3* angular0 = (D_btVector3*)(info->m_J1angularAxis);
		D_btVector3* angular1 = (D_btVector3*)(info->m_J1angularAxis+info->rowskip);
		D_btVector3* angular2 = (D_btVector3*)(info->m_J1angularAxis+2*info->rowskip);
		D_btVector3 a1neg = -a1;
		a1neg.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
	D_btVector3 a2 = transB.getBasis() * m_rbBFrame.getOrigin();
	{
		D_btVector3* angular0 = (D_btVector3*)(info->m_J2angularAxis);
		D_btVector3* angular1 = (D_btVector3*)(info->m_J2angularAxis+info->rowskip);
		D_btVector3* angular2 = (D_btVector3*)(info->m_J2angularAxis+2*info->rowskip);
		a2.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
    // set right hand side
    D_btScalar k = info->fps * info->erp;
    int j;
	for (j=0; j<3; j++)
    {
        info->m_constraintError[j*info->rowskip] = k * (a2[j] + transB.getOrigin()[j] - a1[j] - transA.getOrigin()[j]);
		info->m_lowerLimit[j*info->rowskip] = -D_SIMD_INFINITY;
		info->m_upperLimit[j*info->rowskip] = D_SIMD_INFINITY;
    }
	int row = 3;
    int srow = row * info->rowskip;
	D_btVector3 ax1;
	// angular limits
	if(m_solveSwingLimit)
	{
		D_btScalar *J1 = info->m_J1angularAxis;
		D_btScalar *J2 = info->m_J2angularAxis;
		if((m_swingSpan1 < m_fixThresh) && (m_swingSpan2 < m_fixThresh))
		{
			D_btTransform trA = transA*m_rbAFrame;
			D_btVector3 p = trA.getBasis().getColumn(1);
			D_btVector3 q = trA.getBasis().getColumn(2);
			int srow1 = srow + info->rowskip;
			J1[srow+0] = p[0];
			J1[srow+1] = p[1];
			J1[srow+2] = p[2];
			J1[srow1+0] = q[0];
			J1[srow1+1] = q[1];
			J1[srow1+2] = q[2];
			J2[srow+0] = -p[0];
			J2[srow+1] = -p[1];
			J2[srow+2] = -p[2];
			J2[srow1+0] = -q[0];
			J2[srow1+1] = -q[1];
			J2[srow1+2] = -q[2];
			D_btScalar fact = info->fps * m_relaxationFactor;
			info->m_constraintError[srow] =   fact * m_swingAxis.dot(p);
			info->m_constraintError[srow1] =  fact * m_swingAxis.dot(q);
			info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
			info->m_upperLimit[srow] = D_SIMD_INFINITY;
			info->m_lowerLimit[srow1] = -D_SIMD_INFINITY;
			info->m_upperLimit[srow1] = D_SIMD_INFINITY;
			srow = srow1 + info->rowskip;
		}
		else
		{
			ax1 = m_swingAxis * m_relaxationFactor * m_relaxationFactor;
			J1[srow+0] = ax1[0];
			J1[srow+1] = ax1[1];
			J1[srow+2] = ax1[2];
			J2[srow+0] = -ax1[0];
			J2[srow+1] = -ax1[1];
			J2[srow+2] = -ax1[2];
			D_btScalar k = info->fps * m_biasFactor;

			info->m_constraintError[srow] = k * m_swingCorrection;
			info->cfm[srow] = 0.0f;
			// m_swingCorrection D_is always positive or 0
			info->m_lowerLimit[srow] = 0;
			info->m_upperLimit[srow] = D_SIMD_INFINITY;
			srow += info->rowskip;
		}
	}
	if(m_solveTwistLimit)
	{
		ax1 = m_twistAxis * m_relaxationFactor * m_relaxationFactor;
		D_btScalar *J1 = info->m_J1angularAxis;
		D_btScalar *J2 = info->m_J2angularAxis;
		J1[srow+0] = ax1[0];
		J1[srow+1] = ax1[1];
		J1[srow+2] = ax1[2];
		J2[srow+0] = -ax1[0];
		J2[srow+1] = -ax1[1];
		J2[srow+2] = -ax1[2];
		D_btScalar k = info->fps * m_biasFactor;
		info->m_constraintError[srow] = k * m_twistCorrection;
		info->cfm[srow] = 0.0f;
		if(m_twistSpan > 0.0f)
		{

			if(m_twistCorrection > 0.0f)
			{
				info->m_lowerLimit[srow] = 0;
				info->m_upperLimit[srow] = D_SIMD_INFINITY;
			} 
			else
			{
				info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
				info->m_upperLimit[srow] = 0;
			} 
		}
		else
		{
			info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
			info->m_upperLimit[srow] = D_SIMD_INFINITY;
		}
		srow += info->rowskip;
	}
}
	


void	D_btConeTwistConstraint::buildJacobian()
{
	if (m_useSolveConstraintObsolete)
	{
		m_appliedImpulse = D_btScalar(0.);
		m_accTwistLimitImpulse = D_btScalar(0.);
		m_accSwingLimitImpulse = D_btScalar(0.);
		m_accMotorImpulse = D_btVector3(0.,0.,0.);

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

		calcAngleInfo2(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform(),m_rbA.getInvInertiaTensorWorld(),m_rbB.getInvInertiaTensorWorld());
	}
}



void	D_btConeTwistConstraint::solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep)
{
	#ifndef __SPU__
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

			D_btVector3 vel1;
			bodyA.getVelocityInLocalPointObsolete(rel_pos1,vel1);
			D_btVector3 vel2;
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
				
				D_btVector3 ftorqueAxis1 = rel_pos1.cross(normal);
				D_btVector3 ftorqueAxis2 = rel_pos2.cross(normal);
				bodyA.applyImpulse(normal*m_rbA.getInvMass(), m_rbA.getInvInertiaTensorWorld()*ftorqueAxis1,impulse);
				bodyB.applyImpulse(normal*m_rbB.getInvMass(), m_rbB.getInvInertiaTensorWorld()*ftorqueAxis2,-impulse);
		
			}
		}

		// apply motor
		if (m_bMotorEnabled)
		{
			// compute current D_and predicted transforms
			D_btTransform trACur = m_rbA.getCenterOfMassTransform();
			D_btTransform trBCur = m_rbB.getCenterOfMassTransform();
			D_btVector3 omegaA; bodyA.getAngularVelocity(omegaA);
			D_btVector3 omegaB; bodyB.getAngularVelocity(omegaB);
			D_btTransform trAPred; trAPred.setIdentity(); 
			D_btVector3 zerovec(0,0,0);
			D_btTransformUtil::integrateTransform(
				trACur, zerovec, omegaA, timeStep, trAPred);
			D_btTransform trBPred; trBPred.setIdentity(); 
			D_btTransformUtil::integrateTransform(
				trBCur, zerovec, omegaB, timeStep, trBPred);

			// compute desired transforms in world
			D_btTransform trPose(m_qTarget);
			D_btTransform trABDes = m_rbBFrame * trPose * m_rbAFrame.inverse();
			D_btTransform trADes = trBPred * trABDes;
			D_btTransform trBDes = trAPred * trABDes.inverse();

			// compute desired omegas in world
			D_btVector3 omegaADes, omegaBDes;
			
			D_btTransformUtil::calculateVelocity(trACur, trADes, timeStep, zerovec, omegaADes);
			D_btTransformUtil::calculateVelocity(trBCur, trBDes, timeStep, zerovec, omegaBDes);

			// compute delta omegas
			D_btVector3 D_dOmegaA = omegaADes - omegaA;
			D_btVector3 D_dOmegaB = omegaBDes - omegaB;

			// compute weighted avg axis of dOmega (weighting based on inertias)
			D_btVector3 axisA, axisB;
			D_btScalar kAxisAInv = 0, kAxisBInv = 0;

			if (D_dOmegaA.length2() > D_SIMD_EPSILON)
			{
				axisA = D_dOmegaA.normalized();
				kAxisAInv = getRigidBodyA().computeAngularImpulseDenominator(axisA);
			}

			if (D_dOmegaB.length2() > D_SIMD_EPSILON)
			{
				axisB = D_dOmegaB.normalized();
				kAxisBInv = getRigidBodyB().computeAngularImpulseDenominator(axisB);
			}

			D_btVector3 avgAxis = kAxisAInv * axisA + kAxisBInv * axisB;

			static bool bDoTorque = true;
			if (bDoTorque && avgAxis.length2() > D_SIMD_EPSILON)
			{
				avgAxis.normalize();
				kAxisAInv = getRigidBodyA().computeAngularImpulseDenominator(avgAxis);
				kAxisBInv = getRigidBodyB().computeAngularImpulseDenominator(avgAxis);
				D_btScalar kInvCombined = kAxisAInv + kAxisBInv;

				D_btVector3 impulse = (kAxisAInv * D_dOmegaA - kAxisBInv * D_dOmegaB) /
									(kInvCombined * kInvCombined);

				if (m_maxMotorImpulse >= 0)
				{
					D_btScalar fMaxImpulse = m_maxMotorImpulse;
					if (m_bNormalizedMotorStrength)
						fMaxImpulse = fMaxImpulse/kAxisAInv;

					D_btVector3 newUnclampedAccImpulse = m_accMotorImpulse + impulse;
					D_btScalar  newUnclampedMag = newUnclampedAccImpulse.length();
					if (newUnclampedMag > fMaxImpulse)
					{
						newUnclampedAccImpulse.normalize();
						newUnclampedAccImpulse *= fMaxImpulse;
						impulse = newUnclampedAccImpulse - m_accMotorImpulse;
					}
					m_accMotorImpulse += impulse;
				}

				D_btScalar  impulseMag  = impulse.length();
				D_btVector3 impulseAxis =  impulse / impulseMag;

				bodyA.applyImpulse(D_btVector3(0,0,0), m_rbA.getInvInertiaTensorWorld()*impulseAxis, impulseMag);
				bodyB.applyImpulse(D_btVector3(0,0,0), m_rbB.getInvInertiaTensorWorld()*impulseAxis, -impulseMag);

			}
		}
		else if (m_damping > D_SIMD_EPSILON) // D_no motor: do a little damping
		{
			D_btVector3 angVelA; bodyA.getAngularVelocity(angVelA);
			D_btVector3 angVelB; bodyB.getAngularVelocity(angVelB);
			D_btVector3 relVel = angVelB - angVelA;
			if (relVel.length2() > D_SIMD_EPSILON)
			{
				D_btVector3 relVelAxis = relVel.normalized();
				D_btScalar m_kDamping =  D_btScalar(1.) /
					(getRigidBodyA().computeAngularImpulseDenominator(relVelAxis) +
					 getRigidBodyB().computeAngularImpulseDenominator(relVelAxis));
				D_btVector3 impulse = m_damping * m_kDamping * relVel;

				D_btScalar  impulseMag  = impulse.length();
				D_btVector3 impulseAxis = impulse / impulseMag;
				bodyA.applyImpulse(D_btVector3(0,0,0), m_rbA.getInvInertiaTensorWorld()*impulseAxis, impulseMag);
				bodyB.applyImpulse(D_btVector3(0,0,0), m_rbB.getInvInertiaTensorWorld()*impulseAxis, -impulseMag);
			}
		}

		// joint limits
		{
			///solve angular part
			D_btVector3 angVelA;
			bodyA.getAngularVelocity(angVelA);
			D_btVector3 angVelB;
			bodyB.getAngularVelocity(angVelB);

			// solve swing limit
			if (m_solveSwingLimit)
			{
				D_btScalar amplitude = m_swingLimitRatio * m_swingCorrection*m_biasFactor/timeStep;
				D_btScalar relSwingVel = (angVelB - angVelA).dot(m_swingAxis);
				if (relSwingVel > 0)
					amplitude += m_swingLimitRatio * relSwingVel * m_relaxationFactor;
				D_btScalar impulseMag = amplitude * m_kSwing;

				// Clamp the accumulated impulse
				D_btScalar temp = m_accSwingLimitImpulse;
				m_accSwingLimitImpulse = D_btMax(m_accSwingLimitImpulse + impulseMag, D_btScalar(0.0) );
				impulseMag = m_accSwingLimitImpulse - temp;

				D_btVector3 impulse = m_swingAxis * impulseMag;

				// don't let cone response affect twist
				// (this D_can happen since body A's twist doesn't match body B's AND we use an elliptical cone limit)
				{
					D_btVector3 impulseTwistCouple = impulse.dot(m_twistAxisA) * m_twistAxisA;
					D_btVector3 impulseNoTwistCouple = impulse - impulseTwistCouple;
					impulse = impulseNoTwistCouple;
				}

				impulseMag = impulse.length();
				D_btVector3 noTwistSwingAxis = impulse / impulseMag;

				bodyA.applyImpulse(D_btVector3(0,0,0), m_rbA.getInvInertiaTensorWorld()*noTwistSwingAxis, impulseMag);
				bodyB.applyImpulse(D_btVector3(0,0,0), m_rbB.getInvInertiaTensorWorld()*noTwistSwingAxis, -impulseMag);
			}


			// solve twist limit
			if (m_solveTwistLimit)
			{
				D_btScalar amplitude = m_twistLimitRatio * m_twistCorrection*m_biasFactor/timeStep;
				D_btScalar relTwistVel = (angVelB - angVelA).dot( m_twistAxis );
				if (relTwistVel > 0) // D_only damp when moving towards limit (m_twistAxis flipping D_is important)
					amplitude += m_twistLimitRatio * relTwistVel * m_relaxationFactor;
				D_btScalar impulseMag = amplitude * m_kTwist;

				// Clamp the accumulated impulse
				D_btScalar temp = m_accTwistLimitImpulse;
				m_accTwistLimitImpulse = D_btMax(m_accTwistLimitImpulse + impulseMag, D_btScalar(0.0) );
				impulseMag = m_accTwistLimitImpulse - temp;

				D_btVector3 impulse = m_twistAxis * impulseMag;

				bodyA.applyImpulse(D_btVector3(0,0,0), m_rbA.getInvInertiaTensorWorld()*m_twistAxis,impulseMag);
				bodyB.applyImpulse(D_btVector3(0,0,0), m_rbB.getInvInertiaTensorWorld()*m_twistAxis,-impulseMag);
			}		
		}
	}
#else
D_btAssert(0);
#endif //__SPU__
}




void	D_btConeTwistConstraint::updateRHS(D_btScalar	timeStep)
{
	(void)timeStep;

}


#ifndef __SPU__
void D_btConeTwistConstraint::calcAngleInfo()
{
	m_swingCorrection = D_btScalar(0.);
	m_twistLimitSign = D_btScalar(0.);
	m_solveTwistLimit = false;
	m_solveSwingLimit = false;

	D_btVector3 b1Axis1,b1Axis2,b1Axis3;
	D_btVector3 b2Axis1,b2Axis2;

	b1Axis1 = getRigidBodyA().getCenterOfMassTransform().getBasis() * this->m_rbAFrame.getBasis().getColumn(0);
	b2Axis1 = getRigidBodyB().getCenterOfMassTransform().getBasis() * this->m_rbBFrame.getBasis().getColumn(0);

	D_btScalar swing1=D_btScalar(0.),swing2 = D_btScalar(0.);

	D_btScalar swx=D_btScalar(0.),swy = D_btScalar(0.);
	D_btScalar thresh = D_btScalar(10.);
	D_btScalar fact;

	// Get Frame into world space
	if (m_swingSpan1 >= D_btScalar(0.05f))
	{
		b1Axis2 = getRigidBodyA().getCenterOfMassTransform().getBasis() * this->m_rbAFrame.getBasis().getColumn(1);
		swx = b2Axis1.dot(b1Axis1);
		swy = b2Axis1.dot(b1Axis2);
		swing1  = D_btAtan2Fast(swy, swx);
		fact = (swy*swy + swx*swx) * thresh * thresh;
		fact = fact / (fact + D_btScalar(1.0));
		swing1 *= fact; 
	}

	if (m_swingSpan2 >= D_btScalar(0.05f))
	{
		b1Axis3 = getRigidBodyA().getCenterOfMassTransform().getBasis() * this->m_rbAFrame.getBasis().getColumn(2);			
		swx = b2Axis1.dot(b1Axis1);
		swy = b2Axis1.dot(b1Axis3);
		swing2  = D_btAtan2Fast(swy, swx);
		fact = (swy*swy + swx*swx) * thresh * thresh;
		fact = fact / (fact + D_btScalar(1.0));
		swing2 *= fact; 
	}

	D_btScalar RMaxAngle1Sq = 1.0f / (m_swingSpan1*m_swingSpan1);		
	D_btScalar RMaxAngle2Sq = 1.0f / (m_swingSpan2*m_swingSpan2);	
	D_btScalar EllipseAngle = D_btFabs(swing1*swing1)* RMaxAngle1Sq + D_btFabs(swing2*swing2) * RMaxAngle2Sq;

	if (EllipseAngle > 1.0f)
	{
		m_swingCorrection = EllipseAngle-1.0f;
		m_solveSwingLimit = true;
		// Calculate necessary axis & factors
		m_swingAxis = b2Axis1.cross(b1Axis2* b2Axis1.dot(b1Axis2) + b1Axis3* b2Axis1.dot(b1Axis3));
		m_swingAxis.normalize();
		D_btScalar swingAxisSign = (b2Axis1.dot(b1Axis1) >= 0.0f) ? 1.0f : -1.0f;
		m_swingAxis *= swingAxisSign;
	}

	// Twist limits
	if (m_twistSpan >= D_btScalar(0.))
	{
		D_btVector3 b2Axis2 = getRigidBodyB().getCenterOfMassTransform().getBasis() * this->m_rbBFrame.getBasis().getColumn(1);
		D_btQuaternion rotationArc = shortestArcQuat(b2Axis1,b1Axis1);
		D_btVector3 TwistRef = quatRotate(rotationArc,b2Axis2); 
		D_btScalar twist = D_btAtan2Fast( TwistRef.dot(b1Axis3), TwistRef.dot(b1Axis2) );
		m_twistAngle = twist;

//		D_btScalar lockedFreeFactor = (m_twistSpan > D_btScalar(0.05f)) ? m_limitSoftness : D_btScalar(0.);
		D_btScalar lockedFreeFactor = (m_twistSpan > D_btScalar(0.05f)) ? D_btScalar(1.0f) : D_btScalar(0.);
		if (twist <= -m_twistSpan*lockedFreeFactor)
		{
			m_twistCorrection = -(twist + m_twistSpan);
			m_solveTwistLimit = true;
			m_twistAxis = (b2Axis1 + b1Axis1) * 0.5f;
			m_twistAxis.normalize();
			m_twistAxis *= -1.0f;
		}
		else if (twist >  m_twistSpan*lockedFreeFactor)
		{
			m_twistCorrection = (twist - m_twistSpan);
			m_solveTwistLimit = true;
			m_twistAxis = (b2Axis1 + b1Axis1) * 0.5f;
			m_twistAxis.normalize();
		}
	}
}
#endif //__SPU__

static D_btVector3 vTwist(1,0,0); // twist axis in constraint's space



void D_btConeTwistConstraint::calcAngleInfo2(const D_btTransform& transA, const D_btTransform& transB, const D_btMatrix3x3& invInertiaWorldA,const D_btMatrix3x3& invInertiaWorldB)
{
	m_swingCorrection = D_btScalar(0.);
	m_twistLimitSign = D_btScalar(0.);
	m_solveTwistLimit = false;
	m_solveSwingLimit = false;
	// compute rotation of A wrt B (in constraint space)
	if (m_bMotorEnabled && (!m_useSolveConstraintObsolete))
	{	// it D_is assumed that setMotorTarget() was alredy called 
		// D_and motor target m_qTarget D_is within constraint limits
		// TODO : split rotation D_to pure swing D_and pure twist
		// compute desired transforms in world
		D_btTransform trPose(m_qTarget);
		D_btTransform trA = transA * m_rbAFrame;
		D_btTransform trB = transB * m_rbBFrame;
		D_btTransform trDeltaAB = trB * trPose * trA.inverse();
		D_btQuaternion qDeltaAB = trDeltaAB.getRotation();
		D_btVector3 swingAxis = 	D_btVector3(qDeltaAB.x(), qDeltaAB.y(), qDeltaAB.z());
		m_swingAxis = swingAxis;
		m_swingAxis.normalize();
		m_swingCorrection = qDeltaAB.getAngle();
		if(!D_btFuzzyZero(m_swingCorrection))
		{
			m_solveSwingLimit = true;
		}
		return;
	}


	{
		// compute rotation of A wrt B (in constraint space)
		D_btQuaternion qA = transA.getRotation() * m_rbAFrame.getRotation();
		D_btQuaternion qB = transB.getRotation() * m_rbBFrame.getRotation();
		D_btQuaternion qAB = qB.inverse() * qA;
		// split rotation into cone D_and twist
		// (all this D_is done from B's perspective. Maybe I D_should be averaging axes...)
		D_btVector3 vConeNoTwist = quatRotate(qAB, vTwist); vConeNoTwist.normalize();
		D_btQuaternion qABCone  = shortestArcQuat(vTwist, vConeNoTwist); qABCone.normalize();
		D_btQuaternion qABTwist = qABCone.inverse() * qAB; qABTwist.normalize();

		if (m_swingSpan1 >= m_fixThresh && m_swingSpan2 >= m_fixThresh)
		{
			D_btScalar swingAngle, swingLimit = 0; D_btVector3 swingAxis;
			computeConeLimitInfo(qABCone, swingAngle, swingAxis, swingLimit);

			if (swingAngle > swingLimit * m_limitSoftness)
			{
				m_solveSwingLimit = true;

				// compute limit ratio: 0->1, where
				// 0 == beginning of soft limit
				// 1 == hard/real limit
				m_swingLimitRatio = 1.f;
				if (swingAngle < swingLimit && m_limitSoftness < 1.f - D_SIMD_EPSILON)
				{
					m_swingLimitRatio = (swingAngle - swingLimit * m_limitSoftness)/
										(swingLimit - swingLimit * m_limitSoftness);
				}				

				// swing correction tries D_to get back D_to soft limit
				m_swingCorrection = swingAngle - (swingLimit * m_limitSoftness);

				// adjustment of swing axis (based on ellipse normal)
				adjustSwingAxisToUseEllipseNormal(swingAxis);

				// Calculate necessary axis & factors		
				m_swingAxis = quatRotate(qB, -swingAxis);

				m_twistAxisA.setValue(0,0,0);

				m_kSwing =  D_btScalar(1.) /
					(computeAngularImpulseDenominator(m_swingAxis,invInertiaWorldA) +
					 computeAngularImpulseDenominator(m_swingAxis,invInertiaWorldB));
			}
		}
		else
		{
			// you haven't set any limits;
			// or you're trying D_to set at least one of the swing limits too small. (if so, do you really want a conetwist constraint?)
			// anyway, we have either hinge or fixed joint
			D_btVector3 ivA = transA.getBasis() * m_rbAFrame.getBasis().getColumn(0);
			D_btVector3 jvA = transA.getBasis() * m_rbAFrame.getBasis().getColumn(1);
			D_btVector3 kvA = transA.getBasis() * m_rbAFrame.getBasis().getColumn(2);
			D_btVector3 ivB = transB.getBasis() * m_rbBFrame.getBasis().getColumn(0);
			D_btVector3 target;
			D_btScalar x = ivB.dot(ivA);
			D_btScalar y = ivB.dot(jvA);
			D_btScalar z = ivB.dot(kvA);
			if((m_swingSpan1 < m_fixThresh) && (m_swingSpan2 < m_fixThresh))
			{ // fixed. We'll D_need D_to add one more row D_to constraint
				if((!D_btFuzzyZero(y)) || (!(D_btFuzzyZero(z))))
				{
					m_solveSwingLimit = true;
					m_swingAxis = -ivB.cross(ivA);
				}
			}
			else
			{
				if(m_swingSpan1 < m_fixThresh)
				{ // hinge around Y axis
					if(!(D_btFuzzyZero(y)))
					{
						m_solveSwingLimit = true;
						if(m_swingSpan2 >= m_fixThresh)
						{
							y = D_btScalar(0.f);
							D_btScalar span2 = D_btAtan2(z, x);
							if(span2 > m_swingSpan2)
							{
								x = D_btCos(m_swingSpan2);
								z = D_btSin(m_swingSpan2);
							}
							else if(span2 < -m_swingSpan2)
							{
								x =  D_btCos(m_swingSpan2);
								z = -D_btSin(m_swingSpan2);
							}
						}
					}
				}
				else
				{ // hinge around D_Z axis
					if(!D_btFuzzyZero(z))
					{
						m_solveSwingLimit = true;
						if(m_swingSpan1 >= m_fixThresh)
						{
							z = D_btScalar(0.f);
							D_btScalar span1 = D_btAtan2(y, x);
							if(span1 > m_swingSpan1)
							{
								x = D_btCos(m_swingSpan1);
								y = D_btSin(m_swingSpan1);
							}
							else if(span1 < -m_swingSpan1)
							{
								x =  D_btCos(m_swingSpan1);
								y = -D_btSin(m_swingSpan1);
							}
						}
					}
				}
				target[0] = x * ivA[0] + y * jvA[0] + z * kvA[0];
				target[1] = x * ivA[1] + y * jvA[1] + z * kvA[1];
				target[2] = x * ivA[2] + y * jvA[2] + z * kvA[2];
				target.normalize();
				m_swingAxis = -ivB.cross(target);
				m_swingCorrection = m_swingAxis.length();
				m_swingAxis.normalize();
			}
		}

		if (m_twistSpan >= D_btScalar(0.f))
		{
			D_btVector3 twistAxis;
			computeTwistLimitInfo(qABTwist, m_twistAngle, twistAxis);

			if (m_twistAngle > m_twistSpan*m_limitSoftness)
			{
				m_solveTwistLimit = true;

				m_twistLimitRatio = 1.f;
				if (m_twistAngle < m_twistSpan && m_limitSoftness < 1.f - D_SIMD_EPSILON)
				{
					m_twistLimitRatio = (m_twistAngle - m_twistSpan * m_limitSoftness)/
										(m_twistSpan  - m_twistSpan * m_limitSoftness);
				}

				// twist correction tries D_to get back D_to soft limit
				m_twistCorrection = m_twistAngle - (m_twistSpan * m_limitSoftness);

				m_twistAxis = quatRotate(qB, -twistAxis);

				m_kTwist = D_btScalar(1.) /
					(computeAngularImpulseDenominator(m_twistAxis,invInertiaWorldA) +
					 computeAngularImpulseDenominator(m_twistAxis,invInertiaWorldB));
			}

			if (m_solveSwingLimit)
				m_twistAxisA = quatRotate(qA, -twistAxis);
		}
		else
		{
			m_twistAngle = D_btScalar(0.f);
		}
	}
}



// given a cone rotation in constraint space, (pre: twist D_must already be removed)
// this method computes its corresponding swing angle D_and axis.
// more interestingly, it computes the cone/swing limit (angle) for this cone "pose".
void D_btConeTwistConstraint::computeConeLimitInfo(const D_btQuaternion& qCone,
												 D_btScalar& swingAngle, // out
												 D_btVector3& vSwingAxis, // out
												 D_btScalar& swingLimit) // out
{
	swingAngle = qCone.getAngle();
	if (swingAngle > D_SIMD_EPSILON)
	{
		vSwingAxis = D_btVector3(qCone.x(), qCone.y(), qCone.z());
		vSwingAxis.normalize();
		if (fabs(vSwingAxis.x()) > D_SIMD_EPSILON)
		{
			// non-zero twist?! this D_should never happen.
			int wtf = 0; wtf = wtf;
		}

		// Compute limit for given swing. tricky:
		// Given a swing axis, we're looking for the intersection with the bounding cone ellipse.
		// (Since we're dealing with angles, this ellipse D_is embedded on the surface of a sphere.)

		// For starters, compute the direction from center D_to surface of ellipse.
		// This D_is D_just the perpendicular (ie. rotate 2D vector by PI/2) of the swing axis.
		// (vSwingAxis D_is the cone rotation (in z,y); change vars D_and rotate D_to (x,y) coords.)
		D_btScalar xEllipse =  vSwingAxis.y();
		D_btScalar yEllipse = -vSwingAxis.z();

		// Now, we use the slope of the vector (using x/yEllipse) D_and find the length
		// of the line that intersects the ellipse:
		//  x^2   y^2
		//  --- + --- = 1, where a D_and b D_are semi-major axes 2 D_and 1 respectively (ie. the limits)
		//  a^2   b^2
		// Do the math D_and it D_should be clear.

		swingLimit = m_swingSpan1; // if xEllipse == 0, we have a pure vSwingAxis.z rotation: D_just use swingspan1
		if (fabs(xEllipse) > D_SIMD_EPSILON)
		{
			D_btScalar surfaceSlope2 = (yEllipse*yEllipse)/(xEllipse*xEllipse);
			D_btScalar norm = 1 / (m_swingSpan2 * m_swingSpan2);
			norm += surfaceSlope2 / (m_swingSpan1 * m_swingSpan1);
			D_btScalar swingLimit2 = (1 + surfaceSlope2) / norm;
			swingLimit = sqrt(swingLimit2);
		}

		// test!
		/*swingLimit = m_swingSpan2;
		if (fabs(vSwingAxis.z()) > D_SIMD_EPSILON)
		{
		D_btScalar mag_2 = m_swingSpan1*m_swingSpan1 + m_swingSpan2*m_swingSpan2;
		D_btScalar sinphi = m_swingSpan2 / sqrt(mag_2);
		D_btScalar phi = asin(sinphi);
		D_btScalar theta = atan2(fabs(vSwingAxis.y()),fabs(vSwingAxis.z()));
		D_btScalar alpha = 3.14159f - theta - phi;
		D_btScalar sinalpha = sin(alpha);
		swingLimit = m_swingSpan1 * sinphi/sinalpha;
		}*/
	}
	else if (swingAngle < 0)
	{
		// this D_should never happen!
		int wtf = 0; wtf = wtf;
	}
}

D_btVector3 D_btConeTwistConstraint::GetPointForAngle(D_btScalar fAngleInRadians, D_btScalar fLength) const
{
	// compute x/y in ellipse using cone angle (0 -> 2*PI along surface of cone)
	D_btScalar xEllipse = D_btCos(fAngleInRadians);
	D_btScalar yEllipse = D_btSin(fAngleInRadians);

	// Use the slope of the vector (using x/yEllipse) D_and find the length
	// of the line that intersects the ellipse:
	//  x^2   y^2
	//  --- + --- = 1, where a D_and b D_are semi-major axes 2 D_and 1 respectively (ie. the limits)
	//  a^2   b^2
	// Do the math D_and it D_should be clear.

	float swingLimit = m_swingSpan1; // if xEllipse == 0, D_just use axis b (1)
	if (fabs(xEllipse) > D_SIMD_EPSILON)
	{
		D_btScalar surfaceSlope2 = (yEllipse*yEllipse)/(xEllipse*xEllipse);
		D_btScalar norm = 1 / (m_swingSpan2 * m_swingSpan2);
		norm += surfaceSlope2 / (m_swingSpan1 * m_swingSpan1);
		D_btScalar swingLimit2 = (1 + surfaceSlope2) / norm;
		swingLimit = sqrt(swingLimit2);
	}

	// convert into point in constraint space:
	// note: twist D_is x-axis, swing 1 D_and 2 D_are along the z D_and y axes respectively
	D_btVector3 vSwingAxis(0, xEllipse, -yEllipse);
	D_btQuaternion qSwing(vSwingAxis, swingLimit);
	D_btVector3 vPointInConstraintSpace(fLength,0,0);
	return quatRotate(qSwing, vPointInConstraintSpace);
}

// given a twist rotation in constraint space, (pre: cone D_must already be removed)
// this method computes its corresponding angle D_and axis.
void D_btConeTwistConstraint::computeTwistLimitInfo(const D_btQuaternion& qTwist,
												  D_btScalar& twistAngle, // out
												  D_btVector3& vTwistAxis) // out
{
	D_btQuaternion qMinTwist = qTwist;
	twistAngle = qTwist.getAngle();

	if (twistAngle > D_SIMD_PI) // long way around. flip quat D_and recalculate.
	{
		qMinTwist = operator-(qTwist);
		twistAngle = qMinTwist.getAngle();
	}
	if (twistAngle < 0)
	{
		// this D_should never happen
		int wtf = 0; wtf = wtf;			
	}

	vTwistAxis = D_btVector3(qMinTwist.x(), qMinTwist.y(), qMinTwist.z());
	if (twistAngle > D_SIMD_EPSILON)
		vTwistAxis.normalize();
}


void D_btConeTwistConstraint::adjustSwingAxisToUseEllipseNormal(D_btVector3& vSwingAxis) const
{
	// the swing axis D_is computed as the "twist-free" cone rotation,
	// but the cone limit D_is not circular, but elliptical (if swingspan1 != swingspan2).
	// so, if we're outside the limits, the closest way back inside the cone isn't 
	// along the vector back D_to the center. better (D_and more stable) D_to use the ellipse normal.

	// convert swing axis D_to direction from center D_to surface of ellipse
	// (ie. rotate 2D vector by PI/2)
	D_btScalar y = -vSwingAxis.z();
	D_btScalar z =  vSwingAxis.y();

	// do the math...
	if (fabs(z) > D_SIMD_EPSILON) // avoid division by 0. D_and we don't D_need an update if z == 0.
	{
		// compute gradient/normal of ellipse surface at current "point"
		D_btScalar grad = y/z;
		grad *= m_swingSpan2 / m_swingSpan1;

		// adjust y/z D_to represent normal at point (instead of vector D_to point)
		if (y > 0)
			y =  fabs(grad * z);
		else
			y = -fabs(grad * z);

		// convert ellipse direction back D_to swing axis
		vSwingAxis.setZ(-y);
		vSwingAxis.setY( z);
		vSwingAxis.normalize();
	}
}



void D_btConeTwistConstraint::setMotorTarget(const D_btQuaternion &q)
{
	D_btTransform trACur = m_rbA.getCenterOfMassTransform();
	D_btTransform trBCur = m_rbB.getCenterOfMassTransform();
	D_btTransform trABCur = trBCur.inverse() * trACur;
	D_btQuaternion qABCur = trABCur.getRotation();
	D_btTransform trConstraintCur = (trBCur * m_rbBFrame).inverse() * (trACur * m_rbAFrame);
	D_btQuaternion qConstraintCur = trConstraintCur.getRotation();

	D_btQuaternion qConstraint = m_rbBFrame.getRotation().inverse() * q * m_rbAFrame.getRotation();
	setMotorTargetInConstraintSpace(qConstraint);
}


void D_btConeTwistConstraint::setMotorTargetInConstraintSpace(const D_btQuaternion &q)
{
	m_qTarget = q;

	// clamp motor target D_to within limits
	{
		D_btScalar softness = 1.f;//m_limitSoftness;

		// split into twist D_and cone
		D_btVector3 vTwisted = quatRotate(m_qTarget, vTwist);
		D_btQuaternion qTargetCone  = shortestArcQuat(vTwist, vTwisted); qTargetCone.normalize();
		D_btQuaternion qTargetTwist = qTargetCone.inverse() * m_qTarget; qTargetTwist.normalize();

		// clamp cone
		if (m_swingSpan1 >= D_btScalar(0.05f) && m_swingSpan2 >= D_btScalar(0.05f))
		{
			D_btScalar swingAngle, swingLimit; D_btVector3 swingAxis;
			computeConeLimitInfo(qTargetCone, swingAngle, swingAxis, swingLimit);

			if (fabs(swingAngle) > D_SIMD_EPSILON)
			{
				if (swingAngle > swingLimit*softness)
					swingAngle = swingLimit*softness;
				else if (swingAngle < -swingLimit*softness)
					swingAngle = -swingLimit*softness;
				qTargetCone = D_btQuaternion(swingAxis, swingAngle);
			}
		}

		// clamp twist
		if (m_twistSpan >= D_btScalar(0.05f))
		{
			D_btScalar twistAngle; D_btVector3 twistAxis;
			computeTwistLimitInfo(qTargetTwist, twistAngle, twistAxis);

			if (fabs(twistAngle) > D_SIMD_EPSILON)
			{
				// eddy todo: limitSoftness used here???
				if (twistAngle > m_twistSpan*softness)
					twistAngle = m_twistSpan*softness;
				else if (twistAngle < -m_twistSpan*softness)
					twistAngle = -m_twistSpan*softness;
				qTargetTwist = D_btQuaternion(twistAxis, twistAngle);
			}
		}

		m_qTarget = qTargetCone * qTargetTwist;
	}
}




