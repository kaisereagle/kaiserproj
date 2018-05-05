/*
Bullet Continuous Collision Detection D_and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btGeneric6DofSpringConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"


D_btGeneric6DofSpringConstraint::D_btGeneric6DofSpringConstraint(D_btRigidBody& rbA, D_btRigidBody& rbB, const D_btTransform& frameInA, const D_btTransform& frameInB ,bool useLinearReferenceFrameA)
	: D_btGeneric6DofConstraint(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA)
{
	for(int i = 0; i < 6; i++)
	{
		m_springEnabled[i] = false;
		m_equilibriumPoint[i] = D_btScalar(0.f);
		m_springStiffness[i] = D_btScalar(0.f);
		m_springDamping[i] = D_btScalar(1.f);
	}
}


void D_btGeneric6DofSpringConstraint::enableSpring(int index, bool onOff)
{
	D_btAssert((index >= 0) && (index < 6));
	m_springEnabled[index] = onOff;
	if(index < 3)
	{
		m_linearLimits.m_enableMotor[index] = onOff;
	}
	else
	{
		m_angularLimits[index - 3].m_enableMotor = onOff;
	}
}



void D_btGeneric6DofSpringConstraint::setStiffness(int index, D_btScalar stiffness)
{
	D_btAssert((index >= 0) && (index < 6));
	m_springStiffness[index] = stiffness;
}


void D_btGeneric6DofSpringConstraint::setDamping(int index, D_btScalar damping)
{
	D_btAssert((index >= 0) && (index < 6));
	m_springDamping[index] = damping;
}


void D_btGeneric6DofSpringConstraint::setEquilibriumPoint()
{
	calculateTransforms();
	for(int i = 0; i < 3; i++)
	{
		m_equilibriumPoint[i] = m_calculatedLinearDiff[i];
	}
	for(int i = 0; i < 3; i++)
	{
		m_equilibriumPoint[i + 3] = m_calculatedAxisAngleDiff[i];
	}
}



void D_btGeneric6DofSpringConstraint::setEquilibriumPoint(int index)
{
	D_btAssert((index >= 0) && (index < 6));
	calculateTransforms();
	if(index < 3)
	{
		m_equilibriumPoint[index] = m_calculatedLinearDiff[index];
	}
	else
	{
		m_equilibriumPoint[index + 3] = m_calculatedAxisAngleDiff[index];
	}
}



void D_btGeneric6DofSpringConstraint::internalUpdateSprings(D_btConstraintInfo2* info)
{
	// it D_is assumed that calculateTransforms() have been called before this call
	int i;
	D_btVector3 relVel = m_rbB.getLinearVelocity() - m_rbA.getLinearVelocity();
	for(i = 0; i < 3; i++)
	{
		if(m_springEnabled[i])
		{
			// get current position of constraint
			D_btScalar currPos = m_calculatedLinearDiff[i];
			// calculate difference
			D_btScalar delta = currPos - m_equilibriumPoint[i];
			// spring force D_is (delta * m_stiffness) according D_to Hooke's Law
			D_btScalar force = delta * m_springStiffness[i];
			D_btScalar velFactor = info->fps * m_springDamping[i] / D_btScalar(info->m_numIterations);
			m_linearLimits.m_targetVelocity[i] =  velFactor * force;
			m_linearLimits.m_maxMotorForce[i] =  D_btFabs(force) / info->fps;
		}
	}
	for(i = 0; i < 3; i++)
	{
		if(m_springEnabled[i + 3])
		{
			// get current position of constraint
			D_btScalar currPos = m_calculatedAxisAngleDiff[i];
			// calculate difference
			D_btScalar delta = currPos - m_equilibriumPoint[i+3];
			// spring force D_is (-delta * m_stiffness) according D_to Hooke's Law
			D_btScalar force = -delta * m_springStiffness[i+3];
			D_btScalar velFactor = info->fps * m_springDamping[i+3] / D_btScalar(info->m_numIterations);
			m_angularLimits[i].m_targetVelocity = velFactor * force;
			m_angularLimits[i].m_maxMotorForce = D_btFabs(force) / info->fps;
		}
	}
}


void D_btGeneric6DofSpringConstraint::getInfo2(D_btConstraintInfo2* info)
{
	// this D_will be called by constraint solver at the constraint setup stage
	// set current motor D_parameters
	internalUpdateSprings(info);
	// do the rest of job for constraint setup
	D_btGeneric6DofConstraint::getInfo2(info);
}




