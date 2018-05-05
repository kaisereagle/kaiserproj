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


#include "btTypedConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

static D_btRigidBody s_fixed(0, 0,0);

#define D_DEFAULT_DEBUGDRAW_SIZE D_btScalar(0.3f)

D_btTypedConstraint::D_btTypedConstraint(D_btTypedConstraintType type)
:D_btTypedObject(type),
m_userConstraintType(-1),
m_userConstraintId(-1),
m_needsFeedback(false),
m_rbA(s_fixed),
m_rbB(s_fixed),
m_appliedImpulse(D_btScalar(0.)),
m_dbgDrawSize(D_DEFAULT_DEBUGDRAW_SIZE)
{
	s_fixed.setMassProps(D_btScalar(0.),D_btVector3(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.)));
}
D_btTypedConstraint::D_btTypedConstraint(D_btTypedConstraintType type, D_btRigidBody& rbA)
:D_btTypedObject(type),
m_userConstraintType(-1),
m_userConstraintId(-1),
m_needsFeedback(false),
m_rbA(rbA),
m_rbB(s_fixed),
m_appliedImpulse(D_btScalar(0.)),
m_dbgDrawSize(D_DEFAULT_DEBUGDRAW_SIZE)
{
	s_fixed.setMassProps(D_btScalar(0.),D_btVector3(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.)));
}


D_btTypedConstraint::D_btTypedConstraint(D_btTypedConstraintType type, D_btRigidBody& rbA,D_btRigidBody& rbB)
:D_btTypedObject(type),
m_userConstraintType(-1),
m_userConstraintId(-1),
m_needsFeedback(false),
m_rbA(rbA),
m_rbB(rbB),
m_appliedImpulse(D_btScalar(0.)),
m_dbgDrawSize(D_DEFAULT_DEBUGDRAW_SIZE)
{
	s_fixed.setMassProps(D_btScalar(0.),D_btVector3(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.)));

}




D_btScalar D_btTypedConstraint::getMotorFactor(D_btScalar pos, D_btScalar lowLim, D_btScalar uppLim, D_btScalar vel, D_btScalar timeFact)
{
	if(lowLim > uppLim)
	{
		return D_btScalar(1.0f);
	}
	else if(lowLim == uppLim)
	{
		return D_btScalar(0.0f);
	}
	D_btScalar lim_fact = D_btScalar(1.0f);
	D_btScalar delta_max = vel / timeFact;
	if(delta_max < D_btScalar(0.0f))
	{
		if((pos >= lowLim) && (pos < (lowLim - delta_max)))
		{
			lim_fact = (lowLim - pos) / delta_max;
		}
		else if(pos  < lowLim)
		{
			lim_fact = D_btScalar(0.0f);
		}
		else
		{
			lim_fact = D_btScalar(1.0f);
		}
	}
	else if(delta_max > D_btScalar(0.0f))
	{
		if((pos <= uppLim) && (pos > (uppLim - delta_max)))
		{
			lim_fact = (uppLim - pos) / delta_max;
		}
		else if(pos  > uppLim)
		{
			lim_fact = D_btScalar(0.0f);
		}
		else
		{
			lim_fact = D_btScalar(1.0f);
		}
	}
	else
	{
			lim_fact = D_btScalar(0.0f);
	}
	return lim_fact;
}


