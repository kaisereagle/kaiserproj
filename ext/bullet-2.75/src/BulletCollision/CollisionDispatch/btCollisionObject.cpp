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


#include "btCollisionObject.h"

D_btCollisionObject::D_btCollisionObject()
	:	m_anisotropicFriction(1.f,1.f,1.f),
	m_hasAnisotropicFriction(false),
	m_contactProcessingThreshold(D_BT_LARGE_FLOAT),
		m_broadphaseHandle(0),
		m_collisionShape(0),
		m_rootCollisionShape(0),
		m_collisionFlags(D_btCollisionObject::D_CF_STATIC_OBJECT),
		m_islandTag1(-1),
		m_companionId(-1),
		m_activationState1(1),
		m_deactivationTime(D_btScalar(0.)),
		m_friction(D_btScalar(0.5)),
		m_restitution(D_btScalar(0.)),
		m_userObjectPointer(0),
		m_internalType(D_CO_COLLISION_OBJECT),
		m_hitFraction(D_btScalar(1.)),
		m_ccdSweptSphereRadius(D_btScalar(0.)),
		m_ccdMotionThreshold(D_btScalar(0.)),
		m_checkCollideWith(false)
{
	m_worldTransform.setIdentity();
}

D_btCollisionObject::~D_btCollisionObject()
{
}

void D_btCollisionObject::setActivationState(int newState) 
{ 
	if ( (m_activationState1 != D_DISABLE_DEACTIVATION) && (m_activationState1 != D_DISABLE_SIMULATION))
		m_activationState1 = newState;
}

void D_btCollisionObject::forceActivationState(int newState)
{
	m_activationState1 = newState;
}

void D_btCollisionObject::activate(bool forceActivation)
{
	if (forceActivation || !(m_collisionFlags & (D_CF_STATIC_OBJECT|D_CF_KINEMATIC_OBJECT)))
	{
		setActivationState(D_ACTIVE_TAG);
		m_deactivationTime = D_btScalar(0.);
	}
}



