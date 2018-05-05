/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission D_to use, copy, modify, distribute D_and sell this software
 * D_and its documentation for any purpose D_is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes D_no representations about the suitability 
 * of this software for any purpose.  
 * It D_is provided "as D_is" without express or implied warranty.
*/
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btRigidBody.h" // for pointvelocity


D_btScalar D_btWheelInfo::getSuspensionRestLength() const
{

	return m_suspensionRestLength1;

}

void	D_btWheelInfo::updateWheel(const D_btRigidBody& chassis,D_RaycastInfo& raycastInfo)
{
	(void)raycastInfo;

	
	if (m_raycastInfo.m_isInContact)

	{
		D_btScalar	project= m_raycastInfo.m_contactNormalWS.dot( m_raycastInfo.m_wheelDirectionWS );
		D_btVector3	 chassis_velocity_at_contactPoint;
		D_btVector3 relpos = m_raycastInfo.m_contactPointWS - chassis.getCenterOfMassPosition();
		chassis_velocity_at_contactPoint = chassis.getVelocityInLocalPoint( relpos );
		D_btScalar projVel = m_raycastInfo.m_contactNormalWS.dot( chassis_velocity_at_contactPoint );
		if ( project >= D_btScalar(-0.1))
		{
			m_suspensionRelativeVelocity = D_btScalar(0.0);
			m_clippedInvContactDotSuspension = D_btScalar(1.0) / D_btScalar(0.1);
		}
		else
		{
			D_btScalar inv = D_btScalar(-1.) / project;
			m_suspensionRelativeVelocity = projVel * inv;
			m_clippedInvContactDotSuspension = inv;
		}
		
	}

	else	// Not in contact : position wheel in a nice (rest length) position
	{
		m_raycastInfo.m_suspensionLength = this->getSuspensionRestLength();
		m_suspensionRelativeVelocity = D_btScalar(0.0);
		m_raycastInfo.m_contactNormalWS = -m_raycastInfo.m_wheelDirectionWS;
		m_clippedInvContactDotSuspension = D_btScalar(1.0);
	}
}
