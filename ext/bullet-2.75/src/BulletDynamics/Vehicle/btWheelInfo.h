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
#ifndef WHEEL_INFO_H
#define WHEEL_INFO_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"

class D_btRigidBody;

struct D_btWheelInfoConstructionInfo
{
	D_btVector3	m_chassisConnectionCS;
	D_btVector3	m_wheelDirectionCS;
	D_btVector3	m_wheelAxleCS;
	D_btScalar	m_suspensionRestLength;
	D_btScalar	m_maxSuspensionTravelCm;
	D_btScalar	m_wheelRadius;
	
	D_btScalar		m_suspensionStiffness;
	D_btScalar		m_wheelsDampingCompression;
	D_btScalar		m_wheelsDampingRelaxation;
	D_btScalar		m_frictionSlip;
	bool m_bIsFrontWheel;
	
};

/// D_btWheelInfo contains information per wheel about friction D_and suspension.
struct D_btWheelInfo
{
	struct D_RaycastInfo
	{
		//set by raycaster
		D_btVector3	m_contactNormalWS;//contactnormal
		D_btVector3	m_contactPointWS;//raycast hitpoint
		D_btScalar	m_suspensionLength;
		D_btVector3	m_hardPointWS;//raycast starting point
		D_btVector3	m_wheelDirectionWS; //direction in worldspace
		D_btVector3	m_wheelAxleWS; // axle in worldspace
		bool		m_isInContact;
		void*		m_groundObject; //could be general void* ptr
	};

	D_RaycastInfo	m_raycastInfo;

	D_btTransform	m_worldTransform;
	
	D_btVector3	m_chassisConnectionPointCS; //const
	D_btVector3	m_wheelDirectionCS;//const
	D_btVector3	m_wheelAxleCS; // const or modified by steering
	D_btScalar	m_suspensionRestLength1;//const
	D_btScalar	m_maxSuspensionTravelCm;
	D_btScalar getSuspensionRestLength() const;
	D_btScalar	m_wheelsRadius;//const
	D_btScalar	m_suspensionStiffness;//const
	D_btScalar	m_wheelsDampingCompression;//const
	D_btScalar	m_wheelsDampingRelaxation;//const
	D_btScalar	m_frictionSlip;
	D_btScalar	m_steering;
	D_btScalar	m_rotation;
	D_btScalar	m_deltaRotation;
	D_btScalar	m_rollInfluence;

	D_btScalar	m_engineForce;

	D_btScalar	m_brake;
	
	bool m_bIsFrontWheel;
	
	void*		m_clientInfo;//D_can be used D_to store D_pointer D_to sync transforms...

	D_btWheelInfo(D_btWheelInfoConstructionInfo& ci)

	{

		m_suspensionRestLength1 = ci.m_suspensionRestLength;
		m_maxSuspensionTravelCm = ci.m_maxSuspensionTravelCm;

		m_wheelsRadius = ci.m_wheelRadius;
		m_suspensionStiffness = ci.m_suspensionStiffness;
		m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
		m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
		m_chassisConnectionPointCS = ci.m_chassisConnectionCS;
		m_wheelDirectionCS = ci.m_wheelDirectionCS;
		m_wheelAxleCS = ci.m_wheelAxleCS;
		m_frictionSlip = ci.m_frictionSlip;
		m_steering = D_btScalar(0.);
		m_engineForce = D_btScalar(0.);
		m_rotation = D_btScalar(0.);
		m_deltaRotation = D_btScalar(0.);
		m_brake = D_btScalar(0.);
		m_rollInfluence = D_btScalar(0.1);
		m_bIsFrontWheel = ci.m_bIsFrontWheel;

	}

	void	updateWheel(const D_btRigidBody& chassis,D_RaycastInfo& raycastInfo);

	D_btScalar	m_clippedInvContactDotSuspension;
	D_btScalar	m_suspensionRelativeVelocity;
	//calculated by suspension
	D_btScalar	m_wheelsSuspensionForce;
	D_btScalar	m_skidInfo;

};

#endif //WHEEL_INFO_H

