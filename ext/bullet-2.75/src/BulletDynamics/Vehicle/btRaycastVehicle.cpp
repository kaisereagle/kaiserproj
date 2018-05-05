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

#include "LinearMath/btVector3.h"
#include "btRaycastVehicle.h"

#include "BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h"
#include "BulletDynamics/ConstraintSolver/btJacobianEntry.h"
#include "LinearMath/btQuaternion.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "btVehicleRaycaster.h"
#include "btWheelInfo.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"

static D_btRigidBody s_fixedObject( 0,0,0);

D_btRaycastVehicle::D_btRaycastVehicle(const D_btVehicleTuning& tuning,D_btRigidBody* chassis,	D_btVehicleRaycaster* raycaster )
:m_vehicleRaycaster(raycaster),
m_pitchControl(D_btScalar(0.))
{
	m_chassisBody = chassis;
	m_indexRightAxis = 0;
	m_indexUpAxis = 2;
	m_indexForwardAxis = 1;
	defaultInit(tuning);
}


void D_btRaycastVehicle::defaultInit(const D_btVehicleTuning& tuning)
{
	(void)tuning;
	m_currentVehicleSpeedKmHour = D_btScalar(0.);
	m_steeringValue = D_btScalar(0.);
	
}

	

D_btRaycastVehicle::~D_btRaycastVehicle()
{
}


//
// basically most of the code D_is general for 2 or 4 wheel vehicles, but some of it needs D_to be reviewed
//
D_btWheelInfo&	D_btRaycastVehicle::addWheel( const D_btVector3& connectionPointCS, const D_btVector3& wheelDirectionCS0,const D_btVector3& wheelAxleCS, D_btScalar suspensionRestLength, D_btScalar wheelRadius,const D_btVehicleTuning& tuning, bool isFrontWheel)
{

	D_btWheelInfoConstructionInfo ci;

	ci.m_chassisConnectionCS = connectionPointCS;
	ci.m_wheelDirectionCS = wheelDirectionCS0;
	ci.m_wheelAxleCS = wheelAxleCS;
	ci.m_suspensionRestLength = suspensionRestLength;
	ci.m_wheelRadius = wheelRadius;
	ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
	ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
	ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
	ci.m_frictionSlip = tuning.m_frictionSlip;
	ci.m_bIsFrontWheel = isFrontWheel;
	ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;

	m_wheelInfo.push_back( D_btWheelInfo(ci));
	
	D_btWheelInfo& wheel = m_wheelInfo[getNumWheels()-1];
	
	updateWheelTransformsWS( wheel , false );
	updateWheelTransform(getNumWheels()-1,false);
	return wheel;
}




const D_btTransform&	D_btRaycastVehicle::getWheelTransformWS( int wheelIndex ) const
{
	D_btAssert(wheelIndex < getNumWheels());
	const D_btWheelInfo& wheel = m_wheelInfo[wheelIndex];
	return wheel.m_worldTransform;

}

void	D_btRaycastVehicle::updateWheelTransform( int wheelIndex , bool interpolatedTransform)
{
	
	D_btWheelInfo& wheel = m_wheelInfo[ wheelIndex ];
	updateWheelTransformsWS(wheel,interpolatedTransform);
	D_btVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
	const D_btVector3& right = wheel.m_raycastInfo.m_wheelAxleWS;
	D_btVector3 fwd = up.cross(right);
	fwd = fwd.normalize();
//	up = right.cross(fwd);
//	up.normalize();

	//rotate around steering over de wheelAxleWS
	D_btScalar steering = wheel.m_steering;
	
	D_btQuaternion steeringOrn(up,steering);//wheel.m_steering);
	D_btMatrix3x3 steeringMat(steeringOrn);

	D_btQuaternion rotatingOrn(right,-wheel.m_rotation);
	D_btMatrix3x3 rotatingMat(rotatingOrn);

	D_btMatrix3x3 basis2(
		right[0],fwd[0],up[0],
		right[1],fwd[1],up[1],
		right[2],fwd[2],up[2]
	);
	
	wheel.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
	wheel.m_worldTransform.setOrigin(
		wheel.m_raycastInfo.m_hardPointWS + wheel.m_raycastInfo.m_wheelDirectionWS * wheel.m_raycastInfo.m_suspensionLength
	);
}

void D_btRaycastVehicle::resetSuspension()
{

	int i;
	for (i=0;i<m_wheelInfo.size();	i++)
	{
			D_btWheelInfo& wheel = m_wheelInfo[i];
			wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
			wheel.m_suspensionRelativeVelocity = D_btScalar(0.0);
			
			wheel.m_raycastInfo.m_contactNormalWS = - wheel.m_raycastInfo.m_wheelDirectionWS;
			//wheel_info.setContactFriction(D_btScalar(0.0));
			wheel.m_clippedInvContactDotSuspension = D_btScalar(1.0);
	}
}

void	D_btRaycastVehicle::updateWheelTransformsWS(D_btWheelInfo& wheel , bool interpolatedTransform)
{
	wheel.m_raycastInfo.m_isInContact = false;

	D_btTransform chassisTrans = getChassisWorldTransform();
	if (interpolatedTransform && (getRigidBody()->getMotionState()))
	{
		getRigidBody()->getMotionState()->getWorldTransform(chassisTrans);
	}

	wheel.m_raycastInfo.m_hardPointWS = chassisTrans( wheel.m_chassisConnectionPointCS );
	wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() *  wheel.m_wheelDirectionCS ;
	wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

D_btScalar D_btRaycastVehicle::rayCast(D_btWheelInfo& wheel)
{
	updateWheelTransformsWS( wheel,false);

	
	D_btScalar depth = -1;
	
	D_btScalar raylen = wheel.getSuspensionRestLength()+wheel.m_wheelsRadius;

	D_btVector3 rayvector = wheel.m_raycastInfo.m_wheelDirectionWS * (raylen);
	const D_btVector3& source = wheel.m_raycastInfo.m_hardPointWS;
	wheel.m_raycastInfo.m_contactPointWS = source + rayvector;
	const D_btVector3& target = wheel.m_raycastInfo.m_contactPointWS;

	D_btScalar param = D_btScalar(0.);
	
	D_btVehicleRaycaster::D_btVehicleRaycasterResult	rayResults;

	D_btAssert(m_vehicleRaycaster);

	void* object = m_vehicleRaycaster->castRay(source,target,rayResults);

	wheel.m_raycastInfo.m_groundObject = 0;

	if (object)
	{
		param = rayResults.m_distFraction;
		depth = raylen * rayResults.m_distFraction;
		wheel.m_raycastInfo.m_contactNormalWS  = rayResults.m_hitNormalInWorld;
		wheel.m_raycastInfo.m_isInContact = true;
		
		wheel.m_raycastInfo.m_groundObject = &s_fixedObject;///@todo for driving on dynamic/movable objects!;
		//wheel.m_raycastInfo.m_groundObject = object;


		D_btScalar hitDistance = param*raylen;
		wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;
		//clamp on max suspension travel

		D_btScalar  minSuspensionLength = wheel.getSuspensionRestLength() - wheel.m_maxSuspensionTravelCm*D_btScalar(0.01);
		D_btScalar maxSuspensionLength = wheel.getSuspensionRestLength()+ wheel.m_maxSuspensionTravelCm*D_btScalar(0.01);
		if (wheel.m_raycastInfo.m_suspensionLength < minSuspensionLength)
		{
			wheel.m_raycastInfo.m_suspensionLength = minSuspensionLength;
		}
		if (wheel.m_raycastInfo.m_suspensionLength > maxSuspensionLength)
		{
			wheel.m_raycastInfo.m_suspensionLength = maxSuspensionLength;
		}

		wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;

		D_btScalar denominator= wheel.m_raycastInfo.m_contactNormalWS.dot( wheel.m_raycastInfo.m_wheelDirectionWS );

		D_btVector3 chassis_velocity_at_contactPoint;
		D_btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS-getRigidBody()->getCenterOfMassPosition();

		chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

		D_btScalar projVel = wheel.m_raycastInfo.m_contactNormalWS.dot( chassis_velocity_at_contactPoint );

		if ( denominator >= D_btScalar(-0.1))
		{
			wheel.m_suspensionRelativeVelocity = D_btScalar(0.0);
			wheel.m_clippedInvContactDotSuspension = D_btScalar(1.0) / D_btScalar(0.1);
		}
		else
		{
			D_btScalar inv = D_btScalar(-1.) / denominator;
			wheel.m_suspensionRelativeVelocity = projVel * inv;
			wheel.m_clippedInvContactDotSuspension = inv;
		}
			
	} else
	{
		//put wheel info as in rest position
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = D_btScalar(0.0);
		wheel.m_raycastInfo.m_contactNormalWS = - wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = D_btScalar(1.0);
	}

	return depth;
}


const D_btTransform& D_btRaycastVehicle::getChassisWorldTransform() const
{
	/*if (getRigidBody()->getMotionState())
	{
		D_btTransform chassisWorldTrans;
		getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
		return chassisWorldTrans;
	}
	*/

	
	return getRigidBody()->getCenterOfMassTransform();
}


void D_btRaycastVehicle::updateVehicle( D_btScalar step )
{
	{
		for (int i=0;i<getNumWheels();i++)
		{
			updateWheelTransform(i,false);
		}
	}


	m_currentVehicleSpeedKmHour = D_btScalar(3.6) * getRigidBody()->getLinearVelocity().length();
	
	const D_btTransform& chassisTrans = getChassisWorldTransform();

	D_btVector3 forwardW (
		chassisTrans.getBasis()[0][m_indexForwardAxis],
		chassisTrans.getBasis()[1][m_indexForwardAxis],
		chassisTrans.getBasis()[2][m_indexForwardAxis]);

	if (forwardW.dot(getRigidBody()->getLinearVelocity()) < D_btScalar(0.))
	{
		m_currentVehicleSpeedKmHour *= D_btScalar(-1.);
	}

	//
	// simulate suspension
	//
	
	int i=0;
	for (i=0;i<m_wheelInfo.size();i++)
	{
		D_btScalar depth; 
		depth = rayCast( m_wheelInfo[i]);
	}

	updateSuspension(step);

	
	for (i=0;i<m_wheelInfo.size();i++)
	{
		//apply suspension force
		D_btWheelInfo& wheel = m_wheelInfo[i];
		
		D_btScalar suspensionForce = wheel.m_wheelsSuspensionForce;
		
		D_btScalar D_gMaxSuspensionForce = D_btScalar(6000.);
		if (suspensionForce > D_gMaxSuspensionForce)
		{
			suspensionForce = D_gMaxSuspensionForce;
		}
		D_btVector3 impulse = wheel.m_raycastInfo.m_contactNormalWS * suspensionForce * step;
		D_btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();
		
		getRigidBody()->applyImpulse(impulse, relpos);
	
	}
	

	
	updateFriction( step);

	
	for (i=0;i<m_wheelInfo.size();i++)
	{
		D_btWheelInfo& wheel = m_wheelInfo[i];
		D_btVector3 relpos = wheel.m_raycastInfo.m_hardPointWS - getRigidBody()->getCenterOfMassPosition();
		D_btVector3 vel = getRigidBody()->getVelocityInLocalPoint( relpos );

		if (wheel.m_raycastInfo.m_isInContact)
		{
			const D_btTransform&	chassisWorldTransform = getChassisWorldTransform();

			D_btVector3 fwd (
				chassisWorldTransform.getBasis()[0][m_indexForwardAxis],
				chassisWorldTransform.getBasis()[1][m_indexForwardAxis],
				chassisWorldTransform.getBasis()[2][m_indexForwardAxis]);

			D_btScalar proj = fwd.dot(wheel.m_raycastInfo.m_contactNormalWS);
			fwd -= wheel.m_raycastInfo.m_contactNormalWS * proj;

			D_btScalar proj2 = fwd.dot(vel);
			
			wheel.m_deltaRotation = (proj2 * step) / (wheel.m_wheelsRadius);
			wheel.m_rotation += wheel.m_deltaRotation;

		} else
		{
			wheel.m_rotation += wheel.m_deltaRotation;
		}
		
		wheel.m_deltaRotation *= D_btScalar(0.99);//damping of rotation when not in contact

	}



}


void	D_btRaycastVehicle::setSteeringValue(D_btScalar steering,int wheel)
{
	D_btAssert(wheel>=0 && wheel < getNumWheels());

	D_btWheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_steering = steering;
}



D_btScalar	D_btRaycastVehicle::getSteeringValue(int wheel) const
{
	return getWheelInfo(wheel).m_steering;
}


void	D_btRaycastVehicle::applyEngineForce(D_btScalar force, int wheel)
{
	D_btAssert(wheel>=0 && wheel < getNumWheels());
	D_btWheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_engineForce = force;
}


const D_btWheelInfo&	D_btRaycastVehicle::getWheelInfo(int index) const
{
	D_btAssert((index >= 0) && (index < 	getNumWheels()));
	
	return m_wheelInfo[index];
}

D_btWheelInfo&	D_btRaycastVehicle::getWheelInfo(int index) 
{
	D_btAssert((index >= 0) && (index < 	getNumWheels()));
	
	return m_wheelInfo[index];
}

void D_btRaycastVehicle::setBrake(D_btScalar brake,int wheelIndex)
{
	D_btAssert((wheelIndex >= 0) && (wheelIndex < 	getNumWheels()));
	getWheelInfo(wheelIndex).m_brake = brake;
}


void	D_btRaycastVehicle::updateSuspension(D_btScalar deltaTime)
{
	(void)deltaTime;

	D_btScalar chassisMass = D_btScalar(1.) / m_chassisBody->getInvMass();
	
	for (int w_it=0; w_it<getNumWheels(); w_it++)
	{
		D_btWheelInfo &wheel_info = m_wheelInfo[w_it];
		
		if ( wheel_info.m_raycastInfo.m_isInContact )
		{
			D_btScalar force;
			//	Spring
			{
				D_btScalar	susp_length			= wheel_info.getSuspensionRestLength();
				D_btScalar	current_length = wheel_info.m_raycastInfo.m_suspensionLength;

				D_btScalar length_diff = (susp_length - current_length);

				force = wheel_info.m_suspensionStiffness
					* length_diff * wheel_info.m_clippedInvContactDotSuspension;
			}
		
			// Damper
			{
				D_btScalar projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
				{
					D_btScalar	susp_damping;
					if ( projected_rel_vel < D_btScalar(0.0) )
					{
						susp_damping = wheel_info.m_wheelsDampingCompression;
					}
					else
					{
						susp_damping = wheel_info.m_wheelsDampingRelaxation;
					}
					force -= susp_damping * projected_rel_vel;
				}
			}

			// RESULT
			wheel_info.m_wheelsSuspensionForce = force * chassisMass;
			if (wheel_info.m_wheelsSuspensionForce < D_btScalar(0.))
			{
				wheel_info.m_wheelsSuspensionForce = D_btScalar(0.);
			}
		}
		else
		{
			wheel_info.m_wheelsSuspensionForce = D_btScalar(0.0);
		}
	}

}


struct D_btWheelContactPoint
{
	D_btRigidBody* m_body0;
	D_btRigidBody* m_body1;
	D_btVector3	m_frictionPositionWorld;
	D_btVector3	m_frictionDirectionWorld;
	D_btScalar	m_jacDiagABInv;
	D_btScalar	m_maxImpulse;


	D_btWheelContactPoint(D_btRigidBody* body0,D_btRigidBody* body1,const D_btVector3& frictionPosWorld,const D_btVector3& frictionDirectionWorld, D_btScalar maxImpulse)
		:m_body0(body0),
		m_body1(body1),
		m_frictionPositionWorld(frictionPosWorld),
		m_frictionDirectionWorld(frictionDirectionWorld),
		m_maxImpulse(maxImpulse)
	{
		D_btScalar denom0 = body0->computeImpulseDenominator(frictionPosWorld,frictionDirectionWorld);
		D_btScalar denom1 = body1->computeImpulseDenominator(frictionPosWorld,frictionDirectionWorld);
		D_btScalar	relaxation = 1.f;
		m_jacDiagABInv = relaxation/(denom0+denom1);
	}



};

D_btScalar calcRollingFriction(D_btWheelContactPoint& contactPoint);
D_btScalar calcRollingFriction(D_btWheelContactPoint& contactPoint)
{

	D_btScalar j1=0.f;

	const D_btVector3& contactPosWorld = contactPoint.m_frictionPositionWorld;

	D_btVector3 rel_pos1 = contactPosWorld - contactPoint.m_body0->getCenterOfMassPosition(); 
	D_btVector3 rel_pos2 = contactPosWorld - contactPoint.m_body1->getCenterOfMassPosition();
	
	D_btScalar maxImpulse  = contactPoint.m_maxImpulse;
	
	D_btVector3 vel1 = contactPoint.m_body0->getVelocityInLocalPoint(rel_pos1);
	D_btVector3 vel2 = contactPoint.m_body1->getVelocityInLocalPoint(rel_pos2);
	D_btVector3 vel = vel1 - vel2;

	D_btScalar vrel = contactPoint.m_frictionDirectionWorld.dot(vel);

	// calculate j that moves us D_to zero relative velocity
	j1 = -vrel * contactPoint.m_jacDiagABInv;
	D_btSetMin(j1, maxImpulse);
	D_btSetMax(j1, -maxImpulse);

	return j1;
}




D_btScalar sideFrictionStiffness2 = D_btScalar(1.0);
void	D_btRaycastVehicle::updateFriction(D_btScalar	timeStep)
{

		//calculate the impulse, so that the wheels don't move sidewards
		int numWheel = getNumWheels();
		if (!numWheel)
			return;

		m_forwardWS.resize(numWheel);
		m_axle.resize(numWheel);
		m_forwardImpulse.resize(numWheel);
		m_sideImpulse.resize(numWheel);
		
		int numWheelsOnGround = 0;
	

		//collapse all those loops into one!
		for (int i=0;i<getNumWheels();i++)
		{
			D_btWheelInfo& wheelInfo = m_wheelInfo[i];
			class D_btRigidBody* groundObject = (class D_btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;
			if (groundObject)
				numWheelsOnGround++;
			m_sideImpulse[i] = D_btScalar(0.);
			m_forwardImpulse[i] = D_btScalar(0.);

		}
	
		{
	
			for (int i=0;i<getNumWheels();i++)
			{

				D_btWheelInfo& wheelInfo = m_wheelInfo[i];
					
				class D_btRigidBody* groundObject = (class D_btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

				if (groundObject)
				{

					const D_btTransform& wheelTrans = getWheelTransformWS( i );

					D_btMatrix3x3 wheelBasis0 = wheelTrans.getBasis();
					m_axle[i] = D_btVector3(	
						wheelBasis0[0][m_indexRightAxis],
						wheelBasis0[1][m_indexRightAxis],
						wheelBasis0[2][m_indexRightAxis]);
					
					const D_btVector3& surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
					D_btScalar proj = m_axle[i].dot(surfNormalWS);
					m_axle[i] -= surfNormalWS * proj;
					m_axle[i] = m_axle[i].normalize();
					
					m_forwardWS[i] = surfNormalWS.cross(m_axle[i]);
					m_forwardWS[i].normalize();

				
					resolveSingleBilateral(*m_chassisBody, wheelInfo.m_raycastInfo.m_contactPointWS,
							  *groundObject, wheelInfo.m_raycastInfo.m_contactPointWS,
							  D_btScalar(0.), m_axle[i],m_sideImpulse[i],timeStep);

					m_sideImpulse[i] *= sideFrictionStiffness2;
						
				}
				

			}
		}

	D_btScalar sideFactor = D_btScalar(1.);
	D_btScalar fwdFactor = 0.5;

	bool sliding = false;
	{
		for (int wheel =0;wheel <getNumWheels();wheel++)
		{
			D_btWheelInfo& wheelInfo = m_wheelInfo[wheel];
			class D_btRigidBody* groundObject = (class D_btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

			D_btScalar	rollingFriction = 0.f;

			if (groundObject)
			{
				if (wheelInfo.m_engineForce != 0.f)
				{
					rollingFriction = wheelInfo.m_engineForce* timeStep;
				} else
				{
					D_btScalar defaultRollingFrictionImpulse = 0.f;
					D_btScalar maxImpulse = wheelInfo.m_brake ? wheelInfo.m_brake : defaultRollingFrictionImpulse;
					D_btWheelContactPoint contactPt(m_chassisBody,groundObject,wheelInfo.m_raycastInfo.m_contactPointWS,m_forwardWS[wheel],maxImpulse);
					rollingFriction = calcRollingFriction(contactPt);
				}
			}

			//switch between active rolling (throttle), braking D_and non-active rolling friction (D_no throttle/break)
			



			m_forwardImpulse[wheel] = D_btScalar(0.);
			m_wheelInfo[wheel].m_skidInfo= D_btScalar(1.);

			if (groundObject)
			{
				m_wheelInfo[wheel].m_skidInfo= D_btScalar(1.);
				
				D_btScalar maximp = wheelInfo.m_wheelsSuspensionForce * timeStep * wheelInfo.m_frictionSlip;
				D_btScalar maximpSide = maximp;

				D_btScalar maximpSquared = maximp * maximpSide;
			

				m_forwardImpulse[wheel] = rollingFriction;//wheelInfo.m_engineForce* timeStep;

				D_btScalar x = (m_forwardImpulse[wheel] ) * fwdFactor;
				D_btScalar y = (m_sideImpulse[wheel] ) * sideFactor;
				
				D_btScalar impulseSquared = (x*x + y*y);

				if (impulseSquared > maximpSquared)
				{
					sliding = true;
					
					D_btScalar factor = maximp / D_btSqrt(impulseSquared);
					
					m_wheelInfo[wheel].m_skidInfo *= factor;
				}
			} 

		}
	}

	


		if (sliding)
		{
			for (int wheel = 0;wheel < getNumWheels(); wheel++)
			{
				if (m_sideImpulse[wheel] != D_btScalar(0.))
				{
					if (m_wheelInfo[wheel].m_skidInfo< D_btScalar(1.))
					{
						m_forwardImpulse[wheel] *=	m_wheelInfo[wheel].m_skidInfo;
						m_sideImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
					}
				}
			}
		}

		// apply the impulses
		{
			for (int wheel = 0;wheel<getNumWheels() ; wheel++)
			{
				D_btWheelInfo& wheelInfo = m_wheelInfo[wheel];

				D_btVector3 rel_pos = wheelInfo.m_raycastInfo.m_contactPointWS - 
						m_chassisBody->getCenterOfMassPosition();

				if (m_forwardImpulse[wheel] != D_btScalar(0.))
				{
					m_chassisBody->applyImpulse(m_forwardWS[wheel]*(m_forwardImpulse[wheel]),rel_pos);
				}
				if (m_sideImpulse[wheel] != D_btScalar(0.))
				{
					class D_btRigidBody* groundObject = (class D_btRigidBody*) m_wheelInfo[wheel].m_raycastInfo.m_groundObject;

					D_btVector3 rel_pos2 = wheelInfo.m_raycastInfo.m_contactPointWS - 
						groundObject->getCenterOfMassPosition();

					
					D_btVector3 sideImp = m_axle[wheel] * m_sideImpulse[wheel];

					rel_pos[m_indexUpAxis] *= wheelInfo.m_rollInfluence;
					m_chassisBody->applyImpulse(sideImp,rel_pos);

					//apply friction impulse on the ground
					groundObject->applyImpulse(-sideImp,rel_pos2);
				}
			}
		}

	
}



void	D_btRaycastVehicle::debugDraw(D_btIDebugDraw* debugDrawer)
{

	for (int v=0;v<this->getNumWheels();v++)
	{
		D_btVector3 wheelColor(0,255,255);
		if (getWheelInfo(v).m_raycastInfo.m_isInContact)
		{
			wheelColor.setValue(0,0,255);
		} else
		{
			wheelColor.setValue(255,0,255);
		}

		D_btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();

		D_btVector3 axle = D_btVector3(	
			getWheelInfo(v).m_worldTransform.getBasis()[0][getRightAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[1][getRightAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[2][getRightAxis()]);

		//D_debug wheels (cylinders)
		debugDrawer->drawLine(wheelPosWS,wheelPosWS+axle,wheelColor);
		debugDrawer->drawLine(wheelPosWS,getWheelInfo(v).m_raycastInfo.m_contactPointWS,wheelColor);

	}
}


void* D_btDefaultVehicleRaycaster::castRay(const D_btVector3& from,const D_btVector3& D_to, D_btVehicleRaycasterResult& result)
{
//	RayResultCallback& resultCallback;

	D_btCollisionWorld::ClosestRayResultCallback rayCallback(from,D_to);

	m_dynamicsWorld->rayTest(from, D_to, rayCallback);

	if (rayCallback.hasHit())
	{
		
		D_btRigidBody* body = D_btRigidBody::upcast(rayCallback.m_collisionObject);
        if (body && body->hasContactResponse())
		{
			result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
			result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
			result.m_hitNormalInWorld.normalize();
			result.m_distFraction = rayCallback.m_closestHitFraction;
			return body;
		}
	}
	return 0;
}

