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
#ifndef RAYCASTVEHICLE_H
#define RAYCASTVEHICLE_H

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "btVehicleRaycaster.h"
class D_btDynamicsWorld;
#include "LinearMath/btAlignedObjectArray.h"
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"

class D_btVehicleTuning;

///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
class D_btRaycastVehicle : public D_btActionInterface
{

		D_btAlignedObjectArray<D_btVector3>	m_forwardWS;
		D_btAlignedObjectArray<D_btVector3>	m_axle;
		D_btAlignedObjectArray<D_btScalar>	m_forwardImpulse;
		D_btAlignedObjectArray<D_btScalar>	m_sideImpulse;

public:
	class D_btVehicleTuning
		{
			public:

			D_btVehicleTuning()
				:m_suspensionStiffness(D_btScalar(5.88)),
				m_suspensionCompression(D_btScalar(0.83)),
				m_suspensionDamping(D_btScalar(0.88)),
				m_maxSuspensionTravelCm(D_btScalar(500.)),
				m_frictionSlip(D_btScalar(10.5))
			{
			}
			D_btScalar	m_suspensionStiffness;
			D_btScalar	m_suspensionCompression;
			D_btScalar	m_suspensionDamping;
			D_btScalar	m_maxSuspensionTravelCm;
			D_btScalar	m_frictionSlip;

		};
private:

	D_btScalar	m_tau;
	D_btScalar	m_damping;
	D_btVehicleRaycaster*	m_vehicleRaycaster;
	D_btScalar		m_pitchControl;
	D_btScalar	m_steeringValue; 
	D_btScalar m_currentVehicleSpeedKmHour;

	D_btRigidBody* m_chassisBody;

	int m_indexRightAxis;
	int m_indexUpAxis;
	int	m_indexForwardAxis;

	void defaultInit(const D_btVehicleTuning& tuning);

public:

	//constructor D_to create a car from an existing rigidbody
	D_btRaycastVehicle(const D_btVehicleTuning& tuning,D_btRigidBody* chassis,	D_btVehicleRaycaster* raycaster );

	virtual ~D_btRaycastVehicle() ;


	///D_btActionInterface interface
	virtual void updateAction( D_btCollisionWorld* collisionWorld, D_btScalar step)
	{
		updateVehicle(step);
	}
	

	///D_btActionInterface interface
	void	debugDraw(D_btIDebugDraw* debugDrawer);
			
	const D_btTransform& getChassisWorldTransform() const;
	
	D_btScalar rayCast(D_btWheelInfo& wheel);

	virtual void updateVehicle(D_btScalar step);
	
	
	void resetSuspension();

	D_btScalar	getSteeringValue(int wheel) const;

	void	setSteeringValue(D_btScalar steering,int wheel);


	void	applyEngineForce(D_btScalar force, int wheel);

	const D_btTransform&	getWheelTransformWS( int wheelIndex ) const;

	void	updateWheelTransform( int wheelIndex, bool interpolatedTransform = true );
	
	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, const D_btVector3& hitPoint, const D_btVector3& hitNormal,D_btScalar depth);

	D_btWheelInfo&	addWheel( const D_btVector3& connectionPointCS0, const D_btVector3& wheelDirectionCS0,const D_btVector3& wheelAxleCS,D_btScalar suspensionRestLength,D_btScalar wheelRadius,const D_btVehicleTuning& tuning, bool isFrontWheel);

	inline int		getNumWheels() const {
		return int (m_wheelInfo.size());
	}
	
	D_btAlignedObjectArray<D_btWheelInfo>	m_wheelInfo;


	const D_btWheelInfo&	getWheelInfo(int index) const;

	D_btWheelInfo&	getWheelInfo(int index);

	void	updateWheelTransformsWS(D_btWheelInfo& wheel , bool interpolatedTransform = true);

	
	void setBrake(D_btScalar brake,int wheelIndex);

	void	setPitchControl(D_btScalar pitch)
	{
		m_pitchControl = pitch;
	}
	
	void	updateSuspension(D_btScalar deltaTime);

	virtual void	updateFriction(D_btScalar	timeStep);



	inline D_btRigidBody* getRigidBody()
	{
		return m_chassisBody;
	}

	const D_btRigidBody* getRigidBody() const
	{
		return m_chassisBody;
	}

	inline int	getRightAxis() const
	{
		return m_indexRightAxis;
	}
	inline int getUpAxis() const
	{
		return m_indexUpAxis;
	}

	inline int getForwardAxis() const
	{
		return m_indexForwardAxis;
	}

	
	///Worldspace forward vector
	D_btVector3 getForwardVector() const
	{
		const D_btTransform& chassisTrans = getChassisWorldTransform(); 

		D_btVector3 forwardW ( 
			  chassisTrans.getBasis()[0][m_indexForwardAxis], 
			  chassisTrans.getBasis()[1][m_indexForwardAxis], 
			  chassisTrans.getBasis()[2][m_indexForwardAxis]); 

		return forwardW;
	}

	///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
	D_btScalar	getCurrentSpeedKmHour() const
	{
		return m_currentVehicleSpeedKmHour;
	}

	virtual void	setCoordinateSystem(int rightIndex,int upIndex,int forwardIndex)
	{
		m_indexRightAxis = rightIndex;
		m_indexUpAxis = upIndex;
		m_indexForwardAxis = forwardIndex;
	}



};

class D_btDefaultVehicleRaycaster : public D_btVehicleRaycaster
{
	D_btDynamicsWorld*	m_dynamicsWorld;
public:
	D_btDefaultVehicleRaycaster(D_btDynamicsWorld* world)
		:m_dynamicsWorld(world)
	{
	}

	virtual void* castRay(const D_btVector3& from,const D_btVector3& D_to, D_btVehicleRaycasterResult& result);

};


#endif //RAYCASTVEHICLE_H

