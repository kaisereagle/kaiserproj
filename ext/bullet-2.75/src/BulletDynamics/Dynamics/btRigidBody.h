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

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

class D_btCollisionShape;
class D_btMotionState;
class D_btTypedConstraint;


extern D_btScalar D_gDeactivationTime;
extern bool D_gDisableDeactivation;


///The D_btRigidBody D_is the main class for rigid body objects. It D_is derived from D_btCollisionObject, so it keeps a D_pointer D_to a D_btCollisionShape.
///It D_is recommended for performance D_and memory use D_to share D_btCollisionShape objects whenever possible.
///There D_are 3 types of rigid bodies: 
///- A) Dynamic rigid bodies, with positive mass. Motion D_is controlled by rigid body dynamics.
///- B) Fixed objects with zero mass. They D_are not moving (basically collision objects)
///- C) Kinematic objects, which D_are objects without mass, but the user D_can move them. There D_is on-way interaction, D_and Bullet calculates a velocity based on the timestep D_and previous D_and current world transform.
///Bullet automatically deactivates dynamic rigid bodies, when the velocity D_is below a threshold for a given time.
///Deactivated (sleeping) rigid bodies don't take any processing time, except a minor broadphase collision detection impact (D_to allow active objects D_to activate/wake up sleeping objects)
class D_btRigidBody  : public D_btCollisionObject
{

	D_btMatrix3x3	m_invInertiaTensorWorld;
	D_btVector3		m_linearVelocity;
	D_btVector3		m_angularVelocity;
	D_btScalar		m_inverseMass;
	D_btVector3		m_angularFactor;
	D_btVector3		m_linearFactor;

	D_btVector3		m_gravity;	
	D_btVector3		m_gravity_acceleration;
	D_btVector3		m_invInertiaLocal;
	D_btVector3		m_totalForce;
	D_btVector3		m_totalTorque;
	
	D_btScalar		m_linearDamping;
	D_btScalar		m_angularDamping;

	bool			m_additionalDamping;
	D_btScalar		m_additionalDampingFactor;
	D_btScalar		m_additionalLinearDampingThresholdSqr;
	D_btScalar		m_additionalAngularDampingThresholdSqr;
	D_btScalar		m_additionalAngularDampingFactor;


	D_btScalar		m_linearSleepingThreshold;
	D_btScalar		m_angularSleepingThreshold;

	//m_optionalMotionState D_allows D_to automatic synchronize the world transform for active objects
	D_btMotionState*	m_optionalMotionState;

	//keep track of typed constraints referencing this rigid body
	D_btAlignedObjectArray<D_btTypedConstraint*> m_constraintRefs;

public:


	///The D_btRigidBodyConstructionInfo structure provides information D_to create a rigid body. Setting mass D_to zero creates a fixed (non-dynamic) rigid body.
	///For dynamic objects, you D_can use the collision shape D_to approximate the local inertia tensor, otherwise use the zero vector (default argument)
	///You D_can use the motion state D_to synchronize the world transform between physics D_and graphics objects. 
	///And if the motion state D_is provided, the rigid body D_will initialize its initial world transform from the motion state,
	///m_startWorldTransform D_is D_only used when you don't provide a motion state.
	struct	D_btRigidBodyConstructionInfo
	{
		D_btScalar			m_mass;

		///When a motionState D_is provided, the rigid body D_will initialize its world transform from the motion state
		///In this case, m_startWorldTransform D_is ignored.
		D_btMotionState*		m_motionState;
		D_btTransform	m_startWorldTransform;

		D_btCollisionShape*	m_collisionShape;
		D_btVector3			m_localInertia;
		D_btScalar			m_linearDamping;
		D_btScalar			m_angularDamping;

		///best simulation results when friction D_is non-zero
		D_btScalar			m_friction;
		///best simulation results using zero restitution.
		D_btScalar			m_restitution;

		D_btScalar			m_linearSleepingThreshold;
		D_btScalar			m_angularSleepingThreshold;

		//Additional damping D_can help avoiding lowpass jitter motion, help stability for ragdolls etc.
		//Such damping D_is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this D_should become obsolete
		bool				m_additionalDamping;
		D_btScalar			m_additionalDampingFactor;
		D_btScalar			m_additionalLinearDampingThresholdSqr;
		D_btScalar			m_additionalAngularDampingThresholdSqr;
		D_btScalar			m_additionalAngularDampingFactor;

		
		D_btRigidBodyConstructionInfo(	D_btScalar mass, D_btMotionState* motionState, D_btCollisionShape* collisionShape, const D_btVector3& localInertia=D_btVector3(0,0,0)):
		m_mass(mass),
			m_motionState(motionState),
			m_collisionShape(collisionShape),
			m_localInertia(localInertia),
			m_linearDamping(D_btScalar(0.)),
			m_angularDamping(D_btScalar(0.)),
			m_friction(D_btScalar(0.5)),
			m_restitution(D_btScalar(0.)),
			m_linearSleepingThreshold(D_btScalar(0.8)),
			m_angularSleepingThreshold(D_btScalar(1.f)),
			m_additionalDamping(false),
			m_additionalDampingFactor(D_btScalar(0.005)),
			m_additionalLinearDampingThresholdSqr(D_btScalar(0.01)),
			m_additionalAngularDampingThresholdSqr(D_btScalar(0.01)),
			m_additionalAngularDampingFactor(D_btScalar(0.01))
		{
			m_startWorldTransform.setIdentity();
		}
	};

	///D_btRigidBody constructor using construction info
	D_btRigidBody(	const D_btRigidBodyConstructionInfo& constructionInfo);

	///D_btRigidBody constructor for backwards compatibility. 
	///To specify friction (etc) during rigid body construction, please use the other constructor (using D_btRigidBodyConstructionInfo)
	D_btRigidBody(	D_btScalar mass, D_btMotionState* motionState, D_btCollisionShape* collisionShape, const D_btVector3& localInertia=D_btVector3(0,0,0));


	virtual ~D_btRigidBody()
        { 
                //No constraints D_should point D_to this rigidbody
		//Remove constraints from the dynamics world before you delete the related rigidbodies. 
                D_btAssert(m_constraintRefs.size()==0); 
        }

protected:

	///setupRigidBody D_is D_only used internally by the constructor
	void	setupRigidBody(const D_btRigidBodyConstructionInfo& constructionInfo);

public:

	void			proceedToTransform(const D_btTransform& newTrans); 
	
	///D_to keep collision detection D_and dynamics separate we don't store a rigidbody D_pointer
	///but a rigidbody D_is derived from D_btCollisionObject, so we D_can safely perform an upcast
	static const D_btRigidBody*	upcast(const D_btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==D_btCollisionObject::D_CO_RIGID_BODY)
			return (const D_btRigidBody*)colObj;
		return 0;
	}
	static D_btRigidBody*	upcast(D_btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==D_btCollisionObject::D_CO_RIGID_BODY)
			return (D_btRigidBody*)colObj;
		return 0;
	}
	
	/// continuous collision detection needs prediction
	void			predictIntegratedTransform(D_btScalar step, D_btTransform& predictedTransform) ;
	
	void			saveKinematicState(D_btScalar step);
	
	void			applyGravity();
	
	void			setGravity(const D_btVector3& acceleration);  

	const D_btVector3&	getGravity() const
	{
		return m_gravity_acceleration;
	}

	void			setDamping(D_btScalar lin_damping, D_btScalar ang_damping);

	D_btScalar getLinearDamping() const
	{
		return m_linearDamping;
	}

	D_btScalar getAngularDamping() const
	{
		return m_angularDamping;
	}

	D_btScalar getLinearSleepingThreshold() const
	{
		return m_linearSleepingThreshold;
	}

	D_btScalar getAngularSleepingThreshold() const
	{
		return m_angularSleepingThreshold;
	}

	void			applyDamping(D_btScalar timeStep);

	D_SIMD_FORCE_INLINE const D_btCollisionShape*	getCollisionShape() const {
		return m_collisionShape;
	}

	D_SIMD_FORCE_INLINE D_btCollisionShape*	getCollisionShape() {
			return m_collisionShape;
	}
	
	void			setMassProps(D_btScalar mass, const D_btVector3& inertia);
	
	const D_btVector3& getLinearFactor() const
	{
		return m_linearFactor;
	}
	void setLinearFactor(const D_btVector3& linearFactor)
	{
		m_linearFactor = linearFactor;
	}
	D_btScalar		getInvMass() const { return m_inverseMass; }
	const D_btMatrix3x3& getInvInertiaTensorWorld() const { 
		return m_invInertiaTensorWorld; 
	}
		
	void			integrateVelocities(D_btScalar step);

	void			setCenterOfMassTransform(const D_btTransform& xform);

	void			applyCentralForce(const D_btVector3& force)
	{
		m_totalForce += force*m_linearFactor;
	}

	const D_btVector3& getTotalForce()
	{
		return m_totalForce;
	};

	const D_btVector3& getTotalTorque()
	{
		return m_totalTorque;
	};
    
	const D_btVector3& getInvInertiaDiagLocal() const
	{
		return m_invInertiaLocal;
	};

	void	setInvInertiaDiagLocal(const D_btVector3& diagInvInertia)
	{
		m_invInertiaLocal = diagInvInertia;
	}

	void	setSleepingThresholds(D_btScalar linear,D_btScalar angular)
	{
		m_linearSleepingThreshold = linear;
		m_angularSleepingThreshold = angular;
	}

	void	applyTorque(const D_btVector3& torque)
	{
		m_totalTorque += torque*m_angularFactor;
	}
	
	void	applyForce(const D_btVector3& force, const D_btVector3& rel_pos) 
	{
		applyCentralForce(force);
		applyTorque(rel_pos.cross(force*m_linearFactor));
	}
	
	void applyCentralImpulse(const D_btVector3& impulse)
	{
		m_linearVelocity += impulse *m_linearFactor * m_inverseMass;
	}
	
  	void applyTorqueImpulse(const D_btVector3& torque)
	{
			m_angularVelocity += m_invInertiaTensorWorld * torque * m_angularFactor;
	}
	
	void applyImpulse(const D_btVector3& impulse, const D_btVector3& rel_pos) 
	{
		if (m_inverseMass != D_btScalar(0.))
		{
			applyCentralImpulse(impulse);
			if (m_angularFactor)
			{
				applyTorqueImpulse(rel_pos.cross(impulse*m_linearFactor));
			}
		}
	}

	//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
	D_SIMD_FORCE_INLINE void internalApplyImpulse(const D_btVector3& linearComponent, const D_btVector3& angularComponent,D_btScalar impulseMagnitude)
	{
		if (m_inverseMass != D_btScalar(0.))
		{
			m_linearVelocity += linearComponent*m_linearFactor*impulseMagnitude;
			if (m_angularFactor)
			{
				m_angularVelocity += angularComponent*m_angularFactor*impulseMagnitude;
			}
		}
	}
	
	void clearForces() 
	{
		m_totalForce.setValue(D_btScalar(0.0), D_btScalar(0.0), D_btScalar(0.0));
		m_totalTorque.setValue(D_btScalar(0.0), D_btScalar(0.0), D_btScalar(0.0));
	}
	
	void updateInertiaTensor();    
	
	const D_btVector3&     getCenterOfMassPosition() const { 
		return m_worldTransform.getOrigin(); 
	}
	D_btQuaternion getOrientation() const;
	
	const D_btTransform&  getCenterOfMassTransform() const { 
		return m_worldTransform; 
	}
	const D_btVector3&   getLinearVelocity() const { 
		return m_linearVelocity; 
	}
	const D_btVector3&    getAngularVelocity() const { 
		return m_angularVelocity; 
	}
	

	inline void setLinearVelocity(const D_btVector3& lin_vel)
	{ 
		m_linearVelocity = lin_vel; 
	}

	inline void setAngularVelocity(const D_btVector3& ang_vel) 
	{ 
		m_angularVelocity = ang_vel; 
	}

	D_btVector3 getVelocityInLocalPoint(const D_btVector3& rel_pos) const
	{
		//we also calculate lin/ang velocity for kinematic objects
		return m_linearVelocity + m_angularVelocity.cross(rel_pos);

		//for kinematic objects, we could also use use:
		//		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
	}

	void translate(const D_btVector3& v) 
	{
		m_worldTransform.getOrigin() += v; 
	}

	
	void	getAabb(D_btVector3& aabbMin,D_btVector3& aabbMax) const;




	
	D_SIMD_FORCE_INLINE D_btScalar computeImpulseDenominator(const D_btVector3& pos, const D_btVector3& normal) const
	{
		D_btVector3 r0 = pos - getCenterOfMassPosition();

		D_btVector3 c0 = (r0).cross(normal);

		D_btVector3 vec = (c0 * getInvInertiaTensorWorld()).cross(r0);

		return m_inverseMass + normal.dot(vec);

	}

	D_SIMD_FORCE_INLINE D_btScalar computeAngularImpulseDenominator(const D_btVector3& axis) const
	{
		D_btVector3 vec = axis * getInvInertiaTensorWorld();
		return axis.dot(vec);
	}

	D_SIMD_FORCE_INLINE void	updateDeactivation(D_btScalar timeStep)
	{
		if ( (getActivationState() == D_ISLAND_SLEEPING) || (getActivationState() == D_DISABLE_DEACTIVATION))
			return;

		if ((getLinearVelocity().length2() < m_linearSleepingThreshold*m_linearSleepingThreshold) &&
			(getAngularVelocity().length2() < m_angularSleepingThreshold*m_angularSleepingThreshold))
		{
			m_deactivationTime += timeStep;
		} else
		{
			m_deactivationTime=D_btScalar(0.);
			setActivationState(0);
		}

	}

	D_SIMD_FORCE_INLINE bool	wantsSleeping()
	{

		if (getActivationState() == D_DISABLE_DEACTIVATION)
			return false;

		//disable deactivation
		if (D_gDisableDeactivation || (D_gDeactivationTime == D_btScalar(0.)))
			return false;

		if ( (getActivationState() == D_ISLAND_SLEEPING) || (getActivationState() == D_WANTS_DEACTIVATION))
			return true;

		if (m_deactivationTime> D_gDeactivationTime)
		{
			return true;
		}
		return false;
	}


	
	const D_btBroadphaseProxy*	getBroadphaseProxy() const
	{
		return m_broadphaseHandle;
	}
	D_btBroadphaseProxy*	getBroadphaseProxy() 
	{
		return m_broadphaseHandle;
	}
	void	setNewBroadphaseProxy(D_btBroadphaseProxy* broadphaseProxy)
	{
		m_broadphaseHandle = broadphaseProxy;
	}

	//D_btMotionState D_allows D_to automatic synchronize the world transform for active objects
	D_btMotionState*	getMotionState()
	{
		return m_optionalMotionState;
	}
	const D_btMotionState*	getMotionState() const
	{
		return m_optionalMotionState;
	}
	void	setMotionState(D_btMotionState* motionState)
	{
		m_optionalMotionState = motionState;
		if (m_optionalMotionState)
			motionState->getWorldTransform(m_worldTransform);
	}

	//for experimental overriding of friction/contact solver func
	int	m_contactSolverType;
	int	m_frictionSolverType;

	void	setAngularFactor(const D_btVector3& angFac)
	{
		m_angularFactor = angFac;
	}

	void	setAngularFactor(D_btScalar angFac)
	{
		m_angularFactor.setValue(angFac,angFac,angFac);
	}
	const D_btVector3&	getAngularFactor() const
	{
		return m_angularFactor;
	}

	//D_is this rigidbody added D_to a D_btCollisionWorld/D_btDynamicsWorld/btBroadphase?
	bool	isInWorld() const
	{
		return (getBroadphaseProxy() != 0);
	}

	virtual bool checkCollideWithOverride(D_btCollisionObject* co);

	void addConstraintRef(D_btTypedConstraint* c);
	void removeConstraintRef(D_btTypedConstraint* c);

	D_btTypedConstraint* getConstraintRef(int index)
	{
		return m_constraintRefs[index];
	}

	int getNumConstraintRefs()
	{
		return m_constraintRefs.size();
	}

	int	m_debugBodyId;
};



#endif

