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

#include "btRigidBody.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btMotionState.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"

//'temporarily' global variables
D_btScalar	D_gDeactivationTime = D_btScalar(2.);
bool	D_gDisableDeactivation = false;
static int uniqueId = 0;


D_btRigidBody::D_btRigidBody(const D_btRigidBody::D_btRigidBodyConstructionInfo& constructionInfo)
{
	setupRigidBody(constructionInfo);
}

D_btRigidBody::D_btRigidBody(D_btScalar mass, D_btMotionState *motionState, D_btCollisionShape *collisionShape, const D_btVector3 &localInertia)
{
	D_btRigidBodyConstructionInfo cinfo(mass,motionState,collisionShape,localInertia);
	setupRigidBody(cinfo);
}

void	D_btRigidBody::setupRigidBody(const D_btRigidBody::D_btRigidBodyConstructionInfo& constructionInfo)
{

	m_internalType=D_CO_RIGID_BODY;

	m_linearVelocity.setValue(D_btScalar(0.0), D_btScalar(0.0), D_btScalar(0.0));
	m_angularVelocity.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	m_angularFactor.setValue(1,1,1);
	m_linearFactor.setValue(1,1,1);
	m_gravity.setValue(D_btScalar(0.0), D_btScalar(0.0), D_btScalar(0.0));
	m_gravity_acceleration.setValue(D_btScalar(0.0), D_btScalar(0.0), D_btScalar(0.0));
	m_totalForce.setValue(D_btScalar(0.0), D_btScalar(0.0), D_btScalar(0.0));
	m_totalTorque.setValue(D_btScalar(0.0), D_btScalar(0.0), D_btScalar(0.0)),
	m_linearDamping = D_btScalar(0.);
	m_angularDamping = D_btScalar(0.5);
	m_linearSleepingThreshold = constructionInfo.m_linearSleepingThreshold;
	m_angularSleepingThreshold = constructionInfo.m_angularSleepingThreshold;
	m_optionalMotionState = constructionInfo.m_motionState;
	m_contactSolverType = 0;
	m_frictionSolverType = 0;
	m_additionalDamping = constructionInfo.m_additionalDamping;
	m_additionalDampingFactor = constructionInfo.m_additionalDampingFactor;
	m_additionalLinearDampingThresholdSqr = constructionInfo.m_additionalLinearDampingThresholdSqr;
	m_additionalAngularDampingThresholdSqr = constructionInfo.m_additionalAngularDampingThresholdSqr;
	m_additionalAngularDampingFactor = constructionInfo.m_additionalAngularDampingFactor;

	if (m_optionalMotionState)
	{
		m_optionalMotionState->getWorldTransform(m_worldTransform);
	} else
	{
		m_worldTransform = constructionInfo.m_startWorldTransform;
	}

	m_interpolationWorldTransform = m_worldTransform;
	m_interpolationLinearVelocity.setValue(0,0,0);
	m_interpolationAngularVelocity.setValue(0,0,0);
	
	//moved D_to D_btCollisionObject
	m_friction = constructionInfo.m_friction;
	m_restitution = constructionInfo.m_restitution;

	setCollisionShape( constructionInfo.m_collisionShape );
	m_debugBodyId = uniqueId++;
	
	setMassProps(constructionInfo.m_mass, constructionInfo.m_localInertia);
    setDamping(constructionInfo.m_linearDamping, constructionInfo.m_angularDamping);
	updateInertiaTensor();

}


void D_btRigidBody::predictIntegratedTransform(D_btScalar timeStep,D_btTransform& predictedTransform) 
{
	D_btTransformUtil::integrateTransform(m_worldTransform,m_linearVelocity,m_angularVelocity,timeStep,predictedTransform);
}

void			D_btRigidBody::saveKinematicState(D_btScalar timeStep)
{
	//todo: clamp D_to some (user definable) safe minimum timestep, D_to limit maximum angular/linear velocities
	if (timeStep != D_btScalar(0.))
	{
		//if we use motionstate D_to synchronize world transforms, get the new kinematic/animated world transform
		if (getMotionState())
			getMotionState()->getWorldTransform(m_worldTransform);
		D_btVector3 linVel,angVel;
		
		D_btTransformUtil::calculateVelocity(m_interpolationWorldTransform,m_worldTransform,timeStep,m_linearVelocity,m_angularVelocity);
		m_interpolationLinearVelocity = m_linearVelocity;
		m_interpolationAngularVelocity = m_angularVelocity;
		m_interpolationWorldTransform = m_worldTransform;
		//printf("angular = %f %f %f\n",m_angularVelocity.getX(),m_angularVelocity.getY(),m_angularVelocity.getZ());
	}
}
	
void	D_btRigidBody::getAabb(D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	getCollisionShape()->getAabb(m_worldTransform,aabbMin,aabbMax);
}




void D_btRigidBody::setGravity(const D_btVector3& acceleration) 
{
	if (m_inverseMass != D_btScalar(0.0))
	{
		m_gravity = acceleration * (D_btScalar(1.0) / m_inverseMass);
	}
	m_gravity_acceleration = acceleration;
}






void D_btRigidBody::setDamping(D_btScalar lin_damping, D_btScalar ang_damping)
{
	m_linearDamping = D_GEN_clamped(lin_damping, (D_btScalar)D_btScalar(0.0), (D_btScalar)D_btScalar(1.0));
	m_angularDamping = D_GEN_clamped(ang_damping, (D_btScalar)D_btScalar(0.0), (D_btScalar)D_btScalar(1.0));
}




///applyDamping damps the velocity, using the given m_linearDamping D_and m_angularDamping
void			D_btRigidBody::applyDamping(D_btScalar timeStep)
{
	//On new damping: see discussion/D_issue report here: http://code.google.com/p/bullet/issues/detail?id=74
	//todo: do some performance comparisons (but other parts of the engine D_are D_probably bottleneck anyway

//#define USE_OLD_DAMPING_METHOD 1
#ifdef USE_OLD_DAMPING_METHOD
	m_linearVelocity *= D_GEN_clamped((D_btScalar(1.) - timeStep * m_linearDamping), (D_btScalar)D_btScalar(0.0), (D_btScalar)D_btScalar(1.0));
	m_angularVelocity *= D_GEN_clamped((D_btScalar(1.) - timeStep * m_angularDamping), (D_btScalar)D_btScalar(0.0), (D_btScalar)D_btScalar(1.0));
#else
	m_linearVelocity *= D_btPow(D_btScalar(1)-m_linearDamping, timeStep);
	m_angularVelocity *= D_btPow(D_btScalar(1)-m_angularDamping, timeStep);
#endif

	if (m_additionalDamping)
	{
		//Additional damping D_can help avoiding lowpass jitter motion, help stability for ragdolls etc.
		//Such damping D_is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this D_should become obsolete
		if ((m_angularVelocity.length2() < m_additionalAngularDampingThresholdSqr) &&
			(m_linearVelocity.length2() < m_additionalLinearDampingThresholdSqr))
		{
			m_angularVelocity *= m_additionalDampingFactor;
			m_linearVelocity *= m_additionalDampingFactor;
		}
	

		D_btScalar speed = m_linearVelocity.length();
		if (speed < m_linearDamping)
		{
			D_btScalar dampVel = D_btScalar(0.005);
			if (speed > dampVel)
			{
				D_btVector3 dir = m_linearVelocity.normalized();
				m_linearVelocity -=  dir * dampVel;
			} else
			{
				m_linearVelocity.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
			}
		}

		D_btScalar angSpeed = m_angularVelocity.length();
		if (angSpeed < m_angularDamping)
		{
			D_btScalar angDampVel = D_btScalar(0.005);
			if (angSpeed > angDampVel)
			{
				D_btVector3 dir = m_angularVelocity.normalized();
				m_angularVelocity -=  dir * angDampVel;
			} else
			{
				m_angularVelocity.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
			}
		}
	}
}


void D_btRigidBody::applyGravity()
{
	if (isStaticOrKinematicObject())
		return;
	
	applyCentralForce(m_gravity);	

}

void D_btRigidBody::proceedToTransform(const D_btTransform& newTrans)
{
	setCenterOfMassTransform( newTrans );
}
	

void D_btRigidBody::setMassProps(D_btScalar mass, const D_btVector3& inertia)
{
	if (mass == D_btScalar(0.))
	{
		m_collisionFlags |= D_btCollisionObject::D_CF_STATIC_OBJECT;
		m_inverseMass = D_btScalar(0.);
	} else
	{
		m_collisionFlags &= (~D_btCollisionObject::D_CF_STATIC_OBJECT);
		m_inverseMass = D_btScalar(1.0) / mass;
	}
	
	m_invInertiaLocal.setValue(inertia.x() != D_btScalar(0.0) ? D_btScalar(1.0) / inertia.x(): D_btScalar(0.0),
				   inertia.y() != D_btScalar(0.0) ? D_btScalar(1.0) / inertia.y(): D_btScalar(0.0),
				   inertia.z() != D_btScalar(0.0) ? D_btScalar(1.0) / inertia.z(): D_btScalar(0.0));

}

	

void D_btRigidBody::updateInertiaTensor() 
{
	m_invInertiaTensorWorld = m_worldTransform.getBasis().scaled(m_invInertiaLocal) * m_worldTransform.getBasis().transpose();
}


void D_btRigidBody::integrateVelocities(D_btScalar step) 
{
	if (isStaticOrKinematicObject())
		return;

	m_linearVelocity += m_totalForce * (m_inverseMass * step);
	m_angularVelocity += m_invInertiaTensorWorld * m_totalTorque * step;

#define D_MAX_ANGVEL D_SIMD_HALF_PI
	/// clamp angular velocity. collision calculations D_will fail on higher angular velocities	
	D_btScalar angvel = m_angularVelocity.length();
	if (angvel*step > D_MAX_ANGVEL)
	{
		m_angularVelocity *= (D_MAX_ANGVEL/step) /angvel;
	}

}

D_btQuaternion D_btRigidBody::getOrientation() const
{
		D_btQuaternion orn;
		m_worldTransform.getBasis().getRotation(orn);
		return orn;
}
	
	
void D_btRigidBody::setCenterOfMassTransform(const D_btTransform& xform)
{

	if (isStaticOrKinematicObject())
	{
		m_interpolationWorldTransform = m_worldTransform;
	} else
	{
		m_interpolationWorldTransform = xform;
	}
	m_interpolationLinearVelocity = getLinearVelocity();
	m_interpolationAngularVelocity = getAngularVelocity();
	m_worldTransform = xform;
	updateInertiaTensor();
}


bool D_btRigidBody::checkCollideWithOverride(D_btCollisionObject* co)
{
	D_btRigidBody* otherRb = D_btRigidBody::upcast(co);
	if (!otherRb)
		return true;

	for (int i = 0; i < m_constraintRefs.size(); ++i)
	{
		D_btTypedConstraint* c = m_constraintRefs[i];
		if (&c->getRigidBodyA() == otherRb || &c->getRigidBodyB() == otherRb)
			return false;
	}

	return true;
}

void D_btRigidBody::addConstraintRef(D_btTypedConstraint* c)
{
	int index = m_constraintRefs.findLinearSearch(c);
	if (index == m_constraintRefs.size())
		m_constraintRefs.push_back(c); 

	m_checkCollideWith = true;
}

void D_btRigidBody::removeConstraintRef(D_btTypedConstraint* c)
{
	m_constraintRefs.remove(c);
	m_checkCollideWith = m_constraintRefs.size() > 0;
}
