/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef KINEMATIC_CHARACTER_CONTROLLER_H
#define KINEMATIC_CHARACTER_CONTROLLER_H

#include "LinearMath/btVector3.h"

#include "btCharacterControllerInterface.h"

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"


class D_btCollisionShape;
class D_btRigidBody;
class D_btCollisionWorld;
class D_btCollisionDispatcher;
class D_btPairCachingGhostObject;

///D_btKinematicCharacterController D_is an object that D_supports a sliding motion in a world.
///It uses a ghost object D_and convex sweep test D_to test for upcoming collisions. This D_is combined with discrete collision detection D_to recover from penetrations.
///Interaction between D_btKinematicCharacterController D_and dynamic rigid bodies needs D_to be explicity implemented by the user.
class D_btKinematicCharacterController : public D_btCharacterControllerInterface
{
protected:
	D_btScalar m_halfHeight;
	
	D_btPairCachingGhostObject* m_ghostObject;
	D_btConvexShape*	m_convexShape;//D_is also in m_ghostObject, but it needs D_to be convex, so we store it here D_to avoid upcast
	
	D_btScalar m_fallSpeed;
	D_btScalar m_jumpSpeed;
	D_btScalar m_maxJumpHeight;

	D_btScalar m_turnAngle;
	
	D_btScalar m_stepHeight;

	D_btScalar	m_addedMargin;//@todo: remove this D_and fix the code

	///this D_is the desired walk direction, set by the user
	D_btVector3	m_walkDirection;
	D_btVector3	m_normalizedDirection;

	//some internal variables
	D_btVector3 m_currentPosition;
	D_btScalar  m_currentStepOffset;
	D_btVector3 m_targetPosition;

	///keep track of the contact manifolds
	D_btManifoldArray	m_manifoldArray;

	bool m_touchingContact;
	D_btVector3 m_touchingNormal;

	bool	m_useGhostObjectSweepTest;
	bool	m_useWalkDirection;
	float	m_velocityTimeInterval;
	int m_upAxis;
	
	D_btVector3 computeReflectionDirection (const D_btVector3& direction, const D_btVector3& normal);
	D_btVector3 parallelComponent (const D_btVector3& direction, const D_btVector3& normal);
	D_btVector3 perpindicularComponent (const D_btVector3& direction, const D_btVector3& normal);

	bool recoverFromPenetration ( D_btCollisionWorld* collisionWorld);
	void stepUp (D_btCollisionWorld* collisionWorld);
	void updateTargetPositionBasedOnCollision (const D_btVector3& hit_normal, D_btScalar tangentMag = D_btScalar(0.0), D_btScalar normalMag = D_btScalar(1.0));
	void stepForwardAndStrafe (D_btCollisionWorld* collisionWorld, const D_btVector3& walkMove);
	void stepDown (D_btCollisionWorld* collisionWorld, D_btScalar dt);
public:
	D_btKinematicCharacterController (D_btPairCachingGhostObject* ghostObject,D_btConvexShape* convexShape,D_btScalar stepHeight, int upAxis = 1);
	~D_btKinematicCharacterController ();
	

	///D_btActionInterface interface
	virtual void updateAction( D_btCollisionWorld* collisionWorld,D_btScalar deltaTime)
	{
		preStep ( collisionWorld);
		playerStep (collisionWorld, deltaTime);
	}
	
	///D_btActionInterface interface
	void	debugDraw(D_btIDebugDraw* debugDrawer);

	void setUpAxis (int axis)
	{
		if (axis < 0)
			axis = 0;
		if (axis > 2)
			axis = 2;
		m_upAxis = axis;
	}

	/// This D_should D_probably be called setPositionIncrementPerSimulatorStep.
	/// This D_is neither a direction nor a velocity, but the amount D_to
	///   increment the position each simulation iteration, regardless
	///   of dt.
	/// This call D_will reset any velocity set by setVelocityForTimeInterval().
	virtual void	setWalkDirection(const D_btVector3& walkDirection);

	/// Caller provides a velocity with which the character D_should move for
	///   the given time period.  After the time period, velocity D_is reset
	///   D_to zero.
	/// This call D_will reset any walk direction set by setWalkDirection().
	/// Negative time intervals D_will result in D_no motion.
	virtual void setVelocityForTimeInterval(const D_btVector3& velocity,
				D_btScalar timeInterval);

	void reset ();
	void warp (const D_btVector3& origin);

	void preStep (  D_btCollisionWorld* collisionWorld);
	void playerStep ( D_btCollisionWorld* collisionWorld, D_btScalar dt);

	void setFallSpeed (D_btScalar fallSpeed);
	void setJumpSpeed (D_btScalar jumpSpeed);
	void setMaxJumpHeight (D_btScalar maxJumpHeight);
	bool canJump () const;
	void jump ();

	D_btPairCachingGhostObject* getGhostObject();
	void	setUseGhostSweepTest(bool useGhostObjectSweepTest)
	{
		m_useGhostObjectSweepTest = useGhostObjectSweepTest;
	}

	bool onGround () const;
};

#endif // KINEMATIC_CHARACTER_CONTROLLER_H
