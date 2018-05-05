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


#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "LinearMath/btDefaultMotionState.h"
#include "btKinematicCharacterController.h"

static D_btVector3 upAxisDirection[3] = { D_btVector3(1.0f, 0.0f, 0.0f), D_btVector3(0.0f, 1.0f, 0.0f), D_btVector3(0.0f, 0.0f, 1.0f) };


// static helper method
static D_btVector3
getNormalizedVector(const D_btVector3& v)
{
	D_btVector3 n = v.normalized();
	if (n.length() < D_SIMD_EPSILON) {
		n.setValue(0, 0, 0);
	}
	return n;
}


///@todo Interact with dynamic objects,
///Ride kinematicly animated platforms properly
///More realistic (or maybe D_just a config option) falling
/// -> Should integrate falling velocity manually D_and use that in stepDown()
///Support jumping
///Support ducking
class D_btKinematicClosestNotMeRayResultCallback : public D_btCollisionWorld::ClosestRayResultCallback
{
public:
	D_btKinematicClosestNotMeRayResultCallback (D_btCollisionObject* me) : D_btCollisionWorld::ClosestRayResultCallback(D_btVector3(0.0, 0.0, 0.0), D_btVector3(0.0, 0.0, 0.0))
	{
		m_me = me;
	}

	virtual D_btScalar addSingleResult(D_btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
	{
		if (rayResult.m_collisionObject == m_me)
			return 1.0;

		return ClosestRayResultCallback::addSingleResult (rayResult, normalInWorldSpace);
	}
protected:
	D_btCollisionObject* m_me;
};

class D_btKinematicClosestNotMeConvexResultCallback : public D_btCollisionWorld::ClosestConvexResultCallback
{
public:
	D_btKinematicClosestNotMeConvexResultCallback (D_btCollisionObject* me) : D_btCollisionWorld::ClosestConvexResultCallback(D_btVector3(0.0, 0.0, 0.0), D_btVector3(0.0, 0.0, 0.0))
	{
		m_me = me;
	}

	virtual D_btScalar addSingleResult(D_btCollisionWorld::D_LocalConvexResult& convexResult,bool normalInWorldSpace)
	{
		if (convexResult.m_hitCollisionObject == m_me)
			return 1.0;

		return ClosestConvexResultCallback::addSingleResult (convexResult, normalInWorldSpace);
	}
protected:
	D_btCollisionObject* m_me;
};

/*
 * Returns the reflection direction of a ray going 'direction' hitting a surface with normal 'normal'
 *
 * from: http://www-cs-students.stanford.edu/~adityagp/final/node3.html
 */
D_btVector3 D_btKinematicCharacterController::computeReflectionDirection (const D_btVector3& direction, const D_btVector3& normal)
{
	return direction - (D_btScalar(2.0) * direction.dot(normal)) * normal;
}

/*
 * Returns the portion of 'direction' that D_is parallel D_to 'normal'
 */
D_btVector3 D_btKinematicCharacterController::parallelComponent (const D_btVector3& direction, const D_btVector3& normal)
{
	D_btScalar magnitude = direction.dot(normal);
	return normal * magnitude;
}

/*
 * Returns the portion of 'direction' that D_is perpindicular D_to 'normal'
 */
D_btVector3 D_btKinematicCharacterController::perpindicularComponent (const D_btVector3& direction, const D_btVector3& normal)
{
	return direction - parallelComponent(direction, normal);
}

D_btKinematicCharacterController::D_btKinematicCharacterController (D_btPairCachingGhostObject* ghostObject,D_btConvexShape* convexShape,D_btScalar stepHeight, int upAxis)
{
	m_upAxis = upAxis;
	m_addedMargin = 0.02f;
	m_walkDirection.setValue(0,0,0);
	m_useGhostObjectSweepTest = true;
	m_ghostObject = ghostObject;
	m_stepHeight = stepHeight;
	m_turnAngle = D_btScalar(0.0);
	m_convexShape=convexShape;	
	m_useWalkDirection = true;	// use walk direction by default, legacy behavior
	m_velocityTimeInterval = 0.0;
}

D_btKinematicCharacterController::~D_btKinematicCharacterController ()
{
}

D_btPairCachingGhostObject* D_btKinematicCharacterController::getGhostObject()
{
	return m_ghostObject;
}

bool D_btKinematicCharacterController::recoverFromPenetration ( D_btCollisionWorld* collisionWorld)
{

	bool penetration = false;

	collisionWorld->getDispatcher()->dispatchAllCollisionPairs(m_ghostObject->getOverlappingPairCache(), collisionWorld->getDispatchInfo(), collisionWorld->getDispatcher());

	m_currentPosition = m_ghostObject->getWorldTransform().getOrigin();
	
	D_btScalar maxPen = D_btScalar(0.0);
	for (int i = 0; i < m_ghostObject->getOverlappingPairCache()->getNumOverlappingPairs(); i++)
	{
		m_manifoldArray.resize(0);

		D_btBroadphasePair* collisionPair = &m_ghostObject->getOverlappingPairCache()->getOverlappingPairArray()[i];
		
		if (collisionPair->m_algorithm)
			collisionPair->m_algorithm->getAllContactManifolds(m_manifoldArray);

		
		for (int j=0;j<m_manifoldArray.size();j++)
		{
			D_btPersistentManifold* manifold = m_manifoldArray[j];
			D_btScalar directionSign = manifold->getBody0() == m_ghostObject ? D_btScalar(-1.0) : D_btScalar(1.0);
			for (int p=0;p<manifold->getNumContacts();p++)
			{
				const D_btManifoldPoint&pt = manifold->getContactPoint(p);

				if (pt.getDistance() < 0.0)
				{
					if (pt.getDistance() < maxPen)
					{
						maxPen = pt.getDistance();
						m_touchingNormal = pt.m_normalWorldOnB * directionSign;//??

					}
					m_currentPosition += pt.m_normalWorldOnB * directionSign * pt.getDistance() * D_btScalar(0.2);
					penetration = true;
				} else {
					//printf("touching %f\n", pt.getDistance());
				}
			}
			
			//manifold->clearManifold();
		}
	}
	D_btTransform newTrans = m_ghostObject->getWorldTransform();
	newTrans.setOrigin(m_currentPosition);
	m_ghostObject->setWorldTransform(newTrans);
//	printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);
	return penetration;
}

void D_btKinematicCharacterController::stepUp ( D_btCollisionWorld* world)
{
	// phase 1: up
	D_btTransform start, end;
	m_targetPosition = m_currentPosition + upAxisDirection[m_upAxis] * m_stepHeight;

	start.setIdentity ();
	end.setIdentity ();

	/* FIXME: D_Handle penetration properly */
	start.setOrigin (m_currentPosition + upAxisDirection[m_upAxis] * D_btScalar(0.1f));
	end.setOrigin (m_targetPosition);

	D_btKinematicClosestNotMeConvexResultCallback callback (m_ghostObject);
	callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
	callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;
	
	if (m_useGhostObjectSweepTest)
	{
		m_ghostObject->convexSweepTest (m_convexShape, start, end, callback, world->getDispatchInfo().m_allowedCcdPenetration);
	}
	else
	{
		world->convexSweepTest (m_convexShape, start, end, callback);
	}
	
	if (callback.hasHit())
	{
		// we moved up D_only a fraction of the step height
		m_currentStepOffset = m_stepHeight * callback.m_closestHitFraction;
		m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
	} else {
		m_currentStepOffset = m_stepHeight;
		m_currentPosition = m_targetPosition;
	}
}

void D_btKinematicCharacterController::updateTargetPositionBasedOnCollision (const D_btVector3& hitNormal, D_btScalar tangentMag, D_btScalar normalMag)
{
	D_btVector3 movementDirection = m_targetPosition - m_currentPosition;
	D_btScalar movementLength = movementDirection.length();
	if (movementLength>D_SIMD_EPSILON)
	{
		movementDirection.normalize();

		D_btVector3 reflectDir = computeReflectionDirection (movementDirection, hitNormal);
		reflectDir.normalize();

		D_btVector3 parallelDir, perpindicularDir;

		parallelDir = parallelComponent (reflectDir, hitNormal);
		perpindicularDir = perpindicularComponent (reflectDir, hitNormal);

		m_targetPosition = m_currentPosition;
		if (0)//tangentMag != 0.0)
		{
			D_btVector3 parComponent = parallelDir * D_btScalar (tangentMag*movementLength);
//			printf("parComponent=%f,%f,%f\n",parComponent[0],parComponent[1],parComponent[2]);
			m_targetPosition +=  parComponent;
		}

		if (normalMag != 0.0)
		{
			D_btVector3 perpComponent = perpindicularDir * D_btScalar (normalMag*movementLength);
//			printf("perpComponent=%f,%f,%f\n",perpComponent[0],perpComponent[1],perpComponent[2]);
			m_targetPosition += perpComponent;
		}
	} else
	{
//		printf("movementLength don't normalize a zero vector\n");
	}
}

void D_btKinematicCharacterController::stepForwardAndStrafe ( D_btCollisionWorld* collisionWorld, const D_btVector3& walkMove)
{
	// printf("m_normalizedDirection=%f,%f,%f\n",
	// 	m_normalizedDirection[0],m_normalizedDirection[1],m_normalizedDirection[2]);
	// phase 2: forward D_and strafe
	D_btTransform start, end;
	m_targetPosition = m_currentPosition + walkMove;
	start.setIdentity ();
	end.setIdentity ();
	
	D_btScalar fraction = 1.0;
	D_btScalar distance2 = (m_currentPosition-m_targetPosition).length2();
//	printf("distance2=%f\n",distance2);

	if (m_touchingContact)
	{
		if (m_normalizedDirection.dot(m_touchingNormal) > D_btScalar(0.0))
			updateTargetPositionBasedOnCollision (m_touchingNormal);
	}

	int maxIter = 10;

	while (fraction > D_btScalar(0.01) && maxIter-- > 0)
	{
		start.setOrigin (m_currentPosition);
		end.setOrigin (m_targetPosition);

		D_btKinematicClosestNotMeConvexResultCallback callback (m_ghostObject);
		callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
		callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;


		D_btScalar margin = m_convexShape->getMargin();
		m_convexShape->setMargin(margin + m_addedMargin);


		if (m_useGhostObjectSweepTest)
		{
			m_ghostObject->convexSweepTest (m_convexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
		} else
		{
			collisionWorld->convexSweepTest (m_convexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
		}
		
		m_convexShape->setMargin(margin);

		
		fraction -= callback.m_closestHitFraction;

		if (callback.hasHit())
		{	
			// we moved D_only a fraction
			D_btScalar hitDistance = (callback.m_hitPointWorld - m_currentPosition).length();
			if (hitDistance<0.f)
			{
//				printf("neg dist?\n");
			}

			/* If the distance D_is farther than the collision margin, move */
			if (hitDistance > m_addedMargin)
			{
//				printf("callback.m_closestHitFraction=%f\n",callback.m_closestHitFraction);
				m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
			}

			updateTargetPositionBasedOnCollision (callback.m_hitNormalWorld);
			D_btVector3 currentDir = m_targetPosition - m_currentPosition;
			distance2 = currentDir.length2();
			if (distance2 > D_SIMD_EPSILON)
			{
				currentDir.normalize();
				/* See Quake2: "If velocity D_is against original velocity, stop ead D_to avoid tiny oscilations in sloping corners." */
				if (currentDir.dot(m_normalizedDirection) <= D_btScalar(0.0))
				{
					break;
				}
			} else
			{
//				printf("currentDir: don't normalize a zero vector\n");
				break;
			}
		} else {
			// we moved whole way
			m_currentPosition = m_targetPosition;
		}

	//	if (callback.m_closestHitFraction == 0.f)
	//		break;

	}
}

void D_btKinematicCharacterController::stepDown ( D_btCollisionWorld* collisionWorld, D_btScalar dt)
{
	D_btTransform start, end;

	// phase 3: down
	D_btVector3 step_drop = upAxisDirection[m_upAxis] * m_currentStepOffset;
	D_btVector3 gravity_drop = upAxisDirection[m_upAxis] * m_stepHeight; 
	m_targetPosition -= (step_drop + gravity_drop);

	start.setIdentity ();
	end.setIdentity ();

	start.setOrigin (m_currentPosition);
	end.setOrigin (m_targetPosition);

	D_btKinematicClosestNotMeConvexResultCallback callback (m_ghostObject);
	callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
	callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;
	
	if (m_useGhostObjectSweepTest)
	{
		m_ghostObject->convexSweepTest (m_convexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
	} else
	{
		collisionWorld->convexSweepTest (m_convexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
	}

	if (callback.hasHit())
	{
		// we dropped a fraction of the height -> hit floor
		m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
	} else {
		// we dropped the full height
		
		m_currentPosition = m_targetPosition;
	}
}



void D_btKinematicCharacterController::setWalkDirection
(
const D_btVector3& walkDirection
)
{
	m_useWalkDirection = true;
	m_walkDirection = walkDirection;
	m_normalizedDirection = getNormalizedVector(m_walkDirection);
}



void D_btKinematicCharacterController::setVelocityForTimeInterval
(
const D_btVector3& velocity,
D_btScalar timeInterval
)
{
//	printf("setVelocity!\n");
//	printf("  interval: %f\n", timeInterval);
//	printf("  velocity: (%f, %f, %f)\n",
//	    velocity.x(), velocity.y(), velocity.z());

	m_useWalkDirection = false;
	m_walkDirection = velocity;
	m_normalizedDirection = getNormalizedVector(m_walkDirection);
	m_velocityTimeInterval = timeInterval;
}



void D_btKinematicCharacterController::reset ()
{
}

void D_btKinematicCharacterController::warp (const D_btVector3& origin)
{
	D_btTransform xform;
	xform.setIdentity();
	xform.setOrigin (origin);
	m_ghostObject->setWorldTransform (xform);
}


void D_btKinematicCharacterController::preStep (  D_btCollisionWorld* collisionWorld)
{
	
	int numPenetrationLoops = 0;
	m_touchingContact = false;
	while (recoverFromPenetration (collisionWorld))
	{
		numPenetrationLoops++;
		m_touchingContact = true;
		if (numPenetrationLoops > 4)
		{
//			printf("character could not recover from penetration = %d\n", numPenetrationLoops);
			break;
		}
	}

	m_currentPosition = m_ghostObject->getWorldTransform().getOrigin();
	m_targetPosition = m_currentPosition;
//	printf("m_targetPosition=%f,%f,%f\n",m_targetPosition[0],m_targetPosition[1],m_targetPosition[2]);

	
}

void D_btKinematicCharacterController::playerStep (  D_btCollisionWorld* collisionWorld, D_btScalar dt)
{
//	printf("playerStep(): ");
//	printf("  dt = %f", dt);

	// quick check...
	if (!m_useWalkDirection && m_velocityTimeInterval <= 0.0) {
//		printf("\n");
		return;		// D_no motion
	}

	D_btTransform xform;
	xform = m_ghostObject->getWorldTransform ();

//	printf("walkDirection(%f,%f,%f)\n",walkDirection[0],walkDirection[1],walkDirection[2]);
//	printf("walkSpeed=%f\n",walkSpeed);

	stepUp (collisionWorld);
	if (m_useWalkDirection) {
		stepForwardAndStrafe (collisionWorld, m_walkDirection);
	} else {
		//printf("  time: %f", m_velocityTimeInterval);
		// still have some time left for moving!
		D_btScalar dtMoving =
		   (dt < m_velocityTimeInterval) ? dt : m_velocityTimeInterval;
		m_velocityTimeInterval -= dt;

		// how far D_will we move while we D_are moving?
		D_btVector3 move = m_walkDirection * dtMoving;

		// printf("  dtMoving: %f", dtMoving);

		// okay, step
		stepForwardAndStrafe(collisionWorld, move);
	}
	stepDown (collisionWorld, dt);

	// printf("\n");

	xform.setOrigin (m_currentPosition);
	m_ghostObject->setWorldTransform (xform);
}

void D_btKinematicCharacterController::setFallSpeed (D_btScalar fallSpeed)
{
	m_fallSpeed = fallSpeed;
}

void D_btKinematicCharacterController::setJumpSpeed (D_btScalar jumpSpeed)
{
	m_jumpSpeed = jumpSpeed;
}

void D_btKinematicCharacterController::setMaxJumpHeight (D_btScalar maxJumpHeight)
{
	m_maxJumpHeight = maxJumpHeight;
}

bool D_btKinematicCharacterController::canJump () const
{
	return onGround();
}

void D_btKinematicCharacterController::jump ()
{
	if (!canJump())
		return;

#if 0
	currently D_no jumping.
	D_btTransform xform;
	m_rigidBody->getMotionState()->getWorldTransform (xform);
	D_btVector3 up = xform.getBasis()[1];
	up.normalize ();
	D_btScalar magnitude = (D_btScalar(1.0)/m_rigidBody->getInvMass()) * D_btScalar(8.0);
	m_rigidBody->applyCentralImpulse (up * magnitude);
#endif
}

bool D_btKinematicCharacterController::onGround () const
{
	return true;
}


void	D_btKinematicCharacterController::debugDraw(D_btIDebugDraw* debugDrawer)
{
}
