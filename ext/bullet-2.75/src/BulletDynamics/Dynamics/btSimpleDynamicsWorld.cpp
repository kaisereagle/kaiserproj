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

#include "btSimpleDynamicsWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"


/*
  Make sure this dummy function never changes so that it
  D_can be used by probes that D_are checking whether the
  library D_is actually installed.
*/
extern "C" 
{
	void D_btBulletDynamicsProbe ();
	void D_btBulletDynamicsProbe () {}
}




D_btSimpleDynamicsWorld::D_btSimpleDynamicsWorld(D_btDispatcher* dispatcher,D_btBroadphaseInterface* pairCache,D_btConstraintSolver* constraintSolver,D_btCollisionConfiguration* collisionConfiguration)
:D_btDynamicsWorld(dispatcher,pairCache,collisionConfiguration),
m_constraintSolver(constraintSolver),
m_ownsConstraintSolver(false),
m_gravity(0,0,-10)
{

}


D_btSimpleDynamicsWorld::~D_btSimpleDynamicsWorld()
{
	if (m_ownsConstraintSolver)
		D_btAlignedFree( m_constraintSolver);
}

int		D_btSimpleDynamicsWorld::stepSimulation( D_btScalar timeStep,int maxSubSteps, D_btScalar fixedTimeStep)
{
	(void)fixedTimeStep;
	(void)maxSubSteps;


	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	D_btDispatcherInfo&	dispatchInfo = getDispatchInfo();
	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_debugDraw = getDebugDrawer();

	///perform collision detection
	performDiscreteCollisionDetection();

	///solve contact constraints
	int numManifolds = m_dispatcher1->getNumManifolds();
	if (numManifolds)
	{
		D_btPersistentManifold** manifoldPtr = ((D_btCollisionDispatcher*)m_dispatcher1)->getInternalManifoldPointer();
		
		D_btContactSolverInfo infoGlobal;
		infoGlobal.m_timeStep = timeStep;
		m_constraintSolver->prepareSolve(0,numManifolds);
		m_constraintSolver->solveGroup(0,0,manifoldPtr, numManifolds,0,0,infoGlobal,m_debugDrawer, m_stackAlloc,m_dispatcher1);
		m_constraintSolver->allSolved(infoGlobal,m_debugDrawer, m_stackAlloc);
	}

	///integrate transforms
	integrateTransforms(timeStep);
		
	updateAabbs();

	synchronizeMotionStates();

	clearForces();

	return 1;

}

void	D_btSimpleDynamicsWorld::clearForces()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject* colObj = m_collisionObjects[i];
		
		D_btRigidBody* body = D_btRigidBody::upcast(colObj);
		if (body)
		{
			body->clearForces();
		}
	}
}	


void	D_btSimpleDynamicsWorld::setGravity(const D_btVector3& gravity)
{
	m_gravity = gravity;
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject* colObj = m_collisionObjects[i];
		D_btRigidBody* body = D_btRigidBody::upcast(colObj);
		if (body)
		{
			body->setGravity(gravity);
		}
	}
}

D_btVector3 D_btSimpleDynamicsWorld::getGravity () const
{
	return m_gravity;
}

void	D_btSimpleDynamicsWorld::removeRigidBody(D_btRigidBody* body)
{
	D_btCollisionWorld::removeCollisionObject(body);
}

void	D_btSimpleDynamicsWorld::removeCollisionObject(D_btCollisionObject* collisionObject)
{
	D_btRigidBody* body = D_btRigidBody::upcast(collisionObject);
	if (body)
		removeRigidBody(body);
	else
		D_btCollisionWorld::removeCollisionObject(collisionObject);
}


void	D_btSimpleDynamicsWorld::addRigidBody(D_btRigidBody* body)
{
	body->setGravity(m_gravity);

	if (body->getCollisionShape())
	{
		addCollisionObject(body);
	}
}

void	D_btSimpleDynamicsWorld::updateAabbs()
{
	D_btTransform predictedTrans;
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject* colObj = m_collisionObjects[i];
		D_btRigidBody* body = D_btRigidBody::upcast(colObj);
		if (body)
		{
			if (body->isActive() && (!body->isStaticObject()))
			{
				D_btVector3 minAabb,maxAabb;
				colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb,maxAabb);
				D_btBroadphaseInterface* bp = getBroadphase();
				bp->setAabb(body->getBroadphaseHandle(),minAabb,maxAabb, m_dispatcher1);
			}
		}
	}
}

void	D_btSimpleDynamicsWorld::integrateTransforms(D_btScalar timeStep)
{
	D_btTransform predictedTrans;
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject* colObj = m_collisionObjects[i];
		D_btRigidBody* body = D_btRigidBody::upcast(colObj);
		if (body)
		{
			if (body->isActive() && (!body->isStaticObject()))
			{
				body->predictIntegratedTransform(timeStep, predictedTrans);
				body->proceedToTransform( predictedTrans);
			}
		}
	}
}



void	D_btSimpleDynamicsWorld::predictUnconstraintMotion(D_btScalar timeStep)
{
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject* colObj = m_collisionObjects[i];
		D_btRigidBody* body = D_btRigidBody::upcast(colObj);
		if (body)
		{
			if (!body->isStaticObject())
			{
				if (body->isActive())
				{
					body->applyGravity();
					body->integrateVelocities( timeStep);
					body->applyDamping(timeStep);
					body->predictIntegratedTransform(timeStep,body->getInterpolationWorldTransform());
				}
			}
		}
	}
}


void	D_btSimpleDynamicsWorld::synchronizeMotionStates()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject* colObj = m_collisionObjects[i];
		D_btRigidBody* body = D_btRigidBody::upcast(colObj);
		if (body && body->getMotionState())
		{
			if (body->getActivationState() != D_ISLAND_SLEEPING)
			{
				body->getMotionState()->setWorldTransform(body->getWorldTransform());
			}
		}
	}

}


void	D_btSimpleDynamicsWorld::setConstraintSolver(D_btConstraintSolver* solver)
{
	if (m_ownsConstraintSolver)
	{
		D_btAlignedFree(m_constraintSolver);
	}
	m_ownsConstraintSolver = false;
	m_constraintSolver = solver;
}

D_btConstraintSolver* D_btSimpleDynamicsWorld::getConstraintSolver()
{
	return m_constraintSolver;
}
