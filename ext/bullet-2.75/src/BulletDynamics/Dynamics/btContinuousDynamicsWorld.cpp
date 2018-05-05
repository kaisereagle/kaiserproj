/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btContinuousDynamicsWorld.h"
#include "LinearMath/btQuickprof.h"

//collision detection
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

//rigidbody & constraints
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"



#include <stdio.h>

D_btContinuousDynamicsWorld::D_btContinuousDynamicsWorld(D_btDispatcher* dispatcher,D_btBroadphaseInterface* pairCache,D_btConstraintSolver* constraintSolver,D_btCollisionConfiguration* collisionConfiguration)
:D_btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration)
{
}

D_btContinuousDynamicsWorld::~D_btContinuousDynamicsWorld()
{
}

	
void	D_btContinuousDynamicsWorld::internalSingleStepSimulation( D_btScalar timeStep)
{
	
	startProfiling(timeStep);
	
	if(0 != m_internalPreTickCallback) {
		(*m_internalPreTickCallback)(this, timeStep);
	}


	///update aabbs information
	updateAabbs();
	//static int frame=0;
//	printf("frame %d\n",frame++);

	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	D_btDispatcherInfo& dispatchInfo = getDispatchInfo();

	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_debugDraw = getDebugDrawer();

	///perform collision detection
	performDiscreteCollisionDetection();

	calculateSimulationIslands();

	
	getSolverInfo().m_timeStep = timeStep;
	


	///solve contact D_and other joint constraints
	solveConstraints(getSolverInfo());
	
	///CallbackTriggers();
	calculateTimeOfImpacts(timeStep);

	D_btScalar toi = dispatchInfo.m_timeOfImpact;
//	if (toi < 1.f)
//		printf("toi = %f\n",toi);
	if (toi < 0.f)
		printf("toi = %f\n",toi);


	///integrate transforms
	integrateTransforms(timeStep * toi);

	///update vehicle simulation
	updateActions(timeStep);

	updateActivationState( timeStep );
	
	if(0 != m_internalTickCallback) {
		(*m_internalTickCallback)(this, timeStep);
	}
}

void	D_btContinuousDynamicsWorld::calculateTimeOfImpacts(D_btScalar timeStep)
{
		///these D_should be 'temporal' aabbs!
		updateTemporalAabbs(timeStep);
		
		///'toi' D_is the global smallest time of impact. However, we D_just calculate the time of impact for each object individually.
		///so we handle the case moving versus static properly, D_and we cheat for moving versus moving
		D_btScalar toi = 1.f;
		
	
		D_btDispatcherInfo& dispatchInfo = getDispatchInfo();
		dispatchInfo.m_timeStep = timeStep;
		dispatchInfo.m_timeOfImpact = 1.f;
		dispatchInfo.m_stepCount = 0;
		dispatchInfo.m_dispatchFunc = D_btDispatcherInfo::D_DISPATCH_CONTINUOUS;

		///calculate time of impact for overlapping pairs


		D_btDispatcher* dispatcher = getDispatcher();
		if (dispatcher)
			dispatcher->dispatchAllCollisionPairs(m_broadphasePairCache->getOverlappingPairCache(),dispatchInfo,m_dispatcher1);

		toi = dispatchInfo.m_timeOfImpact;

		dispatchInfo.m_dispatchFunc = D_btDispatcherInfo::D_DISPATCH_DISCRETE;

}

void	D_btContinuousDynamicsWorld::updateTemporalAabbs(D_btScalar timeStep)
{

	D_btVector3 temporalAabbMin,temporalAabbMax;

	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject* colObj = m_collisionObjects[i];
		
		D_btRigidBody* body = D_btRigidBody::upcast(colObj);
		if (body)
		{
			body->getCollisionShape()->getAabb(m_collisionObjects[i]->getWorldTransform(),temporalAabbMin,temporalAabbMax);
			const D_btVector3& linvel = body->getLinearVelocity();

			//make the AABB temporal
			D_btScalar temporalAabbMaxx = temporalAabbMax.getX();
			D_btScalar temporalAabbMaxy = temporalAabbMax.getY();
			D_btScalar temporalAabbMaxz = temporalAabbMax.getZ();
			D_btScalar temporalAabbMinx = temporalAabbMin.getX();
			D_btScalar temporalAabbMiny = temporalAabbMin.getY();
			D_btScalar temporalAabbMinz = temporalAabbMin.getZ();

			// add linear motion
			D_btVector3 linMotion = linvel*timeStep;
		
			if (linMotion.x() > 0.f)
				temporalAabbMaxx += linMotion.x(); 
			else
				temporalAabbMinx += linMotion.x();
			if (linMotion.y() > 0.f)
				temporalAabbMaxy += linMotion.y(); 
			else
				temporalAabbMiny += linMotion.y();
			if (linMotion.z() > 0.f)
				temporalAabbMaxz += linMotion.z(); 
			else
				temporalAabbMinz += linMotion.z();

			//add conservative angular motion
			D_btScalar angularMotion(0);// = angvel.length() * GetAngularMotionDisc() * timeStep;
			D_btVector3 angularMotion3d(angularMotion,angularMotion,angularMotion);
			temporalAabbMin = D_btVector3(temporalAabbMinx,temporalAabbMiny,temporalAabbMinz);
			temporalAabbMax = D_btVector3(temporalAabbMaxx,temporalAabbMaxy,temporalAabbMaxz);

			temporalAabbMin -= angularMotion3d;
			temporalAabbMax += angularMotion3d;

			m_broadphasePairCache->setAabb(body->getBroadphaseHandle(),temporalAabbMin,temporalAabbMax,m_dispatcher1);
		}
	}

	//update aabb (of all moved objects)

	m_broadphasePairCache->calculateOverlappingPairs(m_dispatcher1);
	


}



