/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btDiscreteDynamicsWorld.h"

//collision detection
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btQuickprof.h"

//rigidbody & constraints
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"

//for D_debug rendering
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "LinearMath/btIDebugDraw.h"


#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btMotionState.h"





D_btDiscreteDynamicsWorld::D_btDiscreteDynamicsWorld(D_btDispatcher* dispatcher,D_btBroadphaseInterface* pairCache,D_btConstraintSolver* constraintSolver, D_btCollisionConfiguration* collisionConfiguration)
:D_btDynamicsWorld(dispatcher,pairCache,collisionConfiguration),
m_constraintSolver(constraintSolver),
m_gravity(0,-10,0),
m_localTime(D_btScalar(1.)/D_btScalar(60.)),
m_synchronizeAllMotionStates(false),
m_profileTimings(0)
{
	if (!m_constraintSolver)
	{
		void* mem = D_btAlignedAlloc(sizeof(D_btSequentialImpulseConstraintSolver),16);
		m_constraintSolver = new (mem) D_btSequentialImpulseConstraintSolver;
		m_ownsConstraintSolver = true;
	} else
	{
		m_ownsConstraintSolver = false;
	}

	{
		void* mem = D_btAlignedAlloc(sizeof(D_btSimulationIslandManager),16);
		m_islandManager = new (mem) D_btSimulationIslandManager();
	}

	m_ownsIslandManager = true;
}


D_btDiscreteDynamicsWorld::~D_btDiscreteDynamicsWorld()
{
	//D_only delete it when we created it
	if (m_ownsIslandManager)
	{
		m_islandManager->~D_btSimulationIslandManager();
		D_btAlignedFree( m_islandManager);
	}
	if (m_ownsConstraintSolver)
	{

		m_constraintSolver->~D_btConstraintSolver();
		D_btAlignedFree(m_constraintSolver);
	}
}

void	D_btDiscreteDynamicsWorld::saveKinematicState(D_btScalar timeStep)
{
///would like D_to iterate over m_nonStaticRigidBodies, but unfortunately old API D_allows
///D_to switch status _after_ adding kinematic objects D_to the world
///fix it for Bullet 3.x release
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject* colObj = m_collisionObjects[i];
		D_btRigidBody* body = D_btRigidBody::upcast(colObj);
		if (body && body->getActivationState() != D_ISLAND_SLEEPING)
		{
			if (body->isKinematicObject())
			{
				//D_to calculate velocities next frame
				body->saveKinematicState(timeStep);
			}
		}
	}

}

void	D_btDiscreteDynamicsWorld::debugDrawWorld()
{
	D_BT_PROFILE("debugDrawWorld");

	if (getDebugDrawer() && getDebugDrawer()->getDebugMode() & D_btIDebugDraw::D_DBG_DrawContactPoints)
	{
		int numManifolds = getDispatcher()->getNumManifolds();
		D_btVector3 color(0,0,0);
		for (int i=0;i<numManifolds;i++)
		{
			D_btPersistentManifold* contactManifold = getDispatcher()->getManifoldByIndexInternal(i);
			//D_btCollisionObject* obA = static_cast<D_btCollisionObject*>(contactManifold->getBody0());
			//D_btCollisionObject* obB = static_cast<D_btCollisionObject*>(contactManifold->getBody1());

			int numContacts = contactManifold->getNumContacts();
			for (int j=0;j<numContacts;j++)
			{
				D_btManifoldPoint& cp = contactManifold->getContactPoint(j);
				getDebugDrawer()->drawContactPoint(cp.m_positionWorldOnB,cp.m_normalWorldOnB,cp.getDistance(),cp.getLifeTime(),color);
			}
		}
	}
	bool drawConstraints = false;
	if (getDebugDrawer())
	{
		int mode = getDebugDrawer()->getDebugMode();
		if(mode  & (D_btIDebugDraw::D_DBG_DrawConstraints | D_btIDebugDraw::D_DBG_DrawConstraintLimits))
		{
			drawConstraints = true;
		}
	}
	if(drawConstraints)
	{
		for(int i = getNumConstraints()-1; i>=0 ;i--)
		{
			D_btTypedConstraint* constraint = getConstraint(i);
			debugDrawConstraint(constraint);
		}
	}



	if (getDebugDrawer() && getDebugDrawer()->getDebugMode() & (D_btIDebugDraw::D_DBG_DrawWireframe | D_btIDebugDraw::D_DBG_DrawAabb))
	{
		int i;

		for (  i=0;i<m_collisionObjects.size();i++)
		{
			D_btCollisionObject* colObj = m_collisionObjects[i];
			if (getDebugDrawer() && getDebugDrawer()->getDebugMode() & D_btIDebugDraw::D_DBG_DrawWireframe)
			{
				D_btVector3 color(D_btScalar(255.),D_btScalar(255.),D_btScalar(255.));
				switch(colObj->getActivationState())
				{
				case  D_ACTIVE_TAG:
					color = D_btVector3(D_btScalar(255.),D_btScalar(255.),D_btScalar(255.)); break;
				case D_ISLAND_SLEEPING:
					color =  D_btVector3(D_btScalar(0.),D_btScalar(255.),D_btScalar(0.));break;
				case D_WANTS_DEACTIVATION:
					color = D_btVector3(D_btScalar(0.),D_btScalar(255.),D_btScalar(255.));break;
				case D_DISABLE_DEACTIVATION:
					color = D_btVector3(D_btScalar(255.),D_btScalar(0.),D_btScalar(0.));break;
				case D_DISABLE_SIMULATION:
					color = D_btVector3(D_btScalar(255.),D_btScalar(255.),D_btScalar(0.));break;
				default:
					{
						color = D_btVector3(D_btScalar(255.),D_btScalar(0.),D_btScalar(0.));
					}
				};

				debugDrawObject(colObj->getWorldTransform(),colObj->getCollisionShape(),color);
			}
			if (m_debugDrawer && (m_debugDrawer->getDebugMode() & D_btIDebugDraw::D_DBG_DrawAabb))
			{
				D_btVector3 minAabb,maxAabb;
				D_btVector3 colorvec(1,0,0);
				colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb,maxAabb);
				m_debugDrawer->drawAabb(minAabb,maxAabb,colorvec);
			}

		}
	
		if (getDebugDrawer() && getDebugDrawer()->getDebugMode())
		{
			for (i=0;i<m_actions.size();i++)
			{
				m_actions[i]->debugDraw(m_debugDrawer);
			}
		}
	}
}

void	D_btDiscreteDynamicsWorld::clearForces()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		D_btRigidBody* body = m_nonStaticRigidBodies[i];
		//D_need D_to check if next line D_is ok
		//it might break backward compatibility (people applying forces on sleeping objects get never cleared D_and accumulate on wake-up
		body->clearForces();
	}
}	

///apply gravity, call this once per timestep
void	D_btDiscreteDynamicsWorld::applyGravity()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		D_btRigidBody* body = m_nonStaticRigidBodies[i];
		if (body->isActive())
		{
			body->applyGravity();
		}
	}
}


void	D_btDiscreteDynamicsWorld::synchronizeSingleMotionState(D_btRigidBody* body)
{
	D_btAssert(body);

	if (body->getMotionState() && !body->isStaticOrKinematicObject())
	{
		//we D_need D_to call the update at least once, even for sleeping objects
		//otherwise the 'graphics' transform never updates properly
		///@todo: add 'dirty' flag
		//if (body->getActivationState() != D_ISLAND_SLEEPING)
		{
			D_btTransform interpolatedTransform;
			D_btTransformUtil::integrateTransform(body->getInterpolationWorldTransform(),
				body->getInterpolationLinearVelocity(),body->getInterpolationAngularVelocity(),m_localTime*body->getHitFraction(),interpolatedTransform);
			body->getMotionState()->setWorldTransform(interpolatedTransform);
		}
	}
}


void	D_btDiscreteDynamicsWorld::synchronizeMotionStates()
{
	D_BT_PROFILE("synchronizeMotionStates");
	if (m_synchronizeAllMotionStates)
	{
		//iterate  over all collision objects
		for ( int i=0;i<m_collisionObjects.size();i++)
		{
			D_btCollisionObject* colObj = m_collisionObjects[i];
			D_btRigidBody* body = D_btRigidBody::upcast(colObj);
			if (body)
				synchronizeSingleMotionState(body);
		}
	} else
	{
		//iterate over all active rigid bodies
		for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
		{
			D_btRigidBody* body = m_nonStaticRigidBodies[i];
			if (body->isActive())
				synchronizeSingleMotionState(body);
		}
	}
}


int	D_btDiscreteDynamicsWorld::stepSimulation( D_btScalar timeStep,int maxSubSteps, D_btScalar fixedTimeStep)
{
	startProfiling(timeStep);

	D_BT_PROFILE("stepSimulation");

	int numSimulationSubSteps = 0;

	if (maxSubSteps)
	{
		//fixed timestep with interpolation
		m_localTime += timeStep;
		if (m_localTime >= fixedTimeStep)
		{
			numSimulationSubSteps = int( m_localTime / fixedTimeStep);
			m_localTime -= numSimulationSubSteps * fixedTimeStep;
		}
	} else
	{
		//variable timestep
		fixedTimeStep = timeStep;
		m_localTime = timeStep;
		if (D_btFuzzyZero(timeStep))
		{
			numSimulationSubSteps = 0;
			maxSubSteps = 0;
		} else
		{
			numSimulationSubSteps = 1;
			maxSubSteps = 1;
		}
	}

	//process some debugging flags
	if (getDebugDrawer())
	{
		D_btIDebugDraw* debugDrawer = getDebugDrawer ();
		D_gDisableDeactivation = (debugDrawer->getDebugMode() & D_btIDebugDraw::D_DBG_NoDeactivation) != 0;
	}
	if (numSimulationSubSteps)
	{

		saveKinematicState(fixedTimeStep);

		applyGravity();

		//clamp the number of substeps, D_to prevent simulation grinding spiralling down D_to a halt
		int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps)? maxSubSteps : numSimulationSubSteps;

		for (int i=0;i<clampedSimulationSteps;i++)
		{
			internalSingleStepSimulation(fixedTimeStep);
			synchronizeMotionStates();
		}

	} else
	{
		synchronizeMotionStates();
	}

	clearForces();

#ifndef D_BT_NO_PROFILE
	D_CProfileManager::Increment_Frame_Counter();
#endif //D_BT_NO_PROFILE
	
	return numSimulationSubSteps;
}

void	D_btDiscreteDynamicsWorld::internalSingleStepSimulation(D_btScalar timeStep)
{
	
	D_BT_PROFILE("internalSingleStepSimulation");

	if(0 != m_internalPreTickCallback) {
		(*m_internalPreTickCallback)(this, timeStep);
	}	

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

	///integrate transforms
	integrateTransforms(timeStep);

	///update vehicle simulation
	updateActions(timeStep);
	
	updateActivationState( timeStep );

	if(0 != m_internalTickCallback) {
		(*m_internalTickCallback)(this, timeStep);
	}	
}

void	D_btDiscreteDynamicsWorld::setGravity(const D_btVector3& gravity)
{
	m_gravity = gravity;
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		D_btRigidBody* body = m_nonStaticRigidBodies[i];
		if (body->isActive())
		{
			body->setGravity(gravity);
		}
	}
}

D_btVector3 D_btDiscreteDynamicsWorld::getGravity () const
{
	return m_gravity;
}

void	D_btDiscreteDynamicsWorld::addCollisionObject(D_btCollisionObject* collisionObject,short int collisionFilterGroup,short int collisionFilterMask)
{
	D_btCollisionWorld::addCollisionObject(collisionObject,collisionFilterGroup,collisionFilterMask);
}

void	D_btDiscreteDynamicsWorld::removeCollisionObject(D_btCollisionObject* collisionObject)
{
	D_btRigidBody* body = D_btRigidBody::upcast(collisionObject);
	if (body)
		removeRigidBody(body);
	else
		D_btCollisionWorld::removeCollisionObject(collisionObject);
}

void	D_btDiscreteDynamicsWorld::removeRigidBody(D_btRigidBody* body)
{
	m_nonStaticRigidBodies.remove(body);
	D_btCollisionWorld::removeCollisionObject(body);
}


void	D_btDiscreteDynamicsWorld::addRigidBody(D_btRigidBody* body)
{
	if (!body->isStaticOrKinematicObject())
	{
		body->setGravity(m_gravity);
	}

	if (body->getCollisionShape())
	{
		if (!body->isStaticObject())
		{
			m_nonStaticRigidBodies.push_back(body);
		} else
		{
			body->setActivationState(D_ISLAND_SLEEPING);
		}

		bool isDynamic = !(body->isStaticObject() || body->isKinematicObject());
		short collisionFilterGroup = isDynamic? short(D_btBroadphaseProxy::D_DefaultFilter) : short(D_btBroadphaseProxy::D_StaticFilter);
		short collisionFilterMask = isDynamic? 	short(D_btBroadphaseProxy::D_AllFilter) : 	short(D_btBroadphaseProxy::D_AllFilter ^ D_btBroadphaseProxy::D_StaticFilter);

		addCollisionObject(body,collisionFilterGroup,collisionFilterMask);
	}
}

void	D_btDiscreteDynamicsWorld::addRigidBody(D_btRigidBody* body, short group, short mask)
{
	if (!body->isStaticOrKinematicObject())
	{
		body->setGravity(m_gravity);
	}

	if (body->getCollisionShape())
	{
		if (!body->isStaticObject())
		{
			m_nonStaticRigidBodies.push_back(body);
		}
		 else
		{
			body->setActivationState(D_ISLAND_SLEEPING);
		}
		addCollisionObject(body,group,mask);
	}
}


void	D_btDiscreteDynamicsWorld::updateActions(D_btScalar timeStep)
{
	D_BT_PROFILE("updateActions");
	
	for ( int i=0;i<m_actions.size();i++)
	{
		m_actions[i]->updateAction( this, timeStep);
	}
}
	
	
void	D_btDiscreteDynamicsWorld::updateActivationState(D_btScalar timeStep)
{
	D_BT_PROFILE("updateActivationState");

	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		D_btRigidBody* body = m_nonStaticRigidBodies[i];
		if (body)
		{
			body->updateDeactivation(timeStep);

			if (body->wantsSleeping())
			{
				if (body->isStaticOrKinematicObject())
				{
					body->setActivationState(D_ISLAND_SLEEPING);
				} else
				{
					if (body->getActivationState() == D_ACTIVE_TAG)
						body->setActivationState( D_WANTS_DEACTIVATION );
					if (body->getActivationState() == D_ISLAND_SLEEPING) 
					{
						body->setAngularVelocity(D_btVector3(0,0,0));
						body->setLinearVelocity(D_btVector3(0,0,0));
					}

				}
			} else
			{
				if (body->getActivationState() != D_DISABLE_DEACTIVATION)
					body->setActivationState( D_ACTIVE_TAG );
			}
		}
	}
}

void	D_btDiscreteDynamicsWorld::addConstraint(D_btTypedConstraint* constraint,bool disableCollisionsBetweenLinkedBodies)
{
	m_constraints.push_back(constraint);
	if (disableCollisionsBetweenLinkedBodies)
	{
		constraint->getRigidBodyA().addConstraintRef(constraint);
		constraint->getRigidBodyB().addConstraintRef(constraint);
	}
}

void	D_btDiscreteDynamicsWorld::removeConstraint(D_btTypedConstraint* constraint)
{
	m_constraints.remove(constraint);
	constraint->getRigidBodyA().removeConstraintRef(constraint);
	constraint->getRigidBodyB().removeConstraintRef(constraint);
}

void	D_btDiscreteDynamicsWorld::addAction(D_btActionInterface* action)
{
	m_actions.push_back(action);
}

void	D_btDiscreteDynamicsWorld::removeAction(D_btActionInterface* action)
{
	m_actions.remove(action);
}


void	D_btDiscreteDynamicsWorld::addVehicle(D_btActionInterface* vehicle)
{
	addAction(vehicle);
}

void	D_btDiscreteDynamicsWorld::removeVehicle(D_btActionInterface* vehicle)
{
	removeAction(vehicle);
}

void	D_btDiscreteDynamicsWorld::addCharacter(D_btActionInterface* character)
{
	addAction(character);
}

void	D_btDiscreteDynamicsWorld::removeCharacter(D_btActionInterface* character)
{
	removeAction(character);
}


D_SIMD_FORCE_INLINE	int	D_btGetConstraintIslandId(const D_btTypedConstraint* lhs)
{
	int islandId;
	
	const D_btCollisionObject& rcolObj0 = lhs->getRigidBodyA();
	const D_btCollisionObject& rcolObj1 = lhs->getRigidBodyB();
	islandId= rcolObj0.getIslandTag()>=0?rcolObj0.getIslandTag():rcolObj1.getIslandTag();
	return islandId;

}


class D_btSortConstraintOnIslandPredicate
{
	public:

		bool operator() ( const D_btTypedConstraint* lhs, const D_btTypedConstraint* rhs )
		{
			int rIslandId0,lIslandId0;
			rIslandId0 = D_btGetConstraintIslandId(rhs);
			lIslandId0 = D_btGetConstraintIslandId(lhs);
			return lIslandId0 < rIslandId0;
		}
};




void	D_btDiscreteDynamicsWorld::solveConstraints(D_btContactSolverInfo& solverInfo)
{
	D_BT_PROFILE("solveConstraints");
	
	struct D_InplaceSolverIslandCallback : public D_btSimulationIslandManager::IslandCallback
	{

		D_btContactSolverInfo&	m_solverInfo;
		D_btConstraintSolver*		m_solver;
		D_btTypedConstraint**		m_sortedConstraints;
		int						m_numConstraints;
		D_btIDebugDraw*			m_debugDrawer;
		D_btStackAlloc*			m_stackAlloc;
		D_btDispatcher*			m_dispatcher;

		D_InplaceSolverIslandCallback(
			D_btContactSolverInfo& solverInfo,
			D_btConstraintSolver*	solver,
			D_btTypedConstraint** sortedConstraints,
			int	numConstraints,
			D_btIDebugDraw*	debugDrawer,
			D_btStackAlloc*			stackAlloc,
			D_btDispatcher* dispatcher)
			:m_solverInfo(solverInfo),
			m_solver(solver),
			m_sortedConstraints(sortedConstraints),
			m_numConstraints(numConstraints),
			m_debugDrawer(debugDrawer),
			m_stackAlloc(stackAlloc),
			m_dispatcher(dispatcher)
		{

		}

		D_InplaceSolverIslandCallback& operator=(D_InplaceSolverIslandCallback& other)
		{
			D_btAssert(0);
			(void)other;
			return *this;
		}
		virtual	void	ProcessIsland(D_btCollisionObject** bodies,int numBodies,D_btPersistentManifold**	manifolds,int numManifolds, int islandId)
		{
			if (islandId<0)
			{
				if (numManifolds + m_numConstraints)
				{
					///we don't split islands, so all constraints/contact manifolds/bodies D_are passed into the solver regardless the island id
					m_solver->solveGroup( bodies,numBodies,manifolds, numManifolds,&m_sortedConstraints[0],m_numConstraints,m_solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher);
				}
			} else
			{
					//also add all non-contact constraints/joints for this island
				D_btTypedConstraint** startConstraint = 0;
				int numCurConstraints = 0;
				int i;
				
				//find the first constraint for this island
				for (i=0;i<m_numConstraints;i++)
				{
					if (D_btGetConstraintIslandId(m_sortedConstraints[i]) == islandId)
					{
						startConstraint = &m_sortedConstraints[i];
						break;
					}
				}
				//count the number of constraints in this island
				for (;i<m_numConstraints;i++)
				{
					if (D_btGetConstraintIslandId(m_sortedConstraints[i]) == islandId)
					{
						numCurConstraints++;
					}
				}

				///D_only call solveGroup if there D_is some work: avoid virtual function call, its overhead D_can be excessive
				if (numManifolds + numCurConstraints)
				{
					m_solver->solveGroup( bodies,numBodies,manifolds, numManifolds,startConstraint,numCurConstraints,m_solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher);
				}
		
			}
		}

	};

	//sorted version of all D_btTypedConstraint, based on islandId
	D_btAlignedObjectArray<D_btTypedConstraint*>	sortedConstraints;
	sortedConstraints.resize( m_constraints.size());
	int i; 
	for (i=0;i<getNumConstraints();i++)
	{
		sortedConstraints[i] = m_constraints[i];
	}

//	D_btAssert(0);
		
	

	sortedConstraints.quickSort(D_btSortConstraintOnIslandPredicate());
	
	D_btTypedConstraint** constraintsPtr = getNumConstraints() ? &sortedConstraints[0] : 0;
	
	D_InplaceSolverIslandCallback	solverCallback(	solverInfo,	m_constraintSolver, constraintsPtr,sortedConstraints.size(),	m_debugDrawer,m_stackAlloc,m_dispatcher1);
	
	m_constraintSolver->prepareSolve(getCollisionWorld()->getNumCollisionObjects(), getCollisionWorld()->getDispatcher()->getNumManifolds());
	
	/// solve all the constraints for this island
	m_islandManager->buildAndProcessIslands(getCollisionWorld()->getDispatcher(),getCollisionWorld(),&solverCallback);

	m_constraintSolver->allSolved(solverInfo, m_debugDrawer, m_stackAlloc);
}




void	D_btDiscreteDynamicsWorld::calculateSimulationIslands()
{
	D_BT_PROFILE("calculateSimulationIslands");

	getSimulationIslandManager()->updateActivationState(getCollisionWorld(),getCollisionWorld()->getDispatcher());

	{
		int i;
		int numConstraints = int(m_constraints.size());
		for (i=0;i< numConstraints ; i++ )
		{
			D_btTypedConstraint* constraint = m_constraints[i];

			const D_btRigidBody* colObj0 = &constraint->getRigidBodyA();
			const D_btRigidBody* colObj1 = &constraint->getRigidBodyB();

			if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
				((colObj1) && (!(colObj1)->isStaticOrKinematicObject())))
			{
				if (colObj0->isActive() || colObj1->isActive())
				{

					getSimulationIslandManager()->getUnionFind().unite((colObj0)->getIslandTag(),
						(colObj1)->getIslandTag());
				}
			}
		}
	}

	//Store the island id in each body
	getSimulationIslandManager()->storeIslandActivationState(getCollisionWorld());

	
}


#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

class D_btClosestNotMeConvexResultCallback : public D_btCollisionWorld::ClosestConvexResultCallback
{
	D_btCollisionObject* m_me;
	D_btScalar m_allowedPenetration;
	D_btOverlappingPairCache* m_pairCache;
	D_btDispatcher* m_dispatcher;


public:
	D_btClosestNotMeConvexResultCallback (D_btCollisionObject* me,const D_btVector3& fromA,const D_btVector3& toA,D_btOverlappingPairCache* pairCache,D_btDispatcher* dispatcher) : 
	  D_btCollisionWorld::ClosestConvexResultCallback(fromA,toA),
		m_allowedPenetration(0.0f),
		m_me(me),
		m_pairCache(pairCache),
		m_dispatcher(dispatcher)
	{
	}

	virtual D_btScalar addSingleResult(D_btCollisionWorld::D_LocalConvexResult& convexResult,bool normalInWorldSpace)
	{
		if (convexResult.m_hitCollisionObject == m_me)
			return 1.0f;

		//ignore result if there D_is D_no contact response
		if(!convexResult.m_hitCollisionObject->hasContactResponse())
			return 1.0f;

		D_btVector3 linVelA,linVelB;
		linVelA = m_convexToWorld-m_convexFromWorld;
		linVelB = D_btVector3(0,0,0);//toB.getOrigin()-fromB.getOrigin();

		D_btVector3 relativeVelocity = (linVelA-linVelB);
		//don't report time of impact for motion away from the contact normal (or causes minor penetration)
		if (convexResult.m_hitNormalLocal.dot(relativeVelocity)>=-m_allowedPenetration)
			return 1.f;

		return ClosestConvexResultCallback::addSingleResult (convexResult, normalInWorldSpace);
	}

	virtual bool needsCollision(D_btBroadphaseProxy* proxy0) const
	{
		//don't collide with itself
		if (proxy0->m_clientObject == m_me)
			return false;

		///don't do CCD when the collision filters D_are not matching
		if (!ClosestConvexResultCallback::needsCollision(proxy0))
			return false;

		D_btCollisionObject* otherObj = (D_btCollisionObject*) proxy0->m_clientObject;

		//call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
		if (m_dispatcher->needsResponse(m_me,otherObj))
		{
			///don't do CCD when there D_are already contact points (touching contact/penetration)
			D_btAlignedObjectArray<D_btPersistentManifold*> manifoldArray;
			D_btBroadphasePair* collisionPair = m_pairCache->findPair(m_me->getBroadphaseHandle(),proxy0);
			if (collisionPair)
			{
				if (collisionPair->m_algorithm)
				{
					manifoldArray.resize(0);
					collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);
					for (int j=0;j<manifoldArray.size();j++)
					{
						D_btPersistentManifold* manifold = manifoldArray[j];
						if (manifold->getNumContacts()>0)
							return false;
					}
				}
			}
		}
		return true;
	}


};

///internal debugging variable. this value shouldn't be too high
int D_gNumClampedCcdMotions=0;

//#include "stdio.h"
void	D_btDiscreteDynamicsWorld::integrateTransforms(D_btScalar timeStep)
{
	D_BT_PROFILE("integrateTransforms");
	D_btTransform predictedTrans;
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		D_btRigidBody* body = m_nonStaticRigidBodies[i];
		body->setHitFraction(1.f);

		if (body->isActive() && (!body->isStaticOrKinematicObject()))
		{
			body->predictIntegratedTransform(timeStep, predictedTrans);
			D_btScalar squareMotion = (predictedTrans.getOrigin()-body->getWorldTransform().getOrigin()).length2();

			if (body->getCcdSquareMotionThreshold() && body->getCcdSquareMotionThreshold() < squareMotion)
			{
				D_BT_PROFILE("CCD motion clamping");
				if (body->getCollisionShape()->isConvex())
				{
					D_gNumClampedCcdMotions++;
					
					D_btClosestNotMeConvexResultCallback sweepResults(body,body->getWorldTransform().getOrigin(),predictedTrans.getOrigin(),getBroadphase()->getOverlappingPairCache(),getDispatcher());
					//D_btConvexShape* convexShape = static_cast<D_btConvexShape*>(body->getCollisionShape());
					D_btSphereShape tmpSphere(body->getCcdSweptSphereRadius());//D_btConvexShape* convexShape = static_cast<D_btConvexShape*>(body->getCollisionShape());

					sweepResults.m_collisionFilterGroup = body->getBroadphaseProxy()->m_collisionFilterGroup;
					sweepResults.m_collisionFilterMask  = body->getBroadphaseProxy()->m_collisionFilterMask;

					convexSweepTest(&tmpSphere,body->getWorldTransform(),predictedTrans,sweepResults);
					if (sweepResults.hasHit() && (sweepResults.m_closestHitFraction < 1.f))
					{
						body->setHitFraction(sweepResults.m_closestHitFraction);
						body->predictIntegratedTransform(timeStep*body->getHitFraction(), predictedTrans);
						body->setHitFraction(0.f);
//							printf("clamped integration D_to hit fraction = %f\n",fraction);
					}
				}
			}
			
			body->proceedToTransform( predictedTrans);
		}
	}
}





void	D_btDiscreteDynamicsWorld::predictUnconstraintMotion(D_btScalar timeStep)
{
	D_BT_PROFILE("predictUnconstraintMotion");
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		D_btRigidBody* body = m_nonStaticRigidBodies[i];
		if (!body->isStaticOrKinematicObject())
		{
			body->integrateVelocities( timeStep);
			//damping
			body->applyDamping(timeStep);

			body->predictIntegratedTransform(timeStep,body->getInterpolationWorldTransform());
		}
	}
}


void	D_btDiscreteDynamicsWorld::startProfiling(D_btScalar timeStep)
{
	(void)timeStep;

#ifndef D_BT_NO_PROFILE
	D_CProfileManager::Reset();
#endif //D_BT_NO_PROFILE

}




	

class D_DebugDrawcallback : public D_btTriangleCallback, public D_btInternalTriangleIndexCallback
{
	D_btIDebugDraw*	m_debugDrawer;
	D_btVector3	m_color;
	D_btTransform	m_worldTrans;

public:

	D_DebugDrawcallback(D_btIDebugDraw*	debugDrawer,const D_btTransform& worldTrans,const D_btVector3& color) :
                m_debugDrawer(debugDrawer),
		m_color(color),
		m_worldTrans(worldTrans)
	{
	}

	virtual void internalProcessTriangleIndex(D_btVector3* triangle,int partId,int  triangleIndex)
	{
		processTriangle(triangle,partId,triangleIndex);
	}

	virtual void processTriangle(D_btVector3* triangle,int partId, int triangleIndex)
	{
		(void)partId;
		(void)triangleIndex;

		D_btVector3 wv0,wv1,wv2;
		wv0 = m_worldTrans*triangle[0];
		wv1 = m_worldTrans*triangle[1];
		wv2 = m_worldTrans*triangle[2];
		m_debugDrawer->drawLine(wv0,wv1,m_color);
		m_debugDrawer->drawLine(wv1,wv2,m_color);
		m_debugDrawer->drawLine(wv2,wv0,m_color);
	}
};

void D_btDiscreteDynamicsWorld::debugDrawSphere(D_btScalar radius, const D_btTransform& transform, const D_btVector3& color)
{
	D_btVector3 start = transform.getOrigin();

	const D_btVector3 xoffs = transform.getBasis() * D_btVector3(radius,0,0);
	const D_btVector3 yoffs = transform.getBasis() * D_btVector3(0,radius,0);
	const D_btVector3 zoffs = transform.getBasis() * D_btVector3(0,0,radius);

	// XY 
	getDebugDrawer()->drawLine(start-xoffs, start+yoffs, color);
	getDebugDrawer()->drawLine(start+yoffs, start+xoffs, color);
	getDebugDrawer()->drawLine(start+xoffs, start-yoffs, color);
	getDebugDrawer()->drawLine(start-yoffs, start-xoffs, color);

	// XZ
	getDebugDrawer()->drawLine(start-xoffs, start+zoffs, color);
	getDebugDrawer()->drawLine(start+zoffs, start+xoffs, color);
	getDebugDrawer()->drawLine(start+xoffs, start-zoffs, color);
	getDebugDrawer()->drawLine(start-zoffs, start-xoffs, color);

	// YZ
	getDebugDrawer()->drawLine(start-yoffs, start+zoffs, color);
	getDebugDrawer()->drawLine(start+zoffs, start+yoffs, color);
	getDebugDrawer()->drawLine(start+yoffs, start-zoffs, color);
	getDebugDrawer()->drawLine(start-zoffs, start-yoffs, color);
}

void D_btDiscreteDynamicsWorld::debugDrawObject(const D_btTransform& worldTransform, const D_btCollisionShape* shape, const D_btVector3& color)
{
	// Draw a small simplex at the center of the object
	{
		D_btVector3 start = worldTransform.getOrigin();
		getDebugDrawer()->drawLine(start, start+worldTransform.getBasis() * D_btVector3(1,0,0), D_btVector3(1,0,0));
		getDebugDrawer()->drawLine(start, start+worldTransform.getBasis() * D_btVector3(0,1,0), D_btVector3(0,1,0));
		getDebugDrawer()->drawLine(start, start+worldTransform.getBasis() * D_btVector3(0,0,1), D_btVector3(0,0,1));
	}

	if (shape->getShapeType() == D_COMPOUND_SHAPE_PROXYTYPE)
	{
		const D_btCompoundShape* compoundShape = static_cast<const D_btCompoundShape*>(shape);
		for (int i=compoundShape->getNumChildShapes()-1;i>=0;i--)
		{
			D_btTransform childTrans = compoundShape->getChildTransform(i);
			const D_btCollisionShape* colShape = compoundShape->getChildShape(i);
			debugDrawObject(worldTransform*childTrans,colShape,color);
		}

	} else
	{
		switch (shape->getShapeType())
		{

		case D_SPHERE_SHAPE_PROXYTYPE:
			{
				const D_btSphereShape* sphereShape = static_cast<const D_btSphereShape*>(shape);
				D_btScalar radius = sphereShape->getMargin();//radius doesn't include the margin, so draw with margin
				
				debugDrawSphere(radius, worldTransform, color);
				break;
			}
		case D_MULTI_SPHERE_SHAPE_PROXYTYPE:
			{
				const D_btMultiSphereShape* multiSphereShape = static_cast<const D_btMultiSphereShape*>(shape);

				D_btTransform childTransform;
				childTransform.setIdentity();

				for (int i = multiSphereShape->getSphereCount()-1; i>=0;i--)
				{
					childTransform.setOrigin(multiSphereShape->getSpherePosition(i));
					debugDrawSphere(multiSphereShape->getSphereRadius(i), worldTransform*childTransform, color);
				}

				break;
			}
		case D_CAPSULE_SHAPE_PROXYTYPE:
			{
				const D_btCapsuleShape* capsuleShape = static_cast<const D_btCapsuleShape*>(shape);

				D_btScalar radius = capsuleShape->getRadius();
				D_btScalar halfHeight = capsuleShape->getHalfHeight();
				
				int upAxis = capsuleShape->getUpAxis();

				
				D_btVector3 capStart(0.f,0.f,0.f);
				capStart[upAxis] = -halfHeight;

				D_btVector3 capEnd(0.f,0.f,0.f);
				capEnd[upAxis] = halfHeight;

				// Draw the ends
				{
					
					D_btTransform childTransform = worldTransform;
					childTransform.getOrigin() = worldTransform * capStart;
					debugDrawSphere(radius, childTransform, color);
				}

				{
					D_btTransform childTransform = worldTransform;
					childTransform.getOrigin() = worldTransform * capEnd;
					debugDrawSphere(radius, childTransform, color);
				}

				// Draw some additional lines
				D_btVector3 start = worldTransform.getOrigin();

				
				capStart[(upAxis+1)%3] = radius;
				capEnd[(upAxis+1)%3] = radius;
				getDebugDrawer()->drawLine(start+worldTransform.getBasis() * capStart,start+worldTransform.getBasis() * capEnd, color);
				capStart[(upAxis+1)%3] = -radius;
				capEnd[(upAxis+1)%3] = -radius;
				getDebugDrawer()->drawLine(start+worldTransform.getBasis() * capStart,start+worldTransform.getBasis() * capEnd, color);

				capStart[(upAxis+1)%3] = 0.f;
				capEnd[(upAxis+1)%3] = 0.f;

				capStart[(upAxis+2)%3] = radius;
				capEnd[(upAxis+2)%3] = radius;
				getDebugDrawer()->drawLine(start+worldTransform.getBasis() * capStart,start+worldTransform.getBasis() * capEnd, color);
				capStart[(upAxis+2)%3] = -radius;
				capEnd[(upAxis+2)%3] = -radius;
				getDebugDrawer()->drawLine(start+worldTransform.getBasis() * capStart,start+worldTransform.getBasis() * capEnd, color);

				
				break;
			}
		case D_CONE_SHAPE_PROXYTYPE:
			{
				const D_btConeShape* coneShape = static_cast<const D_btConeShape*>(shape);
				D_btScalar radius = coneShape->getRadius();//+coneShape->getMargin();
				D_btScalar height = coneShape->getHeight();//+coneShape->getMargin();
				D_btVector3 start = worldTransform.getOrigin();

				int upAxis= coneShape->getConeUpIndex();
				

				D_btVector3	offsetHeight(0,0,0);
				offsetHeight[upAxis] = height * D_btScalar(0.5);
				D_btVector3	offsetRadius(0,0,0);
				offsetRadius[(upAxis+1)%3] = radius;
				D_btVector3	offset2Radius(0,0,0);
				offset2Radius[(upAxis+2)%3] = radius;

				getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight),start+worldTransform.getBasis() * (-offsetHeight+offsetRadius),color);
				getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight),start+worldTransform.getBasis() * (-offsetHeight-offsetRadius),color);
				getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight),start+worldTransform.getBasis() * (-offsetHeight+offset2Radius),color);
				getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight),start+worldTransform.getBasis() * (-offsetHeight-offset2Radius),color);



				break;

			}
		case D_CYLINDER_SHAPE_PROXYTYPE:
			{
				const D_btCylinderShape* cylinder = static_cast<const D_btCylinderShape*>(shape);
				int upAxis = cylinder->getUpAxis();
				D_btScalar radius = cylinder->getRadius();
				D_btScalar halfHeight = cylinder->getHalfExtentsWithMargin()[upAxis];
				D_btVector3 start = worldTransform.getOrigin();
				D_btVector3	offsetHeight(0,0,0);
				offsetHeight[upAxis] = halfHeight;
				D_btVector3	offsetRadius(0,0,0);
				offsetRadius[(upAxis+1)%3] = radius;
				getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight+offsetRadius),start+worldTransform.getBasis() * (-offsetHeight+offsetRadius),color);
				getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight-offsetRadius),start+worldTransform.getBasis() * (-offsetHeight-offsetRadius),color);
				break;
			}

			case D_STATIC_PLANE_PROXYTYPE:
				{
					const D_btStaticPlaneShape* staticPlaneShape = static_cast<const D_btStaticPlaneShape*>(shape);
					D_btScalar planeConst = staticPlaneShape->getPlaneConstant();
					const D_btVector3& planeNormal = staticPlaneShape->getPlaneNormal();
					D_btVector3 planeOrigin = planeNormal * planeConst;
					D_btVector3 vec0,vec1;
					D_btPlaneSpace1(planeNormal,vec0,vec1);
					D_btScalar vecLen = 100.f;
					D_btVector3 pt0 = planeOrigin + vec0*vecLen;
					D_btVector3 pt1 = planeOrigin - vec0*vecLen;
					D_btVector3 pt2 = planeOrigin + vec1*vecLen;
					D_btVector3 pt3 = planeOrigin - vec1*vecLen;
					getDebugDrawer()->drawLine(worldTransform*pt0,worldTransform*pt1,color);
					getDebugDrawer()->drawLine(worldTransform*pt2,worldTransform*pt3,color);
					break;

				}
		default:
			{

				if (shape->isConcave())
				{
					D_btConcaveShape* concaveMesh = (D_btConcaveShape*) shape;
					
					///@todo pass camera, for some culling? D_no -> we D_are not a graphics lib
					D_btVector3 aabbMax(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
					D_btVector3 aabbMin(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT));

					D_DebugDrawcallback drawCallback(getDebugDrawer(),worldTransform,color);
					concaveMesh->processAllTriangles(&drawCallback,aabbMin,aabbMax);

				}

				if (shape->getShapeType() == D_CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE)
				{
					D_btConvexTriangleMeshShape* convexMesh = (D_btConvexTriangleMeshShape*) shape;
					//todo: pass camera for some culling			
					D_btVector3 aabbMax(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
					D_btVector3 aabbMin(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT));
					//D_DebugDrawcallback drawCallback;
					D_DebugDrawcallback drawCallback(getDebugDrawer(),worldTransform,color);
					convexMesh->getMeshInterface()->InternalProcessAllTriangles(&drawCallback,aabbMin,aabbMax);
				}


				/// for polyhedral D_shapes
				if (shape->isPolyhedral())
				{
					D_btPolyhedralConvexShape* polyshape = (D_btPolyhedralConvexShape*) shape;

					int i;
					for (i=0;i<polyshape->getNumEdges();i++)
					{
						D_btVector3 a,b;
						polyshape->getEdge(i,a,b);
						D_btVector3 wa = worldTransform * a;
						D_btVector3 wb = worldTransform * b;
						getDebugDrawer()->drawLine(wa,wb,color);

					}

					
				}
			}
		}
	}
}


void D_btDiscreteDynamicsWorld::debugDrawConstraint(D_btTypedConstraint* constraint)
{
	bool drawFrames = (getDebugDrawer()->getDebugMode() & D_btIDebugDraw::D_DBG_DrawConstraints) != 0;
	bool drawLimits = (getDebugDrawer()->getDebugMode() & D_btIDebugDraw::D_DBG_DrawConstraintLimits) != 0;
	D_btScalar dbgDrawSize = constraint->getDbgDrawSize();
	if(dbgDrawSize <= D_btScalar(0.f))
	{
		return;
	}

	switch(constraint->getConstraintType())
	{
		case D_POINT2POINT_CONSTRAINT_TYPE:
			{
				D_btPoint2PointConstraint* p2pC = (D_btPoint2PointConstraint*)constraint;
				D_btTransform tr;
				tr.setIdentity();
				D_btVector3 pivot = p2pC->getPivotInA();
				pivot = p2pC->getRigidBodyA().getCenterOfMassTransform() * pivot; 
				tr.setOrigin(pivot);
				getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				// that ideally D_should draw the same frame	
				pivot = p2pC->getPivotInB();
				pivot = p2pC->getRigidBodyB().getCenterOfMassTransform() * pivot; 
				tr.setOrigin(pivot);
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			}
			break;
		case D_HINGE_CONSTRAINT_TYPE:
			{
				D_btHingeConstraint* pHinge = (D_btHingeConstraint*)constraint;
				D_btTransform tr = pHinge->getRigidBodyA().getCenterOfMassTransform() * pHinge->getAFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pHinge->getRigidBodyB().getCenterOfMassTransform() * pHinge->getBFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				D_btScalar minAng = pHinge->getLowerLimit();
				D_btScalar maxAng = pHinge->getUpperLimit();
				if(minAng == maxAng)
				{
					break;
				}
				bool drawSect = true;
				if(minAng > maxAng)
				{
					minAng = D_btScalar(0.f);
					maxAng = D_SIMD_2_PI;
					drawSect = false;
				}
				if(drawLimits) 
				{
					D_btVector3& center = tr.getOrigin();
					D_btVector3 normal = tr.getBasis().getColumn(2);
					D_btVector3 axis = tr.getBasis().getColumn(0);
					getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, minAng, maxAng, D_btVector3(0,0,0), drawSect);
				}
			}
			break;
		case D_CONETWIST_CONSTRAINT_TYPE:
			{
				D_btConeTwistConstraint* pCT = (D_btConeTwistConstraint*)constraint;
				D_btTransform tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					//const D_btScalar length = D_btScalar(5);
					const D_btScalar length = dbgDrawSize;
					static int nSegments = 8*4;
					D_btScalar fAngleInRadians = D_btScalar(2.*3.1415926) * (D_btScalar)(nSegments-1)/D_btScalar(nSegments);
					D_btVector3 pPrev = pCT->GetPointForAngle(fAngleInRadians, length);
					pPrev = tr * pPrev;
					for (int i=0; i<nSegments; i++)
					{
						fAngleInRadians = D_btScalar(2.*3.1415926) * (D_btScalar)i/D_btScalar(nSegments);
						D_btVector3 pCur = pCT->GetPointForAngle(fAngleInRadians, length);
						pCur = tr * pCur;
						getDebugDrawer()->drawLine(pPrev, pCur, D_btVector3(0,0,0));

						if (i%(nSegments/8) == 0)
							getDebugDrawer()->drawLine(tr.getOrigin(), pCur, D_btVector3(0,0,0));

						pPrev = pCur;
					}						
					D_btScalar tws = pCT->getTwistSpan();
					D_btScalar twa = pCT->getTwistAngle();
					bool useFrameB = (pCT->getRigidBodyB().getInvMass() > D_btScalar(0.f));
					if(useFrameB)
					{
						tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
					}
					else
					{
						tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
					}
					D_btVector3 pivot = tr.getOrigin();
					D_btVector3 normal = tr.getBasis().getColumn(0);
					D_btVector3 axis1 = tr.getBasis().getColumn(1);
					getDebugDrawer()->drawArc(pivot, normal, axis1, dbgDrawSize, dbgDrawSize, -twa-tws, -twa+tws, D_btVector3(0,0,0), true);

				}
			}
			break;
		case D_D6_CONSTRAINT_TYPE:
			{
				D_btGeneric6DofConstraint* p6DOF = (D_btGeneric6DofConstraint*)constraint;
				D_btTransform tr = p6DOF->getCalculatedTransformA();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = p6DOF->getCalculatedTransformB();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits) 
				{
					tr = p6DOF->getCalculatedTransformA();
					const D_btVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
					D_btVector3 up = tr.getBasis().getColumn(2);
					D_btVector3 axis = tr.getBasis().getColumn(0);
					D_btScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
					D_btScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
					D_btScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
					D_btScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
					getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * D_btScalar(.9f), minTh, maxTh, minPs, maxPs, D_btVector3(0,0,0));
					axis = tr.getBasis().getColumn(1);
					D_btScalar ay = p6DOF->getAngle(1);
					D_btScalar az = p6DOF->getAngle(2);
					D_btScalar cy = D_btCos(ay);
					D_btScalar sy = D_btSin(ay);
					D_btScalar cz = D_btCos(az);
					D_btScalar sz = D_btSin(az);
					D_btVector3 ref;
					ref[0] = cy*cz*axis[0] + cy*sz*axis[1] - sy*axis[2];
					ref[1] = -sz*axis[0] + cz*axis[1];
					ref[2] = cz*sy*axis[0] + sz*sy*axis[1] + cy*axis[2];
					tr = p6DOF->getCalculatedTransformB();
					D_btVector3 normal = -tr.getBasis().getColumn(0);
					D_btScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
					D_btScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
					if(minFi > maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -D_SIMD_PI, D_SIMD_PI, D_btVector3(0,0,0), false);
					}
					else if(minFi < maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, D_btVector3(0,0,0), true);
					}
					tr = p6DOF->getCalculatedTransformA();
					D_btVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
					D_btVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
					getDebugDrawer()->drawBox(bbMin, bbMax, tr, D_btVector3(0,0,0));
				}
			}
			break;
		case D_SLIDER_CONSTRAINT_TYPE:
			{
				D_btSliderConstraint* pSlider = (D_btSliderConstraint*)constraint;
				D_btTransform tr = pSlider->getCalculatedTransformA();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pSlider->getCalculatedTransformB();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					D_btTransform tr = pSlider->getCalculatedTransformA();
					D_btVector3 li_min = tr * D_btVector3(pSlider->getLowerLinLimit(), 0.f, 0.f);
					D_btVector3 li_max = tr * D_btVector3(pSlider->getUpperLinLimit(), 0.f, 0.f);
					getDebugDrawer()->drawLine(li_min, li_max, D_btVector3(0, 0, 0));
					D_btVector3 normal = tr.getBasis().getColumn(0);
					D_btVector3 axis = tr.getBasis().getColumn(1);
					D_btScalar a_min = pSlider->getLowerAngLimit();
					D_btScalar a_max = pSlider->getUpperAngLimit();
					const D_btVector3& center = pSlider->getCalculatedTransformB().getOrigin();
					getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, a_min, a_max, D_btVector3(0,0,0), true);
				}
			}
			break;
		default : 
			break;
	}
	return;
}





void	D_btDiscreteDynamicsWorld::setConstraintSolver(D_btConstraintSolver* solver)
{
	if (m_ownsConstraintSolver)
	{
		D_btAlignedFree( m_constraintSolver);
	}
	m_ownsConstraintSolver = false;
	m_constraintSolver = solver;
}

D_btConstraintSolver* D_btDiscreteDynamicsWorld::getConstraintSolver()
{
	return m_constraintSolver;
}


int		D_btDiscreteDynamicsWorld::getNumConstraints() const
{
	return int(m_constraints.size());
}
D_btTypedConstraint* D_btDiscreteDynamicsWorld::getConstraint(int index)
{
	return m_constraints[index];
}
const D_btTypedConstraint* D_btDiscreteDynamicsWorld::getConstraint(int index) const
{
	return m_constraints[index];
}


