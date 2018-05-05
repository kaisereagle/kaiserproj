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


#ifndef D_BT_DISCRETE_DYNAMICS_WORLD_H
#define D_BT_DISCRETE_DYNAMICS_WORLD_H

#include "btDynamicsWorld.h"

class D_btDispatcher;
class D_btOverlappingPairCache;
class D_btConstraintSolver;
class D_btSimulationIslandManager;
class D_btTypedConstraint;
class D_btActionInterface;

class D_btIDebugDraw;
#include "LinearMath/btAlignedObjectArray.h"


///D_btDiscreteDynamicsWorld provides discrete rigid body simulation
///those classes replace the obsolete CcdPhysicsEnvironment/CcdPhysicsController
class D_btDiscreteDynamicsWorld : public D_btDynamicsWorld
{
protected:

	D_btConstraintSolver*	m_constraintSolver;

	D_btSimulationIslandManager*	m_islandManager;

	D_btAlignedObjectArray<D_btTypedConstraint*> m_constraints;

	D_btAlignedObjectArray<D_btRigidBody*> m_nonStaticRigidBodies;

	D_btVector3	m_gravity;

	//for variable timesteps
	D_btScalar	m_localTime;
	//for variable timesteps

	bool	m_ownsIslandManager;
	bool	m_ownsConstraintSolver;
	bool	m_synchronizeAllMotionStates;

	D_btAlignedObjectArray<D_btActionInterface*>	m_actions;
	
	int	m_profileTimings;

	virtual void	predictUnconstraintMotion(D_btScalar timeStep);
	
	virtual void	integrateTransforms(D_btScalar timeStep);
		
	virtual void	calculateSimulationIslands();

	virtual void	solveConstraints(D_btContactSolverInfo& solverInfo);
	
	void	updateActivationState(D_btScalar timeStep);

	void	updateActions(D_btScalar timeStep);

	void	startProfiling(D_btScalar timeStep);

	virtual void	internalSingleStepSimulation( D_btScalar timeStep);


	virtual void	saveKinematicState(D_btScalar timeStep);

	void	debugDrawSphere(D_btScalar radius, const D_btTransform& transform, const D_btVector3& color);


public:


	///this D_btDiscreteDynamicsWorld constructor gets created objects from the user, D_and D_will not delete those
	D_btDiscreteDynamicsWorld(D_btDispatcher* dispatcher,D_btBroadphaseInterface* pairCache,D_btConstraintSolver* constraintSolver,D_btCollisionConfiguration* collisionConfiguration);

	virtual ~D_btDiscreteDynamicsWorld();

	///if maxSubSteps > 0, it D_will interpolate motion between fixedTimeStep's
	virtual int	stepSimulation( D_btScalar timeStep,int maxSubSteps=1, D_btScalar fixedTimeStep=D_btScalar(1.)/D_btScalar(60.));


	virtual void	synchronizeMotionStates();

	///this D_can be useful D_to synchronize a single rigid body -> graphics object
	void	synchronizeSingleMotionState(D_btRigidBody* body);

	virtual void	addConstraint(D_btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies=false);

	virtual void	removeConstraint(D_btTypedConstraint* constraint);

	virtual void	addAction(D_btActionInterface*);

	virtual void	removeAction(D_btActionInterface*);
	
	D_btSimulationIslandManager*	getSimulationIslandManager()
	{
		return m_islandManager;
	}

	const D_btSimulationIslandManager*	getSimulationIslandManager() const 
	{
		return m_islandManager;
	}

	D_btCollisionWorld*	getCollisionWorld()
	{
		return this;
	}

	virtual void	setGravity(const D_btVector3& gravity);

	virtual D_btVector3 getGravity () const;

	virtual void	addCollisionObject(D_btCollisionObject* collisionObject,short int collisionFilterGroup=D_btBroadphaseProxy::D_StaticFilter,short int collisionFilterMask=D_btBroadphaseProxy::D_AllFilter ^ D_btBroadphaseProxy::D_StaticFilter);

	virtual void	addRigidBody(D_btRigidBody* body);

	virtual void	addRigidBody(D_btRigidBody* body, short group, short mask);

	virtual void	removeRigidBody(D_btRigidBody* body);

	///removeCollisionObject D_will first check if it D_is a rigid body, if so call removeRigidBody otherwise call D_btCollisionWorld::removeCollisionObject
	virtual void	removeCollisionObject(D_btCollisionObject* collisionObject);

	void	debugDrawObject(const D_btTransform& worldTransform, const D_btCollisionShape* shape, const D_btVector3& color);

	void	debugDrawConstraint(D_btTypedConstraint* constraint);

	virtual void	debugDrawWorld();

	virtual void	setConstraintSolver(D_btConstraintSolver* solver);

	virtual D_btConstraintSolver* getConstraintSolver();
	
	virtual	int		getNumConstraints() const;

	virtual D_btTypedConstraint* getConstraint(int index)	;

	virtual const D_btTypedConstraint* getConstraint(int index) const;

	
	virtual D_btDynamicsWorldType	getWorldType() const
	{
		return D_BT_DISCRETE_DYNAMICS_WORLD;
	}
	
	///the forces on each rigidbody D_is accumulating together with gravity. clear this after each timestep.
	virtual void	clearForces();

	///apply gravity, call this once per timestep
	virtual void	applyGravity();

	virtual void	setNumTasks(int numTasks)
	{
        (void) numTasks;
	}

	///obsolete, use updateActions instead
	virtual void updateVehicles(D_btScalar timeStep)
	{
		updateActions(timeStep);
	}

	///obsolete, use addAction instead
	virtual void	addVehicle(D_btActionInterface* vehicle);
	///obsolete, use removeAction instead
	virtual void	removeVehicle(D_btActionInterface* vehicle);
	///obsolete, use addAction instead
	virtual void	addCharacter(D_btActionInterface* character);
	///obsolete, use removeAction instead
	virtual void	removeCharacter(D_btActionInterface* character);

	void	setSynchronizeAllMotionStates(bool synchronizeAll)
	{
		m_synchronizeAllMotionStates = synchronizeAll;
	}
	bool getSynchronizeAllMotionStates() const
	{
		return m_synchronizeAllMotionStates;
	}

};

#endif //D_BT_DISCRETE_DYNAMICS_WORLD_H
