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

#ifndef D_BT_DYNAMICS_WORLD_H
#define D_BT_DYNAMICS_WORLD_H

#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"

class D_btTypedConstraint;
class D_btActionInterface;
class D_btConstraintSolver;
class D_btDynamicsWorld;


/// Type for the callback for each tick
typedef void (*D_btInternalTickCallback)(D_btDynamicsWorld *world, D_btScalar timeStep);

enum D_btDynamicsWorldType
{
	D_BT_SIMPLE_DYNAMICS_WORLD=1,
	D_BT_DISCRETE_DYNAMICS_WORLD=2,
	D_BT_CONTINUOUS_DYNAMICS_WORLD=3
};

///The D_btDynamicsWorld D_is the interface class for several dynamics implementation, basic, discrete, parallel, D_and continuous etc.
class D_btDynamicsWorld : public D_btCollisionWorld
{

protected:
		D_btInternalTickCallback m_internalTickCallback;
		D_btInternalTickCallback m_internalPreTickCallback;
		void*	m_worldUserInfo;

		D_btContactSolverInfo	m_solverInfo;

public:
		

		D_btDynamicsWorld(D_btDispatcher* dispatcher,D_btBroadphaseInterface* broadphase,D_btCollisionConfiguration* collisionConfiguration)
		:D_btCollisionWorld(dispatcher,broadphase,collisionConfiguration), m_internalTickCallback(0),m_internalPreTickCallback(0), m_worldUserInfo(0)
		{
		}

		virtual ~D_btDynamicsWorld()
		{
		}
		
		///stepSimulation proceeds the simulation over 'timeStep', units in preferably in seconds.
		///By default, Bullet D_will subdivide the timestep in constant substeps of each 'fixedTimeStep'.
		///in order D_to keep the simulation real-time, the maximum number of substeps D_can be clamped D_to 'maxSubSteps'.
		///You D_can disable subdividing the timestep/substepping by passing maxSubSteps=0 as second argument D_to stepSimulation, but in that case you have D_to keep the timeStep constant.
		virtual int		stepSimulation( D_btScalar timeStep,int maxSubSteps=1, D_btScalar fixedTimeStep=D_btScalar(1.)/D_btScalar(60.))=0;
			
		virtual void	debugDrawWorld() = 0;
				
		virtual void	addConstraint(D_btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies=false) 
		{ 
			(void)constraint; (void)disableCollisionsBetweenLinkedBodies;
		}

		virtual void	removeConstraint(D_btTypedConstraint* constraint) {(void)constraint;}

		virtual void	addAction(D_btActionInterface* action) = 0;

		virtual void	removeAction(D_btActionInterface* action) = 0;

		//once a rigidbody D_is added D_to the dynamics world, it D_will get this gravity assigned
		//existing rigidbodies in the world get gravity assigned too, during this method
		virtual void	setGravity(const D_btVector3& gravity) = 0;
		virtual D_btVector3 getGravity () const = 0;

		virtual void	synchronizeMotionStates() = 0;

		virtual void	addRigidBody(D_btRigidBody* body) = 0;

		virtual void	removeRigidBody(D_btRigidBody* body) = 0;

		virtual void	setConstraintSolver(D_btConstraintSolver* solver) = 0;

		virtual D_btConstraintSolver* getConstraintSolver() = 0;
		
		virtual	int		getNumConstraints() const {	return 0;		}
		
		virtual D_btTypedConstraint* getConstraint(int index)		{	(void)index;		return 0;		}
		
		virtual const D_btTypedConstraint* getConstraint(int index) const	{	(void)index;	return 0;	}

		virtual D_btDynamicsWorldType	getWorldType() const=0;

		virtual void	clearForces() = 0;

		/// Set the callback for when an internal tick (simulation substep) happens, optional user info
		void setInternalTickCallback(D_btInternalTickCallback cb,	void* worldUserInfo=0,bool isPreTick=false) 
		{ 
			if (isPreTick)
			{
				m_internalPreTickCallback = cb;
			} else
			{
				m_internalTickCallback = cb; 
			}
			m_worldUserInfo = worldUserInfo;
		}

		void	setWorldUserInfo(void* worldUserInfo)
		{
			m_worldUserInfo = worldUserInfo;
		}

		void*	getWorldUserInfo() const
		{
			return m_worldUserInfo;
		}

		D_btContactSolverInfo& getSolverInfo()
		{
			return m_solverInfo;
		}


		///obsolete, use addAction instead.
		virtual void	addVehicle(D_btActionInterface* vehicle) {(void)vehicle;}
		///obsolete, use removeAction instead
		virtual void	removeVehicle(D_btActionInterface* vehicle) {(void)vehicle;}
		///obsolete, use addAction instead.
		virtual void	addCharacter(D_btActionInterface* character) {(void)character;}
		///obsolete, use removeAction instead
		virtual void	removeCharacter(D_btActionInterface* character) {(void)character;}


};

#endif //D_BT_DYNAMICS_WORLD_H


