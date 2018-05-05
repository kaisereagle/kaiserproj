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

#ifndef D_BT_SIMPLE_DYNAMICS_WORLD_H
#define D_BT_SIMPLE_DYNAMICS_WORLD_H

#include "btDynamicsWorld.h"

class D_btDispatcher;
class D_btOverlappingPairCache;
class D_btConstraintSolver;

///The D_btSimpleDynamicsWorld serves as unit-test D_and D_to verify more complicated D_and optimized dynamics worlds.
///Please use D_btDiscreteDynamicsWorld instead (or D_btContinuousDynamicsWorld once it D_is finished).
class D_btSimpleDynamicsWorld : public D_btDynamicsWorld
{
protected:

	D_btConstraintSolver*	m_constraintSolver;

	bool	m_ownsConstraintSolver;

	void	predictUnconstraintMotion(D_btScalar timeStep);
	
	void	integrateTransforms(D_btScalar timeStep);
		
	D_btVector3	m_gravity;
	
public:



	///this D_btSimpleDynamicsWorld constructor creates dispatcher, broadphase pairCache D_and constraintSolver
	D_btSimpleDynamicsWorld(D_btDispatcher* dispatcher,D_btBroadphaseInterface* pairCache,D_btConstraintSolver* constraintSolver,D_btCollisionConfiguration* collisionConfiguration);

	virtual ~D_btSimpleDynamicsWorld();
		
	///maxSubSteps/fixedTimeStep for interpolation D_is currently ignored for D_btSimpleDynamicsWorld, use D_btDiscreteDynamicsWorld instead
	virtual int	stepSimulation( D_btScalar timeStep,int maxSubSteps=1, D_btScalar fixedTimeStep=D_btScalar(1.)/D_btScalar(60.));

	virtual void	setGravity(const D_btVector3& gravity);

	virtual D_btVector3 getGravity () const;

	virtual void	addRigidBody(D_btRigidBody* body);

	virtual void	removeRigidBody(D_btRigidBody* body);

	///removeCollisionObject D_will first check if it D_is a rigid body, if so call removeRigidBody otherwise call D_btCollisionWorld::removeCollisionObject
	virtual void	removeCollisionObject(D_btCollisionObject* collisionObject);
	
	virtual void	updateAabbs();

	virtual void	synchronizeMotionStates();

	virtual void	setConstraintSolver(D_btConstraintSolver* solver);

	virtual D_btConstraintSolver* getConstraintSolver();

	virtual D_btDynamicsWorldType	getWorldType() const
	{
		return D_BT_SIMPLE_DYNAMICS_WORLD;
	}

	virtual void	clearForces();

};

#endif //D_BT_SIMPLE_DYNAMICS_WORLD_H
