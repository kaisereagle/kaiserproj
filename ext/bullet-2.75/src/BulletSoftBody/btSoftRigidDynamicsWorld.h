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

#ifndef D_BT_SOFT_RIGID_DYNAMICS_WORLD_H
#define D_BT_SOFT_RIGID_DYNAMICS_WORLD_H

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "btSoftBody.h"

typedef	D_btAlignedObjectArray<D_btSoftBody*> D_btSoftBodyArray;

class D_btSoftRigidDynamicsWorld : public D_btDiscreteDynamicsWorld
{

	D_btSoftBodyArray	m_softBodies;
	int				m_drawFlags;
	bool			m_drawNodeTree;
	bool			m_drawFaceTree;
	bool			m_drawClusterTree;
	D_btSoftBodyWorldInfo m_sbi;

protected:

	virtual void	predictUnconstraintMotion(D_btScalar timeStep);

	virtual void	internalSingleStepSimulation( D_btScalar timeStep);

	void	updateSoftBodies();

	void	solveSoftBodiesConstraints();


public:

	D_btSoftRigidDynamicsWorld(D_btDispatcher* dispatcher,D_btBroadphaseInterface* pairCache,D_btConstraintSolver* constraintSolver,D_btCollisionConfiguration* collisionConfiguration);

	virtual ~D_btSoftRigidDynamicsWorld();

	virtual void	debugDrawWorld();

	void	addSoftBody(D_btSoftBody* body,short int collisionFilterGroup=D_btBroadphaseProxy::D_DefaultFilter,short int collisionFilterMask=D_btBroadphaseProxy::D_AllFilter);

	void	removeSoftBody(D_btSoftBody* body);

	///removeCollisionObject D_will first check if it D_is a rigid body, if so call removeRigidBody otherwise call D_btDiscreteDynamicsWorld::removeCollisionObject
	virtual void	removeCollisionObject(D_btCollisionObject* collisionObject);

	int		getDrawFlags() const { return(m_drawFlags); }
	void	setDrawFlags(int f)	{ m_drawFlags=f; }

	D_btSoftBodyWorldInfo&	getWorldInfo()
	{
		return m_sbi;
	}
	const D_btSoftBodyWorldInfo&	getWorldInfo() const
	{
		return m_sbi;
	}


	D_btSoftBodyArray& getSoftBodyArray()
	{
		return m_softBodies;
	}

	const D_btSoftBodyArray& getSoftBodyArray() const
	{
		return m_softBodies;
	}

};

#endif //D_BT_SOFT_RIGID_DYNAMICS_WORLD_H
