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


#include "btSoftRigidDynamicsWorld.h"
#include "LinearMath/btQuickprof.h"

//softbody & helpers
#include "btSoftBody.h"
#include "btSoftBodyHelpers.h"





D_btSoftRigidDynamicsWorld::D_btSoftRigidDynamicsWorld(D_btDispatcher* dispatcher,D_btBroadphaseInterface* pairCache,D_btConstraintSolver* constraintSolver,D_btCollisionConfiguration* collisionConfiguration)
:D_btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration)
{
	m_drawFlags			=	fDrawFlags::Std;
	m_drawNodeTree		=	true;
	m_drawFaceTree		=	false;
	m_drawClusterTree	=	false;
	m_sbi.m_broadphase = pairCache;
	m_sbi.m_dispatcher = dispatcher;
	m_sbi.m_sparsesdf.Initialize();
	m_sbi.m_sparsesdf.Reset();

}

D_btSoftRigidDynamicsWorld::~D_btSoftRigidDynamicsWorld()
{

}

void	D_btSoftRigidDynamicsWorld::predictUnconstraintMotion(D_btScalar timeStep)
{
	D_btDiscreteDynamicsWorld::predictUnconstraintMotion( timeStep);

	for ( int i=0;i<m_softBodies.size();++i)
	{
		D_btSoftBody*	psb= m_softBodies[i];

		psb->predictMotion(timeStep);		
	}
}

void	D_btSoftRigidDynamicsWorld::internalSingleStepSimulation( D_btScalar timeStep)
{
	D_btDiscreteDynamicsWorld::internalSingleStepSimulation( timeStep );

	///solve soft bodies constraints
	solveSoftBodiesConstraints();

	//self collisions
	for ( int i=0;i<m_softBodies.size();i++)
	{
		D_btSoftBody*	psb=(D_btSoftBody*)m_softBodies[i];
		psb->defaultCollisionHandler(psb);
	}

	///update soft bodies
	updateSoftBodies();

}

void	D_btSoftRigidDynamicsWorld::updateSoftBodies()
{
	D_BT_PROFILE("updateSoftBodies");

	for ( int i=0;i<m_softBodies.size();i++)
	{
		D_btSoftBody*	psb=(D_btSoftBody*)m_softBodies[i];
		psb->integrateMotion();	
	}
}

void	D_btSoftRigidDynamicsWorld::solveSoftBodiesConstraints()
{
	D_BT_PROFILE("solveSoftConstraints");

	if(m_softBodies.size())
	{
		D_btSoftBody::solveClusters(m_softBodies);
	}

	for(int i=0;i<m_softBodies.size();++i)
	{
		D_btSoftBody*	psb=(D_btSoftBody*)m_softBodies[i];
		psb->solveConstraints();
	}	
}

void	D_btSoftRigidDynamicsWorld::addSoftBody(D_btSoftBody* body,short int collisionFilterGroup,short int collisionFilterMask)
{
	m_softBodies.push_back(body);

	D_btCollisionWorld::addCollisionObject(body,
		collisionFilterGroup,
		collisionFilterMask);

}

void	D_btSoftRigidDynamicsWorld::removeSoftBody(D_btSoftBody* body)
{
	m_softBodies.remove(body);

	D_btCollisionWorld::removeCollisionObject(body);
}

void	D_btSoftRigidDynamicsWorld::removeCollisionObject(D_btCollisionObject* collisionObject)
{
	D_btSoftBody* body = D_btSoftBody::upcast(collisionObject);
	if (body)
		removeSoftBody(body);
	else
		D_btDiscreteDynamicsWorld::removeCollisionObject(collisionObject);
}

void	D_btSoftRigidDynamicsWorld::debugDrawWorld()
{
	D_btDiscreteDynamicsWorld::debugDrawWorld();

	if (getDebugDrawer())
	{
		int i;
		for (  i=0;i<this->m_softBodies.size();i++)
		{
			D_btSoftBody*	psb=(D_btSoftBody*)this->m_softBodies[i];
			D_btSoftBodyHelpers::DrawFrame(psb,m_debugDrawer);
			D_btSoftBodyHelpers::Draw(psb,m_debugDrawer,m_drawFlags);
			if (m_debugDrawer && (m_debugDrawer->getDebugMode() & D_btIDebugDraw::D_DBG_DrawAabb))
			{
				if(m_drawNodeTree)		D_btSoftBodyHelpers::DrawNodeTree(psb,m_debugDrawer);
				if(m_drawFaceTree)		D_btSoftBodyHelpers::DrawFaceTree(psb,m_debugDrawer);
				if(m_drawClusterTree)	D_btSoftBodyHelpers::DrawClusterTree(psb,m_debugDrawer);
			}
		}		
	}	
}
