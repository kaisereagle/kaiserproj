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

#include "btConvexPlaneCollisionAlgorithm.h"

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"

//#include <stdio.h>

D_btConvexPlaneCollisionAlgorithm::D_btConvexPlaneCollisionAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* col0,D_btCollisionObject* col1, bool isSwapped, int numPerturbationIterations,int minimumPointsPerturbationThreshold)
: D_btCollisionAlgorithm(ci),
m_ownManifold(false),
m_manifoldPtr(mf),
m_isSwapped(isSwapped),
m_numPerturbationIterations(numPerturbationIterations),
m_minimumPointsPerturbationThreshold(minimumPointsPerturbationThreshold)
{
	D_btCollisionObject* convexObj = m_isSwapped? col1 : col0;
	D_btCollisionObject* planeObj = m_isSwapped? col0 : col1;

	if (!m_manifoldPtr && m_dispatcher->needsCollision(convexObj,planeObj))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(convexObj,planeObj);
		m_ownManifold = true;
	}
}


D_btConvexPlaneCollisionAlgorithm::~D_btConvexPlaneCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void D_btConvexPlaneCollisionAlgorithm::collideSingleContact (const D_btQuaternion& perturbeRot, D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
    D_btCollisionObject* convexObj = m_isSwapped? body1 : body0;
	D_btCollisionObject* planeObj = m_isSwapped? body0: body1;

	D_btConvexShape* convexShape = (D_btConvexShape*) convexObj->getCollisionShape();
	D_btStaticPlaneShape* planeShape = (D_btStaticPlaneShape*) planeObj->getCollisionShape();

    bool hasCollision = false;
	const D_btVector3& planeNormal = planeShape->getPlaneNormal();
	const D_btScalar& planeConstant = planeShape->getPlaneConstant();
	
	D_btTransform convexWorldTransform = convexObj->getWorldTransform();
	D_btTransform convexInPlaneTrans;
	convexInPlaneTrans= planeObj->getWorldTransform().inverse() * convexWorldTransform;
	//now perturbe the convex-world transform
	convexWorldTransform.getBasis()*=D_btMatrix3x3(perturbeRot);
	D_btTransform planeInConvex;
	planeInConvex= convexWorldTransform.inverse() * planeObj->getWorldTransform();
	
	D_btVector3 vtx = convexShape->localGetSupportingVertex(planeInConvex.getBasis()*-planeNormal);

	D_btVector3 vtxInPlane = convexInPlaneTrans(vtx);
	D_btScalar distance = (planeNormal.dot(vtxInPlane) - planeConstant);

	D_btVector3 vtxInPlaneProjected = vtxInPlane - distance*planeNormal;
	D_btVector3 vtxInPlaneWorld = planeObj->getWorldTransform() * vtxInPlaneProjected;

	hasCollision = distance < m_manifoldPtr->getContactBreakingThreshold();
	resultOut->setPersistentManifold(m_manifoldPtr);
	if (hasCollision)
	{
		/// report a contact. internally this D_will be kept persistent, D_and contact reduction D_is done
		D_btVector3 normalOnSurfaceB = planeObj->getWorldTransform().getBasis() * planeNormal;
		D_btVector3 pOnB = vtxInPlaneWorld;
		resultOut->addContactPoint(normalOnSurfaceB,pOnB,distance);
	}
}


void D_btConvexPlaneCollisionAlgorithm::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)dispatchInfo;
	if (!m_manifoldPtr)
		return;

    D_btCollisionObject* convexObj = m_isSwapped? body1 : body0;
	D_btCollisionObject* planeObj = m_isSwapped? body0: body1;

	D_btConvexShape* convexShape = (D_btConvexShape*) convexObj->getCollisionShape();
	D_btStaticPlaneShape* planeShape = (D_btStaticPlaneShape*) planeObj->getCollisionShape();

    
	const D_btVector3& planeNormal = planeShape->getPlaneNormal();
	//const D_btScalar& planeConstant = planeShape->getPlaneConstant();

	//first perform a collision query with the non-perturbated collision objects
	{
		D_btQuaternion rotq(0,0,0,1);
		collideSingleContact(rotq,body0,body1,dispatchInfo,resultOut);
	}

	if (resultOut->getPersistentManifold()->getNumContacts()<m_minimumPointsPerturbationThreshold)
	{
		D_btVector3 v0,v1;
		D_btPlaneSpace1(planeNormal,v0,v1);
		//now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects

		const D_btScalar angleLimit = 0.125f * D_SIMD_PI;
		D_btScalar perturbeAngle;
		D_btScalar radius = convexShape->getAngularMotionDisc();
		perturbeAngle = D_gContactBreakingThreshold / radius;
		if ( perturbeAngle > angleLimit ) 
				perturbeAngle = angleLimit;

		D_btQuaternion perturbeRot(v0,perturbeAngle);
		for (int i=0;i<m_numPerturbationIterations;i++)
		{
			D_btScalar iterationAngle = i*(D_SIMD_2_PI/D_btScalar(m_numPerturbationIterations));
			D_btQuaternion rotq(planeNormal,iterationAngle);
			collideSingleContact(rotq.inverse()*perturbeRot*rotq,body0,body1,dispatchInfo,resultOut);
		}
	}

	if (m_ownManifold)
	{
		if (m_manifoldPtr->getNumContacts())
		{
			resultOut->refreshContactPoints();
		}
	}
}

D_btScalar D_btConvexPlaneCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* col0,D_btCollisionObject* col1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return D_btScalar(1.);
}
