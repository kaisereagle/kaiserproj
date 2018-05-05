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

#include "btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

D_btSphereSphereCollisionAlgorithm::D_btSphereSphereCollisionAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* col0,D_btCollisionObject* col1)
: D_btActivatingCollisionAlgorithm(ci,col0,col1),
m_ownManifold(false),
m_manifoldPtr(mf)
{
	if (!m_manifoldPtr)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(col0,col1);
		m_ownManifold = true;
	}
}

D_btSphereSphereCollisionAlgorithm::~D_btSphereSphereCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void D_btSphereSphereCollisionAlgorithm::processCollision (D_btCollisionObject* col0,D_btCollisionObject* col1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)dispatchInfo;

	if (!m_manifoldPtr)
		return;

	resultOut->setPersistentManifold(m_manifoldPtr);

	D_btSphereShape* sphere0 = (D_btSphereShape*)col0->getCollisionShape();
	D_btSphereShape* sphere1 = (D_btSphereShape*)col1->getCollisionShape();

	D_btVector3 diff = col0->getWorldTransform().getOrigin()-  col1->getWorldTransform().getOrigin();
	D_btScalar len = diff.length();
	D_btScalar radius0 = sphere0->getRadius();
	D_btScalar radius1 = sphere1->getRadius();

#ifdef CLEAR_MANIFOLD
	m_manifoldPtr->clearManifold(); //don't do this, it disables warmstarting
#endif

	///iff distance positive, don't generate a new contact
	if ( len > (radius0+radius1))
	{
#ifndef CLEAR_MANIFOLD
		resultOut->refreshContactPoints();
#endif //CLEAR_MANIFOLD
		return;
	}
	///distance (negative means penetration)
	D_btScalar dist = len - (radius0+radius1);

	D_btVector3 normalOnSurfaceB(1,0,0);
	if (len > D_SIMD_EPSILON)
	{
		normalOnSurfaceB = diff / len;
	}

	///point on A (worldspace)
	///D_btVector3 pos0 = col0->getWorldTransform().getOrigin() - radius0 * normalOnSurfaceB;
	///point on B (worldspace)
	D_btVector3 pos1 = col1->getWorldTransform().getOrigin() + radius1* normalOnSurfaceB;

	/// report a contact. internally this D_will be kept persistent, D_and contact reduction D_is done
	
	
	resultOut->addContactPoint(normalOnSurfaceB,pos1,dist);

#ifndef CLEAR_MANIFOLD
	resultOut->refreshContactPoints();
#endif //CLEAR_MANIFOLD

}

D_btScalar D_btSphereSphereCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* col0,D_btCollisionObject* col1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)col0;
	(void)col1;
	(void)dispatchInfo;
	(void)resultOut;

	//not yet
	return D_btScalar(1.);
}
