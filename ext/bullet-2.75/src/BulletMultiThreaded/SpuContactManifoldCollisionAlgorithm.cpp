/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "SpuContactManifoldCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"




void SpuContactManifoldCollisionAlgorithm::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	D_btAssert(0);
}

D_btScalar SpuContactManifoldCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	D_btAssert(0);
	return 1.f;
}

#ifndef __SPU__
SpuContactManifoldCollisionAlgorithm::SpuContactManifoldCollisionAlgorithm(const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1)
:D_btCollisionAlgorithm(ci)
#ifdef USE_SEPDISTANCE_UTIL
,m_sepDistance(body0->getCollisionShape()->getAngularMotionDisc(),body1->getCollisionShape()->getAngularMotionDisc())
#endif //USE_SEPDISTANCE_UTIL
{
	m_manifoldPtr = m_dispatcher->getNewManifold(body0,body1);
	m_shapeType0 = body0->getCollisionShape()->getShapeType();
	m_shapeType1 = body1->getCollisionShape()->getShapeType();
	m_collisionMargin0 = body0->getCollisionShape()->getMargin();
	m_collisionMargin1 = body1->getCollisionShape()->getMargin();
	m_collisionObject0 = body0;
	m_collisionObject1 = body1;

	if (body0->getCollisionShape()->isPolyhedral())
	{
		D_btPolyhedralConvexShape* convex0 = (D_btPolyhedralConvexShape*)body0->getCollisionShape();
		m_shapeDimensions0 = convex0->getImplicitShapeDimensions();
	}
	if (body1->getCollisionShape()->isPolyhedral())
	{
		D_btPolyhedralConvexShape* convex1 = (D_btPolyhedralConvexShape*)body1->getCollisionShape();
		m_shapeDimensions1 = convex1->getImplicitShapeDimensions();
	}
}
#endif //__SPU__


SpuContactManifoldCollisionAlgorithm::~SpuContactManifoldCollisionAlgorithm()
{
	if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
}
