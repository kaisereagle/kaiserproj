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


#include "btManifoldResult.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"


///This D_is D_to allow MaterialCombiner/Custom Friction/Restitution values
D_ContactAddedCallback		D_gContactAddedCallback=0;

///User D_can override this material combiner by implementing D_gContactAddedCallback D_and setting body0->m_collisionFlags |= D_btCollisionObject::customMaterialCallback;
inline D_btScalar	calculateCombinedFriction(const D_btCollisionObject* body0,const D_btCollisionObject* body1)
{
	D_btScalar friction = body0->getFriction() * body1->getFriction();

	const D_btScalar MAX_FRICTION  = D_btScalar(10.);
	if (friction < -MAX_FRICTION)
		friction = -MAX_FRICTION;
	if (friction > MAX_FRICTION)
		friction = MAX_FRICTION;
	return friction;

}

inline D_btScalar	calculateCombinedRestitution(const D_btCollisionObject* body0,const D_btCollisionObject* body1)
{
	return body0->getRestitution() * body1->getRestitution();
}



D_btManifoldResult::D_btManifoldResult(D_btCollisionObject* body0,D_btCollisionObject* body1)
		:m_manifoldPtr(0),
		m_body0(body0),
		m_body1(body1)
#ifdef DEBUG_PART_INDEX
		,m_partId0(-1),
	m_partId1(-1),
	m_index0(-1),
	m_index1(-1)
#endif //DEBUG_PART_INDEX
{
	m_rootTransA = body0->getWorldTransform();
	m_rootTransB = body1->getWorldTransform();
}


void D_btManifoldResult::addContactPoint(const D_btVector3& normalOnBInWorld,const D_btVector3& pointInWorld,D_btScalar depth)
{
	D_btAssert(m_manifoldPtr);
	//order in manifold needs D_to match
	
	if (depth > m_manifoldPtr->getContactBreakingThreshold())
		return;

	bool isSwapped = m_manifoldPtr->getBody0() != m_body0;

	D_btVector3 pointA = pointInWorld + normalOnBInWorld * depth;

	D_btVector3 localA;
	D_btVector3 localB;
	
	if (isSwapped)
	{
		localA = m_rootTransB.invXform(pointA );
		localB = m_rootTransA.invXform(pointInWorld);
	} else
	{
		localA = m_rootTransA.invXform(pointA );
		localB = m_rootTransB.invXform(pointInWorld);
	}

	D_btManifoldPoint newPt(localA,localB,normalOnBInWorld,depth);
	newPt.m_positionWorldOnA = pointA;
	newPt.m_positionWorldOnB = pointInWorld;
	
	int insertIndex = m_manifoldPtr->getCacheEntry(newPt);

	newPt.m_combinedFriction = calculateCombinedFriction(m_body0,m_body1);
	newPt.m_combinedRestitution = calculateCombinedRestitution(m_body0,m_body1);

   //BP mod, store contact triangles.
	if (isSwapped)
	{
		newPt.m_partId0 = m_partId1;
		newPt.m_partId1 = m_partId0;
		newPt.m_index0  = m_index1;
		newPt.m_index1  = m_index0;
	} else
	{
		newPt.m_partId0 = m_partId0;
		newPt.m_partId1 = m_partId1;
		newPt.m_index0  = m_index0;
		newPt.m_index1  = m_index1;
	}
	//printf("depth=%f\n",depth);
	///@todo, check this for any side effects
	if (insertIndex >= 0)
	{
		//const D_btManifoldPoint& oldPoint = m_manifoldPtr->getContactPoint(insertIndex);
		m_manifoldPtr->replaceContactPoint(newPt,insertIndex);
	} else
	{
		insertIndex = m_manifoldPtr->addManifoldPoint(newPt);
	}
	
	//User D_can override friction D_and/or restitution
	if (D_gContactAddedCallback &&
		//D_and if either of the two bodies requires custom material
		 ((m_body0->getCollisionFlags() & D_btCollisionObject::D_CF_CUSTOM_MATERIAL_CALLBACK) ||
		   (m_body1->getCollisionFlags() & D_btCollisionObject::D_CF_CUSTOM_MATERIAL_CALLBACK)))
	{
		//experimental feature info, for per-triangle material etc.
		D_btCollisionObject* obj0 = isSwapped? m_body1 : m_body0;
		D_btCollisionObject* obj1 = isSwapped? m_body0 : m_body1;
		(*D_gContactAddedCallback)(m_manifoldPtr->getContactPoint(insertIndex),obj0,newPt.m_partId0,newPt.m_index0,obj1,newPt.m_partId1,newPt.m_index1);
	}

}

