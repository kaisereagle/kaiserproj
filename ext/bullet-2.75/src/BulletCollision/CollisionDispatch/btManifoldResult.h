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


#ifndef MANIFOLD_RESULT_H
#define MANIFOLD_RESULT_H

class D_btCollisionObject;
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class D_btManifoldPoint;

#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"

#include "LinearMath/btTransform.h"

typedef bool (*D_ContactAddedCallback)(D_btManifoldPoint& cp,	const D_btCollisionObject* colObj0,int partId0,int index0,const D_btCollisionObject* colObj1,int partId1,int index1);
extern D_ContactAddedCallback		D_gContactAddedCallback;

//#define DEBUG_PART_INDEX 1


///D_btManifoldResult D_is a helper class D_to manage  contact results.
class D_btManifoldResult : public D_btDiscreteCollisionDetectorInterface::D_Result
{
	D_btPersistentManifold* m_manifoldPtr;

	//we D_need this for compounds
	D_btTransform	m_rootTransA;
	D_btTransform	m_rootTransB;

	D_btCollisionObject* m_body0;
	D_btCollisionObject* m_body1;
	int	m_partId0;
	int m_partId1;
	int m_index0;
	int m_index1;
	

public:

	D_btManifoldResult()
#ifdef DEBUG_PART_INDEX
		:
	m_partId0(-1),
	m_partId1(-1),
	m_index0(-1),
	m_index1(-1)
#endif //DEBUG_PART_INDEX
	{
	}

	D_btManifoldResult(D_btCollisionObject* body0,D_btCollisionObject* body1);

	virtual ~D_btManifoldResult() {};

	void	setPersistentManifold(D_btPersistentManifold* manifoldPtr)
	{
		m_manifoldPtr = manifoldPtr;
	}

	const D_btPersistentManifold*	getPersistentManifold() const
	{
		return m_manifoldPtr;
	}
	D_btPersistentManifold*	getPersistentManifold()
	{
		return m_manifoldPtr;
	}

	virtual void setShapeIdentifiersA(int partId0,int index0)
	{
		m_partId0=partId0;
		m_index0=index0;
	}

	virtual void setShapeIdentifiersB(	int partId1,int index1)
	{
		m_partId1=partId1;
		m_index1=index1;
	}


	virtual void addContactPoint(const D_btVector3& normalOnBInWorld,const D_btVector3& pointInWorld,D_btScalar depth);

	D_SIMD_FORCE_INLINE	void refreshContactPoints()
	{
		D_btAssert(m_manifoldPtr);
		if (!m_manifoldPtr->getNumContacts())
			return;

		bool isSwapped = m_manifoldPtr->getBody0() != m_body0;

		if (isSwapped)
		{
			m_manifoldPtr->refreshContactPoints(m_rootTransB,m_rootTransA);
		} else
		{
			m_manifoldPtr->refreshContactPoints(m_rootTransA,m_rootTransB);
		}
	}

	const D_btCollisionObject* getBody0Internal() const
	{
		return m_body0;
	}

	const D_btCollisionObject* getBody1Internal() const
	{
		return m_body1;
	}
	
};

#endif //MANIFOLD_RESULT_H
