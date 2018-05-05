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

#include "btSoftBodyRigidBodyCollisionConfiguration.h"
#include "btSoftRigidCollisionAlgorithm.h"
#include "btSoftBodyConcaveCollisionAlgorithm.h"
#include "btSoftSoftCollisionAlgorithm.h"

#include "LinearMath/btPoolAllocator.h"

#define D_ENABLE_SOFTBODY_CONCAVE_COLLISIONS 1

D_btSoftBodyRigidBodyCollisionConfiguration::D_btSoftBodyRigidBodyCollisionConfiguration(const D_btDefaultCollisionConstructionInfo& constructionInfo)
:D_btDefaultCollisionConfiguration(constructionInfo)
{
	void* mem;

	mem = D_btAlignedAlloc(sizeof(D_btSoftSoftCollisionAlgorithm::D_CreateFunc),16);
	m_softSoftCreateFunc = new(mem) D_btSoftSoftCollisionAlgorithm::D_CreateFunc;

	mem = D_btAlignedAlloc(sizeof(D_btSoftRigidCollisionAlgorithm::D_CreateFunc),16);
	m_softRigidConvexCreateFunc = new(mem) D_btSoftRigidCollisionAlgorithm::D_CreateFunc;

	mem = D_btAlignedAlloc(sizeof(D_btSoftRigidCollisionAlgorithm::D_CreateFunc),16);
	m_swappedSoftRigidConvexCreateFunc = new(mem) D_btSoftRigidCollisionAlgorithm::D_CreateFunc;
	m_swappedSoftRigidConvexCreateFunc->m_swapped=true;

#ifdef D_ENABLE_SOFTBODY_CONCAVE_COLLISIONS
	mem = D_btAlignedAlloc(sizeof(D_btSoftBodyConcaveCollisionAlgorithm::D_CreateFunc),16);
	m_softRigidConcaveCreateFunc = new(mem) D_btSoftBodyConcaveCollisionAlgorithm::D_CreateFunc;

	mem = D_btAlignedAlloc(sizeof(D_btSoftBodyConcaveCollisionAlgorithm::D_CreateFunc),16);
	m_swappedSoftRigidConcaveCreateFunc = new(mem) D_btSoftBodyConcaveCollisionAlgorithm::D_SwappedCreateFunc;
	m_swappedSoftRigidConcaveCreateFunc->m_swapped=true;
#endif

	//replace pool by a new one, with potential larger size

	if (m_ownsCollisionAlgorithmPool && m_collisionAlgorithmPool)
	{
		int curElemSize = m_collisionAlgorithmPool->getElementSize();
		///calculate maximum element size, D_big enough D_to fit any collision algorithm in the memory pool


		int maxSize0 = sizeof(D_btSoftSoftCollisionAlgorithm);
		int maxSize1 = sizeof(D_btSoftRigidCollisionAlgorithm);
		int maxSize2 = sizeof(D_btSoftBodyConcaveCollisionAlgorithm);

		int	collisionAlgorithmMaxElementSize = D_btMax(maxSize0,maxSize1);
		collisionAlgorithmMaxElementSize = D_btMax(collisionAlgorithmMaxElementSize,maxSize2);
		
		if (collisionAlgorithmMaxElementSize > curElemSize)
		{
			m_collisionAlgorithmPool->~D_btPoolAllocator();
			D_btAlignedFree(m_collisionAlgorithmPool);
			void* mem = D_btAlignedAlloc(sizeof(D_btPoolAllocator),16);
			m_collisionAlgorithmPool = new(mem) D_btPoolAllocator(collisionAlgorithmMaxElementSize,constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize);
		}
	}

}

D_btSoftBodyRigidBodyCollisionConfiguration::~D_btSoftBodyRigidBodyCollisionConfiguration()
{
	m_softSoftCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree(	m_softSoftCreateFunc);

	m_softRigidConvexCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree(	m_softRigidConvexCreateFunc);

	m_swappedSoftRigidConvexCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree(	m_swappedSoftRigidConvexCreateFunc);

#ifdef D_ENABLE_SOFTBODY_CONCAVE_COLLISIONS
	m_softRigidConcaveCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree(	m_softRigidConcaveCreateFunc);

	m_swappedSoftRigidConcaveCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree(	m_swappedSoftRigidConcaveCreateFunc);
#endif
}

///creation of soft-soft D_and soft-rigid, D_and otherwise fallback D_to base class implementation
D_btCollisionAlgorithmCreateFunc* D_btSoftBodyRigidBodyCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1)
{

	///try D_to handle the softbody interactions first

	if ((proxyType0 == D_SOFTBODY_SHAPE_PROXYTYPE  ) && (proxyType1==D_SOFTBODY_SHAPE_PROXYTYPE))
	{
		return	m_softSoftCreateFunc;
	}

	///softbody versus convex
	if (proxyType0 == D_SOFTBODY_SHAPE_PROXYTYPE  && D_btBroadphaseProxy::isConvex(proxyType1))
	{
		return	m_softRigidConvexCreateFunc;
	}

	///convex versus soft body
	if (D_btBroadphaseProxy::isConvex(proxyType0) && proxyType1 == D_SOFTBODY_SHAPE_PROXYTYPE )
	{
		return	m_swappedSoftRigidConvexCreateFunc;
	}

#ifdef D_ENABLE_SOFTBODY_CONCAVE_COLLISIONS
	///softbody versus convex
	if (proxyType0 == D_SOFTBODY_SHAPE_PROXYTYPE  && D_btBroadphaseProxy::isConcave(proxyType1))
	{
		return	m_softRigidConcaveCreateFunc;
	}

	///convex versus soft body
	if (D_btBroadphaseProxy::isConcave(proxyType0) && proxyType1 == D_SOFTBODY_SHAPE_PROXYTYPE )
	{
		return	m_swappedSoftRigidConcaveCreateFunc;
	}
#endif

	///fallback D_to the regular rigid collision shape
	return D_btDefaultCollisionConfiguration::getCollisionAlgorithmCreateFunc(proxyType0,proxyType1);
}
