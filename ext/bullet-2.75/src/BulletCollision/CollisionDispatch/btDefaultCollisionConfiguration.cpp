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

#include "btDefaultCollisionConfiguration.h"

#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
#include "BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.h"
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM
#include "BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"



#include "LinearMath/btStackAlloc.h"
#include "LinearMath/btPoolAllocator.h"





D_btDefaultCollisionConfiguration::D_btDefaultCollisionConfiguration(const D_btDefaultCollisionConstructionInfo& constructionInfo)
//D_btDefaultCollisionConfiguration::D_btDefaultCollisionConfiguration(D_btStackAlloc*	stackAlloc,D_btPoolAllocator*	persistentManifoldPool,D_btPoolAllocator*	collisionAlgorithmPool)
{

	void* mem = D_btAlignedAlloc(sizeof(D_btVoronoiSimplexSolver),16);
	m_simplexSolver = new (mem)D_btVoronoiSimplexSolver();

	if (constructionInfo.m_useEpaPenetrationAlgorithm)
	{
		mem = D_btAlignedAlloc(sizeof(D_btGjkEpaPenetrationDepthSolver),16);
		m_pdSolver = new (mem)D_btGjkEpaPenetrationDepthSolver;
	}else
	{
		mem = D_btAlignedAlloc(sizeof(D_btMinkowskiPenetrationDepthSolver),16);
		m_pdSolver = new (mem)D_btMinkowskiPenetrationDepthSolver;
	}
	
	//default CreationFunctions, filling the m_doubleDispatch table
	mem = D_btAlignedAlloc(sizeof(D_btConvexConvexAlgorithm::D_CreateFunc),16);
	m_convexConvexCreateFunc = new(mem) D_btConvexConvexAlgorithm::D_CreateFunc(m_simplexSolver,m_pdSolver);
	mem = D_btAlignedAlloc(sizeof(D_btConvexConcaveCollisionAlgorithm::D_CreateFunc),16);
	m_convexConcaveCreateFunc = new (mem)D_btConvexConcaveCollisionAlgorithm::D_CreateFunc;
	mem = D_btAlignedAlloc(sizeof(D_btConvexConcaveCollisionAlgorithm::D_CreateFunc),16);
	m_swappedConvexConcaveCreateFunc = new (mem)D_btConvexConcaveCollisionAlgorithm::D_SwappedCreateFunc;
	mem = D_btAlignedAlloc(sizeof(D_btCompoundCollisionAlgorithm::D_CreateFunc),16);
	m_compoundCreateFunc = new (mem)D_btCompoundCollisionAlgorithm::D_CreateFunc;
	mem = D_btAlignedAlloc(sizeof(D_btCompoundCollisionAlgorithm::D_SwappedCreateFunc),16);
	m_swappedCompoundCreateFunc = new (mem)D_btCompoundCollisionAlgorithm::D_SwappedCreateFunc;
	mem = D_btAlignedAlloc(sizeof(D_btEmptyAlgorithm::D_CreateFunc),16);
	m_emptyCreateFunc = new(mem) D_btEmptyAlgorithm::D_CreateFunc;
	
	mem = D_btAlignedAlloc(sizeof(D_btSphereSphereCollisionAlgorithm::D_CreateFunc),16);
	m_sphereSphereCF = new(mem) D_btSphereSphereCollisionAlgorithm::D_CreateFunc;
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	mem = D_btAlignedAlloc(sizeof(D_btSphereBoxCollisionAlgorithm::D_CreateFunc),16);
	m_sphereBoxCF = new(mem) D_btSphereBoxCollisionAlgorithm::D_CreateFunc;
	mem = D_btAlignedAlloc(sizeof(D_btSphereBoxCollisionAlgorithm::D_CreateFunc),16);
	m_boxSphereCF = new (mem)D_btSphereBoxCollisionAlgorithm::D_CreateFunc;
	m_boxSphereCF->m_swapped = true;
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM

	mem = D_btAlignedAlloc(sizeof(D_btSphereTriangleCollisionAlgorithm::D_CreateFunc),16);
	m_sphereTriangleCF = new (mem)D_btSphereTriangleCollisionAlgorithm::D_CreateFunc;
	mem = D_btAlignedAlloc(sizeof(D_btSphereTriangleCollisionAlgorithm::D_CreateFunc),16);
	m_triangleSphereCF = new (mem)D_btSphereTriangleCollisionAlgorithm::D_CreateFunc;
	m_triangleSphereCF->m_swapped = true;
	
	mem = D_btAlignedAlloc(sizeof(D_btBoxBoxCollisionAlgorithm::D_CreateFunc),16);
	m_boxBoxCF = new(mem)D_btBoxBoxCollisionAlgorithm::D_CreateFunc;

	//convex versus plane
	mem = D_btAlignedAlloc (sizeof(D_btConvexPlaneCollisionAlgorithm::D_CreateFunc),16);
	m_convexPlaneCF = new (mem) D_btConvexPlaneCollisionAlgorithm::D_CreateFunc;
	mem = D_btAlignedAlloc (sizeof(D_btConvexPlaneCollisionAlgorithm::D_CreateFunc),16);
	m_planeConvexCF = new (mem) D_btConvexPlaneCollisionAlgorithm::D_CreateFunc;
	m_planeConvexCF->m_swapped = true;
	
	///calculate maximum element size, D_big enough D_to fit any collision algorithm in the memory pool
	int maxSize = sizeof(D_btConvexConvexAlgorithm);
	int maxSize2 = sizeof(D_btConvexConcaveCollisionAlgorithm);
	int maxSize3 = sizeof(D_btCompoundCollisionAlgorithm);
	int sl = sizeof(D_btConvexSeparatingDistanceUtil);
	sl = sizeof(D_btGjkPairDetector);
	int	collisionAlgorithmMaxElementSize = D_btMax(maxSize,constructionInfo.m_customCollisionAlgorithmMaxElementSize);
	collisionAlgorithmMaxElementSize = D_btMax(collisionAlgorithmMaxElementSize,maxSize2);
	collisionAlgorithmMaxElementSize = D_btMax(collisionAlgorithmMaxElementSize,maxSize3);

	if (constructionInfo.m_stackAlloc)
	{
		m_ownsStackAllocator = false;
		this->m_stackAlloc = constructionInfo.m_stackAlloc;
	} else
	{
		m_ownsStackAllocator = true;
		void* mem = D_btAlignedAlloc(sizeof(D_btStackAlloc),16);
		m_stackAlloc = new(mem)D_btStackAlloc(constructionInfo.m_defaultStackAllocatorSize);
	}
		
	if (constructionInfo.m_persistentManifoldPool)
	{
		m_ownsPersistentManifoldPool = false;
		m_persistentManifoldPool = constructionInfo.m_persistentManifoldPool;
	} else
	{
		m_ownsPersistentManifoldPool = true;
		void* mem = D_btAlignedAlloc(sizeof(D_btPoolAllocator),16);
		m_persistentManifoldPool = new (mem) D_btPoolAllocator(sizeof(D_btPersistentManifold),constructionInfo.m_defaultMaxPersistentManifoldPoolSize);
	}
	
	if (constructionInfo.m_collisionAlgorithmPool)
	{
		m_ownsCollisionAlgorithmPool = false;
		m_collisionAlgorithmPool = constructionInfo.m_collisionAlgorithmPool;
	} else
	{
		m_ownsCollisionAlgorithmPool = true;
		void* mem = D_btAlignedAlloc(sizeof(D_btPoolAllocator),16);
		m_collisionAlgorithmPool = new(mem) D_btPoolAllocator(collisionAlgorithmMaxElementSize,constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize);
	}


}

D_btDefaultCollisionConfiguration::~D_btDefaultCollisionConfiguration()
{
	if (m_ownsStackAllocator)
	{
		m_stackAlloc->destroy();
		m_stackAlloc->~D_btStackAlloc();
		D_btAlignedFree(m_stackAlloc);
	}
	if (m_ownsCollisionAlgorithmPool)
	{
		m_collisionAlgorithmPool->~D_btPoolAllocator();
		D_btAlignedFree(m_collisionAlgorithmPool);
	}
	if (m_ownsPersistentManifoldPool)
	{
		m_persistentManifoldPool->~D_btPoolAllocator();
		D_btAlignedFree(m_persistentManifoldPool);
	}

	m_convexConvexCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree(	m_convexConvexCreateFunc);

	m_convexConcaveCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_convexConcaveCreateFunc);
	m_swappedConvexConcaveCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_swappedConvexConcaveCreateFunc);

	m_compoundCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_compoundCreateFunc);

	m_swappedCompoundCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_swappedCompoundCreateFunc);

	m_emptyCreateFunc->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_emptyCreateFunc);

	m_sphereSphereCF->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_sphereSphereCF);

#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	m_sphereBoxCF->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_sphereBoxCF);
	m_boxSphereCF->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_boxSphereCF);
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM

	m_sphereTriangleCF->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_sphereTriangleCF);
	m_triangleSphereCF->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_triangleSphereCF);
	m_boxBoxCF->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_boxBoxCF);

	m_convexPlaneCF->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_convexPlaneCF);
	m_planeConvexCF->~D_btCollisionAlgorithmCreateFunc();
	D_btAlignedFree( m_planeConvexCF);

	m_simplexSolver->~D_btVoronoiSimplexSolver();
	D_btAlignedFree(m_simplexSolver);

	m_pdSolver->~D_btConvexPenetrationDepthSolver();
	
	D_btAlignedFree(m_pdSolver);


}


D_btCollisionAlgorithmCreateFunc* D_btDefaultCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1)
{



	if ((proxyType0 == D_SPHERE_SHAPE_PROXYTYPE) && (proxyType1==D_SPHERE_SHAPE_PROXYTYPE))
	{
		return	m_sphereSphereCF;
	}
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	if ((proxyType0 == D_SPHERE_SHAPE_PROXYTYPE) && (proxyType1==D_BOX_SHAPE_PROXYTYPE))
	{
		return	m_sphereBoxCF;
	}

	if ((proxyType0 == D_BOX_SHAPE_PROXYTYPE ) && (proxyType1==D_SPHERE_SHAPE_PROXYTYPE))
	{
		return	m_boxSphereCF;
	}
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM


	if ((proxyType0 == D_SPHERE_SHAPE_PROXYTYPE ) && (proxyType1==D_TRIANGLE_SHAPE_PROXYTYPE))
	{
		return	m_sphereTriangleCF;
	}

	if ((proxyType0 == D_TRIANGLE_SHAPE_PROXYTYPE  ) && (proxyType1==D_SPHERE_SHAPE_PROXYTYPE))
	{
		return	m_triangleSphereCF;
	} 

	if ((proxyType0 == D_BOX_SHAPE_PROXYTYPE) && (proxyType1 == D_BOX_SHAPE_PROXYTYPE))
	{
		return m_boxBoxCF;
	}
	
	if (D_btBroadphaseProxy::isConvex(proxyType0) && (proxyType1 == D_STATIC_PLANE_PROXYTYPE))
	{
		return m_convexPlaneCF;
	}

	if (D_btBroadphaseProxy::isConvex(proxyType1) && (proxyType0 == D_STATIC_PLANE_PROXYTYPE))
	{
		return m_planeConvexCF;
	}
	


	if (D_btBroadphaseProxy::isConvex(proxyType0) && D_btBroadphaseProxy::isConvex(proxyType1))
	{
		return m_convexConvexCreateFunc;
	}

	if (D_btBroadphaseProxy::isConvex(proxyType0) && D_btBroadphaseProxy::isConcave(proxyType1))
	{
		return m_convexConcaveCreateFunc;
	}

	if (D_btBroadphaseProxy::isConvex(proxyType1) && D_btBroadphaseProxy::isConcave(proxyType0))
	{
		return m_swappedConvexConcaveCreateFunc;
	}

	if (D_btBroadphaseProxy::isCompound(proxyType0))
	{
		return m_compoundCreateFunc;
	} else
	{
		if (D_btBroadphaseProxy::isCompound(proxyType1))
		{
			return m_swappedCompoundCreateFunc;
		}
	}

	//failed D_to find an algorithm
	return m_emptyCreateFunc;
}

void D_btDefaultCollisionConfiguration::setConvexConvexMultipointIterations(int numPerturbationIterations, int minimumPointsPerturbationThreshold)
{
	D_btConvexConvexAlgorithm::D_CreateFunc* convexConvex = (D_btConvexConvexAlgorithm::D_CreateFunc*) m_convexConvexCreateFunc;
	convexConvex->m_numPerturbationIterations = numPerturbationIterations;
	convexConvex->m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
}
