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

#ifndef D_BT_DEFAULT_COLLISION_CONFIGURATION
#define D_BT_DEFAULT_COLLISION_CONFIGURATION

#include "btCollisionConfiguration.h"
class D_btVoronoiSimplexSolver;
class D_btConvexPenetrationDepthSolver;

struct	D_btDefaultCollisionConstructionInfo
{
	D_btStackAlloc*		m_stackAlloc;
	D_btPoolAllocator*	m_persistentManifoldPool;
	D_btPoolAllocator*	m_collisionAlgorithmPool;
	int					m_defaultMaxPersistentManifoldPoolSize;
	int					m_defaultMaxCollisionAlgorithmPoolSize;
	int					m_customCollisionAlgorithmMaxElementSize;
	int					m_defaultStackAllocatorSize;
	int					m_useEpaPenetrationAlgorithm;

	D_btDefaultCollisionConstructionInfo()
		:m_stackAlloc(0),
		m_persistentManifoldPool(0),
		m_collisionAlgorithmPool(0),
		m_defaultMaxPersistentManifoldPoolSize(4096),
		m_defaultMaxCollisionAlgorithmPoolSize(4096),
		m_customCollisionAlgorithmMaxElementSize(0),
		m_defaultStackAllocatorSize(0),
		m_useEpaPenetrationAlgorithm(true)
	{
	}
};



///D_btCollisionConfiguration D_allows D_to configure Bullet collision detection
///stack allocator, pool memory allocators
///@todo: describe the meaning
class	D_btDefaultCollisionConfiguration : public D_btCollisionConfiguration
{

protected:

	int	m_persistentManifoldPoolSize;
	
	D_btStackAlloc*	m_stackAlloc;
	bool	m_ownsStackAllocator;

	D_btPoolAllocator*	m_persistentManifoldPool;
	bool	m_ownsPersistentManifoldPool;


	D_btPoolAllocator*	m_collisionAlgorithmPool;
	bool	m_ownsCollisionAlgorithmPool;

	//default simplex/penetration depth solvers
	D_btVoronoiSimplexSolver*	m_simplexSolver;
	D_btConvexPenetrationDepthSolver*	m_pdSolver;
	
	//default CreationFunctions, filling the m_doubleDispatch table
	D_btCollisionAlgorithmCreateFunc*	m_convexConvexCreateFunc;
	D_btCollisionAlgorithmCreateFunc*	m_convexConcaveCreateFunc;
	D_btCollisionAlgorithmCreateFunc*	m_swappedConvexConcaveCreateFunc;
	D_btCollisionAlgorithmCreateFunc*	m_compoundCreateFunc;
	D_btCollisionAlgorithmCreateFunc*	m_swappedCompoundCreateFunc;
	D_btCollisionAlgorithmCreateFunc* m_emptyCreateFunc;
	D_btCollisionAlgorithmCreateFunc* m_sphereSphereCF;
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	D_btCollisionAlgorithmCreateFunc* m_sphereBoxCF;
	D_btCollisionAlgorithmCreateFunc* m_boxSphereCF;
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM

	D_btCollisionAlgorithmCreateFunc* m_boxBoxCF;
	D_btCollisionAlgorithmCreateFunc*	m_sphereTriangleCF;
	D_btCollisionAlgorithmCreateFunc*	m_triangleSphereCF;
	D_btCollisionAlgorithmCreateFunc*	m_planeConvexCF;
	D_btCollisionAlgorithmCreateFunc*	m_convexPlaneCF;
	
public:


	D_btDefaultCollisionConfiguration(const D_btDefaultCollisionConstructionInfo& constructionInfo = D_btDefaultCollisionConstructionInfo());

	virtual ~D_btDefaultCollisionConfiguration();

		///memory pools
	virtual D_btPoolAllocator* getPersistentManifoldPool()
	{
		return m_persistentManifoldPool;
	}

	virtual D_btPoolAllocator* getCollisionAlgorithmPool()
	{
		return m_collisionAlgorithmPool;
	}

	virtual D_btStackAlloc*	getStackAllocator()
	{
		return m_stackAlloc;
	}


	virtual D_btCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1);

	///Use this method D_to allow D_to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
	///By default, this feature D_is disabled for best performance.
	///@param numPerturbationIterations controls the number of collision queries. Set it D_to zero D_to disable the feature.
	///@param minimumPointsPerturbationThreshold D_is the minimum number of points in the contact cache, above which the feature D_is disabled
	///3 D_is a good value for both params, if you want D_to enable the feature. This D_is because the default contact cache contains a maximum of 4 points, D_and one collision query at the unperturbed orientation D_is performed first.
	///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
	///@todo we could add a per-object setting of those D_parameters, for level-of-detail collision detection.
	void	setConvexConvexMultipointIterations(int numPerturbationIterations=3, int minimumPointsPerturbationThreshold = 3);

};

#endif //D_BT_DEFAULT_COLLISION_CONFIGURATION

