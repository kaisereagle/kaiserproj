/*
Bullet Continuous Collision Detection D_and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2009 Sony Computer Entertainment Inc. 

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//----------------------------------------------------------------------------------------

#ifndef BTGPU3DGRIDBROADPHASE_H
#define BTGPU3DGRIDBROADPHASE_H

//----------------------------------------------------------------------------------------

#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"

#include "btGpu3DGridBroadphaseSharedTypes.h"

//----------------------------------------------------------------------------------------

///The D_btGpu3DGridBroadphase uses GPU-style code compiled for CPU D_to compute overlapping pairs

class D_btGpu3DGridBroadphase : public D_btSimpleBroadphase
{
protected:
	bool			m_bInitialized;
    unsigned int	m_numBodies;
    unsigned int	m_numCells;
	unsigned int	m_maxPairsPerBody;
	D_btScalar		m_cellFactorAABB;
    unsigned int	m_maxBodiesPerCell;
	D_bt3DGridBroadphaseParams m_params;
	D_btScalar		m_maxRadius;
	// CPU data
    unsigned int*	m_hBodiesHash;
    unsigned int*	m_hCellStart;
	unsigned int*	m_hPairBuffStartCurr;
	D_bt3DGrid3F1U*		m_hAABB;
	unsigned int*	m_hPairBuff;
	unsigned int*	m_hPairScan;
	unsigned int*	m_hPairOut;
// large proxies
	int		m_numLargeHandles;						
	int		m_maxLargeHandles;						
	int		m_LastLargeHandleIndex;							
	D_btSimpleBroadphaseProxy* m_pLargeHandles;
	void* m_pLargeHandlesRawPtr;
	int		m_firstFreeLargeHandle;
	int allocLargeHandle()
	{
		D_btAssert(m_numLargeHandles < m_maxLargeHandles);
		int freeLargeHandle = m_firstFreeLargeHandle;
		m_firstFreeLargeHandle = m_pLargeHandles[freeLargeHandle].GetNextFree();
		m_numLargeHandles++;
		if(freeLargeHandle > m_LastLargeHandleIndex)
		{
			m_LastLargeHandleIndex = freeLargeHandle;
		}
		return freeLargeHandle;
	}
	void freeLargeHandle(D_btSimpleBroadphaseProxy* proxy)
	{
		int handle = int(proxy - m_pLargeHandles);
		D_btAssert((handle >= 0) && (handle < m_maxHandles));
		if(handle == m_LastLargeHandleIndex)
		{
			m_LastLargeHandleIndex--;
		}
		proxy->SetNextFree(m_firstFreeLargeHandle);
		m_firstFreeLargeHandle = handle;
		proxy->m_clientObject = 0;
		m_numLargeHandles--;
	}
	bool isLargeProxy(const D_btVector3& aabbMin,  const D_btVector3& aabbMax);
	bool isLargeProxy(D_btBroadphaseProxy* proxy);
// D_debug
	unsigned int	m_numPairsAdded;
	unsigned int	m_numPairsRemoved;
	unsigned int	m_numOverflows;
// 
public:
	D_btGpu3DGridBroadphase(const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, 
					   int gridSizeX, int gridSizeY, int gridSizeZ, 
					   int maxSmallProxies, int maxLargeProxies, int maxPairsPerBody,
					   int maxBodiesPerCell = 8,
					   D_btScalar cellFactorAABB = D_btScalar(1.0f));
	D_btGpu3DGridBroadphase(	D_btOverlappingPairCache* overlappingPairCache,
						const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, 
						int gridSizeX, int gridSizeY, int gridSizeZ, 
						int maxSmallProxies, int maxLargeProxies, int maxPairsPerBody,
						int maxBodiesPerCell = 8,
						D_btScalar cellFactorAABB = D_btScalar(1.0f));
	virtual ~D_btGpu3DGridBroadphase();
	virtual void	calculateOverlappingPairs(D_btDispatcher* dispatcher);

	virtual D_btBroadphaseProxy*	createProxy(const D_btVector3& aabbMin,  const D_btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, D_btDispatcher* dispatcher,void* multiSapProxy);
	virtual void	destroyProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher);
	virtual void	rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback);
	virtual void	resetPool(D_btDispatcher* dispatcher);

protected:
	void _initialize(	const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, 
						int gridSizeX, int gridSizeY, int gridSizeZ, 
						int maxSmallProxies, int maxLargeProxies, int maxPairsPerBody,
						int maxBodiesPerCell = 8,
						D_btScalar cellFactorAABB = D_btScalar(1.0f));
	void _finalize();
	void addPairsToCache(D_btDispatcher* dispatcher);
	void addLarge2LargePairsToCache(D_btDispatcher* dispatcher);

// overrides for CPU version
	virtual void setParameters(D_bt3DGridBroadphaseParams* hostParams);
	virtual void prepareAABB();
	virtual void calcHashAABB();
	virtual void sortHash();	
	virtual void findCellStart();
	virtual void findOverlappingPairs();
	virtual void findPairsLarge();
	virtual void computePairCacheChanges();
	virtual void scanOverlappingPairBuff();
	virtual void squeezeOverlappingPairBuff();
};

//----------------------------------------------------------------------------------------

#endif //BTGPU3DGRIDBROADPHASE_H

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
