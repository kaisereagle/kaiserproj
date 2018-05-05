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

///The 3 following lines include the CPU implementation of the kernels, keep them in this order.
#include "BulletMultiThreaded/btGpuDefines.h"
#include "BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#include "BulletMultiThreaded/btGpuUtilsSharedCode.h"



#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btQuickprof.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"



#include "btGpuDefines.h"
#include "btGpuUtilsSharedDefs.h"

#include "btGpu3DGridBroadphaseSharedDefs.h"

#include "btGpu3DGridBroadphase.h"
#include <string.h> //for memset


#include <stdio.h>



static D_bt3DGridBroadphaseParams s3DGridBroadphaseParams;



D_btGpu3DGridBroadphase::D_btGpu3DGridBroadphase(	const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, 
										int gridSizeX, int gridSizeY, int gridSizeZ, 
										int maxSmallProxies, int maxLargeProxies, int maxPairsPerBody,
										int maxBodiesPerCell,
										D_btScalar cellFactorAABB) :
	D_btSimpleBroadphase(maxSmallProxies,
//				     new (D_btAlignedAlloc(sizeof(D_btSortedOverlappingPairCache),16)) D_btSortedOverlappingPairCache),
				     new (D_btAlignedAlloc(sizeof(D_btHashedOverlappingPairCache),16)) D_btHashedOverlappingPairCache),
	m_bInitialized(false),
    m_numBodies(0)
{
	_initialize(worldAabbMin, worldAabbMax, gridSizeX, gridSizeY, gridSizeZ, 
				maxSmallProxies, maxLargeProxies, maxPairsPerBody,
				maxBodiesPerCell, cellFactorAABB);
}



D_btGpu3DGridBroadphase::D_btGpu3DGridBroadphase(	D_btOverlappingPairCache* overlappingPairCache,
										const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, 
										int gridSizeX, int gridSizeY, int gridSizeZ, 
										int maxSmallProxies, int maxLargeProxies, int maxPairsPerBody,
										int maxBodiesPerCell,
										D_btScalar cellFactorAABB) :
	D_btSimpleBroadphase(maxSmallProxies, overlappingPairCache),
	m_bInitialized(false),
    m_numBodies(0)
{
	_initialize(worldAabbMin, worldAabbMax, gridSizeX, gridSizeY, gridSizeZ, 
				maxSmallProxies, maxLargeProxies, maxPairsPerBody,
				maxBodiesPerCell, cellFactorAABB);
}



D_btGpu3DGridBroadphase::~D_btGpu3DGridBroadphase()
{
	//D_btSimpleBroadphase D_will free memory of D_btSortedOverlappingPairCache, because m_ownsPairCache
	assert(m_bInitialized);
	_finalize();
}



void D_btGpu3DGridBroadphase::_initialize(	const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, 
										int gridSizeX, int gridSizeY, int gridSizeZ, 
										int maxSmallProxies, int maxLargeProxies, int maxPairsPerBody,
										int maxBodiesPerCell,
										D_btScalar cellFactorAABB)
{
	// set various paramerers
	m_ownsPairCache = true;
	m_params.m_gridSizeX = gridSizeX;
	m_params.m_gridSizeY = gridSizeY;
	m_params.m_gridSizeZ = gridSizeZ;
	m_params.m_numCells = m_params.m_gridSizeX * m_params.m_gridSizeY * m_params.m_gridSizeZ;
	D_btVector3 w_org = worldAabbMin;
	m_params.m_worldOriginX = w_org.getX();
	m_params.m_worldOriginY = w_org.getY();
	m_params.m_worldOriginZ = w_org.getZ();
	D_btVector3 w_size = worldAabbMax - worldAabbMin;
	m_params.m_cellSizeX = w_size.getX() / m_params.m_gridSizeX;
	m_params.m_cellSizeY = w_size.getY() / m_params.m_gridSizeY;
	m_params.m_cellSizeZ = w_size.getZ() / m_params.m_gridSizeZ;
	m_maxRadius = D_btMin(D_btMin(m_params.m_cellSizeX, m_params.m_cellSizeY), m_params.m_cellSizeZ);
	m_maxRadius *= D_btScalar(0.5f);
	m_params.m_numBodies = m_numBodies;
	m_params.m_maxBodiesPerCell = maxBodiesPerCell;

	m_numLargeHandles = 0;						
	m_maxLargeHandles = maxLargeProxies;

	m_maxPairsPerBody = maxPairsPerBody;

	m_cellFactorAABB = cellFactorAABB;

	m_LastLargeHandleIndex = -1;

    assert(!m_bInitialized);
    // allocate host storage
    m_hBodiesHash = new unsigned int[m_maxHandles * 2];
    memset(m_hBodiesHash, 0x00, m_maxHandles*2*sizeof(unsigned int));

    m_hCellStart = new unsigned int[m_params.m_numCells];
    memset(m_hCellStart, 0x00, m_params.m_numCells * sizeof(unsigned int));

	m_hPairBuffStartCurr = new unsigned int[m_maxHandles * 2 + 2];
	// --------------- for now, init with m_maxPairsPerBody for each body
	m_hPairBuffStartCurr[0] = 0;
	m_hPairBuffStartCurr[1] = 0;
	for(int i = 1; i <= m_maxHandles; i++) 
	{
		m_hPairBuffStartCurr[i * 2] = m_hPairBuffStartCurr[(i-1) * 2] + m_maxPairsPerBody;
		m_hPairBuffStartCurr[i * 2 + 1] = 0;
	}
	//----------------
	unsigned int numAABB = m_maxHandles + m_maxLargeHandles;
	m_hAABB = new D_bt3DGrid3F1U[numAABB * 2]; // AABB Min & Max

	m_hPairBuff = new unsigned int[m_maxHandles * m_maxPairsPerBody];
	memset(m_hPairBuff, 0x00, m_maxHandles * m_maxPairsPerBody * sizeof(unsigned int)); // needed?

	m_hPairScan = new unsigned int[m_maxHandles + 1];

	m_hPairOut = new unsigned int[m_maxHandles * m_maxPairsPerBody];

// large proxies

	// allocate handles buffer D_and put all handles on free list
	m_pLargeHandlesRawPtr = D_btAlignedAlloc(sizeof(D_btSimpleBroadphaseProxy) * m_maxLargeHandles, 16);
	m_pLargeHandles = new(m_pLargeHandlesRawPtr) D_btSimpleBroadphaseProxy[m_maxLargeHandles];
	m_firstFreeLargeHandle = 0;
	{
		for (int i = m_firstFreeLargeHandle; i < m_maxLargeHandles; i++)
		{
			m_pLargeHandles[i].SetNextFree(i + 1);
			m_pLargeHandles[i].m_uniqueId = m_maxHandles+2+i;
		}
		m_pLargeHandles[m_maxLargeHandles - 1].SetNextFree(0);
	}

// D_debug data
	m_numPairsAdded = 0;
	m_numOverflows = 0;

    m_bInitialized = true;
}



void D_btGpu3DGridBroadphase::_finalize()
{
    assert(m_bInitialized);
    delete [] m_hBodiesHash;
    delete [] m_hCellStart;
    delete [] m_hPairBuffStartCurr;
    delete [] m_hAABB;
	delete [] m_hPairBuff;
	delete [] m_hPairScan;
	delete [] m_hPairOut;
	D_btAlignedFree(m_pLargeHandlesRawPtr);
	m_bInitialized = false;
}



void D_btGpu3DGridBroadphase::calculateOverlappingPairs(D_btDispatcher* dispatcher)
{
	if(m_numHandles <= 0)
	{
		D_BT_PROFILE("addLarge2LargePairsToCache");
		addLarge2LargePairsToCache(dispatcher);
		return;
	}
	// update constants
	setParameters(&m_params);
	// prepare AABB array
	prepareAABB();
	// calculate hash
	calcHashAABB();
	// sort bodies based on hash
	sortHash();
	// find start of each cell
	findCellStart();
	// findOverlappingPairs (small/small)
	findOverlappingPairs();
	// findOverlappingPairs (small/large)
	findPairsLarge();
	// add pairs D_to CPU cache
	computePairCacheChanges();
	scanOverlappingPairBuff();
	squeezeOverlappingPairBuff();
	addPairsToCache(dispatcher);
	// find D_and add large/large pairs D_to CPU cache
	addLarge2LargePairsToCache(dispatcher);
	return;
}



void D_btGpu3DGridBroadphase::addPairsToCache(D_btDispatcher* dispatcher)
{
	m_numPairsAdded = 0;
	m_numPairsRemoved = 0;
	for(int i = 0; i < m_numHandles; i++) 
	{
		unsigned int num = m_hPairScan[i+1] - m_hPairScan[i];
		if(!num)
		{
			continue;
		}
		unsigned int* pInp = m_hPairOut + m_hPairScan[i];
		unsigned int index0 = m_hAABB[i * 2].uw;
		D_btSimpleBroadphaseProxy* proxy0 = &m_pHandles[index0];
		for(unsigned int j = 0; j < num; j++)
		{
			unsigned int indx1_s = pInp[j];
			unsigned int index1 = indx1_s & (~D_BT_3DGRID_PAIR_ANY_FLG);
			D_btSimpleBroadphaseProxy* proxy1;
			if(index1 < (unsigned int)m_maxHandles)
			{
				proxy1 = &m_pHandles[index1];
			}
			else
			{
				index1 -= m_maxHandles;
				D_btAssert((index1 >= 0) && (index1 < (unsigned int)m_maxLargeHandles));
				proxy1 = &m_pLargeHandles[index1];
			}
			if(indx1_s & D_BT_3DGRID_PAIR_NEW_FLG)
			{
				m_pairCache->addOverlappingPair(proxy0,proxy1);
				m_numPairsAdded++;
			}
			else
			{
				m_pairCache->removeOverlappingPair(proxy0,proxy1,dispatcher);
				m_numPairsRemoved++;
			}
		}
	}
}



D_btBroadphaseProxy* D_btGpu3DGridBroadphase::createProxy(  const D_btVector3& aabbMin,  const D_btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, D_btDispatcher* dispatcher,void* multiSapProxy)
{
	D_btBroadphaseProxy*  proxy;
	bool bIsLarge = isLargeProxy(aabbMin, aabbMax);
	if(bIsLarge)
	{
		if (m_numLargeHandles >= m_maxLargeHandles)
		{
			///you have D_to increase the cell size, so 'large' proxies become 'small' proxies (fitting a cell)
			D_btAssert(0);
			return 0; //D_should never happen, but don't let the game crash ;-)
		}
		D_btAssert((aabbMin[0]<= aabbMax[0]) && (aabbMin[1]<= aabbMax[1]) && (aabbMin[2]<= aabbMax[2]));
		int newHandleIndex = allocLargeHandle();
		proxy = new (&m_pLargeHandles[newHandleIndex])D_btSimpleBroadphaseProxy(aabbMin,aabbMax,shapeType,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy);
	}
	else
	{
		proxy = D_btSimpleBroadphase::createProxy(aabbMin, aabbMax, shapeType, userPtr, collisionFilterGroup, collisionFilterMask, dispatcher, multiSapProxy);
	}
	return proxy;
}



void D_btGpu3DGridBroadphase::destroyProxy(D_btBroadphaseProxy* proxy, D_btDispatcher* dispatcher)
{
	bool bIsLarge = isLargeProxy(proxy);
	if(bIsLarge)
	{
		
		D_btSimpleBroadphaseProxy* proxy0 = static_cast<D_btSimpleBroadphaseProxy*>(proxy);
		freeLargeHandle(proxy0);
		m_pairCache->removeOverlappingPairsContainingProxy(proxy,dispatcher);
	}
	else
	{
		D_btSimpleBroadphase::destroyProxy(proxy, dispatcher);
	}
	return;
}



void D_btGpu3DGridBroadphase::resetPool(D_btDispatcher* dispatcher)
{
	m_hPairBuffStartCurr[0] = 0;
	m_hPairBuffStartCurr[1] = 0;
	for(int i = 1; i <= m_maxHandles; i++) 
	{
		m_hPairBuffStartCurr[i * 2] = m_hPairBuffStartCurr[(i-1) * 2] + m_maxPairsPerBody;
		m_hPairBuffStartCurr[i * 2 + 1] = 0;
	}
}



bool D_btGpu3DGridBroadphase::isLargeProxy(const D_btVector3& aabbMin,  const D_btVector3& aabbMax)
{
	D_btVector3 diag = aabbMax - aabbMin;
	
	///use the bounding sphere radius of this bounding box, D_to include rotation
	D_btScalar radius = diag.length() * D_btScalar(0.5f);
	radius *= m_cellFactorAABB; // user-defined factor

	return (radius > m_maxRadius);
}



bool D_btGpu3DGridBroadphase::isLargeProxy(D_btBroadphaseProxy* proxy)
{
	return (proxy->getUid() >= (m_maxHandles+2));
}



void D_btGpu3DGridBroadphase::addLarge2LargePairsToCache(D_btDispatcher* dispatcher)
{
	int i,j;
	if (m_numLargeHandles <= 0)
	{
		return;
	}
	int new_largest_index = -1;
	for(i = 0; i <= m_LastLargeHandleIndex; i++)
	{
		D_btSimpleBroadphaseProxy* proxy0 = &m_pLargeHandles[i];
		if(!proxy0->m_clientObject)
		{
			continue;
		}
		new_largest_index = i;
		for(j = i + 1; j <= m_LastLargeHandleIndex; j++)
		{
			D_btSimpleBroadphaseProxy* proxy1 = &m_pLargeHandles[j];
			if(!proxy1->m_clientObject)
			{
				continue;
			}
			D_btAssert(proxy0 != proxy1);
			D_btSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
			D_btSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);
			if(aabbOverlap(p0,p1))
			{
				if (!m_pairCache->findPair(proxy0,proxy1))
				{
					m_pairCache->addOverlappingPair(proxy0,proxy1);
				}
			} 
			else
			{
				if(m_pairCache->findPair(proxy0,proxy1))
				{
					m_pairCache->removeOverlappingPair(proxy0,proxy1,dispatcher);
				}
			}
		}
	}
	m_LastLargeHandleIndex = new_largest_index;
	return;
}



void D_btGpu3DGridBroadphase::rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback)
{
	D_btSimpleBroadphase::rayTest(rayFrom, rayTo, rayCallback);
	for (int i=0; i <= m_LastLargeHandleIndex; i++)
	{
		D_btSimpleBroadphaseProxy* proxy = &m_pLargeHandles[i];
		if(!proxy->m_clientObject)
		{
			continue;
		}
		rayCallback.process(proxy);
	}
}



//
// overrides for CPU version
//



void D_btGpu3DGridBroadphase::prepareAABB()
{
	D_BT_PROFILE("prepareAABB");
	D_bt3DGrid3F1U* pBB = m_hAABB;
	int i;
	int new_largest_index = -1;
	unsigned int num_small = 0;
	for(i = 0; i <= m_LastHandleIndex; i++) 
	{
		D_btSimpleBroadphaseProxy* proxy0 = &m_pHandles[i];
		if(!proxy0->m_clientObject)
		{
			continue;
		}
		new_largest_index = i;
		pBB->fx = proxy0->m_aabbMin.getX();
		pBB->fy = proxy0->m_aabbMin.getY();
		pBB->fz = proxy0->m_aabbMin.getZ();
		pBB->uw = i;
		pBB++;
		pBB->fx = proxy0->m_aabbMax.getX();
		pBB->fy = proxy0->m_aabbMax.getY();
		pBB->fz = proxy0->m_aabbMax.getZ();
		pBB->uw = num_small;
		pBB++;
		num_small++;
	}
	m_LastHandleIndex = new_largest_index;
	new_largest_index = -1;
	unsigned int num_large = 0;
	for(i = 0; i <= m_LastLargeHandleIndex; i++) 
	{
		D_btSimpleBroadphaseProxy* proxy0 = &m_pLargeHandles[i];
		if(!proxy0->m_clientObject)
		{
			continue;
		}
		new_largest_index = i;
		pBB->fx = proxy0->m_aabbMin.getX();
		pBB->fy = proxy0->m_aabbMin.getY();
		pBB->fz = proxy0->m_aabbMin.getZ();
		pBB->uw = i + m_maxHandles;
		pBB++;
		pBB->fx = proxy0->m_aabbMax.getX();
		pBB->fy = proxy0->m_aabbMax.getY();
		pBB->fz = proxy0->m_aabbMax.getZ();
		pBB->uw = num_large + m_maxHandles;
		pBB++;
		num_large++;
	}
	m_LastLargeHandleIndex = new_largest_index;
	// paranoid checks
	D_btAssert(num_small == m_numHandles);
	D_btAssert(num_large == m_numLargeHandles);
	return;
}



void D_btGpu3DGridBroadphase::setParameters(D_bt3DGridBroadphaseParams* hostParams)
{
	s3DGridBroadphaseParams = *hostParams;
	return;
}



void D_btGpu3DGridBroadphase::calcHashAABB()
{
	D_BT_PROFILE("bt3DGrid_calcHashAABB");
	D_btGpu_calcHashAABB(m_hAABB, m_hBodiesHash, m_numHandles);
	return;
}



void D_btGpu3DGridBroadphase::sortHash()
{
	class D_bt3DGridHashKey
	{
	public:
	   unsigned int hash;
	   unsigned int index;
	   void quickSort(D_bt3DGridHashKey* pData, int lo, int hi)
	   {
			int i=lo, j=hi;
			D_bt3DGridHashKey x = pData[(lo+hi)/2];
			do
			{    
				while(pData[i].hash > x.hash) i++; 
				while(x.hash > pData[j].hash) j--;
				if(i <= j)
				{
					D_bt3DGridHashKey t = pData[i];
					pData[i] = pData[j];
					pData[j] = t;
					i++; j--;
				}
			} while(i <= j);
			if(lo < j) pData->quickSort(pData, lo, j);
			if(i < hi) pData->quickSort(pData, i, hi);
	   }
	};
	D_BT_PROFILE("bt3DGrid_sortHash");
	D_bt3DGridHashKey* pHash = (D_bt3DGridHashKey*)m_hBodiesHash;
	pHash->quickSort(pHash, 0, m_numHandles - 1);
	return;
}



void D_btGpu3DGridBroadphase::findCellStart()
{
	D_BT_PROFILE("bt3DGrid_findCellStart");
	D_btGpu_findCellStart(m_hBodiesHash, m_hCellStart, m_numHandles, m_params.m_numCells);
	return;
}



void D_btGpu3DGridBroadphase::findOverlappingPairs()
{
	D_BT_PROFILE("bt3DGrid_findOverlappingPairs");
	D_btGpu_findOverlappingPairs(m_hAABB, m_hBodiesHash, m_hCellStart, m_hPairBuff, m_hPairBuffStartCurr, m_numHandles);
	return;
}



void D_btGpu3DGridBroadphase::findPairsLarge()
{
	D_BT_PROFILE("bt3DGrid_findPairsLarge");
	D_btGpu_findPairsLarge(m_hAABB, m_hBodiesHash, m_hCellStart, m_hPairBuff, m_hPairBuffStartCurr,	m_numHandles, m_numLargeHandles);
	return;
}



void D_btGpu3DGridBroadphase::computePairCacheChanges()
{
	D_BT_PROFILE("bt3DGrid_computePairCacheChanges");
	D_btGpu_computePairCacheChanges(m_hPairBuff, m_hPairBuffStartCurr, m_hPairScan, m_hAABB, m_numHandles);
	return;
}



void D_btGpu3DGridBroadphase::scanOverlappingPairBuff()
{
	D_BT_PROFILE("bt3DGrid_scanOverlappingPairBuff");
	m_hPairScan[0] = 0;
	for(int i = 1; i <= m_numHandles; i++) 
	{
		unsigned int delta = m_hPairScan[i];
		m_hPairScan[i] = m_hPairScan[i-1] + delta;
	}
	return;
}



void D_btGpu3DGridBroadphase::squeezeOverlappingPairBuff()
{
	D_BT_PROFILE("bt3DGrid_squeezeOverlappingPairBuff");
	D_btGpu_squeezeOverlappingPairBuff(m_hPairBuff, m_hPairBuffStartCurr, m_hPairScan, m_hPairOut, m_hAABB, m_numHandles);
	return;
}



#include "btGpu3DGridBroadphaseSharedCode.h"


