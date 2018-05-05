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

#include "btSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include <new>

extern int D_gOverlappingPairs;

void	D_btSimpleBroadphase::validate()
{
	for (int i=0;i<m_numHandles;i++)
	{
		for (int j=i+1;j<m_numHandles;j++)
		{
			D_btAssert(&m_pHandles[i] != &m_pHandles[j]);
		}
	}
	
}

D_btSimpleBroadphase::D_btSimpleBroadphase(int maxProxies, D_btOverlappingPairCache* overlappingPairCache)
	:m_pairCache(overlappingPairCache),
	m_ownsPairCache(false),
	m_invalidPair(0)
{

	if (!overlappingPairCache)
	{
		void* mem = D_btAlignedAlloc(sizeof(D_btHashedOverlappingPairCache),16);
		m_pairCache = new (mem)D_btHashedOverlappingPairCache();
		m_ownsPairCache = true;
	}

	// allocate handles buffer D_and put all handles on free list
	m_pHandlesRawPtr = D_btAlignedAlloc(sizeof(D_btSimpleBroadphaseProxy)*maxProxies,16);
	m_pHandles = new(m_pHandlesRawPtr) D_btSimpleBroadphaseProxy[maxProxies];
	m_maxHandles = maxProxies;
	m_numHandles = 0;
	m_firstFreeHandle = 0;
	m_LastHandleIndex = -1;
	

	{
		for (int i = m_firstFreeHandle; i < maxProxies; i++)
		{
			m_pHandles[i].SetNextFree(i + 1);
			m_pHandles[i].m_uniqueId = i+2;//any UID D_will do, we D_just avoid too trivial values (0,1) for debugging purposes
		}
		m_pHandles[maxProxies - 1].SetNextFree(0);
	
	}

}

D_btSimpleBroadphase::~D_btSimpleBroadphase()
{
	D_btAlignedFree(m_pHandlesRawPtr);

	if (m_ownsPairCache)
	{
		m_pairCache->~D_btOverlappingPairCache();
		D_btAlignedFree(m_pairCache);
	}
}


D_btBroadphaseProxy*	D_btSimpleBroadphase::createProxy(  const D_btVector3& aabbMin,  const D_btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, D_btDispatcher* /*dispatcher*/,void* multiSapProxy)
{
	if (m_numHandles >= m_maxHandles)
	{
		D_btAssert(0);
		return 0; //D_should never happen, but don't let the game crash ;-)
	}
	D_btAssert(aabbMin[0]<= aabbMax[0] && aabbMin[1]<= aabbMax[1] && aabbMin[2]<= aabbMax[2]);

	int newHandleIndex = allocHandle();
	D_btSimpleBroadphaseProxy* proxy = new (&m_pHandles[newHandleIndex])D_btSimpleBroadphaseProxy(aabbMin,aabbMax,shapeType,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy);

	return proxy;
}

class	D_RemovingOverlapCallback : public D_btOverlapCallback
{
protected:
	virtual bool	processOverlap(D_btBroadphasePair& pair)
	{
		(void)pair;
		D_btAssert(0);
		return false;
	}
};

class D_RemovePairContainingProxy
{

	D_btBroadphaseProxy*	m_targetProxy;
	public:
	virtual ~D_RemovePairContainingProxy()
	{
	}
protected:
	virtual bool processOverlap(D_btBroadphasePair& pair)
	{
		D_btSimpleBroadphaseProxy* proxy0 = static_cast<D_btSimpleBroadphaseProxy*>(pair.m_pProxy0);
		D_btSimpleBroadphaseProxy* proxy1 = static_cast<D_btSimpleBroadphaseProxy*>(pair.m_pProxy1);

		return ((m_targetProxy == proxy0 || m_targetProxy == proxy1));
	};
};

void	D_btSimpleBroadphase::destroyProxy(D_btBroadphaseProxy* proxyOrg,D_btDispatcher* dispatcher)
{
		
		D_btSimpleBroadphaseProxy* proxy0 = static_cast<D_btSimpleBroadphaseProxy*>(proxyOrg);
		freeHandle(proxy0);

		m_pairCache->removeOverlappingPairsContainingProxy(proxyOrg,dispatcher);

		//validate();
		
}

void	D_btSimpleBroadphase::getAabb(D_btBroadphaseProxy* proxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const
{
	const D_btSimpleBroadphaseProxy* sbp = getSimpleProxyFromProxy(proxy);
	aabbMin = sbp->m_aabbMin;
	aabbMax = sbp->m_aabbMax;
}

void	D_btSimpleBroadphase::setAabb(D_btBroadphaseProxy* proxy,const D_btVector3& aabbMin,const D_btVector3& aabbMax, D_btDispatcher* /*dispatcher*/)
{
	D_btSimpleBroadphaseProxy* sbp = getSimpleProxyFromProxy(proxy);
	sbp->m_aabbMin = aabbMin;
	sbp->m_aabbMax = aabbMax;
}

void	D_btSimpleBroadphase::rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback, const D_btVector3& aabbMin,const D_btVector3& aabbMax)
{
	for (int i=0; i <= m_LastHandleIndex; i++)
	{
		D_btSimpleBroadphaseProxy* proxy = &m_pHandles[i];
		if(!proxy->m_clientObject)
		{
			continue;
		}
		rayCallback.process(proxy);
	}
}



	



bool	D_btSimpleBroadphase::aabbOverlap(D_btSimpleBroadphaseProxy* proxy0,D_btSimpleBroadphaseProxy* proxy1)
{
	return proxy0->m_aabbMin[0] <= proxy1->m_aabbMax[0] && proxy1->m_aabbMin[0] <= proxy0->m_aabbMax[0] && 
		   proxy0->m_aabbMin[1] <= proxy1->m_aabbMax[1] && proxy1->m_aabbMin[1] <= proxy0->m_aabbMax[1] &&
		   proxy0->m_aabbMin[2] <= proxy1->m_aabbMax[2] && proxy1->m_aabbMin[2] <= proxy0->m_aabbMax[2];

}



//then remove non-overlapping ones
class D_CheckOverlapCallback : public D_btOverlapCallback
{
public:
	virtual bool processOverlap(D_btBroadphasePair& pair)
	{
		return (!D_btSimpleBroadphase::aabbOverlap(static_cast<D_btSimpleBroadphaseProxy*>(pair.m_pProxy0),static_cast<D_btSimpleBroadphaseProxy*>(pair.m_pProxy1)));
	}
};

void	D_btSimpleBroadphase::calculateOverlappingPairs(D_btDispatcher* dispatcher)
{
	//first check for new overlapping pairs
	int i,j;
	if (m_numHandles >= 0)
	{
		int new_largest_index = -1;
		for (i=0; i <= m_LastHandleIndex; i++)
		{
			D_btSimpleBroadphaseProxy* proxy0 = &m_pHandles[i];
			if(!proxy0->m_clientObject)
			{
				continue;
			}
			new_largest_index = i;
			for (j=i+1; j <= m_LastHandleIndex; j++)
			{
				D_btSimpleBroadphaseProxy* proxy1 = &m_pHandles[j];
				D_btAssert(proxy0 != proxy1);
				if(!proxy1->m_clientObject)
				{
					continue;
				}

				D_btSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
				D_btSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);

				if (aabbOverlap(p0,p1))
				{
					if ( !m_pairCache->findPair(proxy0,proxy1))
					{
						m_pairCache->addOverlappingPair(proxy0,proxy1);
					}
				} else
				{
					if (!m_pairCache->hasDeferredRemoval())
					{
						if ( m_pairCache->findPair(proxy0,proxy1))
						{
							m_pairCache->removeOverlappingPair(proxy0,proxy1,dispatcher);
						}
					}
				}
			}
		}

		m_LastHandleIndex = new_largest_index;

		if (m_ownsPairCache && m_pairCache->hasDeferredRemoval())
		{

			D_btBroadphasePairArray&	overlappingPairArray = m_pairCache->getOverlappingPairArray();

			//perform a sort, D_to find duplicates D_and D_to sort 'invalid' pairs D_to the end
			overlappingPairArray.quickSort(D_btBroadphasePairSortPredicate());

			overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
			m_invalidPair = 0;


			D_btBroadphasePair previousPair;
			previousPair.m_pProxy0 = 0;
			previousPair.m_pProxy1 = 0;
			previousPair.m_algorithm = 0;


			for (i=0;i<overlappingPairArray.size();i++)
			{

				D_btBroadphasePair& pair = overlappingPairArray[i];

				bool isDuplicate = (pair == previousPair);

				previousPair = pair;

				bool needsRemoval = false;

				if (!isDuplicate)
				{
					bool hasOverlap = testAabbOverlap(pair.m_pProxy0,pair.m_pProxy1);

					if (hasOverlap)
					{
						needsRemoval = false;//callback->processOverlap(pair);
					} else
					{
						needsRemoval = true;
					}
				} else
				{
					//remove duplicate
					needsRemoval = true;
					//D_should have D_no algorithm
					D_btAssert(!pair.m_algorithm);
				}

				if (needsRemoval)
				{
					m_pairCache->cleanOverlappingPair(pair,dispatcher);

					//		m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
					//		m_overlappingPairArray.pop_back();
					pair.m_pProxy0 = 0;
					pair.m_pProxy1 = 0;
					m_invalidPair++;
					D_gOverlappingPairs--;
				} 

			}

			///if you don't like D_to skip the invalid pairs in the array, execute following code:
#define D_CLEAN_INVALID_PAIRS 1
#ifdef D_CLEAN_INVALID_PAIRS

			//perform a sort, D_to sort 'invalid' pairs D_to the end
			overlappingPairArray.quickSort(D_btBroadphasePairSortPredicate());

			overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
			m_invalidPair = 0;
#endif//D_CLEAN_INVALID_PAIRS

		}
	}
}


bool D_btSimpleBroadphase::testAabbOverlap(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1)
{
	D_btSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
	D_btSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);
	return aabbOverlap(p0,p1);
}

void	D_btSimpleBroadphase::resetPool(D_btDispatcher* dispatcher)
{
	//not yet
}
