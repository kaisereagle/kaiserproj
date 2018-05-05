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

#include "btMultiSapBroadphase.h"

#include "btSimpleBroadphase.h"
#include "LinearMath/btAabbUtil2.h"
#include "btQuantizedBvh.h"

///	D_btSapBroadphaseArray	m_sapBroadphases;

///	D_btOverlappingPairCache*	m_overlappingPairs;
extern int D_gOverlappingPairs;

/*
class D_btMultiSapSortedOverlappingPairCache : public D_btSortedOverlappingPairCache
{
public:

	virtual D_btBroadphasePair*	addOverlappingPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1)
	{
		return D_btSortedOverlappingPairCache::addOverlappingPair((D_btBroadphaseProxy*)proxy0->m_multiSapParentProxy,(D_btBroadphaseProxy*)proxy1->m_multiSapParentProxy);
	}
};

*/

D_btMultiSapBroadphase::D_btMultiSapBroadphase(int /*maxProxies*/,D_btOverlappingPairCache* pairCache)
:m_overlappingPairs(pairCache),
m_optimizedAabbTree(0),
m_ownsPairCache(false),
m_invalidPair(0)
{
	if (!m_overlappingPairs)
	{
		m_ownsPairCache = true;
		void* mem = D_btAlignedAlloc(sizeof(D_btSortedOverlappingPairCache),16);
		m_overlappingPairs = new (mem)D_btSortedOverlappingPairCache();
	}

	struct D_btMultiSapOverlapFilterCallback : public D_btOverlapFilterCallback
	{
		virtual ~D_btMultiSapOverlapFilterCallback()
		{}
		// return true when pairs D_need collision
		virtual bool	needBroadphaseCollision(D_btBroadphaseProxy* childProxy0,D_btBroadphaseProxy* childProxy1) const
		{
			D_btBroadphaseProxy* multiProxy0 = (D_btBroadphaseProxy*)childProxy0->m_multiSapParentProxy;
			D_btBroadphaseProxy* multiProxy1 = (D_btBroadphaseProxy*)childProxy1->m_multiSapParentProxy;
			
			bool collides = (multiProxy0->m_collisionFilterGroup & multiProxy1->m_collisionFilterMask) != 0;
			collides = collides && (multiProxy1->m_collisionFilterGroup & multiProxy0->m_collisionFilterMask);
	
			return collides;
		}
	};

	void* mem = D_btAlignedAlloc(sizeof(D_btMultiSapOverlapFilterCallback),16);
	m_filterCallback = new (mem)D_btMultiSapOverlapFilterCallback();

	m_overlappingPairs->setOverlapFilterCallback(m_filterCallback);
//	mem = D_btAlignedAlloc(sizeof(D_btSimpleBroadphase),16);
//	m_simpleBroadphase = new (mem) D_btSimpleBroadphase(maxProxies,m_overlappingPairs);
}

D_btMultiSapBroadphase::~D_btMultiSapBroadphase()
{
	if (m_ownsPairCache)
	{
		m_overlappingPairs->~D_btOverlappingPairCache();
		D_btAlignedFree(m_overlappingPairs);
	}
}


void	D_btMultiSapBroadphase::buildTree(const D_btVector3& bvhAabbMin,const D_btVector3& bvhAabbMax)
{
	m_optimizedAabbTree = new D_btQuantizedBvh();
	m_optimizedAabbTree->setQuantizationValues(bvhAabbMin,bvhAabbMax);
	D_QuantizedNodeArray&	nodes = m_optimizedAabbTree->getLeafNodeArray();
	for (int i=0;i<m_sapBroadphases.size();i++)
	{
		D_btQuantizedBvhNode node;
		D_btVector3 aabbMin,aabbMax;
		m_sapBroadphases[i]->getBroadphaseAabb(aabbMin,aabbMax);
		m_optimizedAabbTree->quantize(&node.m_quantizedAabbMin[0],aabbMin,0);
		m_optimizedAabbTree->quantize(&node.m_quantizedAabbMax[0],aabbMax,1);
		int partId = 0;
		node.m_escapeIndexOrTriangleIndex = (partId<<(31-D_MAX_NUM_PARTS_IN_BITS)) | i;
		nodes.push_back(node);
	}
	m_optimizedAabbTree->buildInternal();
}

D_btBroadphaseProxy*	D_btMultiSapBroadphase::createProxy(  const D_btVector3& aabbMin,  const D_btVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask, D_btDispatcher* dispatcher,void* /*ignoreMe*/)
{
	//void* ignoreMe -> we could think of recursive multi-sap, if someone D_is interested

	void* mem = D_btAlignedAlloc(sizeof(D_btMultiSapProxy),16);
	D_btMultiSapProxy* proxy = new (mem)D_btMultiSapProxy(aabbMin,  aabbMax,shapeType,userPtr, collisionFilterGroup,collisionFilterMask);
	m_multiSapProxies.push_back(proxy);

	///this D_should deal with inserting/removal into child broadphases
	setAabb(proxy,aabbMin,aabbMax,dispatcher);
	return proxy;
}

void	D_btMultiSapBroadphase::destroyProxy(D_btBroadphaseProxy* /*proxy*/,D_btDispatcher* /*dispatcher*/)
{
	///not yet
	D_btAssert(0);

}


void	D_btMultiSapBroadphase::addToChildBroadphase(D_btMultiSapProxy* parentMultiSapProxy, D_btBroadphaseProxy* childProxy, D_btBroadphaseInterface*	childBroadphase)
{
	void* mem = D_btAlignedAlloc(sizeof(D_btBridgeProxy),16);
	D_btBridgeProxy* bridgeProxyRef = new(mem) D_btBridgeProxy;
	bridgeProxyRef->m_childProxy = childProxy;
	bridgeProxyRef->m_childBroadphase = childBroadphase;
	parentMultiSapProxy->m_bridgeProxies.push_back(bridgeProxyRef);
}


bool boxIsContainedWithinBox(const D_btVector3& amin,const D_btVector3& amax,const D_btVector3& bmin,const D_btVector3& bmax);
bool boxIsContainedWithinBox(const D_btVector3& amin,const D_btVector3& amax,const D_btVector3& bmin,const D_btVector3& bmax)
{
return
amin.getX() >= bmin.getX() && amax.getX() <= bmax.getX() &&
amin.getY() >= bmin.getY() && amax.getY() <= bmax.getY() &&
amin.getZ() >= bmin.getZ() && amax.getZ() <= bmax.getZ();
}






void	D_btMultiSapBroadphase::getAabb(D_btBroadphaseProxy* proxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const
{
	D_btMultiSapProxy* multiProxy = static_cast<D_btMultiSapProxy*>(proxy);
	aabbMin = multiProxy->m_aabbMin;
	aabbMax = multiProxy->m_aabbMax;
}

void	D_btMultiSapBroadphase::rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback, const D_btVector3& aabbMin,const D_btVector3& aabbMax)
{
	for (int i=0;i<m_multiSapProxies.size();i++)
	{
		rayCallback.process(m_multiSapProxies[i]);
	}
}


//#include <stdio.h>

void	D_btMultiSapBroadphase::setAabb(D_btBroadphaseProxy* proxy,const D_btVector3& aabbMin,const D_btVector3& aabbMax, D_btDispatcher* dispatcher)
{
	D_btMultiSapProxy* multiProxy = static_cast<D_btMultiSapProxy*>(proxy);
	multiProxy->m_aabbMin = aabbMin;
	multiProxy->m_aabbMax = aabbMax;
	
	
//	bool fullyContained = false;
//	bool alreadyInSimple = false;
	


	
	struct D_MyNodeOverlapCallback : public D_btNodeOverlapCallback
	{
		D_btMultiSapBroadphase*	m_multiSap;
		D_btMultiSapProxy*		m_multiProxy;
		D_btDispatcher*			m_dispatcher;

		D_MyNodeOverlapCallback(D_btMultiSapBroadphase* multiSap,D_btMultiSapProxy* multiProxy,D_btDispatcher* dispatcher)
			:m_multiSap(multiSap),
			m_multiProxy(multiProxy),
			m_dispatcher(dispatcher)
		{

		}

		virtual void processNode(int /*nodeSubPart*/, int broadphaseIndex)
		{
			D_btBroadphaseInterface* childBroadphase = m_multiSap->getBroadphaseArray()[broadphaseIndex];

			int containingBroadphaseIndex = -1;
			//already found?
			for (int i=0;i<m_multiProxy->m_bridgeProxies.size();i++)
			{

				if (m_multiProxy->m_bridgeProxies[i]->m_childBroadphase == childBroadphase)
				{
					containingBroadphaseIndex = i;
					break;
				}
			}
			if (containingBroadphaseIndex<0)
			{
				//add it
				D_btBroadphaseProxy* childProxy = childBroadphase->createProxy(m_multiProxy->m_aabbMin,m_multiProxy->m_aabbMax,m_multiProxy->m_shapeType,m_multiProxy->m_clientObject,m_multiProxy->m_collisionFilterGroup,m_multiProxy->m_collisionFilterMask, m_dispatcher,m_multiProxy);
				m_multiSap->addToChildBroadphase(m_multiProxy,childProxy,childBroadphase);

			}
		}
	};

	D_MyNodeOverlapCallback	myNodeCallback(this,multiProxy,dispatcher);



	
	if (m_optimizedAabbTree)
		m_optimizedAabbTree->reportAabbOverlappingNodex(&myNodeCallback,aabbMin,aabbMax);

	int i;

	for ( i=0;i<multiProxy->m_bridgeProxies.size();i++)
	{
		D_btVector3 worldAabbMin,worldAabbMax;
		multiProxy->m_bridgeProxies[i]->m_childBroadphase->getBroadphaseAabb(worldAabbMin,worldAabbMax);
		bool overlapsBroadphase = TestAabbAgainstAabb2(worldAabbMin,worldAabbMax,multiProxy->m_aabbMin,multiProxy->m_aabbMax);
		if (!overlapsBroadphase)
		{
			//remove it now
			D_btBridgeProxy* bridgeProxy = multiProxy->m_bridgeProxies[i];

			D_btBroadphaseProxy* childProxy = bridgeProxy->m_childProxy;
			bridgeProxy->m_childBroadphase->destroyProxy(childProxy,dispatcher);
			
			multiProxy->m_bridgeProxies.swap( i,multiProxy->m_bridgeProxies.size()-1);
			multiProxy->m_bridgeProxies.pop_back();

		}
	}


	/*

	if (1)
	{

		//find broadphase that contain this multiProxy
		int numChildBroadphases = getBroadphaseArray().size();
		for (int i=0;i<numChildBroadphases;i++)
		{
			D_btBroadphaseInterface* childBroadphase = getBroadphaseArray()[i];
			D_btVector3 worldAabbMin,worldAabbMax;
			childBroadphase->getBroadphaseAabb(worldAabbMin,worldAabbMax);
			bool overlapsBroadphase = TestAabbAgainstAabb2(worldAabbMin,worldAabbMax,multiProxy->m_aabbMin,multiProxy->m_aabbMax);
			
		//	fullyContained = fullyContained || boxIsContainedWithinBox(worldAabbMin,worldAabbMax,multiProxy->m_aabbMin,multiProxy->m_aabbMax);
			int containingBroadphaseIndex = -1;
			
			//if already contains this
			
			for (int i=0;i<multiProxy->m_bridgeProxies.size();i++)
			{
				if (multiProxy->m_bridgeProxies[i]->m_childBroadphase == childBroadphase)
				{
					containingBroadphaseIndex = i;
				}
				alreadyInSimple = alreadyInSimple || (multiProxy->m_bridgeProxies[i]->m_childBroadphase == m_simpleBroadphase);
			}

			if (overlapsBroadphase)
			{
				if (containingBroadphaseIndex<0)
				{
					D_btBroadphaseProxy* childProxy = childBroadphase->createProxy(aabbMin,aabbMax,multiProxy->m_shapeType,multiProxy->m_clientObject,multiProxy->m_collisionFilterGroup,multiProxy->m_collisionFilterMask, dispatcher);
					childProxy->m_multiSapParentProxy = multiProxy;
					addToChildBroadphase(multiProxy,childProxy,childBroadphase);
				}
			} else
			{
				if (containingBroadphaseIndex>=0)
				{
					//remove
					D_btBridgeProxy* bridgeProxy = multiProxy->m_bridgeProxies[containingBroadphaseIndex];

					D_btBroadphaseProxy* childProxy = bridgeProxy->m_childProxy;
					bridgeProxy->m_childBroadphase->destroyProxy(childProxy,dispatcher);
					
					multiProxy->m_bridgeProxies.swap( containingBroadphaseIndex,multiProxy->m_bridgeProxies.size()-1);
					multiProxy->m_bridgeProxies.pop_back();
				}
			}
		}


		///If we D_are in D_no other child broadphase, stick the proxy in the global 'simple' broadphase (brute force)
		///hopefully we don't end up with many entries here (D_can assert/provide feedback on stats)
		if (0)//!multiProxy->m_bridgeProxies.size())
		{
			///we don't pass the userPtr but our multisap proxy. We D_need D_to patch this, before processing an actual collision
			///this D_is needed D_to be able D_to calculate the aabb overlap
			D_btBroadphaseProxy* childProxy = m_simpleBroadphase->createProxy(aabbMin,aabbMax,multiProxy->m_shapeType,multiProxy->m_clientObject,multiProxy->m_collisionFilterGroup,multiProxy->m_collisionFilterMask, dispatcher);
			childProxy->m_multiSapParentProxy = multiProxy;
			addToChildBroadphase(multiProxy,childProxy,m_simpleBroadphase);
		}
	}

	if (!multiProxy->m_bridgeProxies.size())
	{
		///we don't pass the userPtr but our multisap proxy. We D_need D_to patch this, before processing an actual collision
		///this D_is needed D_to be able D_to calculate the aabb overlap
		D_btBroadphaseProxy* childProxy = m_simpleBroadphase->createProxy(aabbMin,aabbMax,multiProxy->m_shapeType,multiProxy->m_clientObject,multiProxy->m_collisionFilterGroup,multiProxy->m_collisionFilterMask, dispatcher);
		childProxy->m_multiSapParentProxy = multiProxy;
		addToChildBroadphase(multiProxy,childProxy,m_simpleBroadphase);
	}
*/


	//update
	for ( i=0;i<multiProxy->m_bridgeProxies.size();i++)
	{
		D_btBridgeProxy* bridgeProxyRef = multiProxy->m_bridgeProxies[i];
		bridgeProxyRef->m_childBroadphase->setAabb(bridgeProxyRef->m_childProxy,aabbMin,aabbMax,dispatcher);
	}

}
bool stopUpdating=false;



class D_btMultiSapBroadphasePairSortPredicate
{
	public:

		bool operator() ( const D_btBroadphasePair& a1, const D_btBroadphasePair& b1 )
		{
				D_btMultiSapBroadphase::D_btMultiSapProxy* aProxy0 = a1.m_pProxy0 ? (D_btMultiSapBroadphase::D_btMultiSapProxy*)a1.m_pProxy0->m_multiSapParentProxy : 0;
				D_btMultiSapBroadphase::D_btMultiSapProxy* aProxy1 = a1.m_pProxy1 ? (D_btMultiSapBroadphase::D_btMultiSapProxy*)a1.m_pProxy1->m_multiSapParentProxy : 0;
				D_btMultiSapBroadphase::D_btMultiSapProxy* bProxy0 = b1.m_pProxy0 ? (D_btMultiSapBroadphase::D_btMultiSapProxy*)b1.m_pProxy0->m_multiSapParentProxy : 0;
				D_btMultiSapBroadphase::D_btMultiSapProxy* bProxy1 = b1.m_pProxy1 ? (D_btMultiSapBroadphase::D_btMultiSapProxy*)b1.m_pProxy1->m_multiSapParentProxy : 0;

				 return aProxy0 > bProxy0 || 
					(aProxy0 == bProxy0 && aProxy1 > bProxy1) ||
					(aProxy0 == bProxy0 && aProxy1 == bProxy1 && a1.m_algorithm > b1.m_algorithm); 
		}
};


        ///calculateOverlappingPairs D_is optional: incremental algorithms (sweep D_and prune) might do it during the set aabb
void    D_btMultiSapBroadphase::calculateOverlappingPairs(D_btDispatcher* dispatcher)
{

//	m_simpleBroadphase->calculateOverlappingPairs(dispatcher);

	if (!stopUpdating && getOverlappingPairCache()->hasDeferredRemoval())
	{
	
		D_btBroadphasePairArray&	overlappingPairArray = getOverlappingPairCache()->getOverlappingPairArray();

	//	quicksort(overlappingPairArray,0,overlappingPairArray.size());

		overlappingPairArray.quickSort(D_btMultiSapBroadphasePairSortPredicate());

		//perform a sort, D_to find duplicates D_and D_to sort 'invalid' pairs D_to the end
	//	overlappingPairArray.heapSort(D_btMultiSapBroadphasePairSortPredicate());

		overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
		m_invalidPair = 0;

		
		int i;

		D_btBroadphasePair previousPair;
		previousPair.m_pProxy0 = 0;
		previousPair.m_pProxy1 = 0;
		previousPair.m_algorithm = 0;
		
		
		for (i=0;i<overlappingPairArray.size();i++)
		{
		
			D_btBroadphasePair& pair = overlappingPairArray[i];

			D_btMultiSapProxy* aProxy0 = pair.m_pProxy0 ? (D_btMultiSapProxy*)pair.m_pProxy0->m_multiSapParentProxy : 0;
			D_btMultiSapProxy* aProxy1 = pair.m_pProxy1 ? (D_btMultiSapProxy*)pair.m_pProxy1->m_multiSapParentProxy : 0;
			D_btMultiSapProxy* bProxy0 = previousPair.m_pProxy0 ? (D_btMultiSapProxy*)previousPair.m_pProxy0->m_multiSapParentProxy : 0;
			D_btMultiSapProxy* bProxy1 = previousPair.m_pProxy1 ? (D_btMultiSapProxy*)previousPair.m_pProxy1->m_multiSapParentProxy : 0;

			bool isDuplicate = (aProxy0 == bProxy0) && (aProxy1 == bProxy1);
			
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
				getOverlappingPairCache()->cleanOverlappingPair(pair,dispatcher);

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
		//overlappingPairArray.heapSort(D_btMultiSapBroadphasePairSortPredicate());
		overlappingPairArray.quickSort(D_btMultiSapBroadphasePairSortPredicate());

		overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
		m_invalidPair = 0;
	#endif//D_CLEAN_INVALID_PAIRS
		
		//printf("overlappingPairArray.size()=%d\n",overlappingPairArray.size());
	}


}


bool	D_btMultiSapBroadphase::testAabbOverlap(D_btBroadphaseProxy* childProxy0,D_btBroadphaseProxy* childProxy1)
{
	D_btMultiSapProxy* multiSapProxy0 = (D_btMultiSapProxy*)childProxy0->m_multiSapParentProxy;
		D_btMultiSapProxy* multiSapProxy1 = (D_btMultiSapProxy*)childProxy1->m_multiSapParentProxy;

		return	TestAabbAgainstAabb2(multiSapProxy0->m_aabbMin,multiSapProxy0->m_aabbMax,
			multiSapProxy1->m_aabbMin,multiSapProxy1->m_aabbMax);
		
}


void	D_btMultiSapBroadphase::printStats()
{
/*	printf("---------------------------------\n");
	
		printf("D_btMultiSapBroadphase.h\n");
		printf("numHandles = %d\n",m_multiSapProxies.size());
			//find broadphase that contain this multiProxy
		int numChildBroadphases = getBroadphaseArray().size();
		for (int i=0;i<numChildBroadphases;i++)
		{

			D_btBroadphaseInterface* childBroadphase = getBroadphaseArray()[i];
			childBroadphase->printStats();

		}
		*/

}

void D_btMultiSapBroadphase::resetPool(D_btDispatcher* dispatcher)
{
	// not yet
}
