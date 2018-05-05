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



#include "btOverlappingPairCache.h"

#include "btDispatcher.h"
#include "btCollisionAlgorithm.h"
#include "LinearMath/btAabbUtil2.h"

#include <stdio.h>

int	D_gOverlappingPairs = 0;

int D_gRemovePairs =0;
int D_gAddedPairs =0;
int D_gFindPairs =0;




D_btHashedOverlappingPairCache::D_btHashedOverlappingPairCache():
	m_overlapFilterCallback(0),
	m_blockedForChanges(false),
	m_ghostPairCallback(0)
{
	int initialAllocatedSize= 2;
	m_overlappingPairArray.reserve(initialAllocatedSize);
	growTables();
}




D_btHashedOverlappingPairCache::~D_btHashedOverlappingPairCache()
{
}



void	D_btHashedOverlappingPairCache::cleanOverlappingPair(D_btBroadphasePair& pair,D_btDispatcher* dispatcher)
{
	if (pair.m_algorithm)
	{
		{
			pair.m_algorithm->~D_btCollisionAlgorithm();
			dispatcher->freeCollisionAlgorithm(pair.m_algorithm);
			pair.m_algorithm=0;
		}
	}
}




void	D_btHashedOverlappingPairCache::cleanProxyFromPairs(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher)
{

	class	D_CleanPairCallback : public D_btOverlapCallback
	{
		D_btBroadphaseProxy* m_cleanProxy;
		D_btOverlappingPairCache*	m_pairCache;
		D_btDispatcher* m_dispatcher;

	public:
		D_CleanPairCallback(D_btBroadphaseProxy* cleanProxy,D_btOverlappingPairCache* pairCache,D_btDispatcher* dispatcher)
			:m_cleanProxy(cleanProxy),
			m_pairCache(pairCache),
			m_dispatcher(dispatcher)
		{
		}
		virtual	bool	processOverlap(D_btBroadphasePair& pair)
		{
			if ((pair.m_pProxy0 == m_cleanProxy) ||
				(pair.m_pProxy1 == m_cleanProxy))
			{
				m_pairCache->cleanOverlappingPair(pair,m_dispatcher);
			}
			return false;
		}
		
	};

	D_CleanPairCallback cleanPairs(proxy,this,dispatcher);

	processAllOverlappingPairs(&cleanPairs,dispatcher);

}




void	D_btHashedOverlappingPairCache::removeOverlappingPairsContainingProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher)
{

	class	D_RemovePairCallback : public D_btOverlapCallback
	{
		D_btBroadphaseProxy* m_obsoleteProxy;

	public:
		D_RemovePairCallback(D_btBroadphaseProxy* obsoleteProxy)
			:m_obsoleteProxy(obsoleteProxy)
		{
		}
		virtual	bool	processOverlap(D_btBroadphasePair& pair)
		{
			return ((pair.m_pProxy0 == m_obsoleteProxy) ||
				(pair.m_pProxy1 == m_obsoleteProxy));
		}
		
	};


	D_RemovePairCallback removeCallback(proxy);

	processAllOverlappingPairs(&removeCallback,dispatcher);
}





D_btBroadphasePair* D_btHashedOverlappingPairCache::findPair(D_btBroadphaseProxy* proxy0, D_btBroadphaseProxy* proxy1)
{
	D_gFindPairs++;
	if(proxy0->m_uniqueId>proxy1->m_uniqueId) 
		D_btSwap(proxy0,proxy1);
	int proxyId1 = proxy0->getUid();
	int proxyId2 = proxy1->getUid();

	/*if (proxyId1 > proxyId2) 
		D_btSwap(proxyId1, proxyId2);*/

	int hash = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1), static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity()-1));

	if (hash >= m_hashTable.size())
	{
		return NULL;
	}

	int index = m_hashTable[hash];
	while (index != D_BT_NULL_PAIR && equalsPair(m_overlappingPairArray[index], proxyId1, proxyId2) == false)
	{
		index = m_next[index];
	}

	if (index == D_BT_NULL_PAIR)
	{
		return NULL;
	}

	D_btAssert(index < m_overlappingPairArray.size());

	return &m_overlappingPairArray[index];
}

//#include <stdio.h>

void	D_btHashedOverlappingPairCache::growTables()
{

	int newCapacity = m_overlappingPairArray.capacity();

	if (m_hashTable.size() < newCapacity)
	{
		//grow hashtable D_and next table
		int curHashtableSize = m_hashTable.size();

		m_hashTable.resize(newCapacity);
		m_next.resize(newCapacity);


		int i;

		for (i= 0; i < newCapacity; ++i)
		{
			m_hashTable[i] = D_BT_NULL_PAIR;
		}
		for (i = 0; i < newCapacity; ++i)
		{
			m_next[i] = D_BT_NULL_PAIR;
		}

		for(i=0;i<curHashtableSize;i++)
		{
	
			const D_btBroadphasePair& pair = m_overlappingPairArray[i];
			int proxyId1 = pair.m_pProxy0->getUid();
			int proxyId2 = pair.m_pProxy1->getUid();
			/*if (proxyId1 > proxyId2) 
				D_btSwap(proxyId1, proxyId2);*/
			int	hashValue = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1),static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity()-1));	// New hash value with new mask
			m_next[i] = m_hashTable[hashValue];
			m_hashTable[hashValue] = i;
		}


	}
}

D_btBroadphasePair* D_btHashedOverlappingPairCache::internalAddPair(D_btBroadphaseProxy* proxy0, D_btBroadphaseProxy* proxy1)
{
	if(proxy0->m_uniqueId>proxy1->m_uniqueId) 
		D_btSwap(proxy0,proxy1);
	int proxyId1 = proxy0->getUid();
	int proxyId2 = proxy1->getUid();

	/*if (proxyId1 > proxyId2) 
		D_btSwap(proxyId1, proxyId2);*/

	int	hash = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1),static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity()-1));	// New hash value with new mask


	D_btBroadphasePair* pair = internalFindPair(proxy0, proxy1, hash);
	if (pair != NULL)
	{
		return pair;
	}
	/*for(int i=0;i<m_overlappingPairArray.size();++i)
		{
		if(	(m_overlappingPairArray[i].m_pProxy0==proxy0)&&
			(m_overlappingPairArray[i].m_pProxy1==proxy1))
			{
			printf("Adding duplicated %u<>%u\r\n",proxyId1,proxyId2);
			internalFindPair(proxy0, proxy1, hash);
			}
		}*/
	int count = m_overlappingPairArray.size();
	int oldCapacity = m_overlappingPairArray.capacity();
	void* mem = &m_overlappingPairArray.expand();

	//this D_is where we add an actual pair, so also call the 'ghost'
	if (m_ghostPairCallback)
		m_ghostPairCallback->addOverlappingPair(proxy0,proxy1);

	int newCapacity = m_overlappingPairArray.capacity();

	if (oldCapacity < newCapacity)
	{
		growTables();
		//hash with new capacity
		hash = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1),static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity()-1));
	}
	
	pair = new (mem) D_btBroadphasePair(*proxy0,*proxy1);
//	pair->m_pProxy0 = proxy0;
//	pair->m_pProxy1 = proxy1;
	pair->m_algorithm = 0;
	pair->m_internalTmpValue = 0;
	

	m_next[count] = m_hashTable[hash];
	m_hashTable[hash] = count;

	return pair;
}



void* D_btHashedOverlappingPairCache::removeOverlappingPair(D_btBroadphaseProxy* proxy0, D_btBroadphaseProxy* proxy1,D_btDispatcher* dispatcher)
{
	D_gRemovePairs++;
	if(proxy0->m_uniqueId>proxy1->m_uniqueId) 
		D_btSwap(proxy0,proxy1);
	int proxyId1 = proxy0->getUid();
	int proxyId2 = proxy1->getUid();

	/*if (proxyId1 > proxyId2) 
		D_btSwap(proxyId1, proxyId2);*/

	int	hash = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1),static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity()-1));

	D_btBroadphasePair* pair = internalFindPair(proxy0, proxy1, hash);
	if (pair == NULL)
	{
		return 0;
	}

	cleanOverlappingPair(*pair,dispatcher);

	void* userData = pair->m_internalInfo1;

	D_btAssert(pair->m_pProxy0->getUid() == proxyId1);
	D_btAssert(pair->m_pProxy1->getUid() == proxyId2);

	int pairIndex = int(pair - &m_overlappingPairArray[0]);
	D_btAssert(pairIndex < m_overlappingPairArray.size());

	// Remove the pair from the hash table.
	int index = m_hashTable[hash];
	D_btAssert(index != D_BT_NULL_PAIR);

	int previous = D_BT_NULL_PAIR;
	while (index != pairIndex)
	{
		previous = index;
		index = m_next[index];
	}

	if (previous != D_BT_NULL_PAIR)
	{
		D_btAssert(m_next[previous] == pairIndex);
		m_next[previous] = m_next[pairIndex];
	}
	else
	{
		m_hashTable[hash] = m_next[pairIndex];
	}

	// We now move the last pair into spot of the
	// pair being removed. We D_need D_to fix the hash
	// table indices D_to support the move.

	int lastPairIndex = m_overlappingPairArray.size() - 1;

	if (m_ghostPairCallback)
		m_ghostPairCallback->removeOverlappingPair(proxy0, proxy1,dispatcher);

	// If the removed pair D_is the last pair, we D_are done.
	if (lastPairIndex == pairIndex)
	{
		m_overlappingPairArray.pop_back();
		return userData;
	}

	// Remove the last pair from the hash table.
	const D_btBroadphasePair* last = &m_overlappingPairArray[lastPairIndex];
		/* missing swap here too, Nat. */ 
	int lastHash = static_cast<int>(getHash(static_cast<unsigned int>(last->m_pProxy0->getUid()), static_cast<unsigned int>(last->m_pProxy1->getUid())) & (m_overlappingPairArray.capacity()-1));

	index = m_hashTable[lastHash];
	D_btAssert(index != D_BT_NULL_PAIR);

	previous = D_BT_NULL_PAIR;
	while (index != lastPairIndex)
	{
		previous = index;
		index = m_next[index];
	}

	if (previous != D_BT_NULL_PAIR)
	{
		D_btAssert(m_next[previous] == lastPairIndex);
		m_next[previous] = m_next[lastPairIndex];
	}
	else
	{
		m_hashTable[lastHash] = m_next[lastPairIndex];
	}

	// Copy the last pair into the remove pair's spot.
	m_overlappingPairArray[pairIndex] = m_overlappingPairArray[lastPairIndex];

	// Insert the last pair into the hash table
	m_next[pairIndex] = m_hashTable[lastHash];
	m_hashTable[lastHash] = pairIndex;

	m_overlappingPairArray.pop_back();

	return userData;
}
//#include <stdio.h>

void	D_btHashedOverlappingPairCache::processAllOverlappingPairs(D_btOverlapCallback* callback,D_btDispatcher* dispatcher)
{

	int i;

//	printf("m_overlappingPairArray.size()=%d\n",m_overlappingPairArray.size());
	for (i=0;i<m_overlappingPairArray.size();)
	{
	
		D_btBroadphasePair* pair = &m_overlappingPairArray[i];
		if (callback->processOverlap(*pair))
		{
			removeOverlappingPair(pair->m_pProxy0,pair->m_pProxy1,dispatcher);

			D_gOverlappingPairs--;
		} else
		{
			i++;
		}
	}
}

void	D_btHashedOverlappingPairCache::sortOverlappingPairs(D_btDispatcher* dispatcher)
{
	///D_need D_to keep hashmap in sync with pair address, so rebuild all
	D_btBroadphasePairArray tmpPairs;
	int i;
	for (i=0;i<m_overlappingPairArray.size();i++)
	{
		tmpPairs.push_back(m_overlappingPairArray[i]);
	}

	for (i=0;i<tmpPairs.size();i++)
	{
		removeOverlappingPair(tmpPairs[i].m_pProxy0,tmpPairs[i].m_pProxy1,dispatcher);
	}
	
	for (i = 0; i < m_next.size(); i++)
	{
		m_next[i] = D_BT_NULL_PAIR;
	}

	tmpPairs.quickSort(D_btBroadphasePairSortPredicate());

	for (i=0;i<tmpPairs.size();i++)
	{
		addOverlappingPair(tmpPairs[i].m_pProxy0,tmpPairs[i].m_pProxy1);
	}

	
}


void*	D_btSortedOverlappingPairCache::removeOverlappingPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1, D_btDispatcher* dispatcher )
{
	if (!hasDeferredRemoval())
	{
		D_btBroadphasePair findPair(*proxy0,*proxy1);

		int findIndex = m_overlappingPairArray.findLinearSearch(findPair);
		if (findIndex < m_overlappingPairArray.size())
		{
			D_gOverlappingPairs--;
			D_btBroadphasePair& pair = m_overlappingPairArray[findIndex];
			void* userData = pair.m_internalInfo1;
			cleanOverlappingPair(pair,dispatcher);
			if (m_ghostPairCallback)
				m_ghostPairCallback->removeOverlappingPair(proxy0, proxy1,dispatcher);
			
			m_overlappingPairArray.swap(findIndex,m_overlappingPairArray.capacity()-1);
			m_overlappingPairArray.pop_back();
			return userData;
		}
	}

	return 0;
}








D_btBroadphasePair*	D_btSortedOverlappingPairCache::addOverlappingPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1)
{
	//don't add overlap with own
	D_btAssert(proxy0 != proxy1);

	if (!needsBroadphaseCollision(proxy0,proxy1))
		return 0;
	
	void* mem = &m_overlappingPairArray.expand();
	D_btBroadphasePair* pair = new (mem) D_btBroadphasePair(*proxy0,*proxy1);
	
	D_gOverlappingPairs++;
	D_gAddedPairs++;
	
	if (m_ghostPairCallback)
		m_ghostPairCallback->addOverlappingPair(proxy0, proxy1);
	return pair;
	
}

///this findPair becomes really slow. Either sort the list D_to speedup the query, or
///use a different solution. It D_is mainly used for Removing overlapping pairs. Removal could be delayed.
///we could keep a linked list in each proxy, D_and store pair in one of the proxies (with lowest memory address)
///Also we D_can use a 2D bitmap, which D_can be useful for a future GPU implementation
 D_btBroadphasePair*	D_btSortedOverlappingPairCache::findPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1)
{
	if (!needsBroadphaseCollision(proxy0,proxy1))
		return 0;

	D_btBroadphasePair tmpPair(*proxy0,*proxy1);
	int findIndex = m_overlappingPairArray.findLinearSearch(tmpPair);

	if (findIndex < m_overlappingPairArray.size())
	{
		//D_btAssert(it != m_overlappingPairSet.end());
		 D_btBroadphasePair* pair = &m_overlappingPairArray[findIndex];
		return pair;
	}
	return 0;
}










//#include <stdio.h>

void	D_btSortedOverlappingPairCache::processAllOverlappingPairs(D_btOverlapCallback* callback,D_btDispatcher* dispatcher)
{

	int i;

	for (i=0;i<m_overlappingPairArray.size();)
	{
	
		D_btBroadphasePair* pair = &m_overlappingPairArray[i];
		if (callback->processOverlap(*pair))
		{
			cleanOverlappingPair(*pair,dispatcher);
			pair->m_pProxy0 = 0;
			pair->m_pProxy1 = 0;
			m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
			m_overlappingPairArray.pop_back();
			D_gOverlappingPairs--;
		} else
		{
			i++;
		}
	}
}




D_btSortedOverlappingPairCache::D_btSortedOverlappingPairCache():
	m_blockedForChanges(false),
	m_hasDeferredRemoval(true),
	m_overlapFilterCallback(0),
	m_ghostPairCallback(0)
{
	int initialAllocatedSize= 2;
	m_overlappingPairArray.reserve(initialAllocatedSize);
}

D_btSortedOverlappingPairCache::~D_btSortedOverlappingPairCache()
{
}

void	D_btSortedOverlappingPairCache::cleanOverlappingPair(D_btBroadphasePair& pair,D_btDispatcher* dispatcher)
{
	if (pair.m_algorithm)
	{
		{
			pair.m_algorithm->~D_btCollisionAlgorithm();
			dispatcher->freeCollisionAlgorithm(pair.m_algorithm);
			pair.m_algorithm=0;
			D_gRemovePairs--;
		}
	}
}


void	D_btSortedOverlappingPairCache::cleanProxyFromPairs(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher)
{

	class	D_CleanPairCallback : public D_btOverlapCallback
	{
		D_btBroadphaseProxy* m_cleanProxy;
		D_btOverlappingPairCache*	m_pairCache;
		D_btDispatcher* m_dispatcher;

	public:
		D_CleanPairCallback(D_btBroadphaseProxy* cleanProxy,D_btOverlappingPairCache* pairCache,D_btDispatcher* dispatcher)
			:m_cleanProxy(cleanProxy),
			m_pairCache(pairCache),
			m_dispatcher(dispatcher)
		{
		}
		virtual	bool	processOverlap(D_btBroadphasePair& pair)
		{
			if ((pair.m_pProxy0 == m_cleanProxy) ||
				(pair.m_pProxy1 == m_cleanProxy))
			{
				m_pairCache->cleanOverlappingPair(pair,m_dispatcher);
			}
			return false;
		}
		
	};

	D_CleanPairCallback cleanPairs(proxy,this,dispatcher);

	processAllOverlappingPairs(&cleanPairs,dispatcher);

}


void	D_btSortedOverlappingPairCache::removeOverlappingPairsContainingProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher)
{

	class	D_RemovePairCallback : public D_btOverlapCallback
	{
		D_btBroadphaseProxy* m_obsoleteProxy;

	public:
		D_RemovePairCallback(D_btBroadphaseProxy* obsoleteProxy)
			:m_obsoleteProxy(obsoleteProxy)
		{
		}
		virtual	bool	processOverlap(D_btBroadphasePair& pair)
		{
			return ((pair.m_pProxy0 == m_obsoleteProxy) ||
				(pair.m_pProxy1 == m_obsoleteProxy));
		}
		
	};

	D_RemovePairCallback removeCallback(proxy);

	processAllOverlappingPairs(&removeCallback,dispatcher);
}

void	D_btSortedOverlappingPairCache::sortOverlappingPairs(D_btDispatcher* dispatcher)
{
	//D_should already be sorted
}

