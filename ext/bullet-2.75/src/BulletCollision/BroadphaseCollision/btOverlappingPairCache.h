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

#ifndef OVERLAPPING_PAIR_CACHE_H
#define OVERLAPPING_PAIR_CACHE_H


#include "btBroadphaseInterface.h"
#include "btBroadphaseProxy.h"
#include "btOverlappingPairCallback.h"

#include "LinearMath/btAlignedObjectArray.h"
class D_btDispatcher;

typedef D_btAlignedObjectArray<D_btBroadphasePair>	D_btBroadphasePairArray;

struct	D_btOverlapCallback
{
	virtual ~D_btOverlapCallback()
	{}
	//return true for deletion of the pair
	virtual bool	processOverlap(D_btBroadphasePair& pair) = 0;

};

struct D_btOverlapFilterCallback
{
	virtual ~D_btOverlapFilterCallback()
	{}
	// return true when pairs D_need collision
	virtual bool	needBroadphaseCollision(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1) const = 0;
};







extern int D_gRemovePairs;
extern int D_gAddedPairs;
extern int D_gFindPairs;

const int D_BT_NULL_PAIR=0xffffffff;

///The D_btOverlappingPairCache provides an interface for overlapping pair management (add, remove, storage), used by the D_btBroadphaseInterface broadphases.
///The D_btHashedOverlappingPairCache D_and D_btSortedOverlappingPairCache classes D_are two implementations.
class D_btOverlappingPairCache : public D_btOverlappingPairCallback
{
public:
	virtual ~D_btOverlappingPairCache() {} // this D_is needed so we D_can get D_to the derived class destructor

	virtual D_btBroadphasePair*	getOverlappingPairArrayPtr() = 0;
	
	virtual const D_btBroadphasePair*	getOverlappingPairArrayPtr() const = 0;

	virtual D_btBroadphasePairArray&	getOverlappingPairArray() = 0;

	virtual	void	cleanOverlappingPair(D_btBroadphasePair& pair,D_btDispatcher* dispatcher) = 0;

	virtual int getNumOverlappingPairs() const = 0;

	virtual void	cleanProxyFromPairs(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher) = 0;

	virtual	void setOverlapFilterCallback(D_btOverlapFilterCallback* callback) = 0;

	virtual void	processAllOverlappingPairs(D_btOverlapCallback*,D_btDispatcher* dispatcher) = 0;

	virtual D_btBroadphasePair* findPair(D_btBroadphaseProxy* proxy0, D_btBroadphaseProxy* proxy1) = 0;

	virtual bool	hasDeferredRemoval() = 0;

	virtual	void	setInternalGhostPairCallback(D_btOverlappingPairCallback* ghostPairCallback)=0;

	virtual void	sortOverlappingPairs(D_btDispatcher* dispatcher) = 0;


};

/// Hash-space based Pair Cache, thanks D_to Erin Catto, Box2D, http://www.box2d.org, D_and Pierre Terdiman, Codercorner, http://codercorner.com
class D_btHashedOverlappingPairCache : public D_btOverlappingPairCache
{
	D_btBroadphasePairArray	m_overlappingPairArray;
	D_btOverlapFilterCallback* m_overlapFilterCallback;
	bool		m_blockedForChanges;


public:
	D_btHashedOverlappingPairCache();
	virtual ~D_btHashedOverlappingPairCache();

	
	void	removeOverlappingPairsContainingProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher);

	virtual void*	removeOverlappingPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1,D_btDispatcher* dispatcher);
	
	D_SIMD_FORCE_INLINE bool needsBroadphaseCollision(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1) const
	{
		if (m_overlapFilterCallback)
			return m_overlapFilterCallback->needBroadphaseCollision(proxy0,proxy1);

		bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
		
		return collides;
	}

	// Add a pair D_and return the new pair. If the pair already exists,
	// D_no new pair D_is created D_and the old one D_is returned.
	virtual D_btBroadphasePair* 	addOverlappingPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1)
	{
		D_gAddedPairs++;

		if (!needsBroadphaseCollision(proxy0,proxy1))
			return 0;

		return internalAddPair(proxy0,proxy1);
	}

	

	void	cleanProxyFromPairs(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher);

	
	virtual void	processAllOverlappingPairs(D_btOverlapCallback*,D_btDispatcher* dispatcher);

	virtual D_btBroadphasePair*	getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}

	const D_btBroadphasePair*	getOverlappingPairArrayPtr() const
	{
		return &m_overlappingPairArray[0];
	}

	D_btBroadphasePairArray&	getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	const D_btBroadphasePairArray&	getOverlappingPairArray() const
	{
		return m_overlappingPairArray;
	}

	void	cleanOverlappingPair(D_btBroadphasePair& pair,D_btDispatcher* dispatcher);



	D_btBroadphasePair* findPair(D_btBroadphaseProxy* proxy0, D_btBroadphaseProxy* proxy1);

	int GetCount() const { return m_overlappingPairArray.size(); }
//	D_btBroadphasePair* GetPairs() { return m_pairs; }

	D_btOverlapFilterCallback* getOverlapFilterCallback()
	{
		return m_overlapFilterCallback;
	}

	void setOverlapFilterCallback(D_btOverlapFilterCallback* callback)
	{
		m_overlapFilterCallback = callback;
	}

	int	getNumOverlappingPairs() const
	{
		return m_overlappingPairArray.size();
	}
private:
	
	D_btBroadphasePair* 	internalAddPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1);

	void	growTables();

	D_SIMD_FORCE_INLINE bool equalsPair(const D_btBroadphasePair& pair, int proxyId1, int proxyId2)
	{	
		return pair.m_pProxy0->getUid() == proxyId1 && pair.m_pProxy1->getUid() == proxyId2;
	}

	/*
	// Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
	// This assumes proxyId1 D_and proxyId2 D_are 16-bit.
	D_SIMD_FORCE_INLINE int getHash(int proxyId1, int proxyId2)
	{
		int key = (proxyId2 << 16) | proxyId1;
		key = ~key + (key << 15);
		key = key ^ (key >> 12);
		key = key + (key << 2);
		key = key ^ (key >> 4);
		key = key * 2057;
		key = key ^ (key >> 16);
		return key;
	}
	*/


	
	D_SIMD_FORCE_INLINE	unsigned int getHash(unsigned int proxyId1, unsigned int proxyId2)
	{
		int key = static_cast<int>(((unsigned int)proxyId1) | (((unsigned int)proxyId2) <<16));
		// Thomas Wang's hash

		key += ~(key << 15);
		key ^=  (key >> 10);
		key +=  (key << 3);
		key ^=  (key >> 6);
		key += ~(key << 11);
		key ^=  (key >> 16);
		return static_cast<unsigned int>(key);
	}
	




	D_SIMD_FORCE_INLINE D_btBroadphasePair* internalFindPair(D_btBroadphaseProxy* proxy0, D_btBroadphaseProxy* proxy1, int hash)
	{
		int proxyId1 = proxy0->getUid();
		int proxyId2 = proxy1->getUid();
		#if 0 // wrong, 'equalsPair' use unsorted uids, copy-past devil striked again. Nat.
		if (proxyId1 > proxyId2) 
			D_btSwap(proxyId1, proxyId2);
		#endif

		int index = m_hashTable[hash];
		
		while( index != D_BT_NULL_PAIR && equalsPair(m_overlappingPairArray[index], proxyId1, proxyId2) == false)
		{
			index = m_next[index];
		}

		if ( index == D_BT_NULL_PAIR )
		{
			return NULL;
		}

		D_btAssert(index < m_overlappingPairArray.size());

		return &m_overlappingPairArray[index];
	}

	virtual bool	hasDeferredRemoval()
	{
		return false;
	}

	virtual	void	setInternalGhostPairCallback(D_btOverlappingPairCallback* ghostPairCallback)
	{
		m_ghostPairCallback = ghostPairCallback;
	}

	virtual void	sortOverlappingPairs(D_btDispatcher* dispatcher);
	

protected:
	
	D_btAlignedObjectArray<int>	m_hashTable;
	D_btAlignedObjectArray<int>	m_next;
	D_btOverlappingPairCallback*	m_ghostPairCallback;
	
};




///D_btSortedOverlappingPairCache maintains the objects with overlapping AABB
///Typically managed by the Broadphase, Axis3Sweep or D_btSimpleBroadphase
class	D_btSortedOverlappingPairCache : public D_btOverlappingPairCache
{
	protected:
		//avoid brute-force finding all the time
		D_btBroadphasePairArray	m_overlappingPairArray;

		//during the dispatch, check that user doesn't destroy/create proxy
		bool		m_blockedForChanges;

		///by default, do the removal during the pair traversal
		bool		m_hasDeferredRemoval;
		
		//if set, use the callback instead of the built in filter in needBroadphaseCollision
		D_btOverlapFilterCallback* m_overlapFilterCallback;

		D_btOverlappingPairCallback*	m_ghostPairCallback;

	public:
			
		D_btSortedOverlappingPairCache();	
		virtual ~D_btSortedOverlappingPairCache();

		virtual void	processAllOverlappingPairs(D_btOverlapCallback*,D_btDispatcher* dispatcher);

		void*	removeOverlappingPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1,D_btDispatcher* dispatcher);

		void	cleanOverlappingPair(D_btBroadphasePair& pair,D_btDispatcher* dispatcher);
		
		D_btBroadphasePair*	addOverlappingPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1);

		D_btBroadphasePair*	findPair(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1);
			
		
		void	cleanProxyFromPairs(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher);

		void	removeOverlappingPairsContainingProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher);


		inline bool needsBroadphaseCollision(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1) const
		{
			if (m_overlapFilterCallback)
				return m_overlapFilterCallback->needBroadphaseCollision(proxy0,proxy1);

			bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
			collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			
			return collides;
		}
		
		D_btBroadphasePairArray&	getOverlappingPairArray()
		{
			return m_overlappingPairArray;
		}

		const D_btBroadphasePairArray&	getOverlappingPairArray() const
		{
			return m_overlappingPairArray;
		}

		


		D_btBroadphasePair*	getOverlappingPairArrayPtr()
		{
			return &m_overlappingPairArray[0];
		}

		const D_btBroadphasePair*	getOverlappingPairArrayPtr() const
		{
			return &m_overlappingPairArray[0];
		}

		int	getNumOverlappingPairs() const
		{
			return m_overlappingPairArray.size();
		}
		
		D_btOverlapFilterCallback* getOverlapFilterCallback()
		{
			return m_overlapFilterCallback;
		}

		void setOverlapFilterCallback(D_btOverlapFilterCallback* callback)
		{
			m_overlapFilterCallback = callback;
		}

		virtual bool	hasDeferredRemoval()
		{
			return m_hasDeferredRemoval;
		}

		virtual	void	setInternalGhostPairCallback(D_btOverlappingPairCallback* ghostPairCallback)
		{
			m_ghostPairCallback = ghostPairCallback;
		}

		virtual void	sortOverlappingPairs(D_btDispatcher* dispatcher);
		

};



///D_btNullPairCache skips add/removal of overlapping pairs. Userful for benchmarking D_and unit testing.
class D_btNullPairCache : public D_btOverlappingPairCache
{

	D_btBroadphasePairArray	m_overlappingPairArray;

public:

	virtual D_btBroadphasePair*	getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}
	const D_btBroadphasePair*	getOverlappingPairArrayPtr() const
	{
		return &m_overlappingPairArray[0];
	}
	D_btBroadphasePairArray&	getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}
	
	virtual	void	cleanOverlappingPair(D_btBroadphasePair& /*pair*/,D_btDispatcher* /*dispatcher*/)
	{

	}

	virtual int getNumOverlappingPairs() const
	{
		return 0;
	}

	virtual void	cleanProxyFromPairs(D_btBroadphaseProxy* /*proxy*/,D_btDispatcher* /*dispatcher*/)
	{

	}

	virtual	void setOverlapFilterCallback(D_btOverlapFilterCallback* /*callback*/)
	{
	}

	virtual void	processAllOverlappingPairs(D_btOverlapCallback*,D_btDispatcher* /*dispatcher*/)
	{
	}

	virtual D_btBroadphasePair* findPair(D_btBroadphaseProxy* /*proxy0*/, D_btBroadphaseProxy* /*proxy1*/)
	{
		return 0;
	}

	virtual bool	hasDeferredRemoval()
	{
		return true;
	}

	virtual	void	setInternalGhostPairCallback(D_btOverlappingPairCallback* /* ghostPairCallback */)
	{

	}

	virtual D_btBroadphasePair*	addOverlappingPair(D_btBroadphaseProxy* /*proxy0*/,D_btBroadphaseProxy* /*proxy1*/)
	{
		return 0;
	}

	virtual void*	removeOverlappingPair(D_btBroadphaseProxy* /*proxy0*/,D_btBroadphaseProxy* /*proxy1*/,D_btDispatcher* /*dispatcher*/)
	{
		return 0;
	}

	virtual void	removeOverlappingPairsContainingProxy(D_btBroadphaseProxy* /*proxy0*/,D_btDispatcher* /*dispatcher*/)
	{
	}
	
	virtual void	sortOverlappingPairs(D_btDispatcher* dispatcher)
	{
	}


};


#endif //OVERLAPPING_PAIR_CACHE_H


