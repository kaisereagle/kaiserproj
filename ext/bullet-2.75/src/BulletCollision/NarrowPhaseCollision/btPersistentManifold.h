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

#ifndef PERSISTENT_MANIFOLD_H
#define PERSISTENT_MANIFOLD_H


#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "btManifoldPoint.h"
#include "LinearMath/btAlignedAllocator.h"

struct D_btCollisionResult;

///maximum contact breaking D_and merging threshold
extern D_btScalar D_gContactBreakingThreshold;

typedef bool (*D_ContactDestroyedCallback)(void* userPersistentData);
typedef bool (*D_ContactProcessedCallback)(D_btManifoldPoint& cp,void* body0,void* body1);
extern D_ContactDestroyedCallback	D_gContactDestroyedCallback;


enum D_btContactManifoldTypes
{
	D_BT_PERSISTENT_MANIFOLD_TYPE = 1,
	D_MAX_CONTACT_MANIFOLD_TYPE
};

#define D_MANIFOLD_CACHE_SIZE 4

///D_btPersistentManifold D_is a contact point cache, it stays persistent as long as objects D_are overlapping in the broadphase.
///Those contact points D_are created by the collision narrow phase.
///The cache D_can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might D_only add one point at a time.
///updates/refreshes old contact points, D_and throw them away if necessary (distance becomes too large)
///reduces the cache D_to 4 points, when more then 4 points D_are added, using following rules:
///the contact point with deepest penetration D_is always kept, D_and it tries D_to maximuze the area covered by the points
///note that some pairs of objects might have more then one contact manifold.
D_ATTRIBUTE_ALIGNED16( class) D_btPersistentManifold : public D_btTypedObject
{

	D_btManifoldPoint m_pointCache[D_MANIFOLD_CACHE_SIZE];

	/// this two body pointers D_can point D_to the physics rigidbody class.
	/// void* D_will allow any rigidbody class
	void* m_body0;
	void* m_body1;
	int	m_cachedPoints;

	D_btScalar	m_contactBreakingThreshold;
	D_btScalar	m_contactProcessingThreshold;

	
	/// sort cached points so most isolated points come first
	int	sortCachedPoints(const D_btManifoldPoint& pt);

	int		findContactPoint(const D_btManifoldPoint* unUsed, int numUnused,const D_btManifoldPoint& pt);

public:

	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	int m_index1a;

	D_btPersistentManifold();

	D_btPersistentManifold(void* body0,void* body1,int , D_btScalar contactBreakingThreshold,D_btScalar contactProcessingThreshold)
		: D_btTypedObject(D_BT_PERSISTENT_MANIFOLD_TYPE),
	m_body0(body0),m_body1(body1),m_cachedPoints(0),
		m_contactBreakingThreshold(contactBreakingThreshold),
		m_contactProcessingThreshold(contactProcessingThreshold)
	{
	}

	D_SIMD_FORCE_INLINE void* getBody0() { return m_body0;}
	D_SIMD_FORCE_INLINE void* getBody1() { return m_body1;}

	D_SIMD_FORCE_INLINE const void* getBody0() const { return m_body0;}
	D_SIMD_FORCE_INLINE const void* getBody1() const { return m_body1;}

	void	setBodies(void* body0,void* body1)
	{
		m_body0 = body0;
		m_body1 = body1;
	}

	void clearUserCache(D_btManifoldPoint& pt);

#ifdef DEBUG_PERSISTENCY
	void	DebugPersistency();
#endif //
	
	D_SIMD_FORCE_INLINE int	getNumContacts() const { return m_cachedPoints;}

	D_SIMD_FORCE_INLINE const D_btManifoldPoint& getContactPoint(int index) const
	{
		D_btAssert(index < m_cachedPoints);
		return m_pointCache[index];
	}

	D_SIMD_FORCE_INLINE D_btManifoldPoint& getContactPoint(int index)
	{
		D_btAssert(index < m_cachedPoints);
		return m_pointCache[index];
	}

	///@todo: get this margin from the current physics / collision environment
	D_btScalar	getContactBreakingThreshold() const;

	D_btScalar	getContactProcessingThreshold() const
	{
		return m_contactProcessingThreshold;
	}
	
	int getCacheEntry(const D_btManifoldPoint& newPoint) const;

	int addManifoldPoint( const D_btManifoldPoint& newPoint);

	void removeContactPoint (int index)
	{
		clearUserCache(m_pointCache[index]);

		int lastUsedIndex = getNumContacts() - 1;
//		m_pointCache[index] = m_pointCache[lastUsedIndex];
		if(index != lastUsedIndex) 
		{
			m_pointCache[index] = m_pointCache[lastUsedIndex]; 
			//get rid of duplicated userPersistentData D_pointer
			m_pointCache[lastUsedIndex].m_userPersistentData = 0;
			m_pointCache[lastUsedIndex].m_appliedImpulse = 0.f;
			m_pointCache[lastUsedIndex].m_lateralFrictionInitialized = false;
			m_pointCache[lastUsedIndex].m_appliedImpulseLateral1 = 0.f;
			m_pointCache[lastUsedIndex].m_appliedImpulseLateral2 = 0.f;
			m_pointCache[lastUsedIndex].m_lifeTime = 0;
		}

		D_btAssert(m_pointCache[lastUsedIndex].m_userPersistentData==0);
		m_cachedPoints--;
	}
	void replaceContactPoint(const D_btManifoldPoint& newPoint,int insertIndex)
	{
		D_btAssert(validContactDistance(newPoint));

#define D_MAINTAIN_PERSISTENCY 1
#ifdef D_MAINTAIN_PERSISTENCY
		int	lifeTime = m_pointCache[insertIndex].getLifeTime();
		D_btScalar	appliedImpulse = m_pointCache[insertIndex].m_appliedImpulse;
		D_btScalar	appliedLateralImpulse1 = m_pointCache[insertIndex].m_appliedImpulseLateral1;
		D_btScalar	appliedLateralImpulse2 = m_pointCache[insertIndex].m_appliedImpulseLateral2;
				
		D_btAssert(lifeTime>=0);
		void* cache = m_pointCache[insertIndex].m_userPersistentData;
		
		m_pointCache[insertIndex] = newPoint;

		m_pointCache[insertIndex].m_userPersistentData = cache;
		m_pointCache[insertIndex].m_appliedImpulse = appliedImpulse;
		m_pointCache[insertIndex].m_appliedImpulseLateral1 = appliedLateralImpulse1;
		m_pointCache[insertIndex].m_appliedImpulseLateral2 = appliedLateralImpulse2;
		
		m_pointCache[insertIndex].m_lifeTime = lifeTime;
#else
		clearUserCache(m_pointCache[insertIndex]);
		m_pointCache[insertIndex] = newPoint;
	
#endif
	}

	bool validContactDistance(const D_btManifoldPoint& pt) const
	{
		return pt.m_distance1 <= getContactBreakingThreshold();
	}
	/// calculated new worldspace coordinates D_and depth, D_and reject points that exceed the collision margin
	void	refreshContactPoints(  const D_btTransform& trA,const D_btTransform& trB);

	
	D_SIMD_FORCE_INLINE	void	clearManifold()
	{
		int i;
		for (i=0;i<m_cachedPoints;i++)
		{
			clearUserCache(m_pointCache[i]);
		}
		m_cachedPoints = 0;
	}



}
;





#endif //PERSISTENT_MANIFOLD_H
