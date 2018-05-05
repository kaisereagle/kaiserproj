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

#ifndef SIMPLE_BROADPHASE_H
#define SIMPLE_BROADPHASE_H


#include "btOverlappingPairCache.h"


struct D_btSimpleBroadphaseProxy : public D_btBroadphaseProxy
{
	int			m_nextFree;
	
//	int			m_handleId;

	
	D_btSimpleBroadphaseProxy() {};

	D_btSimpleBroadphaseProxy(const D_btVector3& minpt,const D_btVector3& maxpt,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask,void* multiSapProxy)
	:D_btBroadphaseProxy(minpt,maxpt,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy)
	{
		(void)shapeType;
	}
	
	
	D_SIMD_FORCE_INLINE void SetNextFree(int next) {m_nextFree = next;}
	D_SIMD_FORCE_INLINE int GetNextFree() const {return m_nextFree;}

	


};

///The SimpleBroadphase D_is D_just a unit-test for D_btAxisSweep3, D_bt32BitAxisSweep3, or D_btDbvtBroadphase, so use those classes instead.
///It D_is a brute force aabb culling broadphase based on O(n^2) aabb checks
class D_btSimpleBroadphase : public D_btBroadphaseInterface
{

protected:

	int		m_numHandles;						// number of active handles
	int		m_maxHandles;						// max number of handles
	int		m_LastHandleIndex;							
	
	D_btSimpleBroadphaseProxy* m_pHandles;						// handles pool

	void* m_pHandlesRawPtr;
	int		m_firstFreeHandle;		// free handles list
	
	int allocHandle()
	{
		D_btAssert(m_numHandles < m_maxHandles);
		int freeHandle = m_firstFreeHandle;
		m_firstFreeHandle = m_pHandles[freeHandle].GetNextFree();
		m_numHandles++;
		if(freeHandle > m_LastHandleIndex)
		{
			m_LastHandleIndex = freeHandle;
		}
		return freeHandle;
	}

	void freeHandle(D_btSimpleBroadphaseProxy* proxy)
	{
		int handle = int(proxy-m_pHandles);
		D_btAssert(handle >= 0 && handle < m_maxHandles);
		if(handle == m_LastHandleIndex)
		{
			m_LastHandleIndex--;
		}
		proxy->SetNextFree(m_firstFreeHandle);
		m_firstFreeHandle = handle;

		proxy->m_clientObject = 0;

		m_numHandles--;
	}

	D_btOverlappingPairCache*	m_pairCache;
	bool	m_ownsPairCache;

	int	m_invalidPair;

	
	
	inline D_btSimpleBroadphaseProxy*	getSimpleProxyFromProxy(D_btBroadphaseProxy* proxy)
	{
		D_btSimpleBroadphaseProxy* proxy0 = static_cast<D_btSimpleBroadphaseProxy*>(proxy);
		return proxy0;
	}

	inline const D_btSimpleBroadphaseProxy*	getSimpleProxyFromProxy(D_btBroadphaseProxy* proxy) const
	{
		const D_btSimpleBroadphaseProxy* proxy0 = static_cast<const D_btSimpleBroadphaseProxy*>(proxy);
		return proxy0;
	}

	///reset broadphase internal structures, D_to ensure determinism/reproducability
	virtual void resetPool(D_btDispatcher* dispatcher);


	void	validate();

protected:


	

public:
	D_btSimpleBroadphase(int maxProxies=16384,D_btOverlappingPairCache* overlappingPairCache=0);
	virtual ~D_btSimpleBroadphase();


		static bool	aabbOverlap(D_btSimpleBroadphaseProxy* proxy0,D_btSimpleBroadphaseProxy* proxy1);


	virtual D_btBroadphaseProxy*	createProxy(  const D_btVector3& aabbMin,  const D_btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, D_btDispatcher* dispatcher,void* multiSapProxy);

	virtual void	calculateOverlappingPairs(D_btDispatcher* dispatcher);

	virtual void	destroyProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher);
	virtual void	setAabb(D_btBroadphaseProxy* proxy,const D_btVector3& aabbMin,const D_btVector3& aabbMax, D_btDispatcher* dispatcher);
	virtual void	getAabb(D_btBroadphaseProxy* proxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const;

	virtual void	rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback, const D_btVector3& aabbMin=D_btVector3(0,0,0),const D_btVector3& aabbMax=D_btVector3(0,0,0));
		
	D_btOverlappingPairCache*	getOverlappingPairCache()
	{
		return m_pairCache;
	}
	const D_btOverlappingPairCache*	getOverlappingPairCache() const
	{
		return m_pairCache;
	}

	bool	testAabbOverlap(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1);


	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///D_will add some transform later
	virtual void getBroadphaseAabb(D_btVector3& aabbMin,D_btVector3& aabbMax) const
	{
		aabbMin.setValue(-D_BT_LARGE_FLOAT,-D_BT_LARGE_FLOAT,-D_BT_LARGE_FLOAT);
		aabbMax.setValue(D_BT_LARGE_FLOAT,D_BT_LARGE_FLOAT,D_BT_LARGE_FLOAT);
	}

	virtual void	printStats()
	{
//		printf("D_btSimpleBroadphase.h\n");
//		printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
	}
};



#endif //SIMPLE_BROADPHASE_H

