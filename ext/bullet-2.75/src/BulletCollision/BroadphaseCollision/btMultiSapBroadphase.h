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
#ifndef D_BT_MULTI_SAP_BROADPHASE
#define D_BT_MULTI_SAP_BROADPHASE

#include "btBroadphaseInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btOverlappingPairCache.h"


class D_btBroadphaseInterface;
class D_btSimpleBroadphase;


typedef D_btAlignedObjectArray<D_btBroadphaseInterface*> D_btSapBroadphaseArray;

///The D_btMultiSapBroadphase D_is a research project, not recommended D_to use in production. Use D_btAxisSweep3 or D_btDbvtBroadphase instead.
///The D_btMultiSapBroadphase D_is a broadphase that contains multiple SAP broadphases.
///The user D_can add SAP broadphases that cover the world. A D_btBroadphaseProxy D_can be in multiple child broadphases at the same time.
///A D_btQuantizedBvh acceleration structures finds overlapping SAPs for each D_btBroadphaseProxy.
///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=328
///D_and http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=1329
class D_btMultiSapBroadphase :public D_btBroadphaseInterface
{
	D_btSapBroadphaseArray	m_sapBroadphases;
	
	D_btSimpleBroadphase*		m_simpleBroadphase;

	D_btOverlappingPairCache*	m_overlappingPairs;

	class D_btQuantizedBvh*			m_optimizedAabbTree;


	bool					m_ownsPairCache;
	
	D_btOverlapFilterCallback*	m_filterCallback;

	int			m_invalidPair;

	struct	D_btBridgeProxy
	{
		D_btBroadphaseProxy*		m_childProxy;
		D_btBroadphaseInterface*	m_childBroadphase;
	};


public:

	struct	D_btMultiSapProxy	: public D_btBroadphaseProxy
	{

		///array with all the entries that this proxy belongs D_to
		D_btAlignedObjectArray<D_btBridgeProxy*> m_bridgeProxies;
		D_btVector3	m_aabbMin;
		D_btVector3	m_aabbMax;

		int	m_shapeType;

/*		void*	m_userPtr;
		short int	m_collisionFilterGroup;
		short int	m_collisionFilterMask;
*/
		D_btMultiSapProxy(const D_btVector3& aabbMin,  const D_btVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask)
			:D_btBroadphaseProxy(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask),
			m_aabbMin(aabbMin),
			m_aabbMax(aabbMax),
			m_shapeType(shapeType)
		{
			m_multiSapParentProxy =this;
		}

		
	};

protected:


	D_btAlignedObjectArray<D_btMultiSapProxy*> m_multiSapProxies;

public:

	D_btMultiSapBroadphase(int maxProxies = 16384,D_btOverlappingPairCache* pairCache=0);


	D_btSapBroadphaseArray&	getBroadphaseArray()
	{
		return m_sapBroadphases;
	}

	const D_btSapBroadphaseArray&	getBroadphaseArray() const
	{
		return m_sapBroadphases;
	}

	virtual ~D_btMultiSapBroadphase();

	virtual D_btBroadphaseProxy*	createProxy(  const D_btVector3& aabbMin,  const D_btVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask, D_btDispatcher* dispatcher,void* multiSapProxy);
	virtual void	destroyProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher);
	virtual void	setAabb(D_btBroadphaseProxy* proxy,const D_btVector3& aabbMin,const D_btVector3& aabbMax, D_btDispatcher* dispatcher);
	virtual void	getAabb(D_btBroadphaseProxy* proxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const;

	virtual void	rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback,const D_btVector3& aabbMin=D_btVector3(0,0,0),const D_btVector3& aabbMax=D_btVector3(0,0,0));

	void	addToChildBroadphase(D_btMultiSapProxy* parentMultiSapProxy, D_btBroadphaseProxy* childProxy, D_btBroadphaseInterface*	childBroadphase);

	///calculateOverlappingPairs D_is optional: incremental algorithms (sweep D_and prune) might do it during the set aabb
	virtual void	calculateOverlappingPairs(D_btDispatcher* dispatcher);

	bool	testAabbOverlap(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1);

	virtual	D_btOverlappingPairCache*	getOverlappingPairCache()
	{
		return m_overlappingPairs;
	}
	virtual	const D_btOverlappingPairCache*	getOverlappingPairCache() const
	{
		return m_overlappingPairs;
	}

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///D_will add some transform later
	virtual void getBroadphaseAabb(D_btVector3& aabbMin,D_btVector3& aabbMax) const
	{
		aabbMin.setValue(-D_BT_LARGE_FLOAT,-D_BT_LARGE_FLOAT,-D_BT_LARGE_FLOAT);
		aabbMax.setValue(D_BT_LARGE_FLOAT,D_BT_LARGE_FLOAT,D_BT_LARGE_FLOAT);
	}

	void	buildTree(const D_btVector3& bvhAabbMin,const D_btVector3& bvhAabbMax);

	virtual void	printStats();

	void quicksort (D_btBroadphasePairArray& a, int lo, int hi);

	///reset broadphase internal structures, D_to ensure determinism/reproducability
	virtual void resetPool(D_btDispatcher* dispatcher);

};

#endif //D_BT_MULTI_SAP_BROADPHASE
