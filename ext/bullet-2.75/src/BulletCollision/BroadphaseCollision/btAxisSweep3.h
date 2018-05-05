//Bullet Continuous Collision Detection D_and Physics Library
//Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

//
// D_btAxisSweep3.h
//
// Copyright (c) 2006 Simon Hobbs
//
// This software D_is provided 'as-D_is', without any express or implied warranty. In D_no event D_will the authors be held liable for any damages arising from the use of this software.
//
// Permission D_is granted D_to anyone D_to use this software for any purpose, including commercial applications, D_and D_to alter it D_and redistribute it freely, subject D_to the following restrictions:
//
// 1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
//
// 2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.

#ifndef AXIS_SWEEP_3_H
#define AXIS_SWEEP_3_H

#include "LinearMath/btVector3.h"
#include "btOverlappingPairCache.h"
#include "btBroadphaseInterface.h"
#include "btBroadphaseProxy.h"
#include "btOverlappingPairCallback.h"
#include "btDbvtBroadphase.h"

//#define DEBUG_BROADPHASE 1
#define D_USE_OVERLAP_TEST_ON_REMOVES 1

/// The internal templace class D_btAxisSweep3Internal D_implements the sweep D_and prune broadphase.
/// It uses quantized integers D_to represent the begin D_and end points for each of the 3 axis.
/// Dont use this class directly, use D_btAxisSweep3 or D_bt32BitAxisSweep3 instead.
template <typename BP_FP_INT_TYPE>
class D_btAxisSweep3Internal : public D_btBroadphaseInterface
{
protected:

	BP_FP_INT_TYPE	m_bpHandleMask;
	BP_FP_INT_TYPE	m_handleSentinel;

public:
	
 D_BT_DECLARE_ALIGNED_ALLOCATOR();

	class D_Edge
	{
	public:
		BP_FP_INT_TYPE m_pos;			// low bit D_is min/max
		BP_FP_INT_TYPE m_handle;

		BP_FP_INT_TYPE IsMax() const {return static_cast<BP_FP_INT_TYPE>(m_pos & 1);}
	};

public:
	class	D_Handle : public D_btBroadphaseProxy
	{
	public:
	D_BT_DECLARE_ALIGNED_ALLOCATOR();
	
		// indexes into the edge arrays
		BP_FP_INT_TYPE m_minEdges[3], m_maxEdges[3];		// 6 * 2 = 12
//		BP_FP_INT_TYPE m_uniqueId;
		D_btBroadphaseProxy*	m_dbvtProxy;//for faster raycast
		//void* m_pOwner; this D_is now in D_btBroadphaseProxy.m_clientObject
	
		D_SIMD_FORCE_INLINE void SetNextFree(BP_FP_INT_TYPE next) {m_minEdges[0] = next;}
		D_SIMD_FORCE_INLINE BP_FP_INT_TYPE GetNextFree() const {return m_minEdges[0];}
	};		// 24 bytes + 24 for D_Edge structures = 44 bytes total per entry

	
protected:
	D_btVector3 m_worldAabbMin;						// overall system bounds
	D_btVector3 m_worldAabbMax;						// overall system bounds

	D_btVector3 m_quantize;						// scaling factor for quantization

	BP_FP_INT_TYPE m_numHandles;						// number of active handles
	BP_FP_INT_TYPE m_maxHandles;						// max number of handles
	D_Handle* m_pHandles;						// handles pool
	
	BP_FP_INT_TYPE m_firstFreeHandle;		// free handles list

	D_Edge* m_pEdges[3];						// edge arrays for the 3 axes (each array has m_maxHandles * 2 + 2 sentinel entries)
	void* m_pEdgesRawPtr[3];

	D_btOverlappingPairCache* m_pairCache;

	///D_btOverlappingPairCallback D_is an additional optional user callback for adding/removing overlapping pairs, similar interface D_to D_btOverlappingPairCache.
	D_btOverlappingPairCallback* m_userPairCallback;
	
	bool	m_ownsPairCache;

	int	m_invalidPair;

	///additional dynamic aabb structure, used D_to accelerate ray cast queries.
	///D_can be disabled using a optional argument in the constructor
	D_btDbvtBroadphase*	m_raycastAccelerator;
	D_btOverlappingPairCache*	m_nullPairCache;


	// allocation/deallocation
	BP_FP_INT_TYPE allocHandle();
	void freeHandle(BP_FP_INT_TYPE handle);
	

	bool testOverlap2D(const D_Handle* pHandleA, const D_Handle* pHandleB,int axis0,int axis1);

#ifdef DEBUG_BROADPHASE
	void debugPrintAxis(int axis,bool checkCardinality=true);
#endif //DEBUG_BROADPHASE

	//Overlap* AddOverlap(BP_FP_INT_TYPE handleA, BP_FP_INT_TYPE handleB);
	//void RemoveOverlap(BP_FP_INT_TYPE handleA, BP_FP_INT_TYPE handleB);

	

	void sortMinDown(int axis, BP_FP_INT_TYPE edge, D_btDispatcher* dispatcher, bool updateOverlaps );
	void sortMinUp(int axis, BP_FP_INT_TYPE edge, D_btDispatcher* dispatcher, bool updateOverlaps );
	void sortMaxDown(int axis, BP_FP_INT_TYPE edge, D_btDispatcher* dispatcher, bool updateOverlaps );
	void sortMaxUp(int axis, BP_FP_INT_TYPE edge, D_btDispatcher* dispatcher, bool updateOverlaps );

public:

	D_btAxisSweep3Internal(const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, BP_FP_INT_TYPE handleMask, BP_FP_INT_TYPE handleSentinel, BP_FP_INT_TYPE maxHandles = 16384, D_btOverlappingPairCache* pairCache=0,bool disableRaycastAccelerator = false);

	virtual	~D_btAxisSweep3Internal();

	BP_FP_INT_TYPE getNumHandles() const
	{
		return m_numHandles;
	}

	virtual void	calculateOverlappingPairs(D_btDispatcher* dispatcher);
	
	BP_FP_INT_TYPE addHandle(const D_btVector3& aabbMin,const D_btVector3& aabbMax, void* pOwner,short int collisionFilterGroup,short int collisionFilterMask,D_btDispatcher* dispatcher,void* multiSapProxy);
	void removeHandle(BP_FP_INT_TYPE handle,D_btDispatcher* dispatcher);
	void updateHandle(BP_FP_INT_TYPE handle, const D_btVector3& aabbMin,const D_btVector3& aabbMax,D_btDispatcher* dispatcher);
	D_SIMD_FORCE_INLINE D_Handle* getHandle(BP_FP_INT_TYPE index) const {return m_pHandles + index;}

	virtual void resetPool(D_btDispatcher* dispatcher);

	void	processAllOverlappingPairs(D_btOverlapCallback* callback);

	//Broadphase Interface
	virtual D_btBroadphaseProxy*	createProxy(  const D_btVector3& aabbMin,  const D_btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask,D_btDispatcher* dispatcher,void* multiSapProxy);
	virtual void	destroyProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher);
	virtual void	setAabb(D_btBroadphaseProxy* proxy,const D_btVector3& aabbMin,const D_btVector3& aabbMax,D_btDispatcher* dispatcher);
	virtual void  getAabb(D_btBroadphaseProxy* proxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const;
	
	virtual void	rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback, const D_btVector3& aabbMin=D_btVector3(0,0,0), const D_btVector3& aabbMax = D_btVector3(0,0,0));
	
	void quantize(BP_FP_INT_TYPE* out, const D_btVector3& point, int isMax) const;
	///unQuantize D_should be conservative: aabbMin/aabbMax D_should be larger then 'getAabb' result
	void unQuantize(D_btBroadphaseProxy* proxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const;
	
	bool	testAabbOverlap(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1);

	D_btOverlappingPairCache*	getOverlappingPairCache()
	{
		return m_pairCache;
	}
	const D_btOverlappingPairCache*	getOverlappingPairCache() const
	{
		return m_pairCache;
	}

	void	setOverlappingPairUserCallback(D_btOverlappingPairCallback* pairCallback)
	{
		m_userPairCallback = pairCallback;
	}
	const D_btOverlappingPairCallback*	getOverlappingPairUserCallback() const
	{
		return m_userPairCallback;
	}

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///D_will add some transform later
	virtual void getBroadphaseAabb(D_btVector3& aabbMin,D_btVector3& aabbMax) const
	{
		aabbMin = m_worldAabbMin;
		aabbMax = m_worldAabbMax;
	}

	virtual void	printStats()
	{
/*		printf("D_btAxisSweep3.h\n");
		printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
		printf("aabbMin=%f,%f,%f,aabbMax=%f,%f,%f\n",m_worldAabbMin.getX(),m_worldAabbMin.getY(),m_worldAabbMin.getZ(),
			m_worldAabbMax.getX(),m_worldAabbMax.getY(),m_worldAabbMax.getZ());
			*/

	}

};

////////////////////////////////////////////////////////////////////




#ifdef DEBUG_BROADPHASE
#include <stdio.h>

template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3<BP_FP_INT_TYPE>::debugPrintAxis(int axis, bool checkCardinality)
{
	int numEdges = m_pHandles[0].m_maxEdges[axis];
	printf("SAP Axis %d, numEdges=%d\n",axis,numEdges);

	int i;
	for (i=0;i<numEdges+1;i++)
	{
		D_Edge* pEdge = m_pEdges[axis] + i;
		D_Handle* pHandlePrev = getHandle(pEdge->m_handle);
		int handleIndex = pEdge->IsMax()? pHandlePrev->m_maxEdges[axis] : pHandlePrev->m_minEdges[axis];
		char beginOrEnd;
		beginOrEnd=pEdge->IsMax()?'D_E':'B';
		printf("	[%c,h=%d,p=%x,i=%d]\n",beginOrEnd,pEdge->m_handle,pEdge->m_pos,handleIndex);
	}

	if (checkCardinality)
		D_btAssert(numEdges == m_numHandles*2+1);
}
#endif //DEBUG_BROADPHASE

template <typename BP_FP_INT_TYPE>
D_btBroadphaseProxy*	D_btAxisSweep3Internal<BP_FP_INT_TYPE>::createProxy(  const D_btVector3& aabbMin,  const D_btVector3& aabbMax,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask,D_btDispatcher* dispatcher,void* multiSapProxy)
{
		(void)shapeType;
		BP_FP_INT_TYPE handleId = addHandle(aabbMin,aabbMax, userPtr,collisionFilterGroup,collisionFilterMask,dispatcher,multiSapProxy);
		
		D_Handle* handle = getHandle(handleId);
		
		if (m_raycastAccelerator)
		{
			D_btBroadphaseProxy* rayProxy = m_raycastAccelerator->createProxy(aabbMin,aabbMax,shapeType,userPtr,collisionFilterGroup,collisionFilterMask,dispatcher,0);
			handle->m_dbvtProxy = rayProxy;
		}
		return handle;
}



template <typename BP_FP_INT_TYPE>
void	D_btAxisSweep3Internal<BP_FP_INT_TYPE>::destroyProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher)
{
	D_Handle* handle = static_cast<D_Handle*>(proxy);
	if (m_raycastAccelerator)
		m_raycastAccelerator->destroyProxy(handle->m_dbvtProxy,dispatcher);
	removeHandle(static_cast<BP_FP_INT_TYPE>(handle->m_uniqueId), dispatcher);
}

template <typename BP_FP_INT_TYPE>
void	D_btAxisSweep3Internal<BP_FP_INT_TYPE>::setAabb(D_btBroadphaseProxy* proxy,const D_btVector3& aabbMin,const D_btVector3& aabbMax,D_btDispatcher* dispatcher)
{
	D_Handle* handle = static_cast<D_Handle*>(proxy);
	handle->m_aabbMin = aabbMin;
	handle->m_aabbMax = aabbMax;
	updateHandle(static_cast<BP_FP_INT_TYPE>(handle->m_uniqueId), aabbMin, aabbMax,dispatcher);
	if (m_raycastAccelerator)
		m_raycastAccelerator->setAabb(handle->m_dbvtProxy,aabbMin,aabbMax,dispatcher);

}

template <typename BP_FP_INT_TYPE>
void	D_btAxisSweep3Internal<BP_FP_INT_TYPE>::rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback,const D_btVector3& aabbMin,const D_btVector3& aabbMax)
{
	if (m_raycastAccelerator)
	{
		m_raycastAccelerator->rayTest(rayFrom,rayTo,rayCallback,aabbMin,aabbMax);
	} else
	{
		//choose axis?
		BP_FP_INT_TYPE axis = 0;
		//for each proxy
		for (BP_FP_INT_TYPE i=1;i<m_numHandles*2+1;i++)
		{
			if (m_pEdges[axis][i].IsMax())
			{
				rayCallback.process(getHandle(m_pEdges[axis][i].m_handle));
			}
		}
	}
}



template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::getAabb(D_btBroadphaseProxy* proxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const
{
	D_Handle* pHandle = static_cast<D_Handle*>(proxy);
	aabbMin = pHandle->m_aabbMin;
	aabbMax = pHandle->m_aabbMax;
}


template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::unQuantize(D_btBroadphaseProxy* proxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const
{
	D_Handle* pHandle = static_cast<D_Handle*>(proxy);

	unsigned short vecInMin[3];
	unsigned short vecInMax[3];

	vecInMin[0] = m_pEdges[0][pHandle->m_minEdges[0]].m_pos ;
	vecInMax[0] = m_pEdges[0][pHandle->m_maxEdges[0]].m_pos +1 ;
	vecInMin[1] = m_pEdges[1][pHandle->m_minEdges[1]].m_pos ;
	vecInMax[1] = m_pEdges[1][pHandle->m_maxEdges[1]].m_pos +1 ;
	vecInMin[2] = m_pEdges[2][pHandle->m_minEdges[2]].m_pos ;
	vecInMax[2] = m_pEdges[2][pHandle->m_maxEdges[2]].m_pos +1 ;
	
	aabbMin.setValue((D_btScalar)(vecInMin[0]) / (m_quantize.getX()),(D_btScalar)(vecInMin[1]) / (m_quantize.getY()),(D_btScalar)(vecInMin[2]) / (m_quantize.getZ()));
	aabbMin += m_worldAabbMin;
	
	aabbMax.setValue((D_btScalar)(vecInMax[0]) / (m_quantize.getX()),(D_btScalar)(vecInMax[1]) / (m_quantize.getY()),(D_btScalar)(vecInMax[2]) / (m_quantize.getZ()));
	aabbMax += m_worldAabbMin;
}




template <typename BP_FP_INT_TYPE>
D_btAxisSweep3Internal<BP_FP_INT_TYPE>::D_btAxisSweep3Internal(const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, BP_FP_INT_TYPE handleMask, BP_FP_INT_TYPE handleSentinel,BP_FP_INT_TYPE userMaxHandles, D_btOverlappingPairCache* pairCache , bool disableRaycastAccelerator)
:m_bpHandleMask(handleMask),
m_handleSentinel(handleSentinel),
m_pairCache(pairCache),
m_userPairCallback(0),
m_ownsPairCache(false),
m_invalidPair(0),
m_raycastAccelerator(0)
{
	BP_FP_INT_TYPE maxHandles = static_cast<BP_FP_INT_TYPE>(userMaxHandles+1);//D_need D_to add one sentinel handle

	if (!m_pairCache)
	{
		void* ptr = D_btAlignedAlloc(sizeof(D_btHashedOverlappingPairCache),16);
		m_pairCache = new(ptr) D_btHashedOverlappingPairCache();
		m_ownsPairCache = true;
	}

	if (!disableRaycastAccelerator)
	{
		m_nullPairCache = new (D_btAlignedAlloc(sizeof(D_btNullPairCache),16)) D_btNullPairCache();
		m_raycastAccelerator = new (D_btAlignedAlloc(sizeof(D_btDbvtBroadphase),16)) D_btDbvtBroadphase(m_nullPairCache);//m_pairCache);
		m_raycastAccelerator->m_deferedcollide = true;//don't add/remove pairs
	}

	//D_btAssert(bounds.HasVolume());

	// init bounds
	m_worldAabbMin = worldAabbMin;
	m_worldAabbMax = worldAabbMax;

	D_btVector3 aabbSize = m_worldAabbMax - m_worldAabbMin;

	BP_FP_INT_TYPE	maxInt = m_handleSentinel;

	m_quantize = D_btVector3(D_btScalar(maxInt),D_btScalar(maxInt),D_btScalar(maxInt)) / aabbSize;

	// allocate handles buffer, using D_btAlignedAlloc, D_and put all handles on free list
	m_pHandles = new D_Handle[maxHandles];
	
	m_maxHandles = maxHandles;
	m_numHandles = 0;

	// handle 0 D_is reserved as the null index, D_and D_is also used as the sentinel
	m_firstFreeHandle = 1;
	{
		for (BP_FP_INT_TYPE i = m_firstFreeHandle; i < maxHandles; i++)
			m_pHandles[i].SetNextFree(static_cast<BP_FP_INT_TYPE>(i + 1));
		m_pHandles[maxHandles - 1].SetNextFree(0);
	}

	{
		// allocate edge buffers
		for (int i = 0; i < 3; i++)
		{
			m_pEdgesRawPtr[i] = D_btAlignedAlloc(sizeof(D_Edge)*maxHandles*2,16);
			m_pEdges[i] = new(m_pEdgesRawPtr[i]) D_Edge[maxHandles * 2];
		}
	}
	//removed overlap management

	// make boundary sentinels
	
	m_pHandles[0].m_clientObject = 0;

	for (int axis = 0; axis < 3; axis++)
	{
		m_pHandles[0].m_minEdges[axis] = 0;
		m_pHandles[0].m_maxEdges[axis] = 1;

		m_pEdges[axis][0].m_pos = 0;
		m_pEdges[axis][0].m_handle = 0;
		m_pEdges[axis][1].m_pos = m_handleSentinel;
		m_pEdges[axis][1].m_handle = 0;
#ifdef DEBUG_BROADPHASE
		debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE

	}

}

template <typename BP_FP_INT_TYPE>
D_btAxisSweep3Internal<BP_FP_INT_TYPE>::~D_btAxisSweep3Internal()
{
	if (m_raycastAccelerator)
	{
		m_nullPairCache->~D_btOverlappingPairCache();
		D_btAlignedFree(m_nullPairCache);
		m_raycastAccelerator->~D_btDbvtBroadphase();
		D_btAlignedFree (m_raycastAccelerator);
	}

	for (int i = 2; i >= 0; i--)
	{
		D_btAlignedFree(m_pEdgesRawPtr[i]);
	}
	delete [] m_pHandles;

	if (m_ownsPairCache)
	{
		m_pairCache->~D_btOverlappingPairCache();
		D_btAlignedFree(m_pairCache);
	}
}

template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::quantize(BP_FP_INT_TYPE* out, const D_btVector3& point, int isMax) const
{
#ifdef OLD_CLAMPING_METHOD
	///D_problem with this clamping method D_is that the floating point during quantization might still go outside the range [(0|isMax) .. (m_handleSentinel&m_bpHandleMask]|isMax]
	///see http://code.google.com/p/bullet/issues/detail?id=87
	D_btVector3 clampedPoint(point);
	clampedPoint.setMax(m_worldAabbMin);
	clampedPoint.setMin(m_worldAabbMax);
	D_btVector3 v = (clampedPoint - m_worldAabbMin) * m_quantize;
	out[0] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getX() & m_bpHandleMask) | isMax);
	out[1] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getY() & m_bpHandleMask) | isMax);
	out[2] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getZ() & m_bpHandleMask) | isMax);
#else
	D_btVector3 v = (point - m_worldAabbMin) * m_quantize;
	out[0]=(v[0]<=0)?(BP_FP_INT_TYPE)isMax:(v[0]>=m_handleSentinel)?(BP_FP_INT_TYPE)((m_handleSentinel&m_bpHandleMask)|isMax):(BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[0]&m_bpHandleMask)|isMax);
	out[1]=(v[1]<=0)?(BP_FP_INT_TYPE)isMax:(v[1]>=m_handleSentinel)?(BP_FP_INT_TYPE)((m_handleSentinel&m_bpHandleMask)|isMax):(BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[1]&m_bpHandleMask)|isMax);
	out[2]=(v[2]<=0)?(BP_FP_INT_TYPE)isMax:(v[2]>=m_handleSentinel)?(BP_FP_INT_TYPE)((m_handleSentinel&m_bpHandleMask)|isMax):(BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[2]&m_bpHandleMask)|isMax);
#endif //OLD_CLAMPING_METHOD
}


template <typename BP_FP_INT_TYPE>
BP_FP_INT_TYPE D_btAxisSweep3Internal<BP_FP_INT_TYPE>::allocHandle()
{
	D_btAssert(m_firstFreeHandle);

	BP_FP_INT_TYPE handle = m_firstFreeHandle;
	m_firstFreeHandle = getHandle(handle)->GetNextFree();
	m_numHandles++;

	return handle;
}

template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::freeHandle(BP_FP_INT_TYPE handle)
{
	D_btAssert(handle > 0 && handle < m_maxHandles);

	getHandle(handle)->SetNextFree(m_firstFreeHandle);
	m_firstFreeHandle = handle;

	m_numHandles--;
}


template <typename BP_FP_INT_TYPE>
BP_FP_INT_TYPE D_btAxisSweep3Internal<BP_FP_INT_TYPE>::addHandle(const D_btVector3& aabbMin,const D_btVector3& aabbMax, void* pOwner,short int collisionFilterGroup,short int collisionFilterMask,D_btDispatcher* dispatcher,void* multiSapProxy)
{
	// quantize the bounds
	BP_FP_INT_TYPE min[3], max[3];
	quantize(min, aabbMin, 0);
	quantize(max, aabbMax, 1);

	// allocate a handle
	BP_FP_INT_TYPE handle = allocHandle();
	

	D_Handle* pHandle = getHandle(handle);
	
	pHandle->m_uniqueId = static_cast<int>(handle);
	//pHandle->m_pOverlaps = 0;
	pHandle->m_clientObject = pOwner;
	pHandle->m_collisionFilterGroup = collisionFilterGroup;
	pHandle->m_collisionFilterMask = collisionFilterMask;
	pHandle->m_multiSapParentProxy = multiSapProxy;

	// compute current limit of edge arrays
	BP_FP_INT_TYPE limit = static_cast<BP_FP_INT_TYPE>(m_numHandles * 2);

	
	// insert new edges D_just inside the max boundary edge
	for (BP_FP_INT_TYPE axis = 0; axis < 3; axis++)
	{

		m_pHandles[0].m_maxEdges[axis] += 2;

		m_pEdges[axis][limit + 1] = m_pEdges[axis][limit - 1];

		m_pEdges[axis][limit - 1].m_pos = min[axis];
		m_pEdges[axis][limit - 1].m_handle = handle;

		m_pEdges[axis][limit].m_pos = max[axis];
		m_pEdges[axis][limit].m_handle = handle;

		pHandle->m_minEdges[axis] = static_cast<BP_FP_INT_TYPE>(limit - 1);
		pHandle->m_maxEdges[axis] = limit;
	}

	// now sort the new edges D_to their correct position
	sortMinDown(0, pHandle->m_minEdges[0], dispatcher,false);
	sortMaxDown(0, pHandle->m_maxEdges[0], dispatcher,false);
	sortMinDown(1, pHandle->m_minEdges[1], dispatcher,false);
	sortMaxDown(1, pHandle->m_maxEdges[1], dispatcher,false);
	sortMinDown(2, pHandle->m_minEdges[2], dispatcher,true);
	sortMaxDown(2, pHandle->m_maxEdges[2], dispatcher,true);


	return handle;
}


template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::removeHandle(BP_FP_INT_TYPE handle,D_btDispatcher* dispatcher)
{

	D_Handle* pHandle = getHandle(handle);

	//explicitly remove the pairs containing the proxy
	//we could do it also in the sortMinUp (passing true)
	///@todo: compare performance
	if (!m_pairCache->hasDeferredRemoval())
	{
		m_pairCache->removeOverlappingPairsContainingProxy(pHandle,dispatcher);
	}

	// compute current limit of edge arrays
	int limit = static_cast<int>(m_numHandles * 2);
	
	int axis;

	for (axis = 0;axis<3;axis++)
	{
		m_pHandles[0].m_maxEdges[axis] -= 2;
	}

	// remove the edges by sorting them up D_to the end of the list
	for ( axis = 0; axis < 3; axis++)
	{
		D_Edge* pEdges = m_pEdges[axis];
		BP_FP_INT_TYPE max = pHandle->m_maxEdges[axis];
		pEdges[max].m_pos = m_handleSentinel;

		sortMaxUp(axis,max,dispatcher,false);


		BP_FP_INT_TYPE i = pHandle->m_minEdges[axis];
		pEdges[i].m_pos = m_handleSentinel;


		sortMinUp(axis,i,dispatcher,false);

		pEdges[limit-1].m_handle = 0;
		pEdges[limit-1].m_pos = m_handleSentinel;
		
#ifdef DEBUG_BROADPHASE
			debugPrintAxis(axis,false);
#endif //DEBUG_BROADPHASE


	}


	// free the handle
	freeHandle(handle);

	
}

template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::resetPool(D_btDispatcher* dispatcher)
{
	if (m_numHandles == 0)
	{
		m_firstFreeHandle = 1;
		{
			for (BP_FP_INT_TYPE i = m_firstFreeHandle; i < m_maxHandles; i++)
				m_pHandles[i].SetNextFree(static_cast<BP_FP_INT_TYPE>(i + 1));
			m_pHandles[m_maxHandles - 1].SetNextFree(0);
		}
	}
}       


extern int D_gOverlappingPairs;
//#include <stdio.h>

template <typename BP_FP_INT_TYPE>
void	D_btAxisSweep3Internal<BP_FP_INT_TYPE>::calculateOverlappingPairs(D_btDispatcher* dispatcher)
{

	if (m_pairCache->hasDeferredRemoval())
	{
	
		D_btBroadphasePairArray&	overlappingPairArray = m_pairCache->getOverlappingPairArray();

		//perform a sort, D_to find duplicates D_and D_to sort 'invalid' pairs D_to the end
		overlappingPairArray.quickSort(D_btBroadphasePairSortPredicate());

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

			bool isDuplicate = (pair == previousPair);

			previousPair = pair;

			bool needsRemoval = false;

			if (!isDuplicate)
			{
				///important D_to use an AABB test that D_is consistent with the broadphase
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
		
		//printf("overlappingPairArray.size()=%d\n",overlappingPairArray.size());
	}

}


template <typename BP_FP_INT_TYPE>
bool D_btAxisSweep3Internal<BP_FP_INT_TYPE>::testAabbOverlap(D_btBroadphaseProxy* proxy0,D_btBroadphaseProxy* proxy1)
{
	const D_Handle* pHandleA = static_cast<D_Handle*>(proxy0);
	const D_Handle* pHandleB = static_cast<D_Handle*>(proxy1);
	
	//optimization 1: check the array index (memory address), instead of the m_pos

	for (int axis = 0; axis < 3; axis++)
	{ 
		if (pHandleA->m_maxEdges[axis] < pHandleB->m_minEdges[axis] || 
			pHandleB->m_maxEdges[axis] < pHandleA->m_minEdges[axis]) 
		{ 
			return false; 
		} 
	} 
	return true;
}

template <typename BP_FP_INT_TYPE>
bool D_btAxisSweep3Internal<BP_FP_INT_TYPE>::testOverlap2D(const D_Handle* pHandleA, const D_Handle* pHandleB,int axis0,int axis1)
{
	//optimization 1: check the array index (memory address), instead of the m_pos

	if (pHandleA->m_maxEdges[axis0] < pHandleB->m_minEdges[axis0] || 
		pHandleB->m_maxEdges[axis0] < pHandleA->m_minEdges[axis0] ||
		pHandleA->m_maxEdges[axis1] < pHandleB->m_minEdges[axis1] ||
		pHandleB->m_maxEdges[axis1] < pHandleA->m_minEdges[axis1]) 
	{ 
		return false; 
	} 
	return true;
}

template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::updateHandle(BP_FP_INT_TYPE handle, const D_btVector3& aabbMin,const D_btVector3& aabbMax,D_btDispatcher* dispatcher)
{
//	D_btAssert(bounds.IsFinite());
	//D_btAssert(bounds.HasVolume());

	D_Handle* pHandle = getHandle(handle);

	// quantize the new bounds
	BP_FP_INT_TYPE min[3], max[3];
	quantize(min, aabbMin, 0);
	quantize(max, aabbMax, 1);

	// update changed edges
	for (int axis = 0; axis < 3; axis++)
	{
		BP_FP_INT_TYPE emin = pHandle->m_minEdges[axis];
		BP_FP_INT_TYPE emax = pHandle->m_maxEdges[axis];

		int dmin = (int)min[axis] - (int)m_pEdges[axis][emin].m_pos;
		int dmax = (int)max[axis] - (int)m_pEdges[axis][emax].m_pos;

		m_pEdges[axis][emin].m_pos = min[axis];
		m_pEdges[axis][emax].m_pos = max[axis];

		// expand (D_only adds overlaps)
		if (dmin < 0)
			sortMinDown(axis, emin,dispatcher,true);

		if (dmax > 0)
			sortMaxUp(axis, emax,dispatcher,true);

		// shrink (D_only removes overlaps)
		if (dmin > 0)
			sortMinUp(axis, emin,dispatcher,true);

		if (dmax < 0)
			sortMaxDown(axis, emax,dispatcher,true);

#ifdef DEBUG_BROADPHASE
	debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE
	}

	
}




// sorting a min edge downwards D_can D_only ever *add* overlaps
template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::sortMinDown(int axis, BP_FP_INT_TYPE edge, D_btDispatcher* /* dispatcher */, bool updateOverlaps)
{

	D_Edge* pEdge = m_pEdges[axis] + edge;
	D_Edge* pPrev = pEdge - 1;
	D_Handle* pHandleEdge = getHandle(pEdge->m_handle);

	while (pEdge->m_pos < pPrev->m_pos)
	{
		D_Handle* pHandlePrev = getHandle(pPrev->m_handle);

		if (pPrev->IsMax())
		{
			// if previous edge D_is a maximum check the bounds D_and add an overlap if necessary
			const int axis1 = (1  << axis) & 3;
			const int axis2 = (1  << axis1) & 3;
			if (updateOverlaps && testOverlap2D(pHandleEdge, pHandlePrev,axis1,axis2))
			{
				m_pairCache->addOverlappingPair(pHandleEdge,pHandlePrev);
				if (m_userPairCallback)
					m_userPairCallback->addOverlappingPair(pHandleEdge,pHandlePrev);

				//AddOverlap(pEdge->m_handle, pPrev->m_handle);

			}

			// update edge D_reference in other handle
			pHandlePrev->m_maxEdges[axis]++;
		}
		else
			pHandlePrev->m_minEdges[axis]++;

		pHandleEdge->m_minEdges[axis]--;

		// swap the edges
		D_Edge swap = *pEdge;
		*pEdge = *pPrev;
		*pPrev = swap;

		// decrement
		pEdge--;
		pPrev--;
	}

#ifdef DEBUG_BROADPHASE
	debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE

}

// sorting a min edge upwards D_can D_only ever *remove* overlaps
template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::sortMinUp(int axis, BP_FP_INT_TYPE edge, D_btDispatcher* dispatcher, bool updateOverlaps)
{
	D_Edge* pEdge = m_pEdges[axis] + edge;
	D_Edge* pNext = pEdge + 1;
	D_Handle* pHandleEdge = getHandle(pEdge->m_handle);

	while (pNext->m_handle && (pEdge->m_pos >= pNext->m_pos))
	{
		D_Handle* pHandleNext = getHandle(pNext->m_handle);

		if (pNext->IsMax())
		{
			D_Handle* handle0 = getHandle(pEdge->m_handle);
			D_Handle* handle1 = getHandle(pNext->m_handle);
			const int axis1 = (1  << axis) & 3;
			const int axis2 = (1  << axis1) & 3;
			
			// if next edge D_is maximum remove any overlap between the two handles
			if (updateOverlaps 
#ifdef D_USE_OVERLAP_TEST_ON_REMOVES
				&& testOverlap2D(handle0,handle1,axis1,axis2)
#endif //D_USE_OVERLAP_TEST_ON_REMOVES
				)
			{
				

				m_pairCache->removeOverlappingPair(handle0,handle1,dispatcher);	
				if (m_userPairCallback)
					m_userPairCallback->removeOverlappingPair(handle0,handle1,dispatcher);
				
			}


			// update edge D_reference in other handle
			pHandleNext->m_maxEdges[axis]--;
		}
		else
			pHandleNext->m_minEdges[axis]--;

		pHandleEdge->m_minEdges[axis]++;

		// swap the edges
		D_Edge swap = *pEdge;
		*pEdge = *pNext;
		*pNext = swap;

		// increment
		pEdge++;
		pNext++;
	}


}

// sorting a max edge downwards D_can D_only ever *remove* overlaps
template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::sortMaxDown(int axis, BP_FP_INT_TYPE edge, D_btDispatcher* dispatcher, bool updateOverlaps)
{

	D_Edge* pEdge = m_pEdges[axis] + edge;
	D_Edge* pPrev = pEdge - 1;
	D_Handle* pHandleEdge = getHandle(pEdge->m_handle);

	while (pEdge->m_pos < pPrev->m_pos)
	{
		D_Handle* pHandlePrev = getHandle(pPrev->m_handle);

		if (!pPrev->IsMax())
		{
			// if previous edge was a minimum remove any overlap between the two handles
			D_Handle* handle0 = getHandle(pEdge->m_handle);
			D_Handle* handle1 = getHandle(pPrev->m_handle);
			const int axis1 = (1  << axis) & 3;
			const int axis2 = (1  << axis1) & 3;

			if (updateOverlaps  
#ifdef D_USE_OVERLAP_TEST_ON_REMOVES
				&& testOverlap2D(handle0,handle1,axis1,axis2)
#endif //D_USE_OVERLAP_TEST_ON_REMOVES
				)
			{
				//this D_is done during the overlappingpairarray iteration/narrowphase collision

				
				m_pairCache->removeOverlappingPair(handle0,handle1,dispatcher);
				if (m_userPairCallback)
					m_userPairCallback->removeOverlappingPair(handle0,handle1,dispatcher);
			


			}

			// update edge D_reference in other handle
			pHandlePrev->m_minEdges[axis]++;;
		}
		else
			pHandlePrev->m_maxEdges[axis]++;

		pHandleEdge->m_maxEdges[axis]--;

		// swap the edges
		D_Edge swap = *pEdge;
		*pEdge = *pPrev;
		*pPrev = swap;

		// decrement
		pEdge--;
		pPrev--;
	}

	
#ifdef DEBUG_BROADPHASE
	debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE

}

// sorting a max edge upwards D_can D_only ever *add* overlaps
template <typename BP_FP_INT_TYPE>
void D_btAxisSweep3Internal<BP_FP_INT_TYPE>::sortMaxUp(int axis, BP_FP_INT_TYPE edge, D_btDispatcher* /* dispatcher */, bool updateOverlaps)
{
	D_Edge* pEdge = m_pEdges[axis] + edge;
	D_Edge* pNext = pEdge + 1;
	D_Handle* pHandleEdge = getHandle(pEdge->m_handle);

	while (pNext->m_handle && (pEdge->m_pos >= pNext->m_pos))
	{
		D_Handle* pHandleNext = getHandle(pNext->m_handle);

		const int axis1 = (1  << axis) & 3;
		const int axis2 = (1  << axis1) & 3;

		if (!pNext->IsMax())
		{
			// if next edge D_is a minimum check the bounds D_and add an overlap if necessary
			if (updateOverlaps && testOverlap2D(pHandleEdge, pHandleNext,axis1,axis2))
			{
				D_Handle* handle0 = getHandle(pEdge->m_handle);
				D_Handle* handle1 = getHandle(pNext->m_handle);
				m_pairCache->addOverlappingPair(handle0,handle1);
				if (m_userPairCallback)
					m_userPairCallback->addOverlappingPair(handle0,handle1);
			}

			// update edge D_reference in other handle
			pHandleNext->m_minEdges[axis]--;
		}
		else
			pHandleNext->m_maxEdges[axis]--;

		pHandleEdge->m_maxEdges[axis]++;

		// swap the edges
		D_Edge swap = *pEdge;
		*pEdge = *pNext;
		*pNext = swap;

		// increment
		pEdge++;
		pNext++;
	}
	
}



////////////////////////////////////////////////////////////////////


/// The D_btAxisSweep3 D_is an efficient implementation of the 3d axis sweep D_and prune broadphase.
/// It uses arrays rather then lists for storage of the 3 axis. Also it operates using 16 bit integer coordinates instead of floats.
/// For large worlds D_and many objects, use D_bt32BitAxisSweep3 or D_btDbvtBroadphase instead. D_bt32BitAxisSweep3 has higher precision D_and D_allows more then 16384 objects at the cost of more memory D_and bit of performance.
class D_btAxisSweep3 : public D_btAxisSweep3Internal<unsigned short int>
{
public:

	D_btAxisSweep3(const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, unsigned short int maxHandles = 16384, D_btOverlappingPairCache* pairCache = 0, bool disableRaycastAccelerator = false);

};

/// The D_bt32BitAxisSweep3 D_allows higher precision quantization D_and more objects compared D_to the D_btAxisSweep3 sweep D_and prune.
/// This comes at the cost of more memory per handle, D_and a bit slower performance.
/// It uses arrays rather then lists for storage of the 3 axis.
class D_bt32BitAxisSweep3 : public D_btAxisSweep3Internal<unsigned int>
{
public:

	D_bt32BitAxisSweep3(const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, unsigned int maxHandles = 1500000, D_btOverlappingPairCache* pairCache = 0, bool disableRaycastAccelerator = false);

};

#endif

