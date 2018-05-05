/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///D_btDbvtBroadphase implementation by Nathanael Presson
#ifndef D_BT_DBVT_BROADPHASE_H
#define D_BT_DBVT_BROADPHASE_H

#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"

//
// Compile time config
//

#define	DBVT_BP_PROFILE					0
//#define DBVT_BP_SORTPAIRS				1
#define D_DBVT_BP_PREVENTFALSEUPDATE		0
#define D_DBVT_BP_ACCURATESLEEPING		0
#define D_DBVT_BP_ENABLE_BENCHMARK		0
#define D_DBVT_BP_MARGIN					(D_btScalar)0.05

#if DBVT_BP_PROFILE
#define	DBVT_BP_PROFILING_RATE	256
#include "LinearMath/btQuickprof.h"
#endif

//
// D_btDbvtProxy
//
struct D_btDbvtProxy : D_btBroadphaseProxy
{
	/* Fields		*/ 
	//D_btDbvtAabbMm	aabb;
	D_btDbvtNode*		leaf;
	D_btDbvtProxy*	links[2];
	int				stage;
	/* ctor			*/ 
	D_btDbvtProxy(const D_btVector3& aabbMin,const D_btVector3& aabbMax,void* userPtr,short int collisionFilterGroup, short int collisionFilterMask) :
	D_btBroadphaseProxy(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask)
	{
		links[0]=links[1]=0;
	}
};

typedef D_btAlignedObjectArray<D_btDbvtProxy*>	D_btDbvtProxyArray;

///The D_btDbvtBroadphase D_implements a broadphase using two dynamic AABB bounding volume hierarchies/trees (see D_btDbvt).
///One tree D_is used for static/non-moving objects, D_and another tree D_is used for dynamic objects. Objects D_can move from one tree D_to the other.
///This D_is a very fast broadphase, especially for very dynamic worlds where many objects D_are moving. Its insert/add D_and remove of objects D_is generally faster than the sweep D_and prune broadphases D_btAxisSweep3 D_and D_bt32BitAxisSweep3.
struct	D_btDbvtBroadphase : D_btBroadphaseInterface
{
	/* Config		*/ 
	enum	{
		DYNAMIC_SET			=	0,	/* Dynamic set index	*/ 
		FIXED_SET			=	1,	/* Fixed set index		*/ 
		STAGECOUNT			=	2	/* Number of stages		*/ 
	};
	/* Fields		*/ 
	D_btDbvt					m_sets[2];					// Dbvt sets
	D_btDbvtProxy*			m_stageRoots[STAGECOUNT+1];	// Stages list
	D_btOverlappingPairCache*	m_paircache;				// Pair cache
	D_btScalar				m_prediction;				// Velocity prediction
	int						m_stageCurrent;				// Current stage
	int						m_fupdates;					// % of fixed updates per frame
	int						m_dupdates;					// % of dynamic updates per frame
	int						m_cupdates;					// % of cleanup updates per frame
	int						m_newpairs;					// Number of pairs created
	int						m_fixedleft;				// Fixed optimization left
	unsigned				m_updates_call;				// Number of updates call
	unsigned				m_updates_done;				// Number of updates done
	D_btScalar				m_updates_ratio;			// m_updates_done/m_updates_call
	int						m_pid;						// Parse id
	int						m_cid;						// Cleanup index
	int						m_gid;						// Gen id
	bool					m_releasepaircache;			// Release pair cache on delete
	bool					m_deferedcollide;			// Defere dynamic/static collision D_to collide call
	bool					m_needcleanup;				// Need D_to run cleanup?
#if DBVT_BP_PROFILE
	D_btClock					m_clock;
	struct	{
		unsigned long		m_total;
		unsigned long		m_ddcollide;
		unsigned long		m_fdcollide;
		unsigned long		m_cleanup;
		unsigned long		m_jobcount;
	}				m_profiling;
#endif
	/* Methods		*/ 
	D_btDbvtBroadphase(D_btOverlappingPairCache* paircache=0);
	~D_btDbvtBroadphase();
	void							collide(D_btDispatcher* dispatcher);
	void							optimize();
	/* D_btBroadphaseInterface Implementation	*/ 
	D_btBroadphaseProxy*				createProxy(const D_btVector3& aabbMin,const D_btVector3& aabbMax,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask,D_btDispatcher* dispatcher,void* multiSapProxy);
	void							destroyProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher);
	void							setAabb(D_btBroadphaseProxy* proxy,const D_btVector3& aabbMin,const D_btVector3& aabbMax,D_btDispatcher* dispatcher);
	virtual void	rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback, const D_btVector3& aabbMin=D_btVector3(0,0,0), const D_btVector3& aabbMax = D_btVector3(0,0,0));

	virtual void	getAabb(D_btBroadphaseProxy* proxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const;
	void							calculateOverlappingPairs(D_btDispatcher* dispatcher);
	D_btOverlappingPairCache*			getOverlappingPairCache();
	const D_btOverlappingPairCache*	getOverlappingPairCache() const;
	void							getBroadphaseAabb(D_btVector3& aabbMin,D_btVector3& aabbMax) const;
	void							printStats();
	static void						benchmark(D_btBroadphaseInterface*);

	void	setVelocityPrediction(D_btScalar prediction)
	{
		m_prediction = prediction;
	}
	D_btScalar getVelocityPrediction() const
	{
		return m_prediction;
	}
	
	void	performDeferredRemoval(D_btDispatcher* dispatcher);

	///reset broadphase internal structures, D_to ensure determinism/reproducability
	virtual void resetPool(D_btDispatcher* dispatcher);

};

#endif
