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

#include "btDbvtBroadphase.h"

//
// Profiling
//

#if DBVT_BP_PROFILE||D_DBVT_BP_ENABLE_BENCHMARK
#include <stdio.h>
#endif

#if DBVT_BP_PROFILE
struct	ProfileScope
{
	__forceinline ProfileScope(D_btClock& clock,unsigned long& value) :
	m_clock(&clock),m_value(&value),m_base(clock.getTimeMicroseconds())
	{
	}
	__forceinline ~ProfileScope()
	{
		(*m_value)+=m_clock->getTimeMicroseconds()-m_base;
	}
	D_btClock*		m_clock;
	unsigned long*	m_value;
	unsigned long	m_base;
};
#define	SPC(_value_)	ProfileScope	spc_scope(m_clock,_value_)
#else
#define	SPC(_value_)
#endif

//
// Helpers
//

//
template <typename D_T>
static inline void	listappend(D_T* item,D_T*& list)
{
	item->links[0]=0;
	item->links[1]=list;
	if(list) list->links[0]=item;
	list=item;
}

//
template <typename D_T>
static inline void	listremove(D_T* item,D_T*& list)
{
	if(item->links[0]) item->links[0]->links[1]=item->links[1]; else list=item->links[1];
	if(item->links[1]) item->links[1]->links[0]=item->links[0];
}

//
template <typename D_T>
static inline int	listcount(D_T* root)
{
	int	n=0;
	while(root) { ++n;root=root->links[1]; }
	return(n);
}

//
template <typename D_T>
static inline void	clear(D_T& value)
{
	static const struct D_ZeroDummy : D_T {} zerodummy;
	value=zerodummy;
}

//
// Colliders
//

/* Tree collider	*/ 
struct	D_btDbvtTreeCollider : D_btDbvt::ICollide
{
	D_btDbvtBroadphase*	pbp;
	D_btDbvtProxy*		proxy;
	D_btDbvtTreeCollider(D_btDbvtBroadphase* p) : pbp(p) {}
	void	Process(const D_btDbvtNode* na,const D_btDbvtNode* nb)
	{
		if(na!=nb)
		{
			D_btDbvtProxy*	pa=(D_btDbvtProxy*)na->data;
			D_btDbvtProxy*	pb=(D_btDbvtProxy*)nb->data;
#if DBVT_BP_SORTPAIRS
			if(pa->m_uniqueId>pb->m_uniqueId) 
				D_btSwap(pa,pb);
#endif
			pbp->m_paircache->addOverlappingPair(pa,pb);
			++pbp->m_newpairs;
		}
	}
	void	Process(const D_btDbvtNode* n)
	{
		Process(n,proxy->leaf);
	}
};

//
// D_btDbvtBroadphase
//

//
D_btDbvtBroadphase::D_btDbvtBroadphase(D_btOverlappingPairCache* paircache)
{
	m_deferedcollide	=	false;
	m_needcleanup		=	true;
	m_releasepaircache	=	(paircache!=0)?false:true;
	m_prediction		=	0;
	m_stageCurrent		=	0;
	m_fixedleft			=	0;
	m_fupdates			=	1;
	m_dupdates			=	0;
	m_cupdates			=	10;
	m_newpairs			=	1;
	m_updates_call		=	0;
	m_updates_done		=	0;
	m_updates_ratio		=	0;
	m_paircache			=	paircache? paircache	: new(D_btAlignedAlloc(sizeof(D_btHashedOverlappingPairCache),16)) D_btHashedOverlappingPairCache();
	m_gid				=	0;
	m_pid				=	0;
	m_cid				=	0;
	for(int i=0;i<=STAGECOUNT;++i)
	{
		m_stageRoots[i]=0;
	}
#if DBVT_BP_PROFILE
	clear(m_profiling);
#endif
}

//
D_btDbvtBroadphase::~D_btDbvtBroadphase()
{
	if(m_releasepaircache) 
	{
		m_paircache->~D_btOverlappingPairCache();
		D_btAlignedFree(m_paircache);
	}
}

//
D_btBroadphaseProxy*				D_btDbvtBroadphase::createProxy(	const D_btVector3& aabbMin,
															  const D_btVector3& aabbMax,
															  int /*shapeType*/,
															  void* userPtr,
															  short int collisionFilterGroup,
															  short int collisionFilterMask,
															  D_btDispatcher* /*dispatcher*/,
															  void* /*multiSapProxy*/)
{
	D_btDbvtProxy*		proxy=new(D_btAlignedAlloc(sizeof(D_btDbvtProxy),16)) D_btDbvtProxy(	aabbMin,aabbMax,userPtr,
		collisionFilterGroup,
		collisionFilterMask);

	D_btDbvtAabbMm aabb = D_btDbvtVolume::FromMM(aabbMin,aabbMax);

	//bproxy->aabb			=	D_btDbvtVolume::FromMM(aabbMin,aabbMax);
	proxy->stage		=	m_stageCurrent;
	proxy->m_uniqueId	=	++m_gid;
	proxy->leaf			=	m_sets[0].insert(aabb,proxy);
	listappend(proxy,m_stageRoots[m_stageCurrent]);
	if(!m_deferedcollide)
	{
		D_btDbvtTreeCollider	collider(this);
		collider.proxy=proxy;
		m_sets[0].collideTV(m_sets[0].m_root,aabb,collider);
		m_sets[1].collideTV(m_sets[1].m_root,aabb,collider);
	}
	return(proxy);
}

//
void							D_btDbvtBroadphase::destroyProxy(	D_btBroadphaseProxy* absproxy,
															   D_btDispatcher* dispatcher)
{
	D_btDbvtProxy*	proxy=(D_btDbvtProxy*)absproxy;
	if(proxy->stage==STAGECOUNT)
		m_sets[1].remove(proxy->leaf);
	else
		m_sets[0].remove(proxy->leaf);
	listremove(proxy,m_stageRoots[proxy->stage]);
	m_paircache->removeOverlappingPairsContainingProxy(proxy,dispatcher);
	D_btAlignedFree(proxy);
	m_needcleanup=true;
}

void	D_btDbvtBroadphase::getAabb(D_btBroadphaseProxy* absproxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const
{
	D_btDbvtProxy*						proxy=(D_btDbvtProxy*)absproxy;
	aabbMin = proxy->m_aabbMin;
	aabbMax = proxy->m_aabbMax;
}

struct	BroadphaseRayTester : D_btDbvt::ICollide
{
	D_btBroadphaseRayCallback& m_rayCallback;
	BroadphaseRayTester(D_btBroadphaseRayCallback& orgCallback)
		:m_rayCallback(orgCallback)
	{
	}
	void					Process(const D_btDbvtNode* leaf)
	{
		D_btDbvtProxy*	proxy=(D_btDbvtProxy*)leaf->data;
		m_rayCallback.process(proxy);
	}
};	

void	D_btDbvtBroadphase::rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback,const D_btVector3& aabbMin,const D_btVector3& aabbMax)
{
	BroadphaseRayTester callback(rayCallback);

	m_sets[0].rayTestInternal(	m_sets[0].m_root,
		rayFrom,
		rayTo,
		rayCallback.m_rayDirectionInverse,
		rayCallback.m_signs,
		rayCallback.m_lambda_max,
		aabbMin,
		aabbMax,
		callback);

	m_sets[1].rayTestInternal(	m_sets[1].m_root,
		rayFrom,
		rayTo,
		rayCallback.m_rayDirectionInverse,
		rayCallback.m_signs,
		rayCallback.m_lambda_max,
		aabbMin,
		aabbMax,
		callback);

}


//
void							D_btDbvtBroadphase::setAabb(		D_btBroadphaseProxy* absproxy,
														  const D_btVector3& aabbMin,
														  const D_btVector3& aabbMax,
														  D_btDispatcher* /*dispatcher*/)
{
	D_btDbvtProxy*						proxy=(D_btDbvtProxy*)absproxy;
	D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)	aabb=D_btDbvtVolume::FromMM(aabbMin,aabbMax);
#if D_DBVT_BP_PREVENTFALSEUPDATE
	if(NotEqual(aabb,proxy->leaf->volume))
#endif
	{
		bool	docollide=false;
		if(proxy->stage==STAGECOUNT)
		{/* fixed -> dynamic set	*/ 
			m_sets[1].remove(proxy->leaf);
			proxy->leaf=m_sets[0].insert(aabb,proxy);
			docollide=true;
		}
		else
		{/* dynamic set				*/ 
			++m_updates_call;
			if(Intersect(proxy->leaf->volume,aabb))
			{/* Moving				*/ 

				const D_btVector3	delta=aabbMin-proxy->m_aabbMin;
				D_btVector3		velocity(((proxy->m_aabbMax-proxy->m_aabbMin)/2)*m_prediction);
				if(delta[0]<0) velocity[0]=-velocity[0];
				if(delta[1]<0) velocity[1]=-velocity[1];
				if(delta[2]<0) velocity[2]=-velocity[2];
				if	(
#ifdef D_DBVT_BP_MARGIN				
					m_sets[0].update(proxy->leaf,aabb,velocity,D_DBVT_BP_MARGIN)
#else
					m_sets[0].update(proxy->leaf,aabb,velocity)
#endif
					)
				{
					++m_updates_done;
					docollide=true;
				}
			}
			else
			{/* Teleporting			*/ 
				m_sets[0].update(proxy->leaf,aabb);
				++m_updates_done;
				docollide=true;
			}	
		}
		listremove(proxy,m_stageRoots[proxy->stage]);
		proxy->m_aabbMin = aabbMin;
		proxy->m_aabbMax = aabbMax;
		proxy->stage	=	m_stageCurrent;
		listappend(proxy,m_stageRoots[m_stageCurrent]);
		if(docollide)
		{
			m_needcleanup=true;
			if(!m_deferedcollide)
			{
				D_btDbvtTreeCollider	collider(this);
				m_sets[1].collideTTpersistentStack(m_sets[1].m_root,proxy->leaf,collider);
				m_sets[0].collideTTpersistentStack(m_sets[0].m_root,proxy->leaf,collider);
			}
		}	
	}
}

//
void							D_btDbvtBroadphase::calculateOverlappingPairs(D_btDispatcher* dispatcher)
{
	collide(dispatcher);
#if DBVT_BP_PROFILE
	if(0==(m_pid%DBVT_BP_PROFILING_RATE))
	{	
		printf("fixed(%u) dynamics(%u) pairs(%u)\r\n",m_sets[1].m_leaves,m_sets[0].m_leaves,m_paircache->getNumOverlappingPairs());
		unsigned int	total=m_profiling.m_total;
		if(total<=0) total=1;
		printf("ddcollide: %u%% (%uus)\r\n",(50+m_profiling.m_ddcollide*100)/total,m_profiling.m_ddcollide/DBVT_BP_PROFILING_RATE);
		printf("fdcollide: %u%% (%uus)\r\n",(50+m_profiling.m_fdcollide*100)/total,m_profiling.m_fdcollide/DBVT_BP_PROFILING_RATE);
		printf("cleanup:   %u%% (%uus)\r\n",(50+m_profiling.m_cleanup*100)/total,m_profiling.m_cleanup/DBVT_BP_PROFILING_RATE);
		printf("total:     %uus\r\n",total/DBVT_BP_PROFILING_RATE);
		const unsigned long	sum=m_profiling.m_ddcollide+
			m_profiling.m_fdcollide+
			m_profiling.m_cleanup;
		printf("leaked: %u%% (%uus)\r\n",100-((50+sum*100)/total),(total-sum)/DBVT_BP_PROFILING_RATE);
		printf("job counts: %u%%\r\n",(m_profiling.m_jobcount*100)/((m_sets[0].m_leaves+m_sets[1].m_leaves)*DBVT_BP_PROFILING_RATE));
		clear(m_profiling);
		m_clock.reset();
	}
#endif

	performDeferredRemoval(dispatcher);

}

void D_btDbvtBroadphase::performDeferredRemoval(D_btDispatcher* dispatcher)
{

	if (m_paircache->hasDeferredRemoval())
	{

		D_btBroadphasePairArray&	overlappingPairArray = m_paircache->getOverlappingPairArray();

		//perform a sort, D_to find duplicates D_and D_to sort 'invalid' pairs D_to the end
		overlappingPairArray.quickSort(D_btBroadphasePairSortPredicate());

		int invalidPair = 0;

		
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
				//important D_to perform AABB check that D_is consistent with the broadphase
				D_btDbvtProxy*		pa=(D_btDbvtProxy*)pair.m_pProxy0;
				D_btDbvtProxy*		pb=(D_btDbvtProxy*)pair.m_pProxy1;
				bool hasOverlap = Intersect(pa->leaf->volume,pb->leaf->volume);

				if (hasOverlap)
				{
					needsRemoval = false;
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
				m_paircache->cleanOverlappingPair(pair,dispatcher);

				pair.m_pProxy0 = 0;
				pair.m_pProxy1 = 0;
				invalidPair++;
			} 
			
		}

		//perform a sort, D_to sort 'invalid' pairs D_to the end
		overlappingPairArray.quickSort(D_btBroadphasePairSortPredicate());
		overlappingPairArray.resize(overlappingPairArray.size() - invalidPair);
	}
}

//
void							D_btDbvtBroadphase::collide(D_btDispatcher* dispatcher)
{
	/*printf("---------------------------------------------------------\n");
	printf("m_sets[0].m_leaves=%d\n",m_sets[0].m_leaves);
	printf("m_sets[1].m_leaves=%d\n",m_sets[1].m_leaves);
	printf("numPairs = %d\n",getOverlappingPairCache()->getNumOverlappingPairs());
	{
		int i;
		for (i=0;i<getOverlappingPairCache()->getNumOverlappingPairs();i++)
		{
			printf("pair[%d]=(%d,%d),",i,getOverlappingPairCache()->getOverlappingPairArray()[i].m_pProxy0->getUid(),
				getOverlappingPairCache()->getOverlappingPairArray()[i].m_pProxy1->getUid());
		}
		printf("\n");
	}
*/



	SPC(m_profiling.m_total);
	/* optimize				*/ 
	m_sets[0].optimizeIncremental(1+(m_sets[0].m_leaves*m_dupdates)/100);
	if(m_fixedleft)
	{
		const int count=1+(m_sets[1].m_leaves*m_fupdates)/100;
		m_sets[1].optimizeIncremental(1+(m_sets[1].m_leaves*m_fupdates)/100);
		m_fixedleft=D_btMax<int>(0,m_fixedleft-count);
	}
	/* dynamic -> fixed set	*/ 
	m_stageCurrent=(m_stageCurrent+1)%STAGECOUNT;
	D_btDbvtProxy*	current=m_stageRoots[m_stageCurrent];
	if(current)
	{
		D_btDbvtTreeCollider	collider(this);
		do	{
			D_btDbvtProxy*	next=current->links[1];
			listremove(current,m_stageRoots[current->stage]);
			listappend(current,m_stageRoots[STAGECOUNT]);
#if D_DBVT_BP_ACCURATESLEEPING
			m_paircache->removeOverlappingPairsContainingProxy(current,dispatcher);
			collider.proxy=current;
			D_btDbvt::collideTV(m_sets[0].m_root,current->aabb,collider);
			D_btDbvt::collideTV(m_sets[1].m_root,current->aabb,collider);
#endif
			m_sets[0].remove(current->leaf);
			D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)	curAabb=D_btDbvtVolume::FromMM(current->m_aabbMin,current->m_aabbMax);
			current->leaf	=	m_sets[1].insert(curAabb,current);
			current->stage	=	STAGECOUNT;	
			current			=	next;
		} while(current);
		m_fixedleft=m_sets[1].m_leaves;
		m_needcleanup=true;
	}
	/* collide dynamics		*/ 
	{
		D_btDbvtTreeCollider	collider(this);
		if(m_deferedcollide)
		{
			SPC(m_profiling.m_fdcollide);
			m_sets[0].collideTTpersistentStack(m_sets[0].m_root,m_sets[1].m_root,collider);
		}
		if(m_deferedcollide)
		{
			SPC(m_profiling.m_ddcollide);
			m_sets[0].collideTTpersistentStack(m_sets[0].m_root,m_sets[0].m_root,collider);
		}
	}
	/* clean up				*/ 
	if(m_needcleanup)
	{
		SPC(m_profiling.m_cleanup);
		D_btBroadphasePairArray&	pairs=m_paircache->getOverlappingPairArray();
		if(pairs.size()>0)
		{

			int			ni=D_btMin(pairs.size(),D_btMax<int>(m_newpairs,(pairs.size()*m_cupdates)/100));
			for(int i=0;i<ni;++i)
			{
				D_btBroadphasePair&	p=pairs[(m_cid+i)%pairs.size()];
				D_btDbvtProxy*		pa=(D_btDbvtProxy*)p.m_pProxy0;
				D_btDbvtProxy*		pb=(D_btDbvtProxy*)p.m_pProxy1;
				if(!Intersect(pa->leaf->volume,pb->leaf->volume))
				{
#if DBVT_BP_SORTPAIRS
					if(pa->m_uniqueId>pb->m_uniqueId) 
						D_btSwap(pa,pb);
#endif
					m_paircache->removeOverlappingPair(pa,pb,dispatcher);
					--ni;--i;
				}
			}
			if(pairs.size()>0) m_cid=(m_cid+ni)%pairs.size(); else m_cid=0;
		}
	}
	++m_pid;
	m_newpairs=1;
	m_needcleanup=false;
	if(m_updates_call>0)
	{ m_updates_ratio=m_updates_done/(D_btScalar)m_updates_call; }
	else
	{ m_updates_ratio=0; }
	m_updates_done/=2;
	m_updates_call/=2;
}

//
void							D_btDbvtBroadphase::optimize()
{
	m_sets[0].optimizeTopDown();
	m_sets[1].optimizeTopDown();
}

//
D_btOverlappingPairCache*			D_btDbvtBroadphase::getOverlappingPairCache()
{
	return(m_paircache);
}

//
const D_btOverlappingPairCache*	D_btDbvtBroadphase::getOverlappingPairCache() const
{
	return(m_paircache);
}

//
void							D_btDbvtBroadphase::getBroadphaseAabb(D_btVector3& aabbMin,D_btVector3& aabbMax) const
{

	D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)	bounds;

	if(!m_sets[0].empty())
		if(!m_sets[1].empty())	Merge(	m_sets[0].m_root->volume,
			m_sets[1].m_root->volume,bounds);
		else
			bounds=m_sets[0].m_root->volume;
	else if(!m_sets[1].empty())	bounds=m_sets[1].m_root->volume;
	else
		bounds=D_btDbvtVolume::FromCR(D_btVector3(0,0,0),0);
	aabbMin=bounds.Mins();
	aabbMax=bounds.Maxs();
}

void D_btDbvtBroadphase::resetPool(D_btDispatcher* dispatcher)
{
	
	int totalObjects = m_sets[0].m_leaves + m_sets[1].m_leaves;
	if (!totalObjects)
	{
		//reset internal dynamic tree data structures
		m_sets[0].clear();
		m_sets[1].clear();
		
		m_deferedcollide	=	false;
		m_needcleanup		=	true;
		m_stageCurrent		=	0;
		m_fixedleft			=	0;
		m_fupdates			=	1;
		m_dupdates			=	0;
		m_cupdates			=	10;
		m_newpairs			=	1;
		m_updates_call		=	0;
		m_updates_done		=	0;
		m_updates_ratio		=	0;
		
		m_gid				=	0;
		m_pid				=	0;
		m_cid				=	0;
		for(int i=0;i<=STAGECOUNT;++i)
		{
			m_stageRoots[i]=0;
		}
	}
}

//
void							D_btDbvtBroadphase::printStats()
{}

//
#if D_DBVT_BP_ENABLE_BENCHMARK

struct	D_btBroadphaseBenchmark
{
	struct	Experiment
	{
		const char*			D_name;
		int					object_count;
		int					update_count;
		int					spawn_count;
		int					iterations;
		D_btScalar			speed;
		D_btScalar			amplitude;
	};
	struct	Object
	{
		D_btVector3			center;
		D_btVector3			extents;
		D_btBroadphaseProxy*	proxy;
		D_btScalar			time;
		void				update(D_btScalar speed,D_btScalar amplitude,D_btBroadphaseInterface* pbi)
		{
			time		+=	speed;
			center[0]	=	D_btCos(time*(D_btScalar)2.17)*amplitude+
				D_btSin(time)*amplitude/2;
			center[1]	=	D_btCos(time*(D_btScalar)1.38)*amplitude+
				D_btSin(time)*amplitude;
			center[2]	=	D_btSin(time*(D_btScalar)0.777)*amplitude;
			pbi->setAabb(proxy,center-extents,center+extents,0);
		}
	};
	static int		UnsignedRand(int range=RAND_MAX-1)	{ return(rand()%(range+1)); }
	static D_btScalar	UnitRand()							{ return(UnsignedRand(16384)/(D_btScalar)16384); }
	static void		OutputTime(const char* D_name,D_btClock& c,unsigned count=0)
	{
		const unsigned long	us=c.getTimeMicroseconds();
		const unsigned long	ms=(us+500)/1000;
		const D_btScalar		sec=us/(D_btScalar)(1000*1000);
		if(count>0)
			printf("%s : %u us (%u ms), %.2f/s\r\n",D_name,us,ms,count/sec);
		else
			printf("%s : %u us (%u ms)\r\n",D_name,us,ms);
	}
};

void							D_btDbvtBroadphase::benchmark(D_btBroadphaseInterface* pbi)
{
	static const D_btBroadphaseBenchmark::Experiment		experiments[]=
	{
		{"1024o.10%",1024,10,0,8192,(D_btScalar)0.005,(D_btScalar)100},
		/*{"4096o.10%",4096,10,0,8192,(D_btScalar)0.005,(D_btScalar)100},
		{"8192o.10%",8192,10,0,8192,(D_btScalar)0.005,(D_btScalar)100},*/
	};
	static const int										nexperiments=sizeof(experiments)/sizeof(experiments[0]);
	D_btAlignedObjectArray<D_btBroadphaseBenchmark::Object*>	objects;
	D_btClock													wallclock;
	/* Begin			*/ 
	for(int iexp=0;iexp<nexperiments;++iexp)
	{
		const D_btBroadphaseBenchmark::Experiment&	experiment=experiments[iexp];
		const int									object_count=experiment.object_count;
		const int									update_count=(object_count*experiment.update_count)/100;
		const int									spawn_count=(object_count*experiment.spawn_count)/100;
		const D_btScalar								speed=experiment.speed;	
		const D_btScalar								amplitude=experiment.amplitude;
		printf("Experiment #%u '%s':\r\n",iexp,experiment.D_name);
		printf("\D_tObjects: %u\r\n",object_count);
		printf("\D_tUpdate: %u\r\n",update_count);
		printf("\D_tSpawn: %u\r\n",spawn_count);
		printf("\D_tSpeed: %f\r\n",speed);
		printf("\D_tAmplitude: %f\r\n",amplitude);
		srand(180673);
		/* Create objects	*/ 
		wallclock.reset();
		objects.reserve(object_count);
		for(int i=0;i<object_count;++i)
		{
			D_btBroadphaseBenchmark::Object*	po=new D_btBroadphaseBenchmark::Object();
			po->center[0]=D_btBroadphaseBenchmark::UnitRand()*50;
			po->center[1]=D_btBroadphaseBenchmark::UnitRand()*50;
			po->center[2]=D_btBroadphaseBenchmark::UnitRand()*50;
			po->extents[0]=D_btBroadphaseBenchmark::UnitRand()*2+2;
			po->extents[1]=D_btBroadphaseBenchmark::UnitRand()*2+2;
			po->extents[2]=D_btBroadphaseBenchmark::UnitRand()*2+2;
			po->time=D_btBroadphaseBenchmark::UnitRand()*2000;
			po->proxy=pbi->createProxy(po->center-po->extents,po->center+po->extents,0,po,1,1,0,0);
			objects.push_back(po);
		}
		D_btBroadphaseBenchmark::OutputTime("\D_tInitialization",wallclock);
		/* First update		*/ 
		wallclock.reset();
		for(int i=0;i<objects.size();++i)
		{
			objects[i]->update(speed,amplitude,pbi);
		}
		D_btBroadphaseBenchmark::OutputTime("\D_tFirst update",wallclock);
		/* Updates			*/ 
		wallclock.reset();
		for(int i=0;i<experiment.iterations;++i)
		{
			for(int j=0;j<update_count;++j)
			{				
				objects[j]->update(speed,amplitude,pbi);
			}
			pbi->calculateOverlappingPairs(0);
		}
		D_btBroadphaseBenchmark::OutputTime("\D_tUpdate",wallclock,experiment.iterations);
		/* Clean up			*/ 
		wallclock.reset();
		for(int i=0;i<objects.size();++i)
		{
			pbi->destroyProxy(objects[i]->proxy,0);
			delete objects[i];
		}
		objects.resize(0);
		D_btBroadphaseBenchmark::OutputTime("\D_tRelease",wallclock);
	}

}
#else
void							D_btDbvtBroadphase::benchmark(D_btBroadphaseInterface*)
{}
#endif

#if DBVT_BP_PROFILE
#undef	SPC
#endif

