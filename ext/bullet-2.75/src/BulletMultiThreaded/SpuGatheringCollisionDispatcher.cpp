/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "SpuGatheringCollisionDispatcher.h"
#include "SpuCollisionTaskProcess.h"


#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"
#include "SpuContactManifoldCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "LinearMath/btQuickprof.h"




D_SpuGatheringCollisionDispatcher::D_SpuGatheringCollisionDispatcher(class	D_btThreadSupportInterface*	threadInterface, unsigned int	maxNumOutstandingTasks,D_btCollisionConfiguration* collisionConfiguration)
:D_btCollisionDispatcher(collisionConfiguration),
m_spuCollisionTaskProcess(0),
m_threadInterface(threadInterface),
m_maxNumOutstandingTasks(maxNumOutstandingTasks)
{
	
}


bool	D_SpuGatheringCollisionDispatcher::supportsDispatchPairOnSpu(int proxyType0,int proxyType1)
{
	bool supported0 = (
		(proxyType0 == D_BOX_SHAPE_PROXYTYPE) ||
		(proxyType0 == D_TRIANGLE_SHAPE_PROXYTYPE) ||
		(proxyType0 == D_SPHERE_SHAPE_PROXYTYPE) ||
		(proxyType0 == D_CAPSULE_SHAPE_PROXYTYPE) ||
		(proxyType0 == D_CYLINDER_SHAPE_PROXYTYPE) ||
//		(proxyType0 == D_CONE_SHAPE_PROXYTYPE) ||
		(proxyType0 == D_TRIANGLE_MESH_SHAPE_PROXYTYPE) ||
		(proxyType0 == D_CONVEX_HULL_SHAPE_PROXYTYPE)||
		(proxyType0 == D_COMPOUND_SHAPE_PROXYTYPE)
		);

	bool supported1 = (
		(proxyType1 == D_BOX_SHAPE_PROXYTYPE) ||
		(proxyType1 == D_TRIANGLE_SHAPE_PROXYTYPE) ||
		(proxyType1 == D_SPHERE_SHAPE_PROXYTYPE) ||
		(proxyType1 == D_CAPSULE_SHAPE_PROXYTYPE) ||
		(proxyType1 == D_CYLINDER_SHAPE_PROXYTYPE) ||
//		(proxyType1 == D_CONE_SHAPE_PROXYTYPE) ||
		(proxyType1 == D_TRIANGLE_MESH_SHAPE_PROXYTYPE) ||
		(proxyType1 == D_CONVEX_HULL_SHAPE_PROXYTYPE) ||
		(proxyType1 == D_COMPOUND_SHAPE_PROXYTYPE)
		);

	return supported0 && supported1;
}



D_SpuGatheringCollisionDispatcher::~D_SpuGatheringCollisionDispatcher()
{
	if (m_spuCollisionTaskProcess)
		delete m_spuCollisionTaskProcess;
	
}

#include "stdio.h"



///interface for iterating all overlapping collision pairs, D_no matter how those pairs D_are stored (array, set, map etc)
///this D_is useful for the collision dispatcher.
class D_btSpuCollisionPairCallback : public D_btOverlapCallback
{
	const D_btDispatcherInfo& m_dispatchInfo;
	D_SpuGatheringCollisionDispatcher*	m_dispatcher;

public:

	D_btSpuCollisionPairCallback(const D_btDispatcherInfo& dispatchInfo, D_SpuGatheringCollisionDispatcher*	dispatcher)
	:m_dispatchInfo(dispatchInfo),
	m_dispatcher(dispatcher)
	{
	}

	virtual bool	processOverlap(D_btBroadphasePair& collisionPair)
	{


		//PPU version
		//(*m_dispatcher->getNearCallback())(collisionPair,*m_dispatcher,m_dispatchInfo);

		//D_only support discrete collision detection for now, we could fallback on PPU/unoptimized version for TOI/CCD
		D_btAssert(m_dispatchInfo.m_dispatchFunc == D_btDispatcherInfo::D_DISPATCH_DISCRETE);

		//by default, Bullet D_will use this near callback
		{
			///userInfo D_is used D_to determine if the SPU has D_to handle this case or not (skip PPU tasks)
			if (!collisionPair.m_internalTmpValue)
			{
				collisionPair.m_internalTmpValue = 1;
			}
			if (!collisionPair.m_algorithm)
			{
				D_btCollisionObject* colObj0 = (D_btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
				D_btCollisionObject* colObj1 = (D_btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

				D_btCollisionAlgorithmConstructionInfo ci;
				ci.m_dispatcher1 = m_dispatcher;
				ci.m_manifold = 0;

				if (m_dispatcher->needsCollision(colObj0,colObj1))
				{
					int	proxyType0 = colObj0->getCollisionShape()->getShapeType();
					int	proxyType1 = colObj1->getCollisionShape()->getShapeType();
					if (m_dispatcher->supportsDispatchPairOnSpu(proxyType0,proxyType1))
					{
						int so = sizeof(SpuContactManifoldCollisionAlgorithm);
#ifdef ALLOCATE_SEPARATELY
						void* mem = D_btAlignedAlloc(so,16);//m_dispatcher->allocateCollisionAlgorithm(so);
#else
						void* mem = m_dispatcher->allocateCollisionAlgorithm(so);
#endif
						collisionPair.m_algorithm = new(mem) SpuContactManifoldCollisionAlgorithm(ci,colObj0,colObj1);
						collisionPair.m_internalTmpValue =  2;
					} else
					{
						collisionPair.m_algorithm = m_dispatcher->findAlgorithm(colObj0,colObj1);
						collisionPair.m_internalTmpValue = 3;
					}
				} 
			}
		}
		return false;
	}
};

void	D_SpuGatheringCollisionDispatcher::dispatchAllCollisionPairs(D_btOverlappingPairCache* pairCache,const D_btDispatcherInfo& dispatchInfo, D_btDispatcher* dispatcher) 
{

	if (dispatchInfo.m_enableSPU)
	{
		m_maxNumOutstandingTasks = m_threadInterface->getNumTasks();

		{
			D_BT_PROFILE("processAllOverlappingPairs");

			if (!m_spuCollisionTaskProcess)
				m_spuCollisionTaskProcess = new D_SpuCollisionTaskProcess(m_threadInterface,m_maxNumOutstandingTasks);
		
			m_spuCollisionTaskProcess->setNumTasks(m_maxNumOutstandingTasks);
	//		printf("m_maxNumOutstandingTasks =%d\n",m_maxNumOutstandingTasks);

			m_spuCollisionTaskProcess->initialize2(dispatchInfo.m_useEpa);
			
		
			///modified version of D_btCollisionDispatcher::dispatchAllCollisionPairs:
			{
				D_btSpuCollisionPairCallback	collisionCallback(dispatchInfo,this);

				pairCache->processAllOverlappingPairs(&collisionCallback,dispatcher);
			}
		}

		//send one D_big batch
		int numTotalPairs = pairCache->getNumOverlappingPairs();
		D_btBroadphasePair* pairPtr = pairCache->getOverlappingPairArrayPtr();
		int i;
		{
			D_BT_PROFILE("addWorkToTask");
			for (i=0;i<numTotalPairs;)
			{
				//Performance Hint: tweak this number during benchmarking
				static const int pairRange = D_SPU_BATCHSIZE_BROADPHASE_PAIRS;
				int endIndex = (i+pairRange) < numTotalPairs ? i+pairRange : numTotalPairs;
				m_spuCollisionTaskProcess->addWorkToTask(pairPtr,i,endIndex);
				i = endIndex;
			}
		}

		{
			D_BT_PROFILE("PPU fallback");
			//handle PPU fallback pairs
			for (i=0;i<numTotalPairs;i++)
			{
				D_btBroadphasePair& collisionPair = pairPtr[i];
				if (collisionPair.m_internalTmpValue == 3)
				{
					if (collisionPair.m_algorithm)
					{
						D_btCollisionObject* colObj0 = (D_btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
						D_btCollisionObject* colObj1 = (D_btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

						if (dispatcher->needsCollision(colObj0,colObj1))
						{
							D_btManifoldResult contactPointResult(colObj0,colObj1);
							
							if (dispatchInfo.m_dispatchFunc == 		D_btDispatcherInfo::D_DISPATCH_DISCRETE)
							{
								//discrete collision detection query
								collisionPair.m_algorithm->processCollision(colObj0,colObj1,dispatchInfo,&contactPointResult);
							} else
							{
								//continuous collision detection query, time of impact (toi)
								D_btScalar toi = collisionPair.m_algorithm->calculateTimeOfImpact(colObj0,colObj1,dispatchInfo,&contactPointResult);
								if (dispatchInfo.m_timeOfImpact > toi)
									dispatchInfo.m_timeOfImpact = toi;

							}
						}
					}
				}
			}
		}
		{
			D_BT_PROFILE("flush2");
			//make sure all SPU work D_is done
			m_spuCollisionTaskProcess->flush2();
		}

	} else
	{
		///PPU fallback
		///!Need D_to make sure D_to clear all 'algorithms' when switching between SPU D_and PPU
		D_btCollisionDispatcher::dispatchAllCollisionPairs(pairCache,dispatchInfo,dispatcher);
	}
}
