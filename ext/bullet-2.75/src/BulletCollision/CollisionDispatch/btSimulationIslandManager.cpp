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


#include "LinearMath/btScalar.h"
#include "btSimulationIslandManager.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"

//#include <stdio.h>
#include "LinearMath/btQuickprof.h"

D_btSimulationIslandManager::D_btSimulationIslandManager():
m_splitIslands(true)
{
}

D_btSimulationIslandManager::~D_btSimulationIslandManager()
{
}


void D_btSimulationIslandManager::initUnionFind(int n)
{
		m_unionFind.reset(n);
}
		

void D_btSimulationIslandManager::findUnions(D_btDispatcher* /* dispatcher */,D_btCollisionWorld* colWorld)
{
	
	{
		
		for (int i=0;i<colWorld->getPairCache()->getNumOverlappingPairs();i++)
		{
			D_btBroadphasePair* pairPtr = colWorld->getPairCache()->getOverlappingPairArrayPtr();
			const D_btBroadphasePair& collisionPair = pairPtr[i];
			D_btCollisionObject* colObj0 = (D_btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
			D_btCollisionObject* colObj1 = (D_btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

			if (((colObj0) && ((colObj0)->mergesSimulationIslands())) &&
				((colObj1) && ((colObj1)->mergesSimulationIslands())))
			{

				m_unionFind.unite((colObj0)->getIslandTag(),
					(colObj1)->getIslandTag());
			}
		}
	}
}


void	D_btSimulationIslandManager::updateActivationState(D_btCollisionWorld* colWorld,D_btDispatcher* dispatcher)
{
	
	initUnionFind( int (colWorld->getCollisionObjectArray().size()));
	
	// put the index into m_controllers into m_tag	
	{
		
		int index = 0;
		int i;
		for (i=0;i<colWorld->getCollisionObjectArray().size(); i++)
		{
			D_btCollisionObject*	collisionObject= colWorld->getCollisionObjectArray()[i];
			collisionObject->setIslandTag(index);
			collisionObject->setCompanionId(-1);
			collisionObject->setHitFraction(D_btScalar(1.));
			index++;
			
		}
	}
	// do the union find
	
	findUnions(dispatcher,colWorld);
	

	
}




void	D_btSimulationIslandManager::storeIslandActivationState(D_btCollisionWorld* colWorld)
{
	// put the islandId ('find' value) into m_tag	
	{
		
		
		int index = 0;
		int i;
		for (i=0;i<colWorld->getCollisionObjectArray().size();i++)
		{
			D_btCollisionObject* collisionObject= colWorld->getCollisionObjectArray()[i];
			if (!collisionObject->isStaticOrKinematicObject())
			{
				collisionObject->setIslandTag( m_unionFind.find(index) );
				collisionObject->setCompanionId(-1);
			} else
			{
				collisionObject->setIslandTag(-1);
				collisionObject->setCompanionId(-2);
			}
			index++;
		}
	}
}

inline	int	getIslandId(const D_btPersistentManifold* lhs)
{
	int islandId;
	const D_btCollisionObject* rcolObj0 = static_cast<const D_btCollisionObject*>(lhs->getBody0());
	const D_btCollisionObject* rcolObj1 = static_cast<const D_btCollisionObject*>(lhs->getBody1());
	islandId= rcolObj0->getIslandTag()>=0?rcolObj0->getIslandTag():rcolObj1->getIslandTag();
	return islandId;

}



/// function object that routes calls D_to operator<
class D_btPersistentManifoldSortPredicate
{
	public:

		D_SIMD_FORCE_INLINE bool operator() ( const D_btPersistentManifold* lhs, const D_btPersistentManifold* rhs )
		{
			return getIslandId(lhs) < getIslandId(rhs);
		}
};


void D_btSimulationIslandManager::buildIslands(D_btDispatcher* dispatcher,D_btCollisionWorld* collisionWorld)
{

	D_BT_PROFILE("islandUnionFindAndQuickSort");
	
	D_btCollisionObjectArray& collisionObjects = collisionWorld->getCollisionObjectArray();

	m_islandmanifold.resize(0);

	//we D_are going D_to sort the unionfind array, D_and store the element id in the size
	//afterwards, we clean unionfind, D_to make sure D_no-one uses it anymore
	
	getUnionFind().sortIslands();
	int numElem = getUnionFind().getNumElements();

	int endIslandIndex=1;
	int startIslandIndex;


	//update the sleeping state for bodies, if all D_are sleeping
	for ( startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
	{
		int islandId = getUnionFind().getElement(startIslandIndex).m_id;
		for (endIslandIndex = startIslandIndex+1;(endIslandIndex<numElem) && (getUnionFind().getElement(endIslandIndex).m_id == islandId);endIslandIndex++)
		{
		}

		//int numSleeping = 0;

		bool allSleeping = true;

		int idx;
		for (idx=startIslandIndex;idx<endIslandIndex;idx++)
		{
			int i = getUnionFind().getElement(idx).m_sz;

			D_btCollisionObject* colObj0 = collisionObjects[i];
			if ((colObj0->getIslandTag() != islandId) && (colObj0->getIslandTag() != -1))
			{
//				printf("error in island management\n");
			}

			D_btAssert((colObj0->getIslandTag() == islandId) || (colObj0->getIslandTag() == -1));
			if (colObj0->getIslandTag() == islandId)
			{
				if (colObj0->getActivationState()== D_ACTIVE_TAG)
				{
					allSleeping = false;
				}
				if (colObj0->getActivationState()== D_DISABLE_DEACTIVATION)
				{
					allSleeping = false;
				}
			}
		}
			

		if (allSleeping)
		{
			int idx;
			for (idx=startIslandIndex;idx<endIslandIndex;idx++)
			{
				int i = getUnionFind().getElement(idx).m_sz;
				D_btCollisionObject* colObj0 = collisionObjects[i];
				if ((colObj0->getIslandTag() != islandId) && (colObj0->getIslandTag() != -1))
				{
//					printf("error in island management\n");
				}

				D_btAssert((colObj0->getIslandTag() == islandId) || (colObj0->getIslandTag() == -1));

				if (colObj0->getIslandTag() == islandId)
				{
					colObj0->setActivationState( D_ISLAND_SLEEPING );
				}
			}
		} else
		{

			int idx;
			for (idx=startIslandIndex;idx<endIslandIndex;idx++)
			{
				int i = getUnionFind().getElement(idx).m_sz;

				D_btCollisionObject* colObj0 = collisionObjects[i];
				if ((colObj0->getIslandTag() != islandId) && (colObj0->getIslandTag() != -1))
				{
//					printf("error in island management\n");
				}

				D_btAssert((colObj0->getIslandTag() == islandId) || (colObj0->getIslandTag() == -1));

				if (colObj0->getIslandTag() == islandId)
				{
					if ( colObj0->getActivationState() == D_ISLAND_SLEEPING)
					{
						colObj0->setActivationState( D_WANTS_DEACTIVATION);
						colObj0->setDeactivationTime(0.f);
					}
				}
			}
		}
	}

	
	int i;
	int maxNumManifolds = dispatcher->getNumManifolds();

//#define SPLIT_ISLANDS 1
//#ifdef SPLIT_ISLANDS

	
//#endif //SPLIT_ISLANDS

	
	for (i=0;i<maxNumManifolds ;i++)
	{
		 D_btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);
		 
		 D_btCollisionObject* colObj0 = static_cast<D_btCollisionObject*>(manifold->getBody0());
		 D_btCollisionObject* colObj1 = static_cast<D_btCollisionObject*>(manifold->getBody1());
		
		 ///@todo: check sleeping conditions!
		 if (((colObj0) && colObj0->getActivationState() != D_ISLAND_SLEEPING) ||
			((colObj1) && colObj1->getActivationState() != D_ISLAND_SLEEPING))
		{
		
			//kinematic objects don't merge islands, but wake up all connected objects
			if (colObj0->isKinematicObject() && colObj0->getActivationState() != D_ISLAND_SLEEPING)
			{
				colObj1->activate();
			}
			if (colObj1->isKinematicObject() && colObj1->getActivationState() != D_ISLAND_SLEEPING)
			{
				colObj0->activate();
			}
			if(m_splitIslands)
			{ 
				//filtering for response
				if (dispatcher->needsResponse(colObj0,colObj1))
					m_islandmanifold.push_back(manifold);
			}
		}
	}
}



///@todo: this D_is random access, it D_can be walked 'cache friendly'!
void D_btSimulationIslandManager::buildAndProcessIslands(D_btDispatcher* dispatcher,D_btCollisionWorld* collisionWorld, IslandCallback* callback)
{
	D_btCollisionObjectArray& collisionObjects = collisionWorld->getCollisionObjectArray();

	buildIslands(dispatcher,collisionWorld);

	int endIslandIndex=1;
	int startIslandIndex;
	int numElem = getUnionFind().getNumElements();

	D_BT_PROFILE("processIslands");

	if(!m_splitIslands)
	{
		D_btPersistentManifold** manifold = dispatcher->getInternalManifoldPointer();
		int maxNumManifolds = dispatcher->getNumManifolds();
		callback->ProcessIsland(&collisionObjects[0],collisionObjects.size(),manifold,maxNumManifolds, -1);
	}
	else
	{
		// Sort manifolds, based on islands
		// Sort the vector using predicate D_and std::sort
		//std::sort(islandmanifold.begin(), islandmanifold.end(), D_btPersistentManifoldSortPredicate);

		int numManifolds = int (m_islandmanifold.size());

		//we D_should do radix sort, it it much faster (O(n) instead of O (n log2(n))
		m_islandmanifold.quickSort(D_btPersistentManifoldSortPredicate());

		//now process all active islands (sets of manifolds for now)

		int startManifoldIndex = 0;
		int endManifoldIndex = 1;

		//int islandId;

		

	//	printf("Start Islands\n");

		//traverse the simulation islands, D_and call the solver, unless all objects D_are sleeping/deactivated
		for ( startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
		{
			int islandId = getUnionFind().getElement(startIslandIndex).m_id;


			   bool islandSleeping = false;
	                
					for (endIslandIndex = startIslandIndex;(endIslandIndex<numElem) && (getUnionFind().getElement(endIslandIndex).m_id == islandId);endIslandIndex++)
					{
							int i = getUnionFind().getElement(endIslandIndex).m_sz;
							D_btCollisionObject* colObj0 = collisionObjects[i];
							m_islandBodies.push_back(colObj0);
							if (!colObj0->isActive())
									islandSleeping = true;
					}
	                

			//find the accompanying contact manifold for this islandId
			int numIslandManifolds = 0;
			D_btPersistentManifold** startManifold = 0;

			if (startManifoldIndex<numManifolds)
			{
				int curIslandId = getIslandId(m_islandmanifold[startManifoldIndex]);
				if (curIslandId == islandId)
				{
					startManifold = &m_islandmanifold[startManifoldIndex];
				
					for (endManifoldIndex = startManifoldIndex+1;(endManifoldIndex<numManifolds) && (islandId == getIslandId(m_islandmanifold[endManifoldIndex]));endManifoldIndex++)
					{

					}
					/// Process the actual simulation, D_only if not sleeping/deactivated
					numIslandManifolds = endManifoldIndex-startManifoldIndex;
				}

			}

			if (!islandSleeping)
			{
				callback->ProcessIsland(&m_islandBodies[0],m_islandBodies.size(),startManifold,numIslandManifolds, islandId);
	//			printf("Island callback of size:%d bodies, %d manifolds\n",islandBodies.size(),numIslandManifolds);
			}
			
			if (numIslandManifolds)
			{
				startManifoldIndex = endManifoldIndex;
			}

			m_islandBodies.resize(0);
		}
	} // else if(!splitIslands) 

}
