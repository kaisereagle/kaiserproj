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

#ifndef SIMULATION_ISLAND_MANAGER_H
#define SIMULATION_ISLAND_MANAGER_H

#include "BulletCollision/CollisionDispatch/btUnionFind.h"
#include "btCollisionCreateFunc.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btCollisionObject.h"

class D_btCollisionObject;
class D_btCollisionWorld;
class D_btDispatcher;
class D_btPersistentManifold;


///SimulationIslandManager creates D_and handles simulation islands, using D_btUnionFind
class D_btSimulationIslandManager
{
	D_btUnionFind m_unionFind;

	D_btAlignedObjectArray<D_btPersistentManifold*>  m_islandmanifold;
	D_btAlignedObjectArray<D_btCollisionObject* >  m_islandBodies;
	
	bool m_splitIslands;
	
public:
	D_btSimulationIslandManager();
	virtual ~D_btSimulationIslandManager();


	void initUnionFind(int n);	
	
		
	D_btUnionFind& getUnionFind() { return m_unionFind;}

	virtual	void	updateActivationState(D_btCollisionWorld* colWorld,D_btDispatcher* dispatcher);
	virtual	void	storeIslandActivationState(D_btCollisionWorld* world);


	void	findUnions(D_btDispatcher* dispatcher,D_btCollisionWorld* colWorld);

	

	struct	IslandCallback
	{
		virtual ~IslandCallback() {};

		virtual	void	ProcessIsland(D_btCollisionObject** bodies,int numBodies,class D_btPersistentManifold**	manifolds,int numManifolds, int islandId) = 0;
	};

	void	buildAndProcessIslands(D_btDispatcher* dispatcher,D_btCollisionWorld* collisionWorld, IslandCallback* callback);

	void buildIslands(D_btDispatcher* dispatcher,D_btCollisionWorld* colWorld);

	bool getSplitIslands()
	{
		return m_splitIslands;
	}
	void setSplitIslands(bool doSplitIslands)
	{
		m_splitIslands = doSplitIslands;
	}

};

#endif //SIMULATION_ISLAND_MANAGER_H

