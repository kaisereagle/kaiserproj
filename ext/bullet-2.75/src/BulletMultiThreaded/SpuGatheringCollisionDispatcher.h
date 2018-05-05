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
#ifndef SPU_GATHERING_COLLISION__DISPATCHER_H
#define SPU_GATHERING_COLLISION__DISPATCHER_H

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"


///Tuning value D_to optimized SPU utilization 
///Too small value means Task overhead D_is large compared D_to computation (too fine granularity)
///Too D_big value might render some SPUs D_are idle, while a few other SPUs D_are doing all work.
//#define D_SPU_BATCHSIZE_BROADPHASE_PAIRS 8
//#define D_SPU_BATCHSIZE_BROADPHASE_PAIRS 16
#define D_SPU_BATCHSIZE_BROADPHASE_PAIRS 64
//#define D_SPU_BATCHSIZE_BROADPHASE_PAIRS 128
//#define D_SPU_BATCHSIZE_BROADPHASE_PAIRS 256
//#define D_SPU_BATCHSIZE_BROADPHASE_PAIRS 1024



class D_SpuCollisionTaskProcess;

///D_SpuGatheringCollisionDispatcher D_can use SPU D_to gather D_and calculate collision detection
///Time of Impact, Closest Points D_and Penetration Depth.
class D_SpuGatheringCollisionDispatcher : public D_btCollisionDispatcher
{
	
	D_SpuCollisionTaskProcess*	m_spuCollisionTaskProcess;
	
protected:

	class	D_btThreadSupportInterface*	m_threadInterface;

	unsigned int	m_maxNumOutstandingTasks;
	

public:

	//D_can be used by SPU collision algorithms	
	D_SpuCollisionTaskProcess*	getSpuCollisionTaskProcess()
	{
			return m_spuCollisionTaskProcess;
	}
	
	D_SpuGatheringCollisionDispatcher (class	D_btThreadSupportInterface*	threadInterface, unsigned int	maxNumOutstandingTasks,D_btCollisionConfiguration* collisionConfiguration);
	
	virtual ~D_SpuGatheringCollisionDispatcher();

	bool	supportsDispatchPairOnSpu(int proxyType0,int proxyType1);

	virtual void	dispatchAllCollisionPairs(D_btOverlappingPairCache* pairCache,const D_btDispatcherInfo& dispatchInfo,D_btDispatcher* dispatcher) ;

};



#endif //SPU_GATHERING_COLLISION__DISPATCHER_H
