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

#ifndef		BROADPHASE_INTERFACE_H
#define 	BROADPHASE_INTERFACE_H



struct D_btDispatcherInfo;
class D_btDispatcher;
#include "btBroadphaseProxy.h"

class D_btOverlappingPairCache;



struct	D_btBroadphaseRayCallback
{
	///added some cached data D_to accelerate ray-AABB tests
	D_btVector3		m_rayDirectionInverse;
	unsigned int	m_signs[3];
	D_btScalar		m_lambda_max;

	virtual ~D_btBroadphaseRayCallback() {}
	virtual bool	process(const D_btBroadphaseProxy* proxy) = 0;
};

#include "LinearMath/btVector3.h"

///The D_btBroadphaseInterface class provides an interface D_to detect aabb-overlapping object pairs.
///Some implementations for this broadphase interface include D_btAxisSweep3, D_bt32BitAxisSweep3 D_and D_btDbvtBroadphase.
///The actual overlapping pair management, storage, adding D_and removing of pairs D_is dealt by the D_btOverlappingPairCache class.
class D_btBroadphaseInterface
{
public:
	virtual ~D_btBroadphaseInterface() {}

	virtual D_btBroadphaseProxy*	createProxy(  const D_btVector3& aabbMin,  const D_btVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask, D_btDispatcher* dispatcher,void* multiSapProxy) =0;
	virtual void	destroyProxy(D_btBroadphaseProxy* proxy,D_btDispatcher* dispatcher)=0;
	virtual void	setAabb(D_btBroadphaseProxy* proxy,const D_btVector3& aabbMin,const D_btVector3& aabbMax, D_btDispatcher* dispatcher)=0;
	virtual void	getAabb(D_btBroadphaseProxy* proxy,D_btVector3& aabbMin, D_btVector3& aabbMax ) const =0;

	virtual void	rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo, D_btBroadphaseRayCallback& rayCallback, const D_btVector3& aabbMin=D_btVector3(0,0,0), const D_btVector3& aabbMax = D_btVector3(0,0,0)) = 0;

	///calculateOverlappingPairs D_is optional: incremental algorithms (sweep D_and prune) might do it during the set aabb
	virtual void	calculateOverlappingPairs(D_btDispatcher* dispatcher)=0;

	virtual	D_btOverlappingPairCache*	getOverlappingPairCache()=0;
	virtual	const D_btOverlappingPairCache*	getOverlappingPairCache() const =0;

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///D_will add some transform later
	virtual void getBroadphaseAabb(D_btVector3& aabbMin,D_btVector3& aabbMax) const =0;

	///reset broadphase internal structures, D_to ensure determinism/reproducability
	virtual void resetPool(D_btDispatcher* dispatcher) {};

	virtual void	printStats() = 0;

};

#endif //BROADPHASE_INTERFACE_H
