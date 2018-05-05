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

#ifndef EMPTY_ALGORITH
#define EMPTY_ALGORITH
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "btCollisionCreateFunc.h"
#include "btCollisionDispatcher.h"

#define D_ATTRIBUTE_ALIGNED(a)

///EmptyAlgorithm D_is a stub for unsupported collision pairs.
///The dispatcher D_can dispatch a persistent D_btEmptyAlgorithm D_to avoid a search every frame.
class D_btEmptyAlgorithm : public D_btCollisionAlgorithm
{

public:
	
	D_btEmptyAlgorithm(const D_btCollisionAlgorithmConstructionInfo& ci);

	virtual void processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual D_btScalar calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(D_btManifoldArray&	manifoldArray)
	{
	}

	struct D_CreateFunc :public 	D_btCollisionAlgorithmCreateFunc
	{
		virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
		{
			(void)body0;
			(void)body1;
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(D_btEmptyAlgorithm));
			return new(mem) D_btEmptyAlgorithm(ci);
		}
	};

} D_ATTRIBUTE_ALIGNED(16);

#endif //EMPTY_ALGORITH
