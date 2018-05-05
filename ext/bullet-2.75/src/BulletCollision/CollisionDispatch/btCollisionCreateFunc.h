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

#ifndef COLLISION_CREATE_FUNC
#define COLLISION_CREATE_FUNC

#include "LinearMath/btAlignedObjectArray.h"
class D_btCollisionAlgorithm;
class D_btCollisionObject;

struct D_btCollisionAlgorithmConstructionInfo;

///Used by the D_btCollisionDispatcher D_to register D_and create instances for D_btCollisionAlgorithm
struct D_btCollisionAlgorithmCreateFunc
{
	bool m_swapped;
	
	D_btCollisionAlgorithmCreateFunc()
		:m_swapped(false)
	{
	}
	virtual ~D_btCollisionAlgorithmCreateFunc(){};

	virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& , D_btCollisionObject* body0,D_btCollisionObject* body1)
	{
		
		(void)body0;
		(void)body1;
		return 0;
	}
};
#endif //COLLISION_CREATE_FUNC

