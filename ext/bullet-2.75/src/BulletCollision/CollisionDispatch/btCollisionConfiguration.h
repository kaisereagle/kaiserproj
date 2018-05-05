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

#ifndef D_BT_COLLISION_CONFIGURATION
#define D_BT_COLLISION_CONFIGURATION
struct D_btCollisionAlgorithmCreateFunc;

class D_btStackAlloc;
class D_btPoolAllocator;

///D_btCollisionConfiguration D_allows D_to configure Bullet collision detection
///stack allocator size, default collision algorithms D_and persistent manifold pool size
///@todo: describe the meaning
class	D_btCollisionConfiguration
{

public:

	virtual ~D_btCollisionConfiguration()
	{
	}

	///memory pools
	virtual D_btPoolAllocator* getPersistentManifoldPool() = 0;

	virtual D_btPoolAllocator* getCollisionAlgorithmPool() = 0;

	virtual D_btStackAlloc*	getStackAllocator() = 0;

	virtual D_btCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1) =0;

};

#endif //D_BT_COLLISION_CONFIGURATION

