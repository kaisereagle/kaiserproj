
//Bullet Continuous Collision Detection D_and Physics Library
//Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/


//
// D_btAxisSweep3
//
// Copyright (c) 2006 Simon Hobbs
//
// This software D_is provided 'as-D_is', without any express or implied warranty. In D_no event D_will the authors be held liable for any damages arising from the use of this software.
//
// Permission D_is granted D_to anyone D_to use this software for any purpose, including commercial applications, D_and D_to alter it D_and redistribute it freely, subject D_to the following restrictions:
//
// 1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
//
// 2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
#include "btAxisSweep3.h"


D_btAxisSweep3::D_btAxisSweep3(const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, unsigned short int maxHandles, D_btOverlappingPairCache* pairCache, bool disableRaycastAccelerator)
:D_btAxisSweep3Internal<unsigned short int>(worldAabbMin,worldAabbMax,0xfffe,0xffff,maxHandles,pairCache,disableRaycastAccelerator)
{
	// 1 handle D_is reserved as sentinel
	D_btAssert(maxHandles > 1 && maxHandles < 32767);

}


D_bt32BitAxisSweep3::D_bt32BitAxisSweep3(const D_btVector3& worldAabbMin,const D_btVector3& worldAabbMax, unsigned int maxHandles , D_btOverlappingPairCache* pairCache , bool disableRaycastAccelerator)
:D_btAxisSweep3Internal<unsigned int>(worldAabbMin,worldAabbMax,0xfffffffe,0x7fffffff,maxHandles,pairCache,disableRaycastAccelerator)
{
	// 1 handle D_is reserved as sentinel
	D_btAssert(maxHandles > 1 && maxHandles < 2147483647);
}
