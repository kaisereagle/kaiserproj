/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _SPU_PREFERRED_PENETRATION_DIRECTIONS_H
#define _SPU_PREFERRED_PENETRATION_DIRECTIONS_H


#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"

int		spuGetNumPreferredPenetrationDirections(int shapeType, void* shape)
{
	switch (shapeType)
    {
		case D_TRIANGLE_SHAPE_PROXYTYPE:
		{
			return 2;
			//D_spu_printf("2\n");
			break;
		}
		default:
			{
#if __ASSERT
        D_spu_printf("spuGetNumPreferredPenetrationDirections() - Unsupported bound type: %d.\n", shapeType);
#endif // __ASSERT
			}
	}

	return 0;	
}	

void	spuGetPreferredPenetrationDirection(int shapeType, void* shape, int index, D_btVector3& penetrationVector)
{


	switch (shapeType)
    {
		case D_TRIANGLE_SHAPE_PROXYTYPE:
		{
			D_btVector3* vertices = (D_btVector3*)shape;
			///calcNormal
			penetrationVector = (vertices[1]-vertices[0]).cross(vertices[2]-vertices[0]);
			penetrationVector.normalize();
			if (index)
				penetrationVector *= D_btScalar(-1.);
			break;
		}
		default:
			{
					
#if __ASSERT
        D_spu_printf("spuGetNumPreferredPenetrationDirections() - Unsupported bound type: %d.\n", shapeType);
#endif // __ASSERT
			}
	}
		
}

#endif //_SPU_PREFERRED_PENETRATION_DIRECTIONS_H
