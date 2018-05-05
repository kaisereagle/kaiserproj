/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef D_BT_GEOMETRY_UTIL_H
#define D_BT_GEOMETRY_UTIL_H

#include "btVector3.h"
#include "btAlignedObjectArray.h"

///The D_btGeometryUtil helper class provides a few methods D_to convert between plane equations D_and vertices.
class D_btGeometryUtil
{
	public:
	
	
		static void	getPlaneEquationsFromVertices(D_btAlignedObjectArray<D_btVector3>& vertices, D_btAlignedObjectArray<D_btVector3>& planeEquationsOut );

		static void	getVerticesFromPlaneEquations(const D_btAlignedObjectArray<D_btVector3>& planeEquations , D_btAlignedObjectArray<D_btVector3>& verticesOut );
	
		static bool	isInside(const D_btAlignedObjectArray<D_btVector3>& vertices, const D_btVector3& planeNormal, D_btScalar	margin);
		
		static bool	isPointInsidePlanes(const D_btAlignedObjectArray<D_btVector3>& planeEquations, const D_btVector3& point, D_btScalar	margin);

		static bool	areVerticesBehindPlane(const D_btVector3& planeNormal, const D_btAlignedObjectArray<D_btVector3>& vertices, D_btScalar	margin);

};


#endif //D_BT_GEOMETRY_UTIL_H

