
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


#ifndef SPU_CONVEX_PENETRATION_DEPTH_H
#define SPU_CONVEX_PENETRATION_DEPTH_H



class D_btStackAlloc;
class D_btIDebugDraw;
#include "BulletCollision/NarrowphaseCollision/btConvexPenetrationDepthSolver.h"

#include <LinearMath/D_btTransform.h>


///ConvexPenetrationDepthSolver provides an interface for penetration depth calculation.
class D_SpuConvexPenetrationDepthSolver : public D_btConvexPenetrationDepthSolver
{
public:	
	
	virtual ~D_SpuConvexPenetrationDepthSolver() {};
	virtual bool calcPenDepth( SpuVoronoiSimplexSolver& simplexSolver,
	        void* convexA,void* convexB,int shapeTypeA, int shapeTypeB, float marginA, float marginB,
            D_btTransform& transA,const D_btTransform& transB,
			D_btVector3& v, D_btVector3& pa, D_btVector3& pb,
			class D_btIDebugDraw* debugDraw,D_btStackAlloc* stackAlloc,
			struct D_SpuConvexPolyhedronVertexData* convexVertexDataA,
			struct D_SpuConvexPolyhedronVertexData* convexVertexDataB
			) const = 0;


};



#endif //SPU_CONVEX_PENETRATION_DEPTH_H

