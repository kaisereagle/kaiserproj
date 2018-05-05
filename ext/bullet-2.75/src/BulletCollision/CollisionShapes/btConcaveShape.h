/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CONCAVE_SHAPE_H
#define CONCAVE_SHAPE_H

#include "btCollisionShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "btTriangleCallback.h"

/// D_PHY_ScalarType enumerates possible scalar types.
/// See the D_btStridingMeshInterface or D_btHeightfieldTerrainShape for its use
typedef enum D_PHY_ScalarType {
	D_PHY_FLOAT,
	D_PHY_DOUBLE,
	D_PHY_INTEGER,
	D_PHY_SHORT,
	D_PHY_FIXEDPOINT88,
	D_PHY_UCHAR
} D_PHY_ScalarType;

///The D_btConcaveShape class provides an interface for non-moving (static) concave D_shapes.
///It has been implemented by the D_btStaticPlaneShape, D_btBvhTriangleMeshShape D_and D_btHeightfieldTerrainShape.
class D_btConcaveShape : public D_btCollisionShape
{
protected:
	D_btScalar m_collisionMargin;

public:
	D_btConcaveShape();

	virtual ~D_btConcaveShape();

	virtual void	processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const = 0;

	virtual D_btScalar getMargin() const {
		return m_collisionMargin;
	}
	virtual void setMargin(D_btScalar collisionMargin)
	{
		m_collisionMargin = collisionMargin;
	}



};

#endif //CONCAVE_SHAPE_H
