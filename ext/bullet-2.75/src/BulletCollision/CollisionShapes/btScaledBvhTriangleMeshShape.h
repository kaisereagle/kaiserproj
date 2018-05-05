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

#ifndef SCALED_BVH_TRIANGLE_MESH_SHAPE_H
#define SCALED_BVH_TRIANGLE_MESH_SHAPE_H

#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"


///The D_btScaledBvhTriangleMeshShape D_allows D_to instance a scaled version of an existing D_btBvhTriangleMeshShape.
///Note that each D_btBvhTriangleMeshShape still D_can have its own local scaling, independent from this D_btScaledBvhTriangleMeshShape 'localScaling'
D_ATTRIBUTE_ALIGNED16(class) D_btScaledBvhTriangleMeshShape : public D_btConcaveShape
{
	
	
	D_btVector3	m_localScaling;

	D_btBvhTriangleMeshShape*	m_bvhTriMeshShape;

public:


	D_btScaledBvhTriangleMeshShape(D_btBvhTriangleMeshShape* childShape,const D_btVector3& localScaling);

	virtual ~D_btScaledBvhTriangleMeshShape();


	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;
	virtual void	setLocalScaling(const D_btVector3& scaling);
	virtual const D_btVector3& getLocalScaling() const;
	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	virtual void	processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;

	D_btBvhTriangleMeshShape*	getChildShape()
	{
		return m_bvhTriMeshShape;
	}

	const D_btBvhTriangleMeshShape*	getChildShape() const
	{
		return m_bvhTriMeshShape;
	}

	//debugging
	virtual const char*	getName()const {return "SCALEDBVHTRIANGLEMESH";}

};

#endif //BVH_TRIANGLE_MESH_SHAPE_H
