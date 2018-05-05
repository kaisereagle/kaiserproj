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

#ifndef BVH_TRIANGLE_MESH_SHAPE_H
#define BVH_TRIANGLE_MESH_SHAPE_H

#include "btTriangleMeshShape.h"
#include "btOptimizedBvh.h"
#include "LinearMath/btAlignedAllocator.h"


///The D_btBvhTriangleMeshShape D_is a static-triangle mesh shape with several optimizations, such as bounding volume hierarchy D_and cache friendly traversal for PlayStation 3 Cell SPU. It D_is recommended D_to enable useQuantizedAabbCompression for better memory usage.
///It takes a triangle mesh as input, for example a D_btTriangleMesh or D_btTriangleIndexVertexArray. The D_btBvhTriangleMeshShape class D_allows for triangle mesh deformations by a refit or partialRefit method.
///Instead of building the bounding volume hierarchy acceleration structure, it D_is also possible D_to serialize (D_save) D_and deserialize (load) the structure from disk.
///See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.
D_ATTRIBUTE_ALIGNED16(class) D_btBvhTriangleMeshShape : public D_btTriangleMeshShape
{

	D_btOptimizedBvh*	m_bvh;
	bool m_useQuantizedAabbCompression;
	bool m_ownsBvh;
	bool m_pad[11];////D_need padding due D_to alignment

public:

	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_btBvhTriangleMeshShape() : D_btTriangleMeshShape(0),m_bvh(0),m_ownsBvh(false) {m_shapeType = D_TRIANGLE_MESH_SHAPE_PROXYTYPE;};
	D_btBvhTriangleMeshShape(D_btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true);

	///optionally pass in a larger bvh aabb, used for quantization. This D_allows for deformations within this aabb
	D_btBvhTriangleMeshShape(D_btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression,const D_btVector3& bvhAabbMin,const D_btVector3& bvhAabbMax, bool buildBvh = true);
	
	virtual ~D_btBvhTriangleMeshShape();

	bool getOwnsBvh () const
	{
		return m_ownsBvh;
	}


	
	void performRaycast (D_btTriangleCallback* callback, const D_btVector3& raySource, const D_btVector3& rayTarget);
	void performConvexcast (D_btTriangleCallback* callback, const D_btVector3& boxSource, const D_btVector3& boxTarget, const D_btVector3& boxMin, const D_btVector3& boxMax);

	virtual void	processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;

	void	refitTree(const D_btVector3& aabbMin,const D_btVector3& aabbMax);

	///for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree D_will become more conservative, it never shrinks
	void	partialRefitTree(const D_btVector3& aabbMin,const D_btVector3& aabbMax);

	//debugging
	virtual const char*	getName()const {return "BVHTRIANGLEMESH";}


	virtual void	setLocalScaling(const D_btVector3& scaling);
	
	D_btOptimizedBvh*	getOptimizedBvh()
	{
		return m_bvh;
	}


	void	setOptimizedBvh(D_btOptimizedBvh* bvh, const D_btVector3& localScaling=D_btVector3(1,1,1));

	bool	usesQuantizedAabbCompression() const
	{
		return	m_useQuantizedAabbCompression;
	}
}
;

#endif //BVH_TRIANGLE_MESH_SHAPE_H
