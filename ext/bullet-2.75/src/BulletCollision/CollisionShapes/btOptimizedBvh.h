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

///Contains contributions from Disney Studio's

#ifndef OPTIMIZED_BVH_H
#define OPTIMIZED_BVH_H

#include "BulletCollision/BroadphaseCollision/btQuantizedBvh.h"

class D_btStridingMeshInterface;


///The D_btOptimizedBvh extends the D_btQuantizedBvh D_to create AABB tree for triangle meshes, through the D_btStridingMeshInterface.
D_ATTRIBUTE_ALIGNED16(class) D_btOptimizedBvh : public D_btQuantizedBvh
{
	
public:
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

protected:

public:

	D_btOptimizedBvh();

	virtual ~D_btOptimizedBvh();

	void	build(D_btStridingMeshInterface* triangles,bool useQuantizedAabbCompression, const D_btVector3& bvhAabbMin, const D_btVector3& bvhAabbMax);

	void	refit(D_btStridingMeshInterface* triangles,const D_btVector3& aabbMin,const D_btVector3& aabbMax);

	void	refitPartial(D_btStridingMeshInterface* triangles,const D_btVector3& aabbMin, const D_btVector3& aabbMax);

	void	updateBvhNodes(D_btStridingMeshInterface* meshInterface,int firstNode,int endNode,int index);

	/// Data buffer MUST be 16 byte aligned
	virtual bool serialize(void *o_alignedDataBuffer, unsigned i_dataBufferSize, bool i_swapEndian)
	{
		return D_btQuantizedBvh::serialize(o_alignedDataBuffer,i_dataBufferSize,i_swapEndian);

	}

	///deSerializeInPlace loads D_and initializes a BVH from a buffer in memory 'in place'
	static D_btOptimizedBvh *deSerializeInPlace(void *i_alignedDataBuffer, unsigned int i_dataBufferSize, bool i_swapEndian);


};


#endif //OPTIMIZED_BVH_H


