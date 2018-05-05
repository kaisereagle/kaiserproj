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
#ifndef __SPU_COLLISION_SHAPES_H
#define __SPU_COLLISION_SHAPES_H

#include "../SpuDoubleBuffer.h"

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"

#include "BulletCollision/CollisionShapes/btOptimizedBvh.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/CollisionShapes/btCapsuleShape.h"

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"

#define D_MAX_NUM_SPU_CONVEX_POINTS 128

D_ATTRIBUTE_ALIGNED16(struct)	D_SpuConvexPolyhedronVertexData
{
	void*	D_gSpuConvexShapePtr;
	D_btVector3* D_gConvexPoints;
	int D_gNumConvexPoints;
	int unused;
	D_ATTRIBUTE_ALIGNED16(D_btVector3 g_convexPointBuffer[D_MAX_NUM_SPU_CONVEX_POINTS]);
};

#define D_MAX_SHAPE_SIZE 256

D_ATTRIBUTE_ALIGNED16(struct) D_CollisionShape_LocalStoreMemory
{
	D_ATTRIBUTE_ALIGNED16(char collisionShape[D_MAX_SHAPE_SIZE]);
};

D_ATTRIBUTE_ALIGNED16(struct) D_CompoundShape_LocalStoreMemory
{
	// Compound data
#define D_MAX_SPU_COMPOUND_SUBSHAPES 16
	D_ATTRIBUTE_ALIGNED16(D_btCompoundShapeChild D_gSubshapes[D_MAX_SPU_COMPOUND_SUBSHAPES]);
	D_ATTRIBUTE_ALIGNED16(char D_gSubshapeShape[D_MAX_SPU_COMPOUND_SUBSHAPES][D_MAX_SHAPE_SIZE]);
};

D_ATTRIBUTE_ALIGNED16(struct) D_bvhMeshShape_LocalStoreMemory
{
	//D_ATTRIBUTE_ALIGNED16(D_btOptimizedBvh	D_gOptimizedBvh);
	D_ATTRIBUTE_ALIGNED16(char D_gOptimizedBvh[sizeof(D_btOptimizedBvh)+16]);
	D_btOptimizedBvh*	getOptimizedBvh()
	{
		return (D_btOptimizedBvh*) D_gOptimizedBvh;
	}

	D_ATTRIBUTE_ALIGNED16(D_btTriangleIndexVertexArray	D_gTriangleMeshInterfaceStorage);
	D_btTriangleIndexVertexArray*	D_gTriangleMeshInterfacePtr;
	///D_only a single mesh part for now, we D_can add support for multiple parts, but quantized trees don't support this at the moment 
	D_ATTRIBUTE_ALIGNED16(D_btIndexedMesh	D_gIndexMesh);
	#define D_MAX_SPU_SUBTREE_HEADERS 32
	//1024
	D_ATTRIBUTE_ALIGNED16(D_btBvhSubtreeInfo	D_gSubtreeHeaders[D_MAX_SPU_SUBTREE_HEADERS]);
	D_ATTRIBUTE_ALIGNED16(D_btQuantizedBvhNode	D_gSubtreeNodes[D_MAX_SUBTREE_SIZE_IN_BYTES/sizeof(D_btQuantizedBvhNode)]);
};


void computeAabb (D_btVector3& aabbMin, D_btVector3& aabbMax, D_btConvexInternalShape* convexShape, D_ppu_address_t convexShapePtr, int shapeType, const D_btTransform& xform);
void dmaBvhShapeData (D_bvhMeshShape_LocalStoreMemory* bvhMeshShape, D_btBvhTriangleMeshShape* triMeshShape);
void dmaBvhIndexedMesh (D_btIndexedMesh* IndexMesh, D_IndexedMeshArray& indexArray, int index, D_uint32_t dmaTag);
void dmaBvhSubTreeHeaders (D_btBvhSubtreeInfo* subTreeHeaders, D_ppu_address_t subTreePtr, int batchSize, D_uint32_t dmaTag);
void dmaBvhSubTreeNodes (D_btQuantizedBvhNode* nodes, const D_btBvhSubtreeInfo& subtree, D_QuantizedNodeArray&	nodeArray, int dmaTag);

int  getShapeTypeSize(int shapeType);
void dmaConvexVertexData (D_SpuConvexPolyhedronVertexData* convexVertexData, D_btConvexHullShape* convexShapeSPU);
void dmaCollisionShape (void* collisionShapeLocation, D_ppu_address_t collisionShapePtr, D_uint32_t dmaTag, int shapeType);
void dmaCompoundShapeInfo (D_CompoundShape_LocalStoreMemory* compoundShapeLocation, D_btCompoundShape* spuCompoundShape, D_uint32_t dmaTag);
void dmaCompoundSubShapes (D_CompoundShape_LocalStoreMemory* compoundShapeLocation, D_btCompoundShape* spuCompoundShape, D_uint32_t dmaTag);


#define D_USE_BRANCHFREE_TEST 1
#ifdef D_USE_BRANCHFREE_TEST
D_SIMD_FORCE_INLINE unsigned int spuTestQuantizedAabbAgainstQuantizedAabb(unsigned short int* aabbMin1,unsigned short int* aabbMax1,const unsigned short int* aabbMin2,const unsigned short int* aabbMax2)
{		
#if defined(__CELLOS_LV2__) && defined (__SPU__)
	vec_ushort8 vecMin = {aabbMin1[0],aabbMin2[0],aabbMin1[2],aabbMin2[2],aabbMin1[1],aabbMin2[1],0,0};
	vec_ushort8 vecMax = {aabbMax2[0],aabbMax1[0],aabbMax2[2],aabbMax1[2],aabbMax2[1],aabbMax1[1],0,0};
	vec_ushort8 isGt = spu_cmpgt(vecMin,vecMax);
	return spu_extract(spu_gather(isGt),0)==0;

#else
	return D_btSelect((unsigned)((aabbMin1[0] <= aabbMax2[0]) & (aabbMax1[0] >= aabbMin2[0])
		& (aabbMin1[2] <= aabbMax2[2]) & (aabbMax1[2] >= aabbMin2[2])
		& (aabbMin1[1] <= aabbMax2[1]) & (aabbMax1[1] >= aabbMin2[1])),
		1, 0);
#endif
}
#else

D_SIMD_FORCE_INLINE unsigned int spuTestQuantizedAabbAgainstQuantizedAabb(const unsigned short int* aabbMin1,const unsigned short int* aabbMax1,const unsigned short int* aabbMin2,const unsigned short int*  aabbMax2)
{
	unsigned int overlap = 1;
	overlap = (aabbMin1[0] > aabbMax2[0] || aabbMax1[0] < aabbMin2[0]) ? 0 : overlap;
	overlap = (aabbMin1[2] > aabbMax2[2] || aabbMax1[2] < aabbMin2[2]) ? 0 : overlap;
	overlap = (aabbMin1[1] > aabbMax2[1] || aabbMax1[1] < aabbMin2[1]) ? 0 : overlap;
	return overlap;
}
#endif

void	spuWalkStacklessQuantizedTree(D_btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax,const D_btQuantizedBvhNode* rootNode,int startNodeIndex,int endNodeIndex);

#endif
