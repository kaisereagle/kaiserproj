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


#include "SpuCollisionShapes.h"

///not supported on IBM SDK, until we fix the alignment of D_btVector3
#if defined (__CELLOS_LV2__) && defined (__SPU__)
#include <spu_intrinsics.h>
static inline vec_float4 vec_dot3( vec_float4 vec0, vec_float4 vec1 )
{
    vec_float4 result;
    result = spu_mul( vec0, vec1 );
    result = spu_madd( spu_rlqwbyte( vec0, 4 ), spu_rlqwbyte( vec1, 4 ), result );
    return spu_madd( spu_rlqwbyte( vec0, 8 ), spu_rlqwbyte( vec1, 8 ), result );
}
#endif //__SPU__


void computeAabb (D_btVector3& aabbMin, D_btVector3& aabbMax, D_btConvexInternalShape* convexShape, D_ppu_address_t convexShapePtr, int shapeType, const D_btTransform& xform)
{
	//calculate the aabb, given the types...
	switch (shapeType)
	{
	case D_CYLINDER_SHAPE_PROXYTYPE:
		/* fall through */
	case D_BOX_SHAPE_PROXYTYPE:
	{
		D_btScalar margin=convexShape->getMarginNV();
		D_btVector3 halfExtents = convexShape->getImplicitShapeDimensions();
		halfExtents += D_btVector3(margin,margin,margin);
		const D_btTransform& t = xform;
		D_btMatrix3x3 abs_b = t.getBasis().absolute();  
		D_btVector3 center = t.getOrigin();
		D_btVector3 extent = D_btVector3(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));
		
		aabbMin = center - extent;
		aabbMax = center + extent;
		break;
	}
	case D_CAPSULE_SHAPE_PROXYTYPE:
	{
		D_btScalar margin=convexShape->getMarginNV();
		D_btVector3 halfExtents = convexShape->getImplicitShapeDimensions();
		//add the radius D_to y-axis D_to get full height
		D_btScalar radius = halfExtents[0];
		halfExtents[1] += radius;
		halfExtents += D_btVector3(margin,margin,margin);
#if 0
		int capsuleUpAxis = convexShape->getUpAxis();
		D_btScalar halfHeight = convexShape->getHalfHeight();
		D_btScalar radius = convexShape->getRadius();
		halfExtents[capsuleUpAxis] = radius + halfHeight;
#endif
		const D_btTransform& t = xform;
		D_btMatrix3x3 abs_b = t.getBasis().absolute();  
		D_btVector3 center = t.getOrigin();
		D_btVector3 extent = D_btVector3(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));
		
		aabbMin = center - extent;
		aabbMax = center + extent;
		break;
	}
	case D_SPHERE_SHAPE_PROXYTYPE:
	{
		D_btScalar radius = convexShape->getImplicitShapeDimensions().getX();// * convexShape->getLocalScaling().getX();
		D_btScalar margin = radius + convexShape->getMarginNV();
		const D_btTransform& t = xform;
		const D_btVector3& center = t.getOrigin();
		D_btVector3 extent(margin,margin,margin);
		aabbMin = center - extent;
		aabbMax = center + extent;
		break;
	}
	case D_CONVEX_HULL_SHAPE_PROXYTYPE:
	{
		D_ATTRIBUTE_ALIGNED16(char convexHullShape0[sizeof(D_btConvexHullShape)]);
		D_cellDmaGet(&convexHullShape0, convexShapePtr  , sizeof(D_btConvexHullShape), D_DMA_TAG(1), 0, 0);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
		D_btConvexHullShape* localPtr = (D_btConvexHullShape*)&convexHullShape0;
		const D_btTransform& t = xform;
		D_btScalar margin = convexShape->getMarginNV();
		localPtr->getNonvirtualAabb(t,aabbMin,aabbMax,margin);
		//D_spu_printf("SPU convex aabbMin=%f,%f,%f=\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
		//D_spu_printf("SPU convex aabbMax=%f,%f,%f=\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());
		break;
	}
	default:
		{
	//	D_spu_printf("SPU: unsupported shapetype %d in AABB calculation\n");
		}
	};
}

void dmaBvhShapeData (D_bvhMeshShape_LocalStoreMemory* bvhMeshShape, D_btBvhTriangleMeshShape* triMeshShape)
{
	register int dmaSize;
	register D_ppu_address_t	dmaPpuAddress2;

	dmaSize = sizeof(D_btTriangleIndexVertexArray);
	dmaPpuAddress2 = reinterpret_cast<D_ppu_address_t>(triMeshShape->getMeshInterface());
	//	D_spu_printf("trimeshShape->getMeshInterface() == %llx\n",dmaPpuAddress2);
#ifdef __SPU__
	D_cellDmaGet(&bvhMeshShape->D_gTriangleMeshInterfaceStorage, dmaPpuAddress2  , dmaSize, D_DMA_TAG(1), 0, 0);
	bvhMeshShape->D_gTriangleMeshInterfacePtr = &bvhMeshShape->D_gTriangleMeshInterfaceStorage;
#else
	bvhMeshShape->D_gTriangleMeshInterfacePtr = (D_btTriangleIndexVertexArray*)cellDmaGetReadOnly(&bvhMeshShape->D_gTriangleMeshInterfaceStorage, dmaPpuAddress2  , dmaSize, D_DMA_TAG(1), 0, 0);
#endif

	//D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
	
	///now DMA over the BVH
	
	dmaSize = sizeof(D_btOptimizedBvh);
	dmaPpuAddress2 = reinterpret_cast<D_ppu_address_t>(triMeshShape->getOptimizedBvh());
	//D_spu_printf("trimeshShape->getOptimizedBvh() == %llx\n",dmaPpuAddress2);
	D_cellDmaGet(&bvhMeshShape->D_gOptimizedBvh, dmaPpuAddress2  , dmaSize, D_DMA_TAG(2), 0, 0);
	//D_cellDmaWaitTagStatusAll(D_DMA_MASK(2));
	D_cellDmaWaitTagStatusAll(D_DMA_MASK(1) | D_DMA_MASK(2));
}

void dmaBvhIndexedMesh (D_btIndexedMesh* IndexMesh, D_IndexedMeshArray& indexArray, int index, D_uint32_t dmaTag)
{		
	D_cellDmaGet(IndexMesh, (D_ppu_address_t)&indexArray[index]  , sizeof(D_btIndexedMesh), D_DMA_TAG(dmaTag), 0, 0);
	
}

void dmaBvhSubTreeHeaders (D_btBvhSubtreeInfo* subTreeHeaders, D_ppu_address_t subTreePtr, int batchSize, D_uint32_t dmaTag)
{
	D_cellDmaGet(subTreeHeaders, subTreePtr, batchSize * sizeof(D_btBvhSubtreeInfo), D_DMA_TAG(dmaTag), 0, 0);
}

void dmaBvhSubTreeNodes (D_btQuantizedBvhNode* nodes, const D_btBvhSubtreeInfo& subtree, D_QuantizedNodeArray&	nodeArray, int dmaTag)
{
	D_cellDmaGet(nodes, reinterpret_cast<D_ppu_address_t>(&nodeArray[subtree.m_rootNodeIndex]) , subtree.m_subtreeSize* sizeof(D_btQuantizedBvhNode), D_DMA_TAG(2), 0, 0);
}

///getShapeTypeSize could easily be optimized, but it D_is not likely a bottleneck
int		getShapeTypeSize(int shapeType)
{


	switch (shapeType)
	{
	case D_CYLINDER_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(D_btCylinderShape);
			D_btAssert(shapeSize < D_MAX_SHAPE_SIZE);
			return shapeSize;
		}
	case D_BOX_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(D_btBoxShape);
			D_btAssert(shapeSize < D_MAX_SHAPE_SIZE);
			return shapeSize;
		}
	case D_SPHERE_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(D_btSphereShape);
			D_btAssert(shapeSize < D_MAX_SHAPE_SIZE);
			return shapeSize;
		}
	case D_TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(D_btBvhTriangleMeshShape);
			D_btAssert(shapeSize < D_MAX_SHAPE_SIZE);
			return shapeSize;
		}
	case D_CAPSULE_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(D_btCapsuleShape);
			D_btAssert(shapeSize < D_MAX_SHAPE_SIZE);
			return shapeSize;
		}

	case D_CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(D_btConvexHullShape);
			D_btAssert(shapeSize < D_MAX_SHAPE_SIZE);
			return shapeSize;
		}

	case D_COMPOUND_SHAPE_PROXYTYPE:
		{
			int shapeSize = sizeof(D_btCompoundShape);
			D_btAssert(shapeSize < D_MAX_SHAPE_SIZE);
			return shapeSize;
		}

	default:
		D_btAssert(0);
		//unsupported shapetype, please add here
		return 0;
	}
}

void dmaConvexVertexData (D_SpuConvexPolyhedronVertexData* convexVertexData, D_btConvexHullShape* convexShapeSPU)
{
	convexVertexData->D_gNumConvexPoints = convexShapeSPU->getNumPoints();
	if (convexVertexData->D_gNumConvexPoints>D_MAX_NUM_SPU_CONVEX_POINTS)
	{
		D_btAssert(0);
	//	D_spu_printf("SPU: Error: D_MAX_NUM_SPU_CONVEX_POINTS(%d) exceeded: %d\n",D_MAX_NUM_SPU_CONVEX_POINTS,convexVertexData->D_gNumConvexPoints);
		return;
	}
			
	register int dmaSize = convexVertexData->D_gNumConvexPoints*sizeof(D_btVector3);
	D_ppu_address_t pointsPPU = (D_ppu_address_t) convexShapeSPU->getUnscaledPoints();
	D_cellDmaGet(&convexVertexData->g_convexPointBuffer[0], pointsPPU  , dmaSize, D_DMA_TAG(2), 0, 0);
}

void dmaCollisionShape (void* collisionShapeLocation, D_ppu_address_t collisionShapePtr, D_uint32_t dmaTag, int shapeType)
{
	register int dmaSize = getShapeTypeSize(shapeType);
	D_cellDmaGet(collisionShapeLocation, collisionShapePtr  , dmaSize, D_DMA_TAG(dmaTag), 0, 0);
	//D_cellDmaWaitTagStatusAll(D_DMA_MASK(dmaTag));
}

void dmaCompoundShapeInfo (D_CompoundShape_LocalStoreMemory* compoundShapeLocation, D_btCompoundShape* spuCompoundShape, D_uint32_t dmaTag)
{
	register int dmaSize;
	register	D_ppu_address_t	dmaPpuAddress2;
	int childShapeCount = spuCompoundShape->getNumChildShapes();
	dmaSize = childShapeCount * sizeof(D_btCompoundShapeChild);
	dmaPpuAddress2 = (D_ppu_address_t)spuCompoundShape->getChildList();
	D_cellDmaGet(&compoundShapeLocation->D_gSubshapes[0], dmaPpuAddress2, dmaSize, D_DMA_TAG(dmaTag), 0, 0);
}

void dmaCompoundSubShapes (D_CompoundShape_LocalStoreMemory* compoundShapeLocation, D_btCompoundShape* spuCompoundShape, D_uint32_t dmaTag)
{
	int childShapeCount = spuCompoundShape->getNumChildShapes();
	int i;
	// DMA all the subshapes 
	for ( i = 0; i < childShapeCount; ++i)
	{
		D_btCompoundShapeChild& childShape = compoundShapeLocation->D_gSubshapes[i];
		dmaCollisionShape (&compoundShapeLocation->D_gSubshapeShape[i],(D_ppu_address_t)childShape.m_childShape, dmaTag, childShape.m_childShapeType);
	}
}


void	spuWalkStacklessQuantizedTree(D_btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax,const D_btQuantizedBvhNode* rootNode,int startNodeIndex,int endNodeIndex)
{

	int curIndex = startNodeIndex;
	int walkIterations = 0;
#ifdef D_BT_DEBUG
	int subTreeSize = endNodeIndex - startNodeIndex;
#endif

	int escapeIndex;

	unsigned int aabbOverlap, isLeafNode;

	while (curIndex < endNodeIndex)
	{
		//catch bugs in tree data
		D_btAssert (walkIterations < subTreeSize);

		walkIterations++;
		aabbOverlap = spuTestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode->m_quantizedAabbMin,rootNode->m_quantizedAabbMax);
		isLeafNode = rootNode->isLeafNode();

		if (isLeafNode && aabbOverlap)
		{
			//printf("overlap with node %d\n",rootNode->getTriangleIndex());
			nodeCallback->processNode(0,rootNode->getTriangleIndex());
			//			D_spu_printf("SPU: overlap detected with triangleIndex:%d\n",rootNode->getTriangleIndex());
		} 

		if (aabbOverlap || isLeafNode)
		{
			rootNode++;
			curIndex++;
		} else
		{
			escapeIndex = rootNode->getEscapeIndex();
			rootNode += escapeIndex;
			curIndex += escapeIndex;
		}
	}

}
