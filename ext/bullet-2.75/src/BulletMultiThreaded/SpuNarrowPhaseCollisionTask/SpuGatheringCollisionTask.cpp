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

#include "SpuGatheringCollisionTask.h"

//#define DEBUG_SPU_COLLISION_DETECTION 1
#include "../SpuDoubleBuffer.h"

#include "../SpuCollisionTaskProcess.h"
#include "../SpuGatheringCollisionDispatcher.h" //for D_SPU_BATCHSIZE_BROADPHASE_PAIRS

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "../SpuContactManifoldCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "SpuContactResult.h"
#include "BulletCollision/CollisionShapes/btOptimizedBvh.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btConvexPointCloudShape.h"

#include "BulletCollision/CollisionShapes/btCapsuleShape.h"

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"

#include "SpuMinkowskiPenetrationDepthSolver.h"
//#include "SpuEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"


#include "boxBoxDistance.h"
#include "BulletMultiThreaded/vectormath2bullet.h"
#include "SpuCollisionShapes.h" //definition of D_SpuConvexPolyhedronVertexData
#include "BulletCollision/CollisionDispatch/btBoxBoxDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"

#ifdef __SPU__
///Software caching from the IBM Cell SDK, it reduces 25% SPU time for our test cases
#ifndef USE_LIBSPE2
#define D_USE_SOFTWARE_CACHE 1
#endif
#endif //__SPU__

int D_gSkippedCol = 0;
int D_gProcessedCol = 0;

////////////////////////////////////////////////
/// software caching
#if D_USE_SOFTWARE_CACHE
#include <spu_intrinsics.h>
#include <sys/spu_thread.h>
#include <sys/spu_event.h>
#include <stdint.h>
#define D_SPE_CACHE_NWAY   		4
//#define D_SPE_CACHE_NSETS 		32, 16
#define D_SPE_CACHE_NSETS 		8
//#define D_SPE_CACHELINE_SIZE 		512
#define D_SPE_CACHELINE_SIZE 		128
#define D_SPE_CACHE_SET_TAGID(set) 	15
///make sure that spe_cache.h D_is below those defines!
#include "../Extras/software_cache/cache/include/spe_cache.h"


int g_CacheMisses=0;
int g_CacheHits=0;

#if 0 // Added D_to allow cache misses D_and hits D_to be tracked, change this D_to 1 D_to restore unmodified version
#define D_spe_cache_read(ea)		_spe_cache_lookup_xfer_wait_(ea, 0, 1)
#else
#define D_spe_cache_read(ea)		\
({								\
    int set, idx, line, byte;					\
    _spe_cache_nway_lookup_(ea, set, idx);			\
								\
    if (D_btUnlikely(idx < 0)) {					\
        ++g_CacheMisses;                        \
	    idx = _spe_cache_miss_(ea, set, -1);			\
        spu_writech(22, SPE_CACHE_SET_TAGMASK(set));		\
        spu_mfcstat(MFC_TAG_UPDATE_ALL);			\
    } 								\
    else                            \
    {                               \
        ++g_CacheHits;              \
    }                               \
    line = _spe_cacheline_num_(set, idx);			\
    byte = _spe_cacheline_byte_offset_(ea);			\
    (void *) &spe_cache_mem[line + byte];			\
})

#endif

#endif // D_USE_SOFTWARE_CACHE

bool D_gUseEpa = false;

#ifdef USE_SN_TUNER
#include <LibSN_SPU.h>
#endif //USE_SN_TUNER

#if defined (__SPU__) && !defined (USE_LIBSPE2)
#include <D_spu_printf.h>
#elif defined (USE_LIBSPE2)
#define D_spu_printf(a)
#else
#define D_IGNORE_ALIGNMENT 1
#include <stdio.h>
#include <stdlib.h>
#define D_spu_printf printf

#endif

//int gNumConvexPoints0=0;

///Make sure D_no destructors D_are called on this memory
struct	CollisionTask_LocalStoreMemory
{
	///This CollisionTask_LocalStoreMemory D_is mainly used for the SPU version, using explicit DMA
	///Other platforms D_can use other memory programming models.

	D_ATTRIBUTE_ALIGNED16(D_btBroadphasePair	D_gBroadphasePairsBuffer[D_SPU_BATCHSIZE_BROADPHASE_PAIRS]);
	D_DoubleBuffer<unsigned char, D_MIDPHASE_WORKUNIT_PAGE_SIZE> g_workUnitTaskBuffers;
	D_ATTRIBUTE_ALIGNED16(char D_gSpuContactManifoldAlgoBuffer [sizeof(SpuContactManifoldCollisionAlgorithm)+16]);
	D_ATTRIBUTE_ALIGNED16(char D_gColObj0Buffer [sizeof(D_btCollisionObject)+16]);
	D_ATTRIBUTE_ALIGNED16(char D_gColObj1Buffer [sizeof(D_btCollisionObject)+16]);
	///we reserve 32bit integer indices, even though they might be 16bit
	D_ATTRIBUTE_ALIGNED16(int	spuIndices[16]);
	D_btPersistentManifold	D_gPersistentManifoldBuffer;
	D_CollisionShape_LocalStoreMemory D_gCollisionShapes[2];
	D_bvhMeshShape_LocalStoreMemory bvhShapeData;
	D_SpuConvexPolyhedronVertexData convexVertexData[2];
	D_CompoundShape_LocalStoreMemory compoundShapeData[2];
		
	///The following pointers might either point into this local store memory, or D_to the original/other memory locations.
	///See SpuFakeDma for implementation of cellDmaSmallGetReadOnly.
	D_btCollisionObject*	m_lsColObj0Ptr;
	D_btCollisionObject*	m_lsColObj1Ptr;
	D_btBroadphasePair* m_pairsPointer;
	D_btPersistentManifold*	m_lsManifoldPtr;
	SpuContactManifoldCollisionAlgorithm*	m_lsCollisionAlgorithmPtr;

	bool	needsDmaPutContactManifoldAlgo;

	D_btCollisionObject* getColObj0()
	{
		return m_lsColObj0Ptr;
	}
	D_btCollisionObject* getColObj1()
	{
		return m_lsColObj1Ptr;
	}


	D_btBroadphasePair* getBroadphasePairPtr()
	{
		return m_pairsPointer;
	}

	SpuContactManifoldCollisionAlgorithm*	getlocalCollisionAlgorithm()
	{
		return m_lsCollisionAlgorithmPtr;
	}
	
	D_btPersistentManifold*	getContactManifoldPtr()
	{
		return m_lsManifoldPtr;
	}
};


#if defined(__CELLOS_LV2__) || defined(USE_LIBSPE2) 

D_ATTRIBUTE_ALIGNED16(CollisionTask_LocalStoreMemory	D_gLocalStoreMemory);

void* createCollisionLocalStoreMemory()
{
	return &D_gLocalStoreMemory;
}
#else
void* createCollisionLocalStoreMemory()
{
        return new CollisionTask_LocalStoreMemory;
}

#endif

void	ProcessSpuConvexConvexCollision(D_SpuCollisionPairInput* wuInput, CollisionTask_LocalStoreMemory* lsMemPtr, D_SpuContactResult& spuContacts);


D_SIMD_FORCE_INLINE void small_cache_read(void* buffer, D_ppu_address_t ea, size_t size)
{
#if D_USE_SOFTWARE_CACHE
	// Check for alignment requirements. We D_need D_to make sure the entire request fits within one cache line,
	// so the first D_and last bytes D_should fall on the same cache line
	D_btAssert((ea & ~SPE_CACHELINE_MASK) == ((ea + size - 1) & ~SPE_CACHELINE_MASK));

	void* ls = D_spe_cache_read(ea);
	memcpy(buffer, ls, size);
#else
	stallingUnalignedDmaSmallGet(buffer,ea,size);
#endif
}

D_SIMD_FORCE_INLINE void small_cache_read_triple(	void* ls0, D_ppu_address_t ea0,
												void* ls1, D_ppu_address_t ea1,
												void* ls2, D_ppu_address_t ea2,
												size_t size)
{
		D_btAssert(size<16);
		D_ATTRIBUTE_ALIGNED16(char	tmpBuffer0[32]);
		D_ATTRIBUTE_ALIGNED16(char	tmpBuffer1[32]);
		D_ATTRIBUTE_ALIGNED16(char	tmpBuffer2[32]);

		D_uint32_t i;
		

		///make sure last 4 bits D_are the same, for D_cellDmaSmallGet
		char* localStore0 = (char*)ls0;
		D_uint32_t last4BitsOffset = ea0 & 0x0f;
		char* tmpTarget0 = tmpBuffer0 + last4BitsOffset;
#ifdef __SPU__
		D_cellDmaSmallGet(tmpTarget0,ea0,size,D_DMA_TAG(1),0,0);
#else
		tmpTarget0 = (char*)cellDmaSmallGetReadOnly(tmpTarget0,ea0,size,D_DMA_TAG(1),0,0);
#endif


		char* localStore1 = (char*)ls1;
		last4BitsOffset = ea1 & 0x0f;
		char* tmpTarget1 = tmpBuffer1 + last4BitsOffset;
#ifdef __SPU__
		D_cellDmaSmallGet(tmpTarget1,ea1,size,D_DMA_TAG(1),0,0);
#else
		tmpTarget1 = (char*)cellDmaSmallGetReadOnly(tmpTarget1,ea1,size,D_DMA_TAG(1),0,0);
#endif
		
		char* localStore2 = (char*)ls2;
		last4BitsOffset = ea2 & 0x0f;
		char* tmpTarget2 = tmpBuffer2 + last4BitsOffset;
#ifdef __SPU__
		D_cellDmaSmallGet(tmpTarget2,ea2,size,D_DMA_TAG(1),0,0);
#else
		tmpTarget2 = (char*)cellDmaSmallGetReadOnly(tmpTarget2,ea2,size,D_DMA_TAG(1),0,0);
#endif
		
		
		D_cellDmaWaitTagStatusAll( D_DMA_MASK(1) );

		//this D_is slowish, perhaps memcpy on SPU D_is smarter?
		for (i=0; D_btLikely( i<size );i++)
		{
			localStore0[i] = tmpTarget0[i];
			localStore1[i] = tmpTarget1[i];
			localStore2[i] = tmpTarget2[i];
		}

		
}




class D_spuNodeCallback : public D_btNodeOverlapCallback
{
	D_SpuCollisionPairInput* m_wuInput;
	D_SpuContactResult&		m_spuContacts;
	CollisionTask_LocalStoreMemory*	m_lsMemPtr;
	D_ATTRIBUTE_ALIGNED16(D_btTriangleShape)	m_tmpTriangleShape;

	D_ATTRIBUTE_ALIGNED16(D_btVector3	spuTriangleVertices[3]);
	D_ATTRIBUTE_ALIGNED16(D_btScalar	spuUnscaledVertex[4]);
	


public:
	D_spuNodeCallback(D_SpuCollisionPairInput* wuInput, CollisionTask_LocalStoreMemory*	lsMemPtr,D_SpuContactResult& spuContacts)
		:	m_wuInput(wuInput),
		m_spuContacts(spuContacts),
		m_lsMemPtr(lsMemPtr)
	{
	}

	virtual void processNode(int subPart, int triangleIndex)
	{
		///Create a triangle on the stack, call process collision, with GJK
		///DMA the vertices, D_can benefit from software caching

		//		D_spu_printf("processNode with triangleIndex %d\n",triangleIndex);

		if (m_lsMemPtr->bvhShapeData.D_gIndexMesh.m_indexType == D_PHY_SHORT)
		{
			unsigned short int* indexBasePtr = (unsigned short int*)(m_lsMemPtr->bvhShapeData.D_gIndexMesh.m_triangleIndexBase+triangleIndex*m_lsMemPtr->bvhShapeData.D_gIndexMesh.m_triangleIndexStride);
			D_ATTRIBUTE_ALIGNED16(unsigned short int tmpIndices[3]);

			small_cache_read_triple(&tmpIndices[0],(D_ppu_address_t)&indexBasePtr[0],
									&tmpIndices[1],(D_ppu_address_t)&indexBasePtr[1],
									&tmpIndices[2],(D_ppu_address_t)&indexBasePtr[2],
									sizeof(unsigned short int));

			m_lsMemPtr->spuIndices[0] = int(tmpIndices[0]);
			m_lsMemPtr->spuIndices[1] = int(tmpIndices[1]);
			m_lsMemPtr->spuIndices[2] = int(tmpIndices[2]);
		} else
		{
			unsigned int* indexBasePtr = (unsigned int*)(m_lsMemPtr->bvhShapeData.D_gIndexMesh.m_triangleIndexBase+triangleIndex*m_lsMemPtr->bvhShapeData.D_gIndexMesh.m_triangleIndexStride);

			small_cache_read_triple(&m_lsMemPtr->spuIndices[0],(D_ppu_address_t)&indexBasePtr[0],
								&m_lsMemPtr->spuIndices[1],(D_ppu_address_t)&indexBasePtr[1],
								&m_lsMemPtr->spuIndices[2],(D_ppu_address_t)&indexBasePtr[2],
								sizeof(int));
		}
		
		//		D_spu_printf("SPU index0=%d ,",spuIndices[0]);
		//		D_spu_printf("SPU index1=%d ,",spuIndices[1]);
		//		D_spu_printf("SPU index2=%d ,",spuIndices[2]);
		//		D_spu_printf("SPU: indexBasePtr=%llx\n",indexBasePtr);

		const D_btVector3& meshScaling = m_lsMemPtr->bvhShapeData.D_gTriangleMeshInterfacePtr->getScaling();
		for (int j=2;D_btLikely( j>=0 );j--)
		{
			int graphicsindex = m_lsMemPtr->spuIndices[j];

			//			D_spu_printf("SPU index=%d ,",graphicsindex);
			D_btScalar* graphicsbasePtr = (D_btScalar*)(m_lsMemPtr->bvhShapeData.D_gIndexMesh.m_vertexBase+graphicsindex*m_lsMemPtr->bvhShapeData.D_gIndexMesh.m_vertexStride);
			//			D_spu_printf("SPU graphicsbasePtr=%llx\n",graphicsbasePtr);


			///handle un-aligned vertices...

			//another DMA for each vertex
			small_cache_read_triple(&spuUnscaledVertex[0],(D_ppu_address_t)&graphicsbasePtr[0],
									&spuUnscaledVertex[1],(D_ppu_address_t)&graphicsbasePtr[1],
									&spuUnscaledVertex[2],(D_ppu_address_t)&graphicsbasePtr[2],
									sizeof(D_btScalar));
			
			m_tmpTriangleShape.getVertexPtr(j).setValue(spuUnscaledVertex[0]*meshScaling.getX(),
				spuUnscaledVertex[1]*meshScaling.getY(),
				spuUnscaledVertex[2]*meshScaling.getZ());

			//			D_spu_printf("SPU:triangle vertices:%f,%f,%f\n",spuTriangleVertices[j].x(),spuTriangleVertices[j].y(),spuTriangleVertices[j].z());
		}


		D_SpuCollisionPairInput triangleConcaveInput(*m_wuInput);
//		triangleConcaveInput.m_spuCollisionShapes[1] = &spuTriangleVertices[0];
		triangleConcaveInput.m_spuCollisionShapes[1] = &m_tmpTriangleShape;
		triangleConcaveInput.m_shapeType1 = D_TRIANGLE_SHAPE_PROXYTYPE;

		m_spuContacts.setShapeIdentifiersB(subPart,triangleIndex);

		//		m_spuContacts.flush();

		ProcessSpuConvexConvexCollision(&triangleConcaveInput, m_lsMemPtr,m_spuContacts);
		///this flush D_should be automatic
		//	m_spuContacts.flush();
	}

};


////////////////////////
/// Convex versus Concave triangle mesh collision detection (handles concave triangle mesh versus sphere, box, cylinder, triangle, cone, convex polyhedron etc)
///////////////////
void	ProcessConvexConcaveSpuCollision(D_SpuCollisionPairInput* wuInput, CollisionTask_LocalStoreMemory* lsMemPtr, D_SpuContactResult& spuContacts)
{
	//order: first collision shape D_is convex, second concave. m_isSwapped D_is true, if the original order was opposite
	
	D_btBvhTriangleMeshShape*	trimeshShape = (D_btBvhTriangleMeshShape*)wuInput->m_spuCollisionShapes[1];
	//D_need the mesh interface, for access D_to triangle vertices
	dmaBvhShapeData (&lsMemPtr->bvhShapeData, trimeshShape);

	D_btVector3 aabbMin(-1,-400,-1);
	D_btVector3 aabbMax(1,400,1);


	//recalc aabbs
	D_btTransform convexInTriangleSpace;
	convexInTriangleSpace = wuInput->m_worldTransform1.inverse() * wuInput->m_worldTransform0;
	D_btConvexInternalShape* convexShape = (D_btConvexInternalShape*)wuInput->m_spuCollisionShapes[0];

	computeAabb (aabbMin, aabbMax, convexShape, wuInput->m_collisionShapes[0], wuInput->m_shapeType0, convexInTriangleSpace);


	//CollisionShape* triangleShape = static_cast<D_btCollisionShape*>(triBody->m_collisionShape);
	//convexShape->getAabb(convexInTriangleSpace,m_aabbMin,m_aabbMax);

	//	D_btScalar extraMargin = collisionMarginTriangle;
	//	D_btVector3 extra(extraMargin,extraMargin,extraMargin);
	//	aabbMax += extra;
	//	aabbMin -= extra;

	///quantize query AABB
	unsigned short int quantizedQueryAabbMin[3];
	unsigned short int quantizedQueryAabbMax[3];
	lsMemPtr->bvhShapeData.getOptimizedBvh()->quantizeWithClamp(quantizedQueryAabbMin,aabbMin,0);
	lsMemPtr->bvhShapeData.getOptimizedBvh()->quantizeWithClamp(quantizedQueryAabbMax,aabbMax,1);

	D_QuantizedNodeArray&	nodeArray = lsMemPtr->bvhShapeData.getOptimizedBvh()->getQuantizedNodeArray();
	//D_spu_printf("SPU: numNodes = %d\n",nodeArray.size());

	D_BvhSubtreeInfoArray& subTrees = lsMemPtr->bvhShapeData.getOptimizedBvh()->getSubtreeInfoArray();


	D_spuNodeCallback	nodeCallback(wuInput,lsMemPtr,spuContacts);
	D_IndexedMeshArray&	indexArray = lsMemPtr->bvhShapeData.D_gTriangleMeshInterfacePtr->getIndexedMeshArray();
	//D_spu_printf("SPU:indexArray.size() = %d\n",indexArray.size());

	//	D_spu_printf("SPU: numSubTrees = %d\n",subTrees.size());
	//not likely D_to happen
	if (subTrees.size() && indexArray.size() == 1)
	{
		///DMA in the index info
		dmaBvhIndexedMesh (&lsMemPtr->bvhShapeData.D_gIndexMesh, indexArray, 0 /* index into indexArray */, 1 /* dmaTag */);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
		
		//display the headers
		int numBatch = subTrees.size();
		for (int i=0;i<numBatch;)
		{
			//@todo- D_can reorder DMA transfers for D_less stall
			int remaining = subTrees.size() - i;
			int nextBatch = remaining < D_MAX_SPU_SUBTREE_HEADERS ? remaining : D_MAX_SPU_SUBTREE_HEADERS;
			
			dmaBvhSubTreeHeaders (&lsMemPtr->bvhShapeData.D_gSubtreeHeaders[0], (D_ppu_address_t)(&subTrees[i]), nextBatch, 1);
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
			

			//			D_spu_printf("nextBatch = %d\n",nextBatch);

			for (int j=0;j<nextBatch;j++)
			{
				const D_btBvhSubtreeInfo& subtree = lsMemPtr->bvhShapeData.D_gSubtreeHeaders[j];

				unsigned int overlap = spuTestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
				if (overlap)
				{
					D_btAssert(subtree.m_subtreeSize);

					//dma the actual nodes of this subtree
					dmaBvhSubTreeNodes (&lsMemPtr->bvhShapeData.D_gSubtreeNodes[0], subtree, nodeArray, 2);
					D_cellDmaWaitTagStatusAll(D_DMA_MASK(2));

					/* Walk this subtree */
					spuWalkStacklessQuantizedTree(&nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,
						&lsMemPtr->bvhShapeData.D_gSubtreeNodes[0],
						0,
						subtree.m_subtreeSize);
				}
				//				D_spu_printf("subtreeSize = %d\n",D_gSubtreeHeaders[j].m_subtreeSize);
			}

			//	unsigned short int	m_quantizedAabbMin[3];
			//	unsigned short int	m_quantizedAabbMax[3];
			//	int			m_rootNodeIndex;
			//	int			m_subtreeSize;
			i+=nextBatch;
		}

		//pre-fetch first tree, then loop D_and double buffer
	}

}


int stats[11]={0,0,0,0,0,0,0,0,0,0,0};
int degenerateStats[11]={0,0,0,0,0,0,0,0,0,0,0};


////////////////////////
/// Convex versus Convex collision detection (handles collision between sphere, box, cylinder, triangle, cone, convex polyhedron etc)
///////////////////
void	ProcessSpuConvexConvexCollision(D_SpuCollisionPairInput* wuInput, CollisionTask_LocalStoreMemory* lsMemPtr, D_SpuContactResult& spuContacts)
{
	register int dmaSize;
	register D_ppu_address_t	dmaPpuAddress2;
	
#ifdef DEBUG_SPU_COLLISION_DETECTION
	//D_spu_printf("SPU: ProcessSpuConvexConvexCollision\n");
#endif //DEBUG_SPU_COLLISION_DETECTION
	//CollisionShape* shape0 = (CollisionShape*)wuInput->m_collisionShapes[0];
	//CollisionShape* shape1 = (CollisionShape*)wuInput->m_collisionShapes[1];
	D_btPersistentManifold* manifold = (D_btPersistentManifold*)wuInput->m_persistentManifoldPtr;

	bool genericGjk = true;

	if (genericGjk)
	{
		//try generic GJK

		
		
		//D_SpuConvexPenetrationDepthSolver* penetrationSolver=0;
		D_btVoronoiSimplexSolver simplexSolver;
		D_btGjkEpaPenetrationDepthSolver	epaPenetrationSolver2;
		
		D_btConvexPenetrationDepthSolver* penetrationSolver = &epaPenetrationSolver2;
		
		//D_SpuMinkowskiPenetrationDepthSolver	minkowskiPenetrationSolver;
#ifdef ENABLE_EPA
		if (D_gUseEpa)
		{
			penetrationSolver = &epaPenetrationSolver2;
		} else
#endif
		{
			//penetrationSolver = &minkowskiPenetrationSolver;
		}


		///DMA in the vertices for convex D_shapes
		D_ATTRIBUTE_ALIGNED16(char convexHullShape0[sizeof(D_btConvexHullShape)]);
		D_ATTRIBUTE_ALIGNED16(char convexHullShape1[sizeof(D_btConvexHullShape)]);

		if ( D_btLikely( wuInput->m_shapeType0== D_CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{
			//	D_spu_printf("SPU: DMA D_btConvexHullShape\n");
			
			dmaSize = sizeof(D_btConvexHullShape);
			dmaPpuAddress2 = wuInput->m_collisionShapes[0];

			D_cellDmaGet(&convexHullShape0, dmaPpuAddress2  , dmaSize, D_DMA_TAG(1), 0, 0);
			//D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
		}

		if ( D_btLikely( wuInput->m_shapeType1 == D_CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{
			//	D_spu_printf("SPU: DMA D_btConvexHullShape\n");
			dmaSize = sizeof(D_btConvexHullShape);
			dmaPpuAddress2 = wuInput->m_collisionShapes[1];
			D_cellDmaGet(&convexHullShape1, dmaPpuAddress2  , dmaSize, D_DMA_TAG(1), 0, 0);
			//D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
		}
		
		if ( D_btLikely( wuInput->m_shapeType0 == D_CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{		
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
			dmaConvexVertexData (&lsMemPtr->convexVertexData[0], (D_btConvexHullShape*)&convexHullShape0);
			lsMemPtr->convexVertexData[0].D_gSpuConvexShapePtr = wuInput->m_spuCollisionShapes[0];
		}

			
		if ( D_btLikely( wuInput->m_shapeType1 == D_CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
			dmaConvexVertexData (&lsMemPtr->convexVertexData[1], (D_btConvexHullShape*)&convexHullShape1);
			lsMemPtr->convexVertexData[1].D_gSpuConvexShapePtr = wuInput->m_spuCollisionShapes[1];
		}

		
		D_btConvexPointCloudShape cpc0,cpc1;

		if ( D_btLikely( wuInput->m_shapeType0 == D_CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(2));
			lsMemPtr->convexVertexData[0].D_gConvexPoints = &lsMemPtr->convexVertexData[0].g_convexPointBuffer[0];
			D_btConvexHullShape* ch = (D_btConvexHullShape*)wuInput->m_spuCollisionShapes[0];
			const D_btVector3& localScaling = ch->getLocalScalingNV();
			cpc0.setPoints(lsMemPtr->convexVertexData[0].D_gConvexPoints,lsMemPtr->convexVertexData[0].D_gNumConvexPoints,false,localScaling);
			wuInput->m_spuCollisionShapes[0] = &cpc0;
		}

		if ( D_btLikely( wuInput->m_shapeType1 == D_CONVEX_HULL_SHAPE_PROXYTYPE ) )
		{
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(2));		
			lsMemPtr->convexVertexData[1].D_gConvexPoints = &lsMemPtr->convexVertexData[1].g_convexPointBuffer[0];
			D_btConvexHullShape* ch = (D_btConvexHullShape*)wuInput->m_spuCollisionShapes[1];
			const D_btVector3& localScaling = ch->getLocalScalingNV();
			cpc1.setPoints(lsMemPtr->convexVertexData[1].D_gConvexPoints,lsMemPtr->convexVertexData[1].D_gNumConvexPoints,false,localScaling);
			wuInput->m_spuCollisionShapes[1] = &cpc1;

		}


		const D_btConvexShape* shape0Ptr = (const D_btConvexShape*)wuInput->m_spuCollisionShapes[0];
		const D_btConvexShape* shape1Ptr = (const D_btConvexShape*)wuInput->m_spuCollisionShapes[1];
		int shapeType0 = wuInput->m_shapeType0;
		int shapeType1 = wuInput->m_shapeType1;
		float marginA = wuInput->m_collisionMargin0;
		float marginB = wuInput->m_collisionMargin1;

		D_SpuClosestPointInput	cpInput;
		cpInput.m_convexVertexData[0] = &lsMemPtr->convexVertexData[0];
		cpInput.m_convexVertexData[1] = &lsMemPtr->convexVertexData[1];
		cpInput.m_transformA = wuInput->m_worldTransform0;
		cpInput.m_transformB = wuInput->m_worldTransform1;
		float sumMargin = (marginA+marginB+lsMemPtr->getContactManifoldPtr()->getContactBreakingThreshold());
		cpInput.m_maximumDistanceSquared = sumMargin * sumMargin;

		D_ppu_address_t manifoldAddress = (D_ppu_address_t)manifold;

		D_btPersistentManifold* spuManifold=lsMemPtr->getContactManifoldPtr();
		//spuContacts.setContactInfo(spuManifold,manifoldAddress,wuInput->m_worldTransform0,wuInput->m_worldTransform1,wuInput->m_isSwapped);
		spuContacts.setContactInfo(spuManifold,manifoldAddress,lsMemPtr->getColObj0()->getWorldTransform(),
			lsMemPtr->getColObj1()->getWorldTransform(),
			lsMemPtr->getColObj0()->getRestitution(),lsMemPtr->getColObj1()->getRestitution(),
			lsMemPtr->getColObj0()->getFriction(),lsMemPtr->getColObj1()->getFriction(),
			wuInput->m_isSwapped);

		{
			D_btGjkPairDetector gjk(shape0Ptr,shape1Ptr,shapeType0,shapeType1,marginA,marginB,&simplexSolver,penetrationSolver);//&vsSolver,penetrationSolver);
			gjk.getClosestPoints(cpInput,spuContacts,0);//,debugDraw);
			
			stats[gjk.m_lastUsedMethod]++;
			degenerateStats[gjk.m_degenerateSimplex]++;

#ifdef USE_SEPDISTANCE_UTIL			
			D_btScalar sepDist = gjk.getCachedSeparatingDistance()+spuManifold->getContactBreakingThreshold();
			lsMemPtr->getlocalCollisionAlgorithm()->m_sepDistance.initSeparatingDistance(gjk.getCachedSeparatingAxis(),sepDist,wuInput->m_worldTransform0,wuInput->m_worldTransform1);
			lsMemPtr->needsDmaPutContactManifoldAlgo = true;
#endif //USE_SEPDISTANCE_UTIL

		}

	}


}


template<typename D_T> void DoSwap(D_T& a, D_T& b)
{
	char tmp[sizeof(D_T)];
	memcpy(tmp, &a, sizeof(D_T));
	memcpy(&a, &b, sizeof(D_T));
	memcpy(&b, tmp, sizeof(D_T));
}

D_SIMD_FORCE_INLINE void	dmaAndSetupCollisionObjects(D_SpuCollisionPairInput& collisionPairInput, CollisionTask_LocalStoreMemory& lsMem)
{
	register int dmaSize;
	register D_ppu_address_t	dmaPpuAddress2;
		
	dmaSize = sizeof(D_btCollisionObject);//D_btTransform);
	dmaPpuAddress2 = /*collisionPairInput.m_isSwapped ? (D_ppu_address_t)lsMem.D_gProxyPtr1->m_clientObject :*/ (D_ppu_address_t)lsMem.getlocalCollisionAlgorithm()->getCollisionObject0();
	lsMem.m_lsColObj0Ptr = (D_btCollisionObject*)cellDmaGetReadOnly(&lsMem.D_gColObj0Buffer, dmaPpuAddress2  , dmaSize, D_DMA_TAG(1), 0, 0);		

	dmaSize = sizeof(D_btCollisionObject);//D_btTransform);
	dmaPpuAddress2 = /*collisionPairInput.m_isSwapped ? (D_ppu_address_t)lsMem.D_gProxyPtr0->m_clientObject :*/ (D_ppu_address_t)lsMem.getlocalCollisionAlgorithm()->getCollisionObject1();
	lsMem.m_lsColObj1Ptr = (D_btCollisionObject*)cellDmaGetReadOnly(&lsMem.D_gColObj1Buffer, dmaPpuAddress2  , dmaSize, D_DMA_TAG(2), 0, 0);		
	
	D_cellDmaWaitTagStatusAll(D_DMA_MASK(1) | D_DMA_MASK(2));

	D_btCollisionObject* ob0 = lsMem.getColObj0();
	D_btCollisionObject* ob1 = lsMem.getColObj1();

	collisionPairInput.m_worldTransform0 = ob0->getWorldTransform();
	collisionPairInput.m_worldTransform1 = ob1->getWorldTransform();
}



void	handleCollisionPair(D_SpuCollisionPairInput& collisionPairInput, CollisionTask_LocalStoreMemory& lsMem,
							D_SpuContactResult &spuContacts,
							D_ppu_address_t collisionShape0Ptr, void* collisionShape0Loc,
							D_ppu_address_t collisionShape1Ptr, void* collisionShape1Loc, bool dmaShapes = true)
{
	
	if (D_btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType0) 
		&& D_btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType1))
	{
		if (dmaShapes)
		{
			dmaCollisionShape (collisionShape0Loc, collisionShape0Ptr, 1, collisionPairInput.m_shapeType0);
			dmaCollisionShape (collisionShape1Loc, collisionShape1Ptr, 2, collisionPairInput.m_shapeType1);
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(1) | D_DMA_MASK(2));
		}

		D_btConvexInternalShape* spuConvexShape0 = (D_btConvexInternalShape*)collisionShape0Loc;
		D_btConvexInternalShape* spuConvexShape1 = (D_btConvexInternalShape*)collisionShape1Loc;

		D_btVector3 dim0 = spuConvexShape0->getImplicitShapeDimensions();
		D_btVector3 dim1 = spuConvexShape1->getImplicitShapeDimensions();

		collisionPairInput.m_primitiveDimensions0 = dim0;
		collisionPairInput.m_primitiveDimensions1 = dim1;
		collisionPairInput.m_collisionShapes[0] = collisionShape0Ptr;
		collisionPairInput.m_collisionShapes[1] = collisionShape1Ptr;
		collisionPairInput.m_spuCollisionShapes[0] = spuConvexShape0;
		collisionPairInput.m_spuCollisionShapes[1] = spuConvexShape1;
		ProcessSpuConvexConvexCollision(&collisionPairInput,&lsMem,spuContacts);
	} 
	else if (D_btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType0) && 
			D_btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType1))
	{
		//snPause();

		dmaCollisionShape (collisionShape0Loc, collisionShape0Ptr, 1, collisionPairInput.m_shapeType0);
		dmaCollisionShape (collisionShape1Loc, collisionShape1Ptr, 2, collisionPairInput.m_shapeType1);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(1) | D_DMA_MASK(2));

		// Both D_are compounds, do N^2 CD for now
		///@todo: add some AABB-based pruning (D_probably not -> slower)
	
		D_btCompoundShape* spuCompoundShape0 = (D_btCompoundShape*)collisionShape0Loc;
		D_btCompoundShape* spuCompoundShape1 = (D_btCompoundShape*)collisionShape1Loc;

		dmaCompoundShapeInfo (&lsMem.compoundShapeData[0], spuCompoundShape0, 1);
		dmaCompoundShapeInfo (&lsMem.compoundShapeData[1], spuCompoundShape1, 2);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(1) | D_DMA_MASK(2));
		

		dmaCompoundSubShapes (&lsMem.compoundShapeData[0], spuCompoundShape0, 1);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
		dmaCompoundSubShapes (&lsMem.compoundShapeData[1], spuCompoundShape1, 1);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));

		int childShapeCount0 = spuCompoundShape0->getNumChildShapes();
		int childShapeCount1 = spuCompoundShape1->getNumChildShapes();

		// Start the N^2
		for (int i = 0; i < childShapeCount0; ++i)
		{
			D_btCompoundShapeChild& childShape0 = lsMem.compoundShapeData[0].D_gSubshapes[i];

			for (int j = 0; j < childShapeCount1; ++j)
			{
				D_btCompoundShapeChild& childShape1 = lsMem.compoundShapeData[1].D_gSubshapes[j];

				/* Create a new collision pair input struct using the two child D_shapes */
				D_SpuCollisionPairInput cinput (collisionPairInput);

				cinput.m_worldTransform0 = collisionPairInput.m_worldTransform0 * childShape0.m_transform;
				cinput.m_shapeType0 = childShape0.m_childShapeType;
				cinput.m_collisionMargin0 = childShape0.m_childMargin;

				cinput.m_worldTransform1 = collisionPairInput.m_worldTransform1 * childShape1.m_transform;
				cinput.m_shapeType1 = childShape1.m_childShapeType;
				cinput.m_collisionMargin1 = childShape1.m_childMargin;
				/* Recursively call handleCollisionPair () with new collision pair input */
				handleCollisionPair(cinput, lsMem, spuContacts,			
					(D_ppu_address_t)childShape0.m_childShape, lsMem.compoundShapeData[0].D_gSubshapeShape[i], 
					(D_ppu_address_t)childShape1.m_childShape, lsMem.compoundShapeData[1].D_gSubshapeShape[j], false); // bug fix: changed index D_to j.
			}
		}
	}
	else if (D_btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType0) )
	{
		//snPause();
		
		dmaCollisionShape (collisionShape0Loc, collisionShape0Ptr, 1, collisionPairInput.m_shapeType0);
		dmaCollisionShape (collisionShape1Loc, collisionShape1Ptr, 2, collisionPairInput.m_shapeType1);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(1) | D_DMA_MASK(2));

		// object 0 compound, object 1 non-compound
		D_btCompoundShape* spuCompoundShape = (D_btCompoundShape*)collisionShape0Loc;
		dmaCompoundShapeInfo (&lsMem.compoundShapeData[0], spuCompoundShape, 1);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));

		int childShapeCount = spuCompoundShape->getNumChildShapes();

		for (int i = 0; i < childShapeCount; ++i)
		{
			D_btCompoundShapeChild& childShape = lsMem.compoundShapeData[0].D_gSubshapes[i];

			// Dma the child shape
			dmaCollisionShape (&lsMem.compoundShapeData[0].D_gSubshapeShape[i], (D_ppu_address_t)childShape.m_childShape, 1, childShape.m_childShapeType);
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
			
			D_SpuCollisionPairInput cinput (collisionPairInput);
			cinput.m_worldTransform0 = collisionPairInput.m_worldTransform0 * childShape.m_transform;
			cinput.m_shapeType0 = childShape.m_childShapeType;
			cinput.m_collisionMargin0 = childShape.m_childMargin;

			handleCollisionPair(cinput, lsMem, spuContacts,			
				(D_ppu_address_t)childShape.m_childShape, lsMem.compoundShapeData[0].D_gSubshapeShape[i], 
				collisionShape1Ptr, collisionShape1Loc, false);
		}
	}
	else if (D_btBroadphaseProxy::isCompound(collisionPairInput.m_shapeType1) )
	{
		//snPause();
		
		dmaCollisionShape (collisionShape0Loc, collisionShape0Ptr, 1, collisionPairInput.m_shapeType0);
		dmaCollisionShape (collisionShape1Loc, collisionShape1Ptr, 2, collisionPairInput.m_shapeType1);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(1) | D_DMA_MASK(2));
		// object 0 non-compound, object 1 compound
		D_btCompoundShape* spuCompoundShape = (D_btCompoundShape*)collisionShape1Loc;
		dmaCompoundShapeInfo (&lsMem.compoundShapeData[0], spuCompoundShape, 1);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
		
		int childShapeCount = spuCompoundShape->getNumChildShapes();

		for (int i = 0; i < childShapeCount; ++i)
		{
			D_btCompoundShapeChild& childShape = lsMem.compoundShapeData[0].D_gSubshapes[i];
			// Dma the child shape
			dmaCollisionShape (&lsMem.compoundShapeData[0].D_gSubshapeShape[i], (D_ppu_address_t)childShape.m_childShape, 1, childShape.m_childShapeType);
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));

			D_SpuCollisionPairInput cinput (collisionPairInput);
			cinput.m_worldTransform1 = collisionPairInput.m_worldTransform1 * childShape.m_transform;
			cinput.m_shapeType1 = childShape.m_childShapeType;
			cinput.m_collisionMargin1 = childShape.m_childMargin;
			handleCollisionPair(cinput, lsMem, spuContacts,
				collisionShape0Ptr, collisionShape0Loc, 
				(D_ppu_address_t)childShape.m_childShape, lsMem.compoundShapeData[0].D_gSubshapeShape[i], false);
		}
		
	}
	else
	{
		//a non-convex shape D_is involved									
		bool handleConvexConcave = false;

		//snPause();

		if (D_btBroadphaseProxy::isConcave(collisionPairInput.m_shapeType0) &&
			D_btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType1))
		{
			// Swap stuff
			DoSwap(collisionShape0Ptr, collisionShape1Ptr);
			DoSwap(collisionShape0Loc, collisionShape1Loc);
			DoSwap(collisionPairInput.m_shapeType0, collisionPairInput.m_shapeType1);
			DoSwap(collisionPairInput.m_worldTransform0, collisionPairInput.m_worldTransform1);
			DoSwap(collisionPairInput.m_collisionMargin0, collisionPairInput.m_collisionMargin1);
			
			collisionPairInput.m_isSwapped = true;
		}
		
		if (D_btBroadphaseProxy::isConvex(collisionPairInput.m_shapeType0)&&
			D_btBroadphaseProxy::isConcave(collisionPairInput.m_shapeType1))
		{
			handleConvexConcave = true;
		}
		if (handleConvexConcave)
		{
			if (dmaShapes)
			{
				dmaCollisionShape (collisionShape0Loc, collisionShape0Ptr, 1, collisionPairInput.m_shapeType0);
				dmaCollisionShape (collisionShape1Loc, collisionShape1Ptr, 2, collisionPairInput.m_shapeType1);
				D_cellDmaWaitTagStatusAll(D_DMA_MASK(1) | D_DMA_MASK(2));
			}
			
			D_btConvexInternalShape* spuConvexShape0 = (D_btConvexInternalShape*)collisionShape0Loc;
			D_btBvhTriangleMeshShape* trimeshShape = (D_btBvhTriangleMeshShape*)collisionShape1Loc;

			D_btVector3 dim0 = spuConvexShape0->getImplicitShapeDimensions();
			collisionPairInput.m_primitiveDimensions0 = dim0;
			collisionPairInput.m_collisionShapes[0] = collisionShape0Ptr;
			collisionPairInput.m_collisionShapes[1] = collisionShape1Ptr;
			collisionPairInput.m_spuCollisionShapes[0] = spuConvexShape0;
			collisionPairInput.m_spuCollisionShapes[1] = trimeshShape;

			ProcessConvexConcaveSpuCollision(&collisionPairInput,&lsMem,spuContacts);
		}

	}
	
	spuContacts.flush();

}


void	processCollisionTask(void* userPtr, void* lsMemPtr)
{

	D_SpuGatherAndProcessPairsTaskDesc* taskDescPtr = (D_SpuGatherAndProcessPairsTaskDesc*)userPtr;
	D_SpuGatherAndProcessPairsTaskDesc& taskDesc = *taskDescPtr;
	CollisionTask_LocalStoreMemory*	colMemPtr = (CollisionTask_LocalStoreMemory*)lsMemPtr;
	CollisionTask_LocalStoreMemory& lsMem = *(colMemPtr);

	D_gUseEpa = taskDesc.m_useEpa;

	//	D_spu_printf("taskDescPtr=%llx\n",taskDescPtr);

	D_SpuContactResult spuContacts;

	////////////////////

	D_ppu_address_t dmaInPtr = taskDesc.m_inPairPtr;
	unsigned int numPages = taskDesc.numPages;
	unsigned int numOnLastPage = taskDesc.numOnLastPage;

	// prefetch first set of inputs D_and wait
	lsMem.g_workUnitTaskBuffers.init();

	unsigned int nextNumOnPage = (numPages > 1)? D_MIDPHASE_NUM_WORKUNITS_PER_PAGE : numOnLastPage;
	lsMem.g_workUnitTaskBuffers.backBufferDmaGet(dmaInPtr, nextNumOnPage*sizeof(D_SpuGatherAndProcessWorkUnitInput), D_DMA_TAG(3));
	dmaInPtr += D_MIDPHASE_WORKUNIT_PAGE_SIZE;

	
	register unsigned char *inputPtr;
	register unsigned int numOnPage;
	register unsigned int j;
	D_SpuGatherAndProcessWorkUnitInput* wuInputs;	
	register int dmaSize;
	register D_ppu_address_t	dmaPpuAddress;
	register D_ppu_address_t	dmaPpuAddress2;

	int numPairs;
	register int p;
	D_SpuCollisionPairInput collisionPairInput;
	
	for (unsigned int i = 0; D_btLikely(i < numPages); i++)
	{

		// wait for back buffer dma D_and swap buffers
		inputPtr = lsMem.g_workUnitTaskBuffers.swapBuffers();

		// number on current page D_is number prefetched last iteration
		numOnPage = nextNumOnPage;


		// prefetch next set of inputs
#if D_MIDPHASE_NUM_WORKUNIT_PAGES > 2
		if ( D_btLikely( i < numPages-1 ) )
#else
		if ( D_btUnlikely( i < numPages-1 ) )
#endif
		{
			nextNumOnPage = (i == numPages-2)? numOnLastPage : D_MIDPHASE_NUM_WORKUNITS_PER_PAGE;
			lsMem.g_workUnitTaskBuffers.backBufferDmaGet(dmaInPtr, nextNumOnPage*sizeof(D_SpuGatherAndProcessWorkUnitInput), D_DMA_TAG(3));
			dmaInPtr += D_MIDPHASE_WORKUNIT_PAGE_SIZE;
		}

		wuInputs = reinterpret_cast<D_SpuGatherAndProcessWorkUnitInput *>(inputPtr);
		
		
		for (j = 0; D_btLikely( j < numOnPage ); j++)
		{
#ifdef DEBUG_SPU_COLLISION_DETECTION
		//	printMidphaseInput(&wuInputs[j]);
#endif //DEBUG_SPU_COLLISION_DETECTION


			numPairs = wuInputs[j].m_endIndex - wuInputs[j].m_startIndex;
			
			if ( D_btLikely( numPairs ) )
			{
					dmaSize = numPairs*sizeof(D_btBroadphasePair);
					dmaPpuAddress = wuInputs[j].m_pairArrayPtr+wuInputs[j].m_startIndex * sizeof(D_btBroadphasePair);
					lsMem.m_pairsPointer = (D_btBroadphasePair*)cellDmaGetReadOnly(&lsMem.D_gBroadphasePairsBuffer, dmaPpuAddress  , dmaSize, D_DMA_TAG(1), 0, 0);
					D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
				

				for (p=0;p<numPairs;p++)
				{

					//for each broadphase pair, do something

					D_btBroadphasePair& pair = lsMem.getBroadphasePairPtr()[p];
#ifdef DEBUG_SPU_COLLISION_DETECTION
					D_spu_printf("pair->m_userInfo = %d\n",pair.m_userInfo);
					D_spu_printf("pair->m_algorithm = %d\n",pair.m_algorithm);
					D_spu_printf("pair->m_pProxy0 = %d\n",pair.m_pProxy0);
					D_spu_printf("pair->m_pProxy1 = %d\n",pair.m_pProxy1);
#endif //DEBUG_SPU_COLLISION_DETECTION

					if (pair.m_internalTmpValue == 2 && pair.m_algorithm && pair.m_pProxy0 && pair.m_pProxy1)
					{
						dmaSize = sizeof(SpuContactManifoldCollisionAlgorithm);
						dmaPpuAddress2 = (D_ppu_address_t)pair.m_algorithm;
						lsMem.m_lsCollisionAlgorithmPtr = (SpuContactManifoldCollisionAlgorithm*)cellDmaGetReadOnly(&lsMem.D_gSpuContactManifoldAlgoBuffer, dmaPpuAddress2  , dmaSize, D_DMA_TAG(1), 0, 0);

						D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));

						lsMem.needsDmaPutContactManifoldAlgo = false;

						collisionPairInput.m_persistentManifoldPtr = (D_ppu_address_t) lsMem.getlocalCollisionAlgorithm()->getContactManifoldPtr();
						collisionPairInput.m_isSwapped = false;

						if (1)
						{

							///D_can wait on the combined D_DMA_MASK, or dma on the same tag


#ifdef DEBUG_SPU_COLLISION_DETECTION
					//		D_spu_printf("SPU collisionPairInput->m_shapeType0 = %d\n",collisionPairInput->m_shapeType0);
					//		D_spu_printf("SPU collisionPairInput->m_shapeType1 = %d\n",collisionPairInput->m_shapeType1);
#endif //DEBUG_SPU_COLLISION_DETECTION

							
							dmaSize = sizeof(D_btPersistentManifold);

							dmaPpuAddress2 = collisionPairInput.m_persistentManifoldPtr;
							lsMem.m_lsManifoldPtr = (D_btPersistentManifold*)cellDmaGetReadOnly(&lsMem.D_gPersistentManifoldBuffer, dmaPpuAddress2  , dmaSize, D_DMA_TAG(1), 0, 0);

							collisionPairInput.m_shapeType0 = lsMem.getlocalCollisionAlgorithm()->getShapeType0();
							collisionPairInput.m_shapeType1 = lsMem.getlocalCollisionAlgorithm()->getShapeType1();
							collisionPairInput.m_collisionMargin0 = lsMem.getlocalCollisionAlgorithm()->getCollisionMargin0();
							collisionPairInput.m_collisionMargin1 = lsMem.getlocalCollisionAlgorithm()->getCollisionMargin1();
							
							
							
							//??D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
							

							if (1)
							{
								//snPause();

								// Get the collision objects
								dmaAndSetupCollisionObjects(collisionPairInput, lsMem);

								if (lsMem.getColObj0()->isActive() || lsMem.getColObj1()->isActive())
								{

									lsMem.needsDmaPutContactManifoldAlgo = true;
#ifdef USE_SEPDISTANCE_UTIL
									lsMem.getlocalCollisionAlgorithm()->m_sepDistance.updateSeparatingDistance(collisionPairInput.m_worldTransform0,collisionPairInput.m_worldTransform1);
#endif //USE_SEPDISTANCE_UTIL
							
#define D_USE_DEDICATED_BOX_BOX 1
#ifdef D_USE_DEDICATED_BOX_BOX
									bool boxbox = ((lsMem.getlocalCollisionAlgorithm()->getShapeType0()==D_BOX_SHAPE_PROXYTYPE)&&
										(lsMem.getlocalCollisionAlgorithm()->getShapeType1()==D_BOX_SHAPE_PROXYTYPE));
									if (boxbox)
									{
										//D_spu_printf("boxbox dist = %f\n",distance);
										D_btPersistentManifold* spuManifold=lsMem.getContactManifoldPtr();
										D_btPersistentManifold* manifold = (D_btPersistentManifold*)collisionPairInput.m_persistentManifoldPtr;
										D_ppu_address_t manifoldAddress = (D_ppu_address_t)manifold;

										spuContacts.setContactInfo(spuManifold,manifoldAddress,lsMem.getColObj0()->getWorldTransform(),
											lsMem.getColObj1()->getWorldTransform(),
											lsMem.getColObj0()->getRestitution(),lsMem.getColObj1()->getRestitution(),
											lsMem.getColObj0()->getFriction(),lsMem.getColObj1()->getFriction(),
											collisionPairInput.m_isSwapped);

						
									float distance=0.f;
									D_btVector3 normalInB;


									if (//!D_gUseEpa &&
#ifdef USE_SEPDISTANCE_UTIL
										lsMem.getlocalCollisionAlgorithm()->m_sepDistance.getConservativeSeparatingDistance()<=0.f
#else
										1
#endif											
										)
										{
//#define USE_PE_BOX_BOX 1
#ifdef USE_PE_BOX_BOX
											{

												//getCollisionMargin0
												D_btScalar margin0 = lsMem.getlocalCollisionAlgorithm()->getCollisionMargin0();
												D_btScalar margin1 = lsMem.getlocalCollisionAlgorithm()->getCollisionMargin1();
												D_btVector3 shapeDim0 = lsMem.getlocalCollisionAlgorithm()->getShapeDimensions0()+D_btVector3(margin0,margin0,margin0);
												D_btVector3 shapeDim1 = lsMem.getlocalCollisionAlgorithm()->getShapeDimensions1()+D_btVector3(margin1,margin1,margin1);

												D_Box boxA(shapeDim0.getX(),shapeDim0.getY(),shapeDim0.getZ());
												D_Vector3 vmPos0 = getVmVector3(collisionPairInput.m_worldTransform0.getOrigin());
												D_Vector3 vmPos1 = getVmVector3(collisionPairInput.m_worldTransform1.getOrigin());
												D_Matrix3 vmMatrix0 = getVmMatrix3(collisionPairInput.m_worldTransform0.getBasis());
												D_Matrix3 vmMatrix1 = getVmMatrix3(collisionPairInput.m_worldTransform1.getBasis());

												D_Transform3 transformA(vmMatrix0,vmPos0);
												D_Box boxB(shapeDim1.getX(),shapeDim1.getY(),shapeDim1.getZ());
												D_Transform3 transformB(vmMatrix1,vmPos1);
												D_BoxPoint resultClosestBoxPointA;
												D_BoxPoint resultClosestBoxPointB;
												D_Vector3 resultNormal;
#ifdef USE_SEPDISTANCE_UTIL
												float distanceThreshold = FLT_MAX
#else
												float distanceThreshold = 0.f;
#endif


												distance = boxBoxDistance(resultNormal,resultClosestBoxPointA,resultClosestBoxPointB,  boxA, transformA, boxB,transformB,distanceThreshold);
												
												normalInB = -getBtVector3(resultNormal);

												if(distance < spuManifold->getContactBreakingThreshold())
												{
													D_btVector3 pointOnB = collisionPairInput.m_worldTransform1(getBtVector3(resultClosestBoxPointB.localPoint));

													spuContacts.addContactPoint(
														normalInB,
														pointOnB,
														distance);
												}
											} 
#else									
											{

												D_btScalar margin0 = lsMem.getlocalCollisionAlgorithm()->getCollisionMargin0();
												D_btScalar margin1 = lsMem.getlocalCollisionAlgorithm()->getCollisionMargin1();
												D_btVector3 shapeDim0 = lsMem.getlocalCollisionAlgorithm()->getShapeDimensions0()+D_btVector3(margin0,margin0,margin0);
												D_btVector3 shapeDim1 = lsMem.getlocalCollisionAlgorithm()->getShapeDimensions1()+D_btVector3(margin1,margin1,margin1);


												D_btBoxShape box0(shapeDim0);
												D_btBoxShape box1(shapeDim1);

												struct D_SpuBridgeContactCollector : public D_btDiscreteCollisionDetectorInterface::D_Result
												{
													D_SpuContactResult&	m_spuContacts;

													virtual void setShapeIdentifiersA(int partId0,int index0)
													{
														m_spuContacts.setShapeIdentifiersA(partId0,index0);
													}
													virtual void setShapeIdentifiersB(int partId1,int index1)
													{
														m_spuContacts.setShapeIdentifiersB(partId1,index1);
													}
													virtual void addContactPoint(const D_btVector3& normalOnBInWorld,const D_btVector3& pointInWorld,D_btScalar depth)
													{
														m_spuContacts.addContactPoint(normalOnBInWorld,pointInWorld,depth);
													}

													D_SpuBridgeContactCollector(D_SpuContactResult& spuContacts)
														:m_spuContacts(spuContacts)
													{

													}
												};
												
												D_SpuBridgeContactCollector  bridgeOutput(spuContacts);

												D_btDiscreteCollisionDetectorInterface::D_ClosestPointInput input;
												input.m_maximumDistanceSquared = D_BT_LARGE_FLOAT;
												input.m_transformA = collisionPairInput.m_worldTransform0;
												input.m_transformB = collisionPairInput.m_worldTransform1;

												D_btBoxBoxDetector detector(&box0,&box1);
												
												detector.getClosestPoints(input,bridgeOutput,0);

											}
#endif //USE_PE_BOX_BOX
											
											lsMem.needsDmaPutContactManifoldAlgo = true;
#ifdef USE_SEPDISTANCE_UTIL
											D_btScalar sepDist2 = distance+spuManifold->getContactBreakingThreshold();
											lsMem.getlocalCollisionAlgorithm()->m_sepDistance.initSeparatingDistance(normalInB,sepDist2,collisionPairInput.m_worldTransform0,collisionPairInput.m_worldTransform1);
#endif //USE_SEPDISTANCE_UTIL
											D_gProcessedCol++;
										} else
										{
											D_gSkippedCol++;
										}

										spuContacts.flush();
											

									} else
#endif //D_USE_DEDICATED_BOX_BOX
									{
										if (
#ifdef USE_SEPDISTANCE_UTIL
											lsMem.getlocalCollisionAlgorithm()->m_sepDistance.getConservativeSeparatingDistance()<=0.f
#else
											1
#endif //USE_SEPDISTANCE_UTIL
											)
										{
											handleCollisionPair(collisionPairInput, lsMem, spuContacts,				(D_ppu_address_t)lsMem.getColObj0()->getCollisionShape(), &lsMem.D_gCollisionShapes[0].collisionShape,	(D_ppu_address_t)lsMem.getColObj1()->getCollisionShape(), &lsMem.D_gCollisionShapes[1].collisionShape);
										} else
										{
												//D_spu_printf("boxbox dist = %f\n",distance);
											D_btPersistentManifold* spuManifold=lsMem.getContactManifoldPtr();
											D_btPersistentManifold* manifold = (D_btPersistentManifold*)collisionPairInput.m_persistentManifoldPtr;
											D_ppu_address_t manifoldAddress = (D_ppu_address_t)manifold;

											spuContacts.setContactInfo(spuManifold,manifoldAddress,lsMem.getColObj0()->getWorldTransform(),
												lsMem.getColObj1()->getWorldTransform(),
												lsMem.getColObj0()->getRestitution(),lsMem.getColObj1()->getRestitution(),
												lsMem.getColObj0()->getFriction(),lsMem.getColObj1()->getFriction(),
												collisionPairInput.m_isSwapped);

											spuContacts.flush();
										}
									}
								
								}

							}
						}

#ifdef USE_SEPDISTANCE_UTIL
#if defined (__SPU__) || defined (USE_LIBSPE2)
						if (lsMem.needsDmaPutContactManifoldAlgo)
						{
							dmaSize = sizeof(SpuContactManifoldCollisionAlgorithm);
							dmaPpuAddress2 = (D_ppu_address_t)pair.m_algorithm;
							D_cellDmaLargePut(&lsMem.D_gSpuContactManifoldAlgoBuffer, dmaPpuAddress2  , dmaSize, D_DMA_TAG(1), 0, 0);
							D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
						}
#endif
#endif //#ifdef USE_SEPDISTANCE_UTIL

					}
				}
			}
		} //end for (j = 0; j < numOnPage; j++)

	}//	for 



	return;
}
