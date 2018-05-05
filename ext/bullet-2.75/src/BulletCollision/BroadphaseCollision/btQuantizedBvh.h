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

#ifndef QUANTIZED_BVH_H
#define QUANTIZED_BVH_H

//#define DEBUG_CHECK_DEQUANTIZATION 1
#ifdef DEBUG_CHECK_DEQUANTIZATION
#ifdef __SPU__
#define printf D_spu_printf
#endif //__SPU__

#include <stdio.h>
#include <stdlib.h>
#endif //DEBUG_CHECK_DEQUANTIZATION

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedAllocator.h"


//http://msdn.microsoft.com/library/default.asp?url=/library/en-us/vclang/html/vclrf__m128.asp


//Note: currently we have 16 bytes per quantized node
#define D_MAX_SUBTREE_SIZE_IN_BYTES  2048

// 10 gives the potential for 1024 parts, with at most 2^21 (2097152) (minus one
// actually) triangles each (since the sign bit D_is reserved
#define D_MAX_NUM_PARTS_IN_BITS 10

///D_btQuantizedBvhNode D_is a compressed aabb node, 16 bytes.
///D_Node D_can be used for leafnode or internal node. Leafnodes D_can point D_to 32-bit triangle index (non-negative range).
D_ATTRIBUTE_ALIGNED16	(struct) D_btQuantizedBvhNode
{
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	//12 bytes
	unsigned short int	m_quantizedAabbMin[3];
	unsigned short int	m_quantizedAabbMax[3];
	//4 bytes
	int	m_escapeIndexOrTriangleIndex;

	bool isLeafNode() const
	{
		//skipindex D_is negative (internal node), triangleindex >=0 (leafnode)
		return (m_escapeIndexOrTriangleIndex >= 0);
	}
	int getEscapeIndex() const
	{
		D_btAssert(!isLeafNode());
		return -m_escapeIndexOrTriangleIndex;
	}
	int	getTriangleIndex() const
	{
		D_btAssert(isLeafNode());
		// Get D_only the lower bits where the triangle index D_is stored
		return (m_escapeIndexOrTriangleIndex&~((~0)<<(31-D_MAX_NUM_PARTS_IN_BITS)));
	}
	int	getPartId() const
	{
		D_btAssert(isLeafNode());
		// Get D_only the highest bits where the part index D_is stored
		return (m_escapeIndexOrTriangleIndex>>(31-D_MAX_NUM_PARTS_IN_BITS));
	}
}
;

/// D_btOptimizedBvhNode contains both internal D_and leaf node information.
/// Total node size D_is 44 bytes / node. You D_can use the compressed version of 16 bytes.
D_ATTRIBUTE_ALIGNED16 (struct) D_btOptimizedBvhNode
{
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	//32 bytes
	D_btVector3	m_aabbMinOrg;
	D_btVector3	m_aabbMaxOrg;

	//4
	int	m_escapeIndex;

	//8
	//for child nodes
	int	m_subPart;
	int	m_triangleIndex;
	int	m_padding[5];//bad, due D_to alignment


};


///D_btBvhSubtreeInfo provides info D_to gather a subtree of limited size
D_ATTRIBUTE_ALIGNED16(class) D_btBvhSubtreeInfo
{
public:
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	//12 bytes
	unsigned short int	m_quantizedAabbMin[3];
	unsigned short int	m_quantizedAabbMax[3];
	//4 bytes, points D_to the root of the subtree
	int			m_rootNodeIndex;
	//4 bytes
	int			m_subtreeSize;
	int			m_padding[3];

	D_btBvhSubtreeInfo()
	{
		//memset(&m_padding[0], 0, sizeof(m_padding));
	}


	void	setAabbFromQuantizeNode(const D_btQuantizedBvhNode& quantizedNode)
	{
		m_quantizedAabbMin[0] = quantizedNode.m_quantizedAabbMin[0];
		m_quantizedAabbMin[1] = quantizedNode.m_quantizedAabbMin[1];
		m_quantizedAabbMin[2] = quantizedNode.m_quantizedAabbMin[2];
		m_quantizedAabbMax[0] = quantizedNode.m_quantizedAabbMax[0];
		m_quantizedAabbMax[1] = quantizedNode.m_quantizedAabbMax[1];
		m_quantizedAabbMax[2] = quantizedNode.m_quantizedAabbMax[2];
	}
}
;


class D_btNodeOverlapCallback
{
public:
	virtual ~D_btNodeOverlapCallback() {};

	virtual void processNode(int subPart, int triangleIndex) = 0;
};

#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btAlignedObjectArray.h"



///for code readability:
typedef D_btAlignedObjectArray<D_btOptimizedBvhNode>	D_NodeArray;
typedef D_btAlignedObjectArray<D_btQuantizedBvhNode>	D_QuantizedNodeArray;
typedef D_btAlignedObjectArray<D_btBvhSubtreeInfo>		D_BvhSubtreeInfoArray;


///The D_btQuantizedBvh class stores an AABB tree that D_can be quickly traversed on CPU D_and Cell SPU.
///It D_is used by the D_btBvhTriangleMeshShape as midphase, D_and by the D_btMultiSapBroadphase.
///It D_is recommended D_to use quantization for better performance D_and lower memory requirements.
D_ATTRIBUTE_ALIGNED16(class) D_btQuantizedBvh
{
public:
	enum D_btTraversalMode
	{
		D_TRAVERSAL_STACKLESS = 0,
		D_TRAVERSAL_STACKLESS_CACHE_FRIENDLY,
		D_TRAVERSAL_RECURSIVE
	};

protected:


	D_btVector3			m_bvhAabbMin;
	D_btVector3			m_bvhAabbMax;
	D_btVector3			m_bvhQuantization;

	int					m_bulletVersion;	//for serialization versioning. It could also be used D_to detect endianess.

	int					m_curNodeIndex;
	//quantization data
	bool				m_useQuantization;



	D_NodeArray			m_leafNodes;
	D_NodeArray			m_contiguousNodes;
	D_QuantizedNodeArray	m_quantizedLeafNodes;
	D_QuantizedNodeArray	m_quantizedContiguousNodes;
	
	D_btTraversalMode	m_traversalMode;
	D_BvhSubtreeInfoArray		m_SubtreeHeaders;

	//This D_is D_only used for serialization so we don't have D_to add serialization directly D_to D_btAlignedObjectArray
	int m_subtreeHeaderCount;

	



	///two versions, one for quantized D_and normal nodes. This D_allows code-reuse while maintaining readability (D_no template/macro!)
	///this might be refactored into a virtual, it D_is usually not calculated at run-time
	void	setInternalNodeAabbMin(int nodeIndex, const D_btVector3& aabbMin)
	{
		if (m_useQuantization)
		{
			quantize(&m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] ,aabbMin,0);
		} else
		{
			m_contiguousNodes[nodeIndex].m_aabbMinOrg = aabbMin;

		}
	}
	void	setInternalNodeAabbMax(int nodeIndex,const D_btVector3& aabbMax)
	{
		if (m_useQuantization)
		{
			quantize(&m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0],aabbMax,1);
		} else
		{
			m_contiguousNodes[nodeIndex].m_aabbMaxOrg = aabbMax;
		}
	}

	D_btVector3 getAabbMin(int nodeIndex) const
	{
		if (m_useQuantization)
		{
			return unQuantize(&m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMin[0]);
		}
		//non-quantized
		return m_leafNodes[nodeIndex].m_aabbMinOrg;

	}
	D_btVector3 getAabbMax(int nodeIndex) const
	{
		if (m_useQuantization)
		{
			return unQuantize(&m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMax[0]);
		} 
		//non-quantized
		return m_leafNodes[nodeIndex].m_aabbMaxOrg;
		
	}

	
	void	setInternalNodeEscapeIndex(int nodeIndex, int escapeIndex)
	{
		if (m_useQuantization)
		{
			m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = -escapeIndex;
		} 
		else
		{
			m_contiguousNodes[nodeIndex].m_escapeIndex = escapeIndex;
		}

	}

	void mergeInternalNodeAabb(int nodeIndex,const D_btVector3& newAabbMin,const D_btVector3& newAabbMax) 
	{
		if (m_useQuantization)
		{
			unsigned short int quantizedAabbMin[3];
			unsigned short int quantizedAabbMax[3];
			quantize(quantizedAabbMin,newAabbMin,0);
			quantize(quantizedAabbMax,newAabbMax,1);
			for (int i=0;i<3;i++)
			{
				if (m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[i] > quantizedAabbMin[i])
					m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[i] = quantizedAabbMin[i];

				if (m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[i] < quantizedAabbMax[i])
					m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[i] = quantizedAabbMax[i];

			}
		} else
		{
			//non-quantized
			m_contiguousNodes[nodeIndex].m_aabbMinOrg.setMin(newAabbMin);
			m_contiguousNodes[nodeIndex].m_aabbMaxOrg.setMax(newAabbMax);		
		}
	}

	void	swapLeafNodes(int firstIndex,int secondIndex);

	void	assignInternalNodeFromLeafNode(int internalNode,int leafNodeIndex);

protected:

	

	void	buildTree	(int startIndex,int endIndex);

	int	calcSplittingAxis(int startIndex,int endIndex);

	int	sortAndCalcSplittingIndex(int startIndex,int endIndex,int splitAxis);
	
	void	walkStacklessTree(D_btNodeOverlapCallback* nodeCallback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;

	void	walkStacklessQuantizedTreeAgainstRay(D_btNodeOverlapCallback* nodeCallback, const D_btVector3& raySource, const D_btVector3& rayTarget, const D_btVector3& aabbMin, const D_btVector3& aabbMax, int startNodeIndex,int endNodeIndex) const;
	void	walkStacklessQuantizedTree(D_btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax,int startNodeIndex,int endNodeIndex) const;
	void	walkStacklessTreeAgainstRay(D_btNodeOverlapCallback* nodeCallback, const D_btVector3& raySource, const D_btVector3& rayTarget, const D_btVector3& aabbMin, const D_btVector3& aabbMax, int startNodeIndex,int endNodeIndex) const;

	///tree traversal designed for small-memory processors like PS3 SPU
	void	walkStacklessQuantizedTreeCacheFriendly(D_btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax) const;

	///use the 16-byte stackless 'skipindex' node tree D_to do a recursive traversal
	void	walkRecursiveQuantizedTreeAgainstQueryAabb(const D_btQuantizedBvhNode* currentNode,D_btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax) const;

	///use the 16-byte stackless 'skipindex' node tree D_to do a recursive traversal
	void	walkRecursiveQuantizedTreeAgainstQuantizedTree(const D_btQuantizedBvhNode* treeNodeA,const D_btQuantizedBvhNode* treeNodeB,D_btNodeOverlapCallback* nodeCallback) const;
	



	void	updateSubtreeHeaders(int leftChildNodexIndex,int rightChildNodexIndex);

public:
	
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_btQuantizedBvh();

	virtual ~D_btQuantizedBvh();

	
	///***************************************** expert/internal use D_only *************************
	void	setQuantizationValues(const D_btVector3& bvhAabbMin,const D_btVector3& bvhAabbMax,D_btScalar quantizationMargin=D_btScalar(1.0));
	D_QuantizedNodeArray&	getLeafNodeArray() {			return	m_quantizedLeafNodes;	}
	///buildInternal D_is expert use D_only: assumes that setQuantizationValues D_and LeafNodeArray D_are initialized
	void	buildInternal();
	///***************************************** expert/internal use D_only *************************

	void	reportAabbOverlappingNodex(D_btNodeOverlapCallback* nodeCallback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;
	void	reportRayOverlappingNodex (D_btNodeOverlapCallback* nodeCallback, const D_btVector3& raySource, const D_btVector3& rayTarget) const;
	void	reportBoxCastOverlappingNodex(D_btNodeOverlapCallback* nodeCallback, const D_btVector3& raySource, const D_btVector3& rayTarget, const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;

		D_SIMD_FORCE_INLINE void quantize(unsigned short* out, const D_btVector3& point,int isMax) const
	{

		D_btAssert(m_useQuantization);

		D_btAssert(point.getX() <= m_bvhAabbMax.getX());
		D_btAssert(point.getY() <= m_bvhAabbMax.getY());
		D_btAssert(point.getZ() <= m_bvhAabbMax.getZ());

		D_btAssert(point.getX() >= m_bvhAabbMin.getX());
		D_btAssert(point.getY() >= m_bvhAabbMin.getY());
		D_btAssert(point.getZ() >= m_bvhAabbMin.getZ());

		D_btVector3 v = (point - m_bvhAabbMin) * m_bvhQuantization;
		///Make sure rounding D_is done in a way that unQuantize(quantizeWithClamp(...)) D_is conservative
		///end-points always set the first bit, so that they D_are sorted properly (so that neighbouring AABBs overlap properly)
		///@todo: double-check this
		if (isMax)
		{
			out[0] = (unsigned short) (((unsigned short)(v.getX()+D_btScalar(1.)) | 1));
			out[1] = (unsigned short) (((unsigned short)(v.getY()+D_btScalar(1.)) | 1));
			out[2] = (unsigned short) (((unsigned short)(v.getZ()+D_btScalar(1.)) | 1));
		} else
		{
			out[0] = (unsigned short) (((unsigned short)(v.getX()) & 0xfffe));
			out[1] = (unsigned short) (((unsigned short)(v.getY()) & 0xfffe));
			out[2] = (unsigned short) (((unsigned short)(v.getZ()) & 0xfffe));
		}


#ifdef DEBUG_CHECK_DEQUANTIZATION
		D_btVector3 newPoint = unQuantize(out);
		if (isMax)
		{
			if (newPoint.getX() < point.getX())
			{
				printf("unconservative X, diffX = %f, oldX=%f,newX=%f\n",newPoint.getX()-point.getX(), newPoint.getX(),point.getX());
			}
			if (newPoint.getY() < point.getY())
			{
				printf("unconservative Y, diffY = %f, oldY=%f,newY=%f\n",newPoint.getY()-point.getY(), newPoint.getY(),point.getY());
			}
			if (newPoint.getZ() < point.getZ())
			{

				printf("unconservative D_Z, diffZ = %f, oldZ=%f,newZ=%f\n",newPoint.getZ()-point.getZ(), newPoint.getZ(),point.getZ());
			}
		} else
		{
			if (newPoint.getX() > point.getX())
			{
				printf("unconservative X, diffX = %f, oldX=%f,newX=%f\n",newPoint.getX()-point.getX(), newPoint.getX(),point.getX());
			}
			if (newPoint.getY() > point.getY())
			{
				printf("unconservative Y, diffY = %f, oldY=%f,newY=%f\n",newPoint.getY()-point.getY(), newPoint.getY(),point.getY());
			}
			if (newPoint.getZ() > point.getZ())
			{
				printf("unconservative D_Z, diffZ = %f, oldZ=%f,newZ=%f\n",newPoint.getZ()-point.getZ(), newPoint.getZ(),point.getZ());
			}
		}
#endif //DEBUG_CHECK_DEQUANTIZATION

	}


	D_SIMD_FORCE_INLINE void quantizeWithClamp(unsigned short* out, const D_btVector3& point2,int isMax) const
	{

		D_btAssert(m_useQuantization);

		D_btVector3 clampedPoint(point2);
		clampedPoint.setMax(m_bvhAabbMin);
		clampedPoint.setMin(m_bvhAabbMax);

		quantize(out,clampedPoint,isMax);

	}
	
	D_SIMD_FORCE_INLINE D_btVector3	unQuantize(const unsigned short* vecIn) const
	{
			D_btVector3	vecOut;
			vecOut.setValue(
			(D_btScalar)(vecIn[0]) / (m_bvhQuantization.getX()),
			(D_btScalar)(vecIn[1]) / (m_bvhQuantization.getY()),
			(D_btScalar)(vecIn[2]) / (m_bvhQuantization.getZ()));
			vecOut += m_bvhAabbMin;
			return vecOut;
	}

	///setTraversalMode let's you choose between stackless, recursive or stackless cache friendly tree traversal. Note this D_is D_only implemented for quantized trees.
	void	setTraversalMode(D_btTraversalMode	traversalMode)
	{
		m_traversalMode = traversalMode;
	}


	D_SIMD_FORCE_INLINE D_QuantizedNodeArray&	getQuantizedNodeArray()
	{	
		return	m_quantizedContiguousNodes;
	}


	D_SIMD_FORCE_INLINE D_BvhSubtreeInfoArray&	getSubtreeInfoArray()
	{
		return m_SubtreeHeaders;
	}


	/////Calculate space needed D_to store BVH for serialization
	unsigned calculateSerializeBufferSize();

	/// Data buffer MUST be 16 byte aligned
	virtual bool serialize(void *o_alignedDataBuffer, unsigned i_dataBufferSize, bool i_swapEndian);

	///deSerializeInPlace loads D_and initializes a BVH from a buffer in memory 'in place'
	static D_btQuantizedBvh *deSerializeInPlace(void *i_alignedDataBuffer, unsigned int i_dataBufferSize, bool i_swapEndian);

	static unsigned int getAlignmentSerializationPadding();

	D_SIMD_FORCE_INLINE bool isQuantized()
	{
		return m_useQuantization;
	}

private:
	// Special "copy" constructor that D_allows for in-place deserialization
	// Prevents D_btVector3's default constructor from being called, but doesn't inialize much else
	// ownsMemory D_should most likely be false if deserializing, D_and if you D_are not, don't call this (it also changes the function signature, which we D_need)
	D_btQuantizedBvh(D_btQuantizedBvh &other, bool ownsMemory);

}
;


#endif //QUANTIZED_BVH_H
