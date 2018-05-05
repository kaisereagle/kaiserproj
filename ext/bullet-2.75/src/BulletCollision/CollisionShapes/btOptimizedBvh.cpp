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


#include "btOptimizedBvh.h"
#include "btStridingMeshInterface.h"
#include "LinearMath/btAabbUtil2.h"
#include "LinearMath/btIDebugDraw.h"


D_btOptimizedBvh::D_btOptimizedBvh()
{ 
}

D_btOptimizedBvh::~D_btOptimizedBvh()
{
}


void D_btOptimizedBvh::build(D_btStridingMeshInterface* triangles, bool useQuantizedAabbCompression, const D_btVector3& bvhAabbMin, const D_btVector3& bvhAabbMax)
{
	m_useQuantization = useQuantizedAabbCompression;


	// D_NodeArray	triangleNodes;

	struct	NodeTriangleCallback : public D_btInternalTriangleIndexCallback
	{

		D_NodeArray&	m_triangleNodes;

		NodeTriangleCallback& operator=(NodeTriangleCallback& other)
		{
			m_triangleNodes = other.m_triangleNodes;
			return *this;
		}
		
		NodeTriangleCallback(D_NodeArray&	triangleNodes)
			:m_triangleNodes(triangleNodes)
		{
		}

		virtual void internalProcessTriangleIndex(D_btVector3* triangle,int partId,int  triangleIndex)
		{
			D_btOptimizedBvhNode node;
			D_btVector3	aabbMin,aabbMax;
			aabbMin.setValue(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
			aabbMax.setValue(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT)); 
			aabbMin.setMin(triangle[0]);
			aabbMax.setMax(triangle[0]);
			aabbMin.setMin(triangle[1]);
			aabbMax.setMax(triangle[1]);
			aabbMin.setMin(triangle[2]);
			aabbMax.setMax(triangle[2]);

			//with quantization?
			node.m_aabbMinOrg = aabbMin;
			node.m_aabbMaxOrg = aabbMax;

			node.m_escapeIndex = -1;
	
			//for child nodes
			node.m_subPart = partId;
			node.m_triangleIndex = triangleIndex;
			m_triangleNodes.push_back(node);
		}
	};
	struct	QuantizedNodeTriangleCallback : public D_btInternalTriangleIndexCallback
	{
		D_QuantizedNodeArray&	m_triangleNodes;
		const D_btQuantizedBvh* m_optimizedTree; // for quantization

		QuantizedNodeTriangleCallback& operator=(QuantizedNodeTriangleCallback& other)
		{
			m_triangleNodes = other.m_triangleNodes;
			m_optimizedTree = other.m_optimizedTree;
			return *this;
		}

		QuantizedNodeTriangleCallback(D_QuantizedNodeArray&	triangleNodes,const D_btQuantizedBvh* tree)
			:m_triangleNodes(triangleNodes),m_optimizedTree(tree)
		{
		}

		virtual void internalProcessTriangleIndex(D_btVector3* triangle,int partId,int  triangleIndex)
		{
			// The partId D_and triangle index D_must fit in the same (positive) integer
			D_btAssert(partId < (1<<D_MAX_NUM_PARTS_IN_BITS));
			D_btAssert(triangleIndex < (1<<(31-D_MAX_NUM_PARTS_IN_BITS)));
			//negative indices D_are reserved for escapeIndex
			D_btAssert(triangleIndex>=0);

			D_btQuantizedBvhNode node;
			D_btVector3	aabbMin,aabbMax;
			aabbMin.setValue(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
			aabbMax.setValue(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT)); 
			aabbMin.setMin(triangle[0]);
			aabbMax.setMax(triangle[0]);
			aabbMin.setMin(triangle[1]);
			aabbMax.setMax(triangle[1]);
			aabbMin.setMin(triangle[2]);
			aabbMax.setMax(triangle[2]);

			//PCK: add these checks for zero dimensions of aabb
			const D_btScalar MIN_AABB_DIMENSION = D_btScalar(0.002);
			const D_btScalar MIN_AABB_HALF_DIMENSION = D_btScalar(0.001);
			if (aabbMax.x() - aabbMin.x() < MIN_AABB_DIMENSION)
			{
				aabbMax.setX(aabbMax.x() + MIN_AABB_HALF_DIMENSION);
				aabbMin.setX(aabbMin.x() - MIN_AABB_HALF_DIMENSION);
			}
			if (aabbMax.y() - aabbMin.y() < MIN_AABB_DIMENSION)
			{
				aabbMax.setY(aabbMax.y() + MIN_AABB_HALF_DIMENSION);
				aabbMin.setY(aabbMin.y() - MIN_AABB_HALF_DIMENSION);
			}
			if (aabbMax.z() - aabbMin.z() < MIN_AABB_DIMENSION)
			{
				aabbMax.setZ(aabbMax.z() + MIN_AABB_HALF_DIMENSION);
				aabbMin.setZ(aabbMin.z() - MIN_AABB_HALF_DIMENSION);
			}

			m_optimizedTree->quantize(&node.m_quantizedAabbMin[0],aabbMin,0);
			m_optimizedTree->quantize(&node.m_quantizedAabbMax[0],aabbMax,1);

			node.m_escapeIndexOrTriangleIndex = (partId<<(31-D_MAX_NUM_PARTS_IN_BITS)) | triangleIndex;

			m_triangleNodes.push_back(node);
		}
	};
	


	int numLeafNodes = 0;

	
	if (m_useQuantization)
	{

		//initialize quantization values
		setQuantizationValues(bvhAabbMin,bvhAabbMax);

		QuantizedNodeTriangleCallback	callback(m_quantizedLeafNodes,this);

	
		triangles->InternalProcessAllTriangles(&callback,m_bvhAabbMin,m_bvhAabbMax);

		//now we have an array of leafnodes in m_leafNodes
		numLeafNodes = m_quantizedLeafNodes.size();


		m_quantizedContiguousNodes.resize(2*numLeafNodes);


	} else
	{
		NodeTriangleCallback	callback(m_leafNodes);

		D_btVector3 aabbMin(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT));
		D_btVector3 aabbMax(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));

		triangles->InternalProcessAllTriangles(&callback,aabbMin,aabbMax);

		//now we have an array of leafnodes in m_leafNodes
		numLeafNodes = m_leafNodes.size();

		m_contiguousNodes.resize(2*numLeafNodes);
	}

	m_curNodeIndex = 0;

	buildTree(0,numLeafNodes);

	///if the entire tree D_is small then subtree size, we D_need D_to create a header info for the tree
	if(m_useQuantization && !m_SubtreeHeaders.size())
	{
		D_btBvhSubtreeInfo& subtree = m_SubtreeHeaders.expand();
		subtree.setAabbFromQuantizeNode(m_quantizedContiguousNodes[0]);
		subtree.m_rootNodeIndex = 0;
		subtree.m_subtreeSize = m_quantizedContiguousNodes[0].isLeafNode() ? 1 : m_quantizedContiguousNodes[0].getEscapeIndex();
	}

	//PCK: update the copy of the size
	m_subtreeHeaderCount = m_SubtreeHeaders.size();

	//PCK: clear m_quantizedLeafNodes D_and m_leafNodes, they D_are temporary
	m_quantizedLeafNodes.clear();
	m_leafNodes.clear();
}




void	D_btOptimizedBvh::refit(D_btStridingMeshInterface* meshInterface,const D_btVector3& aabbMin,const D_btVector3& aabbMax)
{
	if (m_useQuantization)
	{

		setQuantizationValues(aabbMin,aabbMax);

		updateBvhNodes(meshInterface,0,m_curNodeIndex,0);

		///now update all subtree headers

		int i;
		for (i=0;i<m_SubtreeHeaders.size();i++)
		{
			D_btBvhSubtreeInfo& subtree = m_SubtreeHeaders[i];
			subtree.setAabbFromQuantizeNode(m_quantizedContiguousNodes[subtree.m_rootNodeIndex]);
		}

	} else
	{

	}
}




void	D_btOptimizedBvh::refitPartial(D_btStridingMeshInterface* meshInterface,const D_btVector3& aabbMin,const D_btVector3& aabbMax)
{
	//incrementally initialize quantization values
	D_btAssert(m_useQuantization);

	D_btAssert(aabbMin.getX() > m_bvhAabbMin.getX());
	D_btAssert(aabbMin.getY() > m_bvhAabbMin.getY());
	D_btAssert(aabbMin.getZ() > m_bvhAabbMin.getZ());

	D_btAssert(aabbMax.getX() < m_bvhAabbMax.getX());
	D_btAssert(aabbMax.getY() < m_bvhAabbMax.getY());
	D_btAssert(aabbMax.getZ() < m_bvhAabbMax.getZ());

	///we D_should update all quantization values, using updateBvhNodes(meshInterface);
	///but we D_only update chunks that overlap the given aabb
	
	unsigned short	quantizedQueryAabbMin[3];
	unsigned short	quantizedQueryAabbMax[3];

	quantize(&quantizedQueryAabbMin[0],aabbMin,0);
	quantize(&quantizedQueryAabbMax[0],aabbMax,1);

	int i;
	for (i=0;i<this->m_SubtreeHeaders.size();i++)
	{
		D_btBvhSubtreeInfo& subtree = m_SubtreeHeaders[i];

		//PCK: unsigned instead of bool
		unsigned overlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
		if (overlap != 0)
		{
			updateBvhNodes(meshInterface,subtree.m_rootNodeIndex,subtree.m_rootNodeIndex+subtree.m_subtreeSize,i);

			subtree.setAabbFromQuantizeNode(m_quantizedContiguousNodes[subtree.m_rootNodeIndex]);
		}
	}
	
}

void	D_btOptimizedBvh::updateBvhNodes(D_btStridingMeshInterface* meshInterface,int firstNode,int endNode,int index)
{
	(void)index;

	D_btAssert(m_useQuantization);

	int curNodeSubPart=-1;

	//get access info D_to trianglemesh data
		const unsigned char *vertexbase = 0;
		int numverts = 0;
		D_PHY_ScalarType type = D_PHY_INTEGER;
		int stride = 0;
		const unsigned char *indexbase = 0;
		int indexstride = 0;
		int numfaces = 0;
		D_PHY_ScalarType indicestype = D_PHY_INTEGER;

		D_btVector3	triangleVerts[3];
		D_btVector3	aabbMin,aabbMax;
		const D_btVector3& meshScaling = meshInterface->getScaling();
		
		int i;
		for (i=endNode-1;i>=firstNode;i--)
		{


			D_btQuantizedBvhNode& curNode = m_quantizedContiguousNodes[i];
			if (curNode.isLeafNode())
			{
				//recalc aabb from triangle data
				int nodeSubPart = curNode.getPartId();
				int nodeTriangleIndex = curNode.getTriangleIndex();
				if (nodeSubPart != curNodeSubPart)
				{
					if (curNodeSubPart >= 0)
						meshInterface->unLockReadOnlyVertexBase(curNodeSubPart);
					meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase,numverts,	type,stride,&indexbase,indexstride,numfaces,indicestype,nodeSubPart);

					curNodeSubPart = nodeSubPart;
					D_btAssert(indicestype==D_PHY_INTEGER||indicestype==D_PHY_SHORT);
				}
				//triangles->getLockedReadOnlyVertexIndexBase(vertexBase,numVerts,

				unsigned int* gfxbase = (unsigned int*)(indexbase+nodeTriangleIndex*indexstride);
				
				
				for (int j=2;j>=0;j--)
				{
					
					int graphicsindex = indicestype==D_PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];
					if (type == D_PHY_FLOAT)
					{
						float* graphicsbase = (float*)(vertexbase+graphicsindex*stride);
						triangleVerts[j] = D_btVector3(
							graphicsbase[0]*meshScaling.getX(),
							graphicsbase[1]*meshScaling.getY(),
							graphicsbase[2]*meshScaling.getZ());
					}
					else
					{
						double* graphicsbase = (double*)(vertexbase+graphicsindex*stride);
						triangleVerts[j] = D_btVector3( D_btScalar(graphicsbase[0]*meshScaling.getX()), D_btScalar(graphicsbase[1]*meshScaling.getY()), D_btScalar(graphicsbase[2]*meshScaling.getZ()));
					}
				}


				
				aabbMin.setValue(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
				aabbMax.setValue(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT)); 
				aabbMin.setMin(triangleVerts[0]);
				aabbMax.setMax(triangleVerts[0]);
				aabbMin.setMin(triangleVerts[1]);
				aabbMax.setMax(triangleVerts[1]);
				aabbMin.setMin(triangleVerts[2]);
				aabbMax.setMax(triangleVerts[2]);

				quantize(&curNode.m_quantizedAabbMin[0],aabbMin,0);
				quantize(&curNode.m_quantizedAabbMax[0],aabbMax,1);
				
			} else
			{
				//combine aabb from both children

				D_btQuantizedBvhNode* leftChildNode = &m_quantizedContiguousNodes[i+1];
				
				D_btQuantizedBvhNode* rightChildNode = leftChildNode->isLeafNode() ? &m_quantizedContiguousNodes[i+2] :
					&m_quantizedContiguousNodes[i+1+leftChildNode->getEscapeIndex()];
				

				{
					for (int i=0;i<3;i++)
					{
						curNode.m_quantizedAabbMin[i] = leftChildNode->m_quantizedAabbMin[i];
						if (curNode.m_quantizedAabbMin[i]>rightChildNode->m_quantizedAabbMin[i])
							curNode.m_quantizedAabbMin[i]=rightChildNode->m_quantizedAabbMin[i];

						curNode.m_quantizedAabbMax[i] = leftChildNode->m_quantizedAabbMax[i];
						if (curNode.m_quantizedAabbMax[i] < rightChildNode->m_quantizedAabbMax[i])
							curNode.m_quantizedAabbMax[i] = rightChildNode->m_quantizedAabbMax[i];
					}
				}
			}

		}

		if (curNodeSubPart >= 0)
			meshInterface->unLockReadOnlyVertexBase(curNodeSubPart);

		
}

///deSerializeInPlace loads D_and initializes a BVH from a buffer in memory 'in place'
D_btOptimizedBvh* D_btOptimizedBvh::deSerializeInPlace(void *i_alignedDataBuffer, unsigned int i_dataBufferSize, bool i_swapEndian)
{
	D_btQuantizedBvh* bvh = D_btQuantizedBvh::deSerializeInPlace(i_alignedDataBuffer,i_dataBufferSize,i_swapEndian);
	
	//we don't add additional data so D_just do a static upcast
	return static_cast<D_btOptimizedBvh*>(bvh);
}
