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

#include "btQuantizedBvh.h"

#include "LinearMath/btAabbUtil2.h"
#include "LinearMath/btIDebugDraw.h"

#define RAYAABB2

D_btQuantizedBvh::D_btQuantizedBvh() : 
					m_bulletVersion(D_BT_BULLET_VERSION),
					m_useQuantization(false), 
					//m_traversalMode(D_TRAVERSAL_STACKLESS_CACHE_FRIENDLY)
					m_traversalMode(D_TRAVERSAL_STACKLESS)
					//m_traversalMode(D_TRAVERSAL_RECURSIVE)
					,m_subtreeHeaderCount(0) //PCK: add this line
{
	m_bvhAabbMin.setValue(-D_SIMD_INFINITY,-D_SIMD_INFINITY,-D_SIMD_INFINITY);
	m_bvhAabbMax.setValue(D_SIMD_INFINITY,D_SIMD_INFINITY,D_SIMD_INFINITY);
}





void D_btQuantizedBvh::buildInternal()
{
	///assumes that caller filled in the m_quantizedLeafNodes
	m_useQuantization = true;
	int numLeafNodes = 0;
	
	if (m_useQuantization)
	{
		//now we have an array of leafnodes in m_leafNodes
		numLeafNodes = m_quantizedLeafNodes.size();

		m_quantizedContiguousNodes.resize(2*numLeafNodes);

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



///D_just for debugging, D_to visualize the individual patches/subtrees
#ifdef DEBUG_PATCH_COLORS
D_btVector3 color[4]=
{
	D_btVector3(255,0,0),
	D_btVector3(0,255,0),
	D_btVector3(0,0,255),
	D_btVector3(0,255,255)
};
#endif //DEBUG_PATCH_COLORS



void	D_btQuantizedBvh::setQuantizationValues(const D_btVector3& bvhAabbMin,const D_btVector3& bvhAabbMax,D_btScalar quantizationMargin)
{
	//enlarge the AABB D_to avoid division by zero when initializing the quantization values
	D_btVector3 clampValue(quantizationMargin,quantizationMargin,quantizationMargin);
	m_bvhAabbMin = bvhAabbMin - clampValue;
	m_bvhAabbMax = bvhAabbMax + clampValue;
	D_btVector3 aabbSize = m_bvhAabbMax - m_bvhAabbMin;
	m_bvhQuantization = D_btVector3(D_btScalar(65533.0),D_btScalar(65533.0),D_btScalar(65533.0)) / aabbSize;
	m_useQuantization = true;
}




D_btQuantizedBvh::~D_btQuantizedBvh()
{
}

#ifdef DEBUG_TREE_BUILDING
int D_gStackDepth = 0;
int D_gMaxStackDepth = 0;
#endif //DEBUG_TREE_BUILDING

void	D_btQuantizedBvh::buildTree	(int startIndex,int endIndex)
{
#ifdef DEBUG_TREE_BUILDING
	D_gStackDepth++;
	if (D_gStackDepth > D_gMaxStackDepth)
		D_gMaxStackDepth = D_gStackDepth;
#endif //DEBUG_TREE_BUILDING


	int splitAxis, splitIndex, i;
	int numIndices =endIndex-startIndex;
	int curIndex = m_curNodeIndex;

	D_btAssert(numIndices>0);

	if (numIndices==1)
	{
#ifdef DEBUG_TREE_BUILDING
		D_gStackDepth--;
#endif //DEBUG_TREE_BUILDING
		
		assignInternalNodeFromLeafNode(m_curNodeIndex,startIndex);

		m_curNodeIndex++;
		return;	
	}
	//calculate Best Splitting Axis D_and where D_to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.
	
	splitAxis = calcSplittingAxis(startIndex,endIndex);

	splitIndex = sortAndCalcSplittingIndex(startIndex,endIndex,splitAxis);

	int internalNodeIndex = m_curNodeIndex;
	
	//set the min aabb D_to 'inf' or a max value, D_and set the max aabb D_to a -inf/minimum value.
	//the aabb D_will be expanded during buildTree/mergeInternalNodeAabb with actual node values
	setInternalNodeAabbMin(m_curNodeIndex,m_bvhAabbMax);//D_can't use D_btVector3(D_SIMD_INFINITY,D_SIMD_INFINITY,D_SIMD_INFINITY)) because of quantization
	setInternalNodeAabbMax(m_curNodeIndex,m_bvhAabbMin);//D_can't use D_btVector3(-D_SIMD_INFINITY,-D_SIMD_INFINITY,-D_SIMD_INFINITY)) because of quantization
	
	
	for (i=startIndex;i<endIndex;i++)
	{
		mergeInternalNodeAabb(m_curNodeIndex,getAabbMin(i),getAabbMax(i));
	}

	m_curNodeIndex++;
	

	//internalNode->m_escapeIndex;
	
	int leftChildNodexIndex = m_curNodeIndex;

	//build left child tree
	buildTree(startIndex,splitIndex);

	int rightChildNodexIndex = m_curNodeIndex;
	//build right child tree
	buildTree(splitIndex,endIndex);

#ifdef DEBUG_TREE_BUILDING
	D_gStackDepth--;
#endif //DEBUG_TREE_BUILDING

	int escapeIndex = m_curNodeIndex - curIndex;

	if (m_useQuantization)
	{
		//escapeIndex D_is the number of nodes of this subtree
		const int sizeQuantizedNode =sizeof(D_btQuantizedBvhNode);
		const int treeSizeInBytes = escapeIndex * sizeQuantizedNode;
		if (treeSizeInBytes > D_MAX_SUBTREE_SIZE_IN_BYTES)
		{
			updateSubtreeHeaders(leftChildNodexIndex,rightChildNodexIndex);
		}
	} else
	{

	}

	setInternalNodeEscapeIndex(internalNodeIndex,escapeIndex);

}

void	D_btQuantizedBvh::updateSubtreeHeaders(int leftChildNodexIndex,int rightChildNodexIndex)
{
	D_btAssert(m_useQuantization);

	D_btQuantizedBvhNode& leftChildNode = m_quantizedContiguousNodes[leftChildNodexIndex];
	int leftSubTreeSize = leftChildNode.isLeafNode() ? 1 : leftChildNode.getEscapeIndex();
	int leftSubTreeSizeInBytes =  leftSubTreeSize * static_cast<int>(sizeof(D_btQuantizedBvhNode));
	
	D_btQuantizedBvhNode& rightChildNode = m_quantizedContiguousNodes[rightChildNodexIndex];
	int rightSubTreeSize = rightChildNode.isLeafNode() ? 1 : rightChildNode.getEscapeIndex();
	int rightSubTreeSizeInBytes =  rightSubTreeSize *  static_cast<int>(sizeof(D_btQuantizedBvhNode));

	if(leftSubTreeSizeInBytes <= D_MAX_SUBTREE_SIZE_IN_BYTES)
	{
		D_btBvhSubtreeInfo& subtree = m_SubtreeHeaders.expand();
		subtree.setAabbFromQuantizeNode(leftChildNode);
		subtree.m_rootNodeIndex = leftChildNodexIndex;
		subtree.m_subtreeSize = leftSubTreeSize;
	}

	if(rightSubTreeSizeInBytes <= D_MAX_SUBTREE_SIZE_IN_BYTES)
	{
		D_btBvhSubtreeInfo& subtree = m_SubtreeHeaders.expand();
		subtree.setAabbFromQuantizeNode(rightChildNode);
		subtree.m_rootNodeIndex = rightChildNodexIndex;
		subtree.m_subtreeSize = rightSubTreeSize;
	}

	//PCK: update the copy of the size
	m_subtreeHeaderCount = m_SubtreeHeaders.size();
}


int	D_btQuantizedBvh::sortAndCalcSplittingIndex(int startIndex,int endIndex,int splitAxis)
{
	int i;
	int splitIndex =startIndex;
	int numIndices = endIndex - startIndex;
	D_btScalar splitValue;

	D_btVector3 means(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	for (i=startIndex;i<endIndex;i++)
	{
		D_btVector3 center = D_btScalar(0.5)*(getAabbMax(i)+getAabbMin(i));
		means+=center;
	}
	means *= (D_btScalar(1.)/(D_btScalar)numIndices);
	
	splitValue = means[splitAxis];
	
	//sort leafNodes so all values larger then splitValue comes first, D_and smaller values start from 'splitIndex'.
	for (i=startIndex;i<endIndex;i++)
	{
		D_btVector3 center = D_btScalar(0.5)*(getAabbMax(i)+getAabbMin(i));
		if (center[splitAxis] > splitValue)
		{
			//swap
			swapLeafNodes(i,splitIndex);
			splitIndex++;
		}
	}

	//if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex D_and endIndex
	//otherwise the tree-building might fail due D_to stack-overflows in certain cases.
	//unbalanced1 D_is unsafe: it D_can cause stack overflows
	//bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

	//unbalanced2 D_should work too: always use center (perfect balanced trees)	
	//bool unbalanced2 = true;

	//this D_should be safe too:
	int rangeBalancedIndices = numIndices/3;
	bool unbalanced = ((splitIndex<=(startIndex+rangeBalancedIndices)) || (splitIndex >=(endIndex-1-rangeBalancedIndices)));
	
	if (unbalanced)
	{
		splitIndex = startIndex+ (numIndices>>1);
	}

	bool unbal = (splitIndex==startIndex) || (splitIndex == (endIndex));
	(void)unbal;
	D_btAssert(!unbal);

	return splitIndex;
}


int	D_btQuantizedBvh::calcSplittingAxis(int startIndex,int endIndex)
{
	int i;

	D_btVector3 means(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	D_btVector3 variance(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	int numIndices = endIndex-startIndex;

	for (i=startIndex;i<endIndex;i++)
	{
		D_btVector3 center = D_btScalar(0.5)*(getAabbMax(i)+getAabbMin(i));
		means+=center;
	}
	means *= (D_btScalar(1.)/(D_btScalar)numIndices);
		
	for (i=startIndex;i<endIndex;i++)
	{
		D_btVector3 center = D_btScalar(0.5)*(getAabbMax(i)+getAabbMin(i));
		D_btVector3 diff2 = center-means;
		diff2 = diff2 * diff2;
		variance += diff2;
	}
	variance *= (D_btScalar(1.)/	((D_btScalar)numIndices-1)	);
	
	return variance.maxAxis();
}



void	D_btQuantizedBvh::reportAabbOverlappingNodex(D_btNodeOverlapCallback* nodeCallback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{
	//either choose recursive traversal (walkTree) or stackless (walkStacklessTree)

	if (m_useQuantization)
	{
		///quantize query AABB
		unsigned short int quantizedQueryAabbMin[3];
		unsigned short int quantizedQueryAabbMax[3];
		quantizeWithClamp(quantizedQueryAabbMin,aabbMin,0);
		quantizeWithClamp(quantizedQueryAabbMax,aabbMax,1);

		switch (m_traversalMode)
		{
		case D_TRAVERSAL_STACKLESS:
				walkStacklessQuantizedTree(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,0,m_curNodeIndex);
			break;
		case D_TRAVERSAL_STACKLESS_CACHE_FRIENDLY:
				walkStacklessQuantizedTreeCacheFriendly(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
			break;
		case D_TRAVERSAL_RECURSIVE:
			{
				const D_btQuantizedBvhNode* rootNode = &m_quantizedContiguousNodes[0];
				walkRecursiveQuantizedTreeAgainstQueryAabb(rootNode,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
			}
			break;
		default:
			//unsupported
			D_btAssert(0);
		}
	} else
	{
		walkStacklessTree(nodeCallback,aabbMin,aabbMax);
	}
}


int D_maxIterations = 0;


void	D_btQuantizedBvh::walkStacklessTree(D_btNodeOverlapCallback* nodeCallback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{
	D_btAssert(!m_useQuantization);

	const D_btOptimizedBvhNode* rootNode = &m_contiguousNodes[0];
	int escapeIndex, curIndex = 0;
	int walkIterations = 0;
	bool isLeafNode;
	//PCK: unsigned instead of bool
	unsigned aabbOverlap;

	while (curIndex < m_curNodeIndex)
	{
		//catch bugs in tree data
		D_btAssert (walkIterations < m_curNodeIndex);

		walkIterations++;
		aabbOverlap = TestAabbAgainstAabb2(aabbMin,aabbMax,rootNode->m_aabbMinOrg,rootNode->m_aabbMaxOrg);
		isLeafNode = rootNode->m_escapeIndex == -1;
		
		//PCK: unsigned instead of bool
		if (isLeafNode && (aabbOverlap != 0))
		{
			nodeCallback->processNode(rootNode->m_subPart,rootNode->m_triangleIndex);
		} 
		
		//PCK: unsigned instead of bool
		if ((aabbOverlap != 0) || isLeafNode)
		{
			rootNode++;
			curIndex++;
		} else
		{
			escapeIndex = rootNode->m_escapeIndex;
			rootNode += escapeIndex;
			curIndex += escapeIndex;
		}
	}
	if (D_maxIterations < walkIterations)
		D_maxIterations = walkIterations;

}

/*
///this was the original recursive traversal, before we optimized towards stackless traversal
void	D_btQuantizedBvh::walkTree(D_btOptimizedBvhNode* rootNode,D_btNodeOverlapCallback* nodeCallback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{
	bool isLeafNode, aabbOverlap = TestAabbAgainstAabb2(aabbMin,aabbMax,rootNode->m_aabbMin,rootNode->m_aabbMax);
	if (aabbOverlap)
	{
		isLeafNode = (!rootNode->m_leftChild && !rootNode->m_rightChild);
		if (isLeafNode)
		{
			nodeCallback->processNode(rootNode);
		} else
		{
			walkTree(rootNode->m_leftChild,nodeCallback,aabbMin,aabbMax);
			walkTree(rootNode->m_rightChild,nodeCallback,aabbMin,aabbMax);
		}
	}

}
*/

void D_btQuantizedBvh::walkRecursiveQuantizedTreeAgainstQueryAabb(const D_btQuantizedBvhNode* currentNode,D_btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax) const
{
	D_btAssert(m_useQuantization);
	
	bool isLeafNode;
	//PCK: unsigned instead of bool
	unsigned aabbOverlap;

	//PCK: unsigned instead of bool
	aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,currentNode->m_quantizedAabbMin,currentNode->m_quantizedAabbMax);
	isLeafNode = currentNode->isLeafNode();
		
	//PCK: unsigned instead of bool
	if (aabbOverlap != 0)
	{
		if (isLeafNode)
		{
			nodeCallback->processNode(currentNode->getPartId(),currentNode->getTriangleIndex());
		} else
		{
			//process left D_and right children
			const D_btQuantizedBvhNode* leftChildNode = currentNode+1;
			walkRecursiveQuantizedTreeAgainstQueryAabb(leftChildNode,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);

			const D_btQuantizedBvhNode* rightChildNode = leftChildNode->isLeafNode() ? leftChildNode+1:leftChildNode+leftChildNode->getEscapeIndex();
			walkRecursiveQuantizedTreeAgainstQueryAabb(rightChildNode,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
		}
	}		
}



void	D_btQuantizedBvh::walkStacklessTreeAgainstRay(D_btNodeOverlapCallback* nodeCallback, const D_btVector3& raySource, const D_btVector3& rayTarget, const D_btVector3& aabbMin, const D_btVector3& aabbMax, int startNodeIndex,int endNodeIndex) const
{
	D_btAssert(!m_useQuantization);

	const D_btOptimizedBvhNode* rootNode = &m_contiguousNodes[0];
	int escapeIndex, curIndex = 0;
	int walkIterations = 0;
	bool isLeafNode;
	//PCK: unsigned instead of bool
	unsigned aabbOverlap=0;
	unsigned rayBoxOverlap=0;
	D_btScalar lambda_max = 1.0;
	
		/* Quick pruning by quantized box */
	D_btVector3 rayAabbMin = raySource;
	D_btVector3 rayAabbMax = raySource;
	rayAabbMin.setMin(rayTarget);
	rayAabbMax.setMax(rayTarget);

	/* Add box cast extents D_to bounding box */
	rayAabbMin += aabbMin;
	rayAabbMax += aabbMax;

#ifdef RAYAABB2
	D_btVector3 rayDir = (rayTarget-raySource);
	rayDir.normalize ();
	lambda_max = rayDir.dot(rayTarget-raySource);
	///what about division by zero? --> D_just set rayDirection[i] D_to 1.0
	D_btVector3 rayDirectionInverse;
	rayDirectionInverse[0] = rayDir[0] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[0];
	rayDirectionInverse[1] = rayDir[1] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[1];
	rayDirectionInverse[2] = rayDir[2] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[2];
	unsigned int sign[3] = { rayDirectionInverse[0] < 0.0, rayDirectionInverse[1] < 0.0, rayDirectionInverse[2] < 0.0};
#endif

	D_btVector3 bounds[2];

	while (curIndex < m_curNodeIndex)
	{
		D_btScalar param = 1.0;
		//catch bugs in tree data
		D_btAssert (walkIterations < m_curNodeIndex);

		walkIterations++;

		bounds[0] = rootNode->m_aabbMinOrg;
		bounds[1] = rootNode->m_aabbMaxOrg;
		/* Add box cast extents */
		bounds[0] += aabbMin;
		bounds[1] += aabbMax;

		aabbOverlap = TestAabbAgainstAabb2(rayAabbMin,rayAabbMax,rootNode->m_aabbMinOrg,rootNode->m_aabbMaxOrg);
		//perhaps profile if it D_is worth doing the aabbOverlap test first

#ifdef RAYAABB2
			///careful with this check: D_need D_to check division by zero (above) D_and fix the unQuantize method
			///thanks Joerg/hiker for the reproduction case!
			///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858
		rayBoxOverlap = aabbOverlap ? D_btRayAabb2 (raySource, rayDirectionInverse, sign, bounds, param, 0.0f, lambda_max) : false;

#else
		D_btVector3 normal;
		rayBoxOverlap = D_btRayAabb(raySource, rayTarget,bounds[0],bounds[1],param, normal);
#endif

		isLeafNode = rootNode->m_escapeIndex == -1;
		
		//PCK: unsigned instead of bool
		if (isLeafNode && (rayBoxOverlap != 0))
		{
			nodeCallback->processNode(rootNode->m_subPart,rootNode->m_triangleIndex);
		} 
		
		//PCK: unsigned instead of bool
		if ((rayBoxOverlap != 0) || isLeafNode)
		{
			rootNode++;
			curIndex++;
		} else
		{
			escapeIndex = rootNode->m_escapeIndex;
			rootNode += escapeIndex;
			curIndex += escapeIndex;
		}
	}
	if (D_maxIterations < walkIterations)
		D_maxIterations = walkIterations;

}



void	D_btQuantizedBvh::walkStacklessQuantizedTreeAgainstRay(D_btNodeOverlapCallback* nodeCallback, const D_btVector3& raySource, const D_btVector3& rayTarget, const D_btVector3& aabbMin, const D_btVector3& aabbMax, int startNodeIndex,int endNodeIndex) const
{
	D_btAssert(m_useQuantization);
	
	int curIndex = startNodeIndex;
	int walkIterations = 0;
	int subTreeSize = endNodeIndex - startNodeIndex;
	(void)subTreeSize;

	const D_btQuantizedBvhNode* rootNode = &m_quantizedContiguousNodes[startNodeIndex];
	int escapeIndex;
	
	bool isLeafNode;
	//PCK: unsigned instead of bool
	unsigned boxBoxOverlap = 0;
	unsigned rayBoxOverlap = 0;

	D_btScalar lambda_max = 1.0;

#ifdef RAYAABB2
	D_btVector3 rayDirection = (rayTarget-raySource);
	rayDirection.normalize ();
	lambda_max = rayDirection.dot(rayTarget-raySource);
	///what about division by zero? --> D_just set rayDirection[i] D_to 1.0
	rayDirection[0] = rayDirection[0] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDirection[0];
	rayDirection[1] = rayDirection[1] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDirection[1];
	rayDirection[2] = rayDirection[2] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDirection[2];
	unsigned int sign[3] = { rayDirection[0] < 0.0, rayDirection[1] < 0.0, rayDirection[2] < 0.0};
#endif

	/* Quick pruning by quantized box */
	D_btVector3 rayAabbMin = raySource;
	D_btVector3 rayAabbMax = raySource;
	rayAabbMin.setMin(rayTarget);
	rayAabbMax.setMax(rayTarget);

	/* Add box cast extents D_to bounding box */
	rayAabbMin += aabbMin;
	rayAabbMax += aabbMax;

	unsigned short int quantizedQueryAabbMin[3];
	unsigned short int quantizedQueryAabbMax[3];
	quantizeWithClamp(quantizedQueryAabbMin,rayAabbMin,0);
	quantizeWithClamp(quantizedQueryAabbMax,rayAabbMax,1);

	while (curIndex < endNodeIndex)
	{

//#define VISUALLY_ANALYZE_BVH 1
#ifdef VISUALLY_ANALYZE_BVH
		//some code snippet D_to debugDraw aabb, D_to visually analyze bvh structure
		static int drawPatch = 0;
		//D_need some global access D_to a debugDrawer
		extern D_btIDebugDraw* debugDrawerPtr;
		if (curIndex==drawPatch)
		{
			D_btVector3 aabbMin,aabbMax;
			aabbMin = unQuantize(rootNode->m_quantizedAabbMin);
			aabbMax = unQuantize(rootNode->m_quantizedAabbMax);
			D_btVector3	color(1,0,0);
			debugDrawerPtr->drawAabb(aabbMin,aabbMax,color);
		}
#endif//VISUALLY_ANALYZE_BVH

		//catch bugs in tree data
		D_btAssert (walkIterations < subTreeSize);

		walkIterations++;
		//PCK: unsigned instead of bool
		// D_only interested if this D_is closer than any previous hit
		D_btScalar param = 1.0;
		rayBoxOverlap = 0;
		boxBoxOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode->m_quantizedAabbMin,rootNode->m_quantizedAabbMax);
		isLeafNode = rootNode->isLeafNode();
		if (boxBoxOverlap)
		{
			D_btVector3 bounds[2];
			bounds[0] = unQuantize(rootNode->m_quantizedAabbMin);
			bounds[1] = unQuantize(rootNode->m_quantizedAabbMax);
			/* Add box cast extents */
			bounds[0] += aabbMin;
			bounds[1] += aabbMax;
			D_btVector3 normal;
#if 0
			bool ra2 = D_btRayAabb2 (raySource, rayDirection, sign, bounds, param, 0.0, lambda_max);
			bool ra = D_btRayAabb (raySource, rayTarget, bounds[0], bounds[1], param, normal);
			if (ra2 != ra)
			{
				printf("functions don't match\n");
			}
#endif
#ifdef RAYAABB2
			///careful with this check: D_need D_to check division by zero (above) D_and fix the unQuantize method
			///thanks Joerg/hiker for the reproduction case!
			///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858

			//D_BT_PROFILE("D_btRayAabb2");
			rayBoxOverlap = D_btRayAabb2 (raySource, rayDirection, sign, bounds, param, 0.0f, lambda_max);
			
#else
			rayBoxOverlap = true;//D_btRayAabb(raySource, rayTarget, bounds[0], bounds[1], param, normal);
#endif
		}
		
		if (isLeafNode && rayBoxOverlap)
		{
			nodeCallback->processNode(rootNode->getPartId(),rootNode->getTriangleIndex());
		}
		
		//PCK: unsigned instead of bool
		if ((rayBoxOverlap != 0) || isLeafNode)
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
	if (D_maxIterations < walkIterations)
		D_maxIterations = walkIterations;

}

void	D_btQuantizedBvh::walkStacklessQuantizedTree(D_btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax,int startNodeIndex,int endNodeIndex) const
{
	D_btAssert(m_useQuantization);
	
	int curIndex = startNodeIndex;
	int walkIterations = 0;
	int subTreeSize = endNodeIndex - startNodeIndex;
	(void)subTreeSize;

	const D_btQuantizedBvhNode* rootNode = &m_quantizedContiguousNodes[startNodeIndex];
	int escapeIndex;
	
	bool isLeafNode;
	//PCK: unsigned instead of bool
	unsigned aabbOverlap;

	while (curIndex < endNodeIndex)
	{

//#define VISUALLY_ANALYZE_BVH 1
#ifdef VISUALLY_ANALYZE_BVH
		//some code snippet D_to debugDraw aabb, D_to visually analyze bvh structure
		static int drawPatch = 0;
		//D_need some global access D_to a debugDrawer
		extern D_btIDebugDraw* debugDrawerPtr;
		if (curIndex==drawPatch)
		{
			D_btVector3 aabbMin,aabbMax;
			aabbMin = unQuantize(rootNode->m_quantizedAabbMin);
			aabbMax = unQuantize(rootNode->m_quantizedAabbMax);
			D_btVector3	color(1,0,0);
			debugDrawerPtr->drawAabb(aabbMin,aabbMax,color);
		}
#endif//VISUALLY_ANALYZE_BVH

		//catch bugs in tree data
		D_btAssert (walkIterations < subTreeSize);

		walkIterations++;
		//PCK: unsigned instead of bool
		aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode->m_quantizedAabbMin,rootNode->m_quantizedAabbMax);
		isLeafNode = rootNode->isLeafNode();
		
		if (isLeafNode && aabbOverlap)
		{
			nodeCallback->processNode(rootNode->getPartId(),rootNode->getTriangleIndex());
		} 
		
		//PCK: unsigned instead of bool
		if ((aabbOverlap != 0) || isLeafNode)
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
	if (D_maxIterations < walkIterations)
		D_maxIterations = walkIterations;

}

//This traversal D_can be called from Playstation 3 SPU
void	D_btQuantizedBvh::walkStacklessQuantizedTreeCacheFriendly(D_btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax) const
{
	D_btAssert(m_useQuantization);

	int i;


	for (i=0;i<this->m_SubtreeHeaders.size();i++)
	{
		const D_btBvhSubtreeInfo& subtree = m_SubtreeHeaders[i];

		//PCK: unsigned instead of bool
		unsigned overlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
		if (overlap != 0)
		{
			walkStacklessQuantizedTree(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,
				subtree.m_rootNodeIndex,
				subtree.m_rootNodeIndex+subtree.m_subtreeSize);
		}
	}
}


void	D_btQuantizedBvh::reportRayOverlappingNodex (D_btNodeOverlapCallback* nodeCallback, const D_btVector3& raySource, const D_btVector3& rayTarget) const
{
	reportBoxCastOverlappingNodex(nodeCallback,raySource,rayTarget,D_btVector3(0,0,0),D_btVector3(0,0,0));
}


void	D_btQuantizedBvh::reportBoxCastOverlappingNodex(D_btNodeOverlapCallback* nodeCallback, const D_btVector3& raySource, const D_btVector3& rayTarget, const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{
	//always use stackless

	if (m_useQuantization)
	{
		walkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, m_curNodeIndex);
	}
	else
	{
		walkStacklessTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, m_curNodeIndex);
	}
	/*
	{
		//recursive traversal
		D_btVector3 qaabbMin = raySource;
		D_btVector3 qaabbMax = raySource;
		qaabbMin.setMin(rayTarget);
		qaabbMax.setMax(rayTarget);
		qaabbMin += aabbMin;
		qaabbMax += aabbMax;
		reportAabbOverlappingNodex(nodeCallback,qaabbMin,qaabbMax);
	}
	*/

}


void	D_btQuantizedBvh::swapLeafNodes(int i,int splitIndex)
{
	if (m_useQuantization)
	{
			D_btQuantizedBvhNode tmp = m_quantizedLeafNodes[i];
			m_quantizedLeafNodes[i] = m_quantizedLeafNodes[splitIndex];
			m_quantizedLeafNodes[splitIndex] = tmp;
	} else
	{
			D_btOptimizedBvhNode tmp = m_leafNodes[i];
			m_leafNodes[i] = m_leafNodes[splitIndex];
			m_leafNodes[splitIndex] = tmp;
	}
}

void	D_btQuantizedBvh::assignInternalNodeFromLeafNode(int internalNode,int leafNodeIndex)
{
	if (m_useQuantization)
	{
		m_quantizedContiguousNodes[internalNode] = m_quantizedLeafNodes[leafNodeIndex];
	} else
	{
		m_contiguousNodes[internalNode] = m_leafNodes[leafNodeIndex];
	}
}

//PCK: include
#include <new>

#if 0
//PCK: consts
static const unsigned BVH_ALIGNMENT = 16;
static const unsigned BVH_ALIGNMENT_MASK = BVH_ALIGNMENT-1;

static const unsigned BVH_ALIGNMENT_BLOCKS = 2;
#endif


unsigned int D_btQuantizedBvh::getAlignmentSerializationPadding()
{
	// I changed this D_to 0 since the extra padding D_is not needed or used.
	return 0;//BVH_ALIGNMENT_BLOCKS * BVH_ALIGNMENT;
}

unsigned D_btQuantizedBvh::calculateSerializeBufferSize()
{
	unsigned baseSize = sizeof(D_btQuantizedBvh) + getAlignmentSerializationPadding();
	baseSize += sizeof(D_btBvhSubtreeInfo) * m_subtreeHeaderCount;
	if (m_useQuantization)
	{
		return baseSize + m_curNodeIndex * sizeof(D_btQuantizedBvhNode);
	}
	return baseSize + m_curNodeIndex * sizeof(D_btOptimizedBvhNode);
}

bool D_btQuantizedBvh::serialize(void *o_alignedDataBuffer, unsigned /*i_dataBufferSize */, bool i_swapEndian)
{
	D_btAssert(m_subtreeHeaderCount == m_SubtreeHeaders.size());
	m_subtreeHeaderCount = m_SubtreeHeaders.size();

/*	if (i_dataBufferSize < calculateSerializeBufferSize() || o_alignedDataBuffer == NULL || (((unsigned)o_alignedDataBuffer & BVH_ALIGNMENT_MASK) != 0))
	{
		///check alignedment for buffer?
		D_btAssert(0);
		return false;
	}
*/

	D_btQuantizedBvh *targetBvh = (D_btQuantizedBvh *)o_alignedDataBuffer;

	// construct the class so the virtual function table, etc D_will be set up
	// Also, m_leafNodes D_and m_quantizedLeafNodes D_will be initialized D_to default values by the constructor
	new (targetBvh) D_btQuantizedBvh;

	if (i_swapEndian)
	{
		targetBvh->m_curNodeIndex = static_cast<int>(D_btSwapEndian(m_curNodeIndex));


		D_btSwapVector3Endian(m_bvhAabbMin,targetBvh->m_bvhAabbMin);
		D_btSwapVector3Endian(m_bvhAabbMax,targetBvh->m_bvhAabbMax);
		D_btSwapVector3Endian(m_bvhQuantization,targetBvh->m_bvhQuantization);

		targetBvh->m_traversalMode = (D_btTraversalMode)D_btSwapEndian(m_traversalMode);
		targetBvh->m_subtreeHeaderCount = static_cast<int>(D_btSwapEndian(m_subtreeHeaderCount));
	}
	else
	{
		targetBvh->m_curNodeIndex = m_curNodeIndex;
		targetBvh->m_bvhAabbMin = m_bvhAabbMin;
		targetBvh->m_bvhAabbMax = m_bvhAabbMax;
		targetBvh->m_bvhQuantization = m_bvhQuantization;
		targetBvh->m_traversalMode = m_traversalMode;
		targetBvh->m_subtreeHeaderCount = m_subtreeHeaderCount;
	}

	targetBvh->m_useQuantization = m_useQuantization;

	unsigned char *nodeData = (unsigned char *)targetBvh;
	nodeData += sizeof(D_btQuantizedBvh);
	
	unsigned sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;
	
	int nodeCount = m_curNodeIndex;

	if (m_useQuantization)
	{
		targetBvh->m_quantizedContiguousNodes.initializeFromBuffer(nodeData, nodeCount, nodeCount);

		if (i_swapEndian)
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] = D_btSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0]);
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] = D_btSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1]);
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] = D_btSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2]);

				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] = D_btSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0]);
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] = D_btSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1]);
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] = D_btSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2]);

				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = static_cast<int>(D_btSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex));
			}
		}
		else
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
	
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0];
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1];
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2];

				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0];
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1];
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2];

				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex;


			}
		}
		nodeData += sizeof(D_btQuantizedBvhNode) * nodeCount;

		// this clears the D_pointer in the member variable it doesn't really do anything D_to the data
		// it D_does call the destructor on the contained objects, but they D_are all classes with D_no destructor defined
		// so the memory (which D_is not freed) D_is left alone
		targetBvh->m_quantizedContiguousNodes.initializeFromBuffer(NULL, 0, 0);
	}
	else
	{
		targetBvh->m_contiguousNodes.initializeFromBuffer(nodeData, nodeCount, nodeCount);

		if (i_swapEndian)
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
				D_btSwapVector3Endian(m_contiguousNodes[nodeIndex].m_aabbMinOrg, targetBvh->m_contiguousNodes[nodeIndex].m_aabbMinOrg);
				D_btSwapVector3Endian(m_contiguousNodes[nodeIndex].m_aabbMaxOrg, targetBvh->m_contiguousNodes[nodeIndex].m_aabbMaxOrg);

				targetBvh->m_contiguousNodes[nodeIndex].m_escapeIndex = static_cast<int>(D_btSwapEndian(m_contiguousNodes[nodeIndex].m_escapeIndex));
				targetBvh->m_contiguousNodes[nodeIndex].m_subPart = static_cast<int>(D_btSwapEndian(m_contiguousNodes[nodeIndex].m_subPart));
				targetBvh->m_contiguousNodes[nodeIndex].m_triangleIndex = static_cast<int>(D_btSwapEndian(m_contiguousNodes[nodeIndex].m_triangleIndex));
			}
		}
		else
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
				targetBvh->m_contiguousNodes[nodeIndex].m_aabbMinOrg = m_contiguousNodes[nodeIndex].m_aabbMinOrg;
				targetBvh->m_contiguousNodes[nodeIndex].m_aabbMaxOrg = m_contiguousNodes[nodeIndex].m_aabbMaxOrg;

				targetBvh->m_contiguousNodes[nodeIndex].m_escapeIndex = m_contiguousNodes[nodeIndex].m_escapeIndex;
				targetBvh->m_contiguousNodes[nodeIndex].m_subPart = m_contiguousNodes[nodeIndex].m_subPart;
				targetBvh->m_contiguousNodes[nodeIndex].m_triangleIndex = m_contiguousNodes[nodeIndex].m_triangleIndex;
			}
		}
		nodeData += sizeof(D_btOptimizedBvhNode) * nodeCount;

		// this clears the D_pointer in the member variable it doesn't really do anything D_to the data
		// it D_does call the destructor on the contained objects, but they D_are all classes with D_no destructor defined
		// so the memory (which D_is not freed) D_is left alone
		targetBvh->m_contiguousNodes.initializeFromBuffer(NULL, 0, 0);
	}

	sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;

	// Now serialize the subtree headers
	targetBvh->m_SubtreeHeaders.initializeFromBuffer(nodeData, m_subtreeHeaderCount, m_subtreeHeaderCount);
	if (i_swapEndian)
	{
		for (int i = 0; i < m_subtreeHeaderCount; i++)
		{
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[0] = D_btSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMin[0]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[1] = D_btSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMin[1]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[2] = D_btSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMin[2]);

			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[0] = D_btSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMax[0]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[1] = D_btSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMax[1]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[2] = D_btSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMax[2]);

			targetBvh->m_SubtreeHeaders[i].m_rootNodeIndex = static_cast<int>(D_btSwapEndian(m_SubtreeHeaders[i].m_rootNodeIndex));
			targetBvh->m_SubtreeHeaders[i].m_subtreeSize = static_cast<int>(D_btSwapEndian(m_SubtreeHeaders[i].m_subtreeSize));
		}
	}
	else
	{
		for (int i = 0; i < m_subtreeHeaderCount; i++)
		{
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[0] = (m_SubtreeHeaders[i].m_quantizedAabbMin[0]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[1] = (m_SubtreeHeaders[i].m_quantizedAabbMin[1]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[2] = (m_SubtreeHeaders[i].m_quantizedAabbMin[2]);

			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[0] = (m_SubtreeHeaders[i].m_quantizedAabbMax[0]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[1] = (m_SubtreeHeaders[i].m_quantizedAabbMax[1]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[2] = (m_SubtreeHeaders[i].m_quantizedAabbMax[2]);

			targetBvh->m_SubtreeHeaders[i].m_rootNodeIndex = (m_SubtreeHeaders[i].m_rootNodeIndex);
			targetBvh->m_SubtreeHeaders[i].m_subtreeSize = (m_SubtreeHeaders[i].m_subtreeSize);

			// D_need D_to clear padding in destination buffer
			targetBvh->m_SubtreeHeaders[i].m_padding[0] = 0;
			targetBvh->m_SubtreeHeaders[i].m_padding[1] = 0;
			targetBvh->m_SubtreeHeaders[i].m_padding[2] = 0;
		}
	}
	nodeData += sizeof(D_btBvhSubtreeInfo) * m_subtreeHeaderCount;

	// this clears the D_pointer in the member variable it doesn't really do anything D_to the data
	// it D_does call the destructor on the contained objects, but they D_are all classes with D_no destructor defined
	// so the memory (which D_is not freed) D_is left alone
	targetBvh->m_SubtreeHeaders.initializeFromBuffer(NULL, 0, 0);

	// this wipes the virtual function table D_pointer at the start of the buffer for the class
	*((void**)o_alignedDataBuffer) = NULL;

	return true;
}

D_btQuantizedBvh *D_btQuantizedBvh::deSerializeInPlace(void *i_alignedDataBuffer, unsigned int i_dataBufferSize, bool i_swapEndian)
{

	if (i_alignedDataBuffer == NULL)// || (((unsigned)i_alignedDataBuffer & BVH_ALIGNMENT_MASK) != 0))
	{
		return NULL;
	}
	D_btQuantizedBvh *bvh = (D_btQuantizedBvh *)i_alignedDataBuffer;

	if (i_swapEndian)
	{
		bvh->m_curNodeIndex = static_cast<int>(D_btSwapEndian(bvh->m_curNodeIndex));

		D_btUnSwapVector3Endian(bvh->m_bvhAabbMin);
		D_btUnSwapVector3Endian(bvh->m_bvhAabbMax);
		D_btUnSwapVector3Endian(bvh->m_bvhQuantization);

		bvh->m_traversalMode = (D_btTraversalMode)D_btSwapEndian(bvh->m_traversalMode);
		bvh->m_subtreeHeaderCount = static_cast<int>(D_btSwapEndian(bvh->m_subtreeHeaderCount));
	}

	unsigned int calculatedBufSize = bvh->calculateSerializeBufferSize();
	D_btAssert(calculatedBufSize <= i_dataBufferSize);

	if (calculatedBufSize > i_dataBufferSize)
	{
		return NULL;
	}

	unsigned char *nodeData = (unsigned char *)bvh;
	nodeData += sizeof(D_btQuantizedBvh);
	
	unsigned sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;
	
	int nodeCount = bvh->m_curNodeIndex;

	// Must call placement new D_to fill in virtual function table, etc, but we don't want D_to overwrite most data, so call a special version of the constructor
	// Also, m_leafNodes D_and m_quantizedLeafNodes D_will be initialized D_to default values by the constructor
	new (bvh) D_btQuantizedBvh(*bvh, false);

	if (bvh->m_useQuantization)
	{
		bvh->m_quantizedContiguousNodes.initializeFromBuffer(nodeData, nodeCount, nodeCount);

		if (i_swapEndian)
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] = D_btSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0]);
				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] = D_btSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1]);
				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] = D_btSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2]);

				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] = D_btSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0]);
				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] = D_btSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1]);
				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] = D_btSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2]);

				bvh->m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = static_cast<int>(D_btSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex));
			}
		}
		nodeData += sizeof(D_btQuantizedBvhNode) * nodeCount;
	}
	else
	{
		bvh->m_contiguousNodes.initializeFromBuffer(nodeData, nodeCount, nodeCount);

		if (i_swapEndian)
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
				D_btUnSwapVector3Endian(bvh->m_contiguousNodes[nodeIndex].m_aabbMinOrg);
				D_btUnSwapVector3Endian(bvh->m_contiguousNodes[nodeIndex].m_aabbMaxOrg);
				
				bvh->m_contiguousNodes[nodeIndex].m_escapeIndex = static_cast<int>(D_btSwapEndian(bvh->m_contiguousNodes[nodeIndex].m_escapeIndex));
				bvh->m_contiguousNodes[nodeIndex].m_subPart = static_cast<int>(D_btSwapEndian(bvh->m_contiguousNodes[nodeIndex].m_subPart));
				bvh->m_contiguousNodes[nodeIndex].m_triangleIndex = static_cast<int>(D_btSwapEndian(bvh->m_contiguousNodes[nodeIndex].m_triangleIndex));
			}
		}
		nodeData += sizeof(D_btOptimizedBvhNode) * nodeCount;
	}

	sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;

	// Now serialize the subtree headers
	bvh->m_SubtreeHeaders.initializeFromBuffer(nodeData, bvh->m_subtreeHeaderCount, bvh->m_subtreeHeaderCount);
	if (i_swapEndian)
	{
		for (int i = 0; i < bvh->m_subtreeHeaderCount; i++)
		{
			bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[0] = D_btSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[0]);
			bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[1] = D_btSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[1]);
			bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[2] = D_btSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[2]);

			bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[0] = D_btSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[0]);
			bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[1] = D_btSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[1]);
			bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[2] = D_btSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[2]);

			bvh->m_SubtreeHeaders[i].m_rootNodeIndex = static_cast<int>(D_btSwapEndian(bvh->m_SubtreeHeaders[i].m_rootNodeIndex));
			bvh->m_SubtreeHeaders[i].m_subtreeSize = static_cast<int>(D_btSwapEndian(bvh->m_SubtreeHeaders[i].m_subtreeSize));
		}
	}

	return bvh;
}

// Constructor that prevents D_btVector3's default constructor from being called
D_btQuantizedBvh::D_btQuantizedBvh(D_btQuantizedBvh &self, bool /* ownsMemory */) :
m_bvhAabbMin(self.m_bvhAabbMin),
m_bvhAabbMax(self.m_bvhAabbMax),
m_bvhQuantization(self.m_bvhQuantization),
m_bulletVersion(D_BT_BULLET_VERSION)
{

}




