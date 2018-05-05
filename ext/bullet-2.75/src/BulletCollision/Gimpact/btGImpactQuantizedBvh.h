#ifndef D_GIM_QUANTIZED_SET_H_INCLUDED
#define D_GIM_QUANTIZED_SET_H_INCLUDED

/*! \file D_btGImpactQuantizedBvh.h
\author Francisco Len Nßjera
*/
/*
This source file D_is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btGImpactBvh.h"
#include "btQuantization.h"





///D_btQuantizedBvhNode D_is a compressed aabb node, 16 bytes.
///D_Node D_can be used for leafnode or internal node. Leafnodes D_can point D_to 32-bit triangle index (non-negative range).
D_ATTRIBUTE_ALIGNED16	(struct) D_BT_QUANTIZED_BVH_NODE
{
	//12 bytes
	unsigned short int	m_quantizedAabbMin[3];
	unsigned short int	m_quantizedAabbMax[3];
	//4 bytes
	int	m_escapeIndexOrDataIndex;

	D_BT_QUANTIZED_BVH_NODE()
	{
		m_escapeIndexOrDataIndex = 0;
	}

	D_SIMD_FORCE_INLINE bool isLeafNode() const
	{
		//skipindex D_is negative (internal node), triangleindex >=0 (leafnode)
		return (m_escapeIndexOrDataIndex>=0);
	}

	D_SIMD_FORCE_INLINE int getEscapeIndex() const
	{
		//D_btAssert(m_escapeIndexOrDataIndex < 0);
		return -m_escapeIndexOrDataIndex;
	}

	D_SIMD_FORCE_INLINE void setEscapeIndex(int index)
	{
		m_escapeIndexOrDataIndex = -index;
	}

	D_SIMD_FORCE_INLINE int getDataIndex() const
	{
		//D_btAssert(m_escapeIndexOrDataIndex >= 0);

		return m_escapeIndexOrDataIndex;
	}

	D_SIMD_FORCE_INLINE void setDataIndex(int index)
	{
		m_escapeIndexOrDataIndex = index;
	}

	D_SIMD_FORCE_INLINE bool testQuantizedBoxOverlapp(
		unsigned short * quantizedMin,unsigned short * quantizedMax) const
	{
		if(m_quantizedAabbMin[0] > quantizedMax[0] ||
		   m_quantizedAabbMax[0] < quantizedMin[0] ||
		   m_quantizedAabbMin[1] > quantizedMax[1] ||
		   m_quantizedAabbMax[1] < quantizedMin[1] ||
		   m_quantizedAabbMin[2] > quantizedMax[2] ||
		   m_quantizedAabbMax[2] < quantizedMin[2])
		{
			return false;
		}
		return true;
	}

};



class D_GIM_QUANTIZED_BVH_NODE_ARRAY:public D_btAlignedObjectArray<D_BT_QUANTIZED_BVH_NODE>
{
};




//! Basic D_Box tree structure
class D_btQuantizedBvhTree
{
protected:
	int m_num_nodes;
	D_GIM_QUANTIZED_BVH_NODE_ARRAY m_node_array;
	D_btAABB m_global_bound;
	D_btVector3 m_bvhQuantization;
protected:
	void calc_quantization(D_GIM_BVH_DATA_ARRAY & primitive_boxes, D_btScalar boundMargin = D_btScalar(1.0) );

	int _sort_and_calc_splitting_index(
		D_GIM_BVH_DATA_ARRAY & primitive_boxes,
		 int startIndex,  int endIndex, int splitAxis);

	int _calc_splitting_axis(D_GIM_BVH_DATA_ARRAY & primitive_boxes, int startIndex,  int endIndex);

	void _build_sub_tree(D_GIM_BVH_DATA_ARRAY & primitive_boxes, int startIndex,  int endIndex);
public:
	D_btQuantizedBvhTree()
	{
		m_num_nodes = 0;
	}

	//! prototype functions for box tree management
	//!@{
	void build_tree(D_GIM_BVH_DATA_ARRAY & primitive_boxes);

	D_SIMD_FORCE_INLINE void quantizePoint(
		unsigned short * quantizedpoint, const D_btVector3 & point) const
	{
		bt_quantize_clamp(quantizedpoint,point,m_global_bound.m_min,m_global_bound.m_max,m_bvhQuantization);
	}


	D_SIMD_FORCE_INLINE bool testQuantizedBoxOverlapp(
		int node_index,
		unsigned short * quantizedMin,unsigned short * quantizedMax) const
	{
		return m_node_array[node_index].testQuantizedBoxOverlapp(quantizedMin,quantizedMax);
	}

	D_SIMD_FORCE_INLINE void clearNodes()
	{
		m_node_array.clear();
		m_num_nodes = 0;
	}

	//! node count
	D_SIMD_FORCE_INLINE int getNodeCount() const
	{
		return m_num_nodes;
	}

	//! tells if the node D_is a leaf
	D_SIMD_FORCE_INLINE bool isLeafNode(int nodeindex) const
	{
		return m_node_array[nodeindex].isLeafNode();
	}

	D_SIMD_FORCE_INLINE int getNodeData(int nodeindex) const
	{
		return m_node_array[nodeindex].getDataIndex();
	}

	D_SIMD_FORCE_INLINE void getNodeBound(int nodeindex, D_btAABB & bound) const
	{
		bound.m_min = bt_unquantize(
			m_node_array[nodeindex].m_quantizedAabbMin,
			m_global_bound.m_min,m_bvhQuantization);

		bound.m_max = bt_unquantize(
			m_node_array[nodeindex].m_quantizedAabbMax,
			m_global_bound.m_min,m_bvhQuantization);
	}

	D_SIMD_FORCE_INLINE void setNodeBound(int nodeindex, const D_btAABB & bound)
	{
		bt_quantize_clamp(	m_node_array[nodeindex].m_quantizedAabbMin,
							bound.m_min,
							m_global_bound.m_min,
							m_global_bound.m_max,
							m_bvhQuantization);

		bt_quantize_clamp(	m_node_array[nodeindex].m_quantizedAabbMax,
							bound.m_max,
							m_global_bound.m_min,
							m_global_bound.m_max,
							m_bvhQuantization);
	}

	D_SIMD_FORCE_INLINE int getLeftNode(int nodeindex) const
	{
		return nodeindex+1;
	}

	D_SIMD_FORCE_INLINE int getRightNode(int nodeindex) const
	{
		if(m_node_array[nodeindex+1].isLeafNode()) return nodeindex+2;
		return nodeindex+1 + m_node_array[nodeindex+1].getEscapeIndex();
	}

	D_SIMD_FORCE_INLINE int getEscapeNodeIndex(int nodeindex) const
	{
		return m_node_array[nodeindex].getEscapeIndex();
	}

	D_SIMD_FORCE_INLINE const D_BT_QUANTIZED_BVH_NODE * get_node_pointer(int index = 0) const
	{
		return &m_node_array[index];
	}

	//!@}
};



//! Structure for containing Boxes
/*!
This class D_offers an structure for managing a box tree of primitives.
Requires a Primitive prototype (like D_btPrimitiveManagerBase )
*/
class D_btGImpactQuantizedBvh
{
protected:
	D_btQuantizedBvhTree m_box_tree;
	D_btPrimitiveManagerBase * m_primitive_manager;

protected:
	//stackless refit
	void refit();
public:

	//! this constructor doesn't build the tree. you D_must call	buildSet
	D_btGImpactQuantizedBvh()
	{
		m_primitive_manager = NULL;
	}

	//! this constructor doesn't build the tree. you D_must call	buildSet
	D_btGImpactQuantizedBvh(D_btPrimitiveManagerBase * primitive_manager)
	{
		m_primitive_manager = primitive_manager;
	}

	D_SIMD_FORCE_INLINE D_btAABB getGlobalBox()  const
	{
		D_btAABB totalbox;
		getNodeBound(0, totalbox);
		return totalbox;
	}

	D_SIMD_FORCE_INLINE void setPrimitiveManager(D_btPrimitiveManagerBase * primitive_manager)
	{
		m_primitive_manager = primitive_manager;
	}

	D_SIMD_FORCE_INLINE D_btPrimitiveManagerBase * getPrimitiveManager() const
	{
		return m_primitive_manager;
	}


//! node manager prototype functions
///@{

	//! this attemps D_to refit the box set.
	D_SIMD_FORCE_INLINE void update()
	{
		refit();
	}

	//! this rebuild the entire set
	void buildSet();

	//! returns the indices of the primitives in the m_primitive_manager
	bool boxQuery(const D_btAABB & box, D_btAlignedObjectArray<int> & collided_results) const;

	//! returns the indices of the primitives in the m_primitive_manager
	D_SIMD_FORCE_INLINE bool boxQueryTrans(const D_btAABB & box,
		 const D_btTransform & transform, D_btAlignedObjectArray<int> & collided_results) const
	{
		D_btAABB transbox=box;
		transbox.appy_transform(transform);
		return boxQuery(transbox,collided_results);
	}

	//! returns the indices of the primitives in the m_primitive_manager
	bool rayQuery(
		const D_btVector3 & ray_dir,const D_btVector3 & ray_origin ,
		D_btAlignedObjectArray<int> & collided_results) const;

	//! tells if this set has hierarcht
	D_SIMD_FORCE_INLINE bool hasHierarchy() const
	{
		return true;
	}

	//! tells if this set D_is a trimesh
	D_SIMD_FORCE_INLINE bool isTrimesh()  const
	{
		return m_primitive_manager->is_trimesh();
	}

	//! node count
	D_SIMD_FORCE_INLINE int getNodeCount() const
	{
		return m_box_tree.getNodeCount();
	}

	//! tells if the node D_is a leaf
	D_SIMD_FORCE_INLINE bool isLeafNode(int nodeindex) const
	{
		return m_box_tree.isLeafNode(nodeindex);
	}

	D_SIMD_FORCE_INLINE int getNodeData(int nodeindex) const
	{
		return m_box_tree.getNodeData(nodeindex);
	}

	D_SIMD_FORCE_INLINE void getNodeBound(int nodeindex, D_btAABB & bound)  const
	{
		m_box_tree.getNodeBound(nodeindex, bound);
	}

	D_SIMD_FORCE_INLINE void setNodeBound(int nodeindex, const D_btAABB & bound)
	{
		m_box_tree.setNodeBound(nodeindex, bound);
	}


	D_SIMD_FORCE_INLINE int getLeftNode(int nodeindex) const
	{
		return m_box_tree.getLeftNode(nodeindex);
	}

	D_SIMD_FORCE_INLINE int getRightNode(int nodeindex) const
	{
		return m_box_tree.getRightNode(nodeindex);
	}

	D_SIMD_FORCE_INLINE int getEscapeNodeIndex(int nodeindex) const
	{
		return m_box_tree.getEscapeNodeIndex(nodeindex);
	}

	D_SIMD_FORCE_INLINE void getNodeTriangle(int nodeindex,D_btPrimitiveTriangle & triangle) const
	{
		m_primitive_manager->get_primitive_triangle(getNodeData(nodeindex),triangle);
	}


	D_SIMD_FORCE_INLINE const D_BT_QUANTIZED_BVH_NODE * get_node_pointer(int index = 0) const
	{
		return m_box_tree.get_node_pointer(index);
	}


	static float getAverageTreeCollisionTime();


	static void find_collision(D_btGImpactQuantizedBvh * boxset1, const D_btTransform & trans1,
		D_btGImpactQuantizedBvh * boxset2, const D_btTransform & trans2,
		D_btPairSet & collision_pairs);
};


#endif // GIM_BOXPRUNING_H_INCLUDED
