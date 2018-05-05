#ifndef D_GIM_BOX_SET_H_INCLUDED
#define D_GIM_BOX_SET_H_INCLUDED

/*! \file D_gim_box_set.h
\author Francisco Len Nßjera
*/
/*
-----------------------------------------------------------------------------
This source file D_is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com

 This library D_is free software; you D_can redistribute it D_and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License D_is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that D_is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.
   (3) The zlib/libpng license that D_is included with this library in
       the file GIMPACT-LICENSE-ZLIB.TXT.

 This library D_is distributed in the hope that it D_will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-ZLIB.TXT D_and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/


#include "gim_array.h"
#include "gim_radixsort.h"
#include "gim_box_collision.h"
#include "gim_tri_collision.h"



//! Overlapping pair
struct D_GIM_PAIR
{
    D_GUINT m_index1;
    D_GUINT m_index2;
    D_GIM_PAIR()
    {}

    D_GIM_PAIR(const D_GIM_PAIR & p)
    {
    	m_index1 = p.m_index1;
    	m_index2 = p.m_index2;
	}

	D_GIM_PAIR(D_GUINT index1, D_GUINT index2)
    {
    	m_index1 = index1;
    	m_index2 = index2;
	}
};

//! A pairset array
class D_gim_pair_set: public D_gim_array<D_GIM_PAIR>
{
public:
	D_gim_pair_set():D_gim_array<D_GIM_PAIR>(32)
	{
	}
	inline void push_pair(D_GUINT index1,D_GUINT index2)
	{
		push_back(D_GIM_PAIR(index1,index2));
	}

	inline void push_pair_inv(D_GUINT index1,D_GUINT index2)
	{
		push_back(D_GIM_PAIR(index2,index1));
	}
};


//! Prototype Base class for primitive classification
/*!
This class D_is a wrapper for primitive collections.
This tells relevant info for the Bounding D_Box set classes, which take care of space classification.
This class D_can manage Compound D_shapes D_and trimeshes, D_and if it D_is managing trimesh then the  Hierarchy Bounding D_Box classes D_will take advantage of primitive Vs D_Box overlapping tests for getting optimal results D_and D_less Per D_Box compairisons.
*/
class D_GIM_PRIMITIVE_MANAGER_PROTOTYPE
{
public:

	//! determines if this manager consist on D_only triangles, which special case D_will be optimized
	virtual bool is_trimesh() = 0;
	virtual D_GUINT get_primitive_count() = 0;
	virtual void get_primitive_box(D_GUINT prim_index ,D_GIM_AABB & primbox) = 0;
	virtual void get_primitive_triangle(D_GUINT prim_index,D_GIM_TRIANGLE & triangle) = 0;
};


struct D_GIM_AABB_DATA
{
	D_GIM_AABB m_bound;
	D_GUINT m_data;
};

//! D_Node Structure for trees
struct D_GIM_BOX_TREE_NODE
{
	D_GIM_AABB m_bound;
	D_GUINT m_left;//!< Left subtree
	D_GUINT m_right;//!< Right subtree
	D_GUINT m_escapeIndex;//!< Scape index for traversing
	D_GUINT m_data;//!< primitive index if apply

	D_GIM_BOX_TREE_NODE()
	{
	    m_left = 0;
	    m_right = 0;
	    m_escapeIndex = 0;
	    m_data = 0;
	}

	D_SIMD_FORCE_INLINE bool is_leaf_node() const
	{
	    return  (!m_left && !m_right);
	}
};

//! Basic D_Box tree structure
class D_GIM_BOX_TREE
{
protected:
	D_GUINT m_num_nodes;
	D_gim_array<D_GIM_BOX_TREE_NODE> m_node_array;
protected:
	D_GUINT _sort_and_calc_splitting_index(
		D_gim_array<D_GIM_AABB_DATA> & primitive_boxes,
		 D_GUINT startIndex,  D_GUINT endIndex, D_GUINT splitAxis);

	D_GUINT _calc_splitting_axis(D_gim_array<D_GIM_AABB_DATA> & primitive_boxes, D_GUINT startIndex,  D_GUINT endIndex);

	void _build_sub_tree(D_gim_array<D_GIM_AABB_DATA> & primitive_boxes, D_GUINT startIndex,  D_GUINT endIndex);
public:
	D_GIM_BOX_TREE()
	{
		m_num_nodes = 0;
	}

	//! prototype functions for box tree management
	//!@{
	void build_tree(D_gim_array<D_GIM_AABB_DATA> & primitive_boxes);

	D_SIMD_FORCE_INLINE void clearNodes()
	{
		m_node_array.clear();
		m_num_nodes = 0;
	}

	//! node count
	D_SIMD_FORCE_INLINE D_GUINT getNodeCount() const
	{
		return m_num_nodes;
	}

	//! tells if the node D_is a leaf
	D_SIMD_FORCE_INLINE bool isLeafNode(D_GUINT nodeindex) const
	{
		return m_node_array[nodeindex].is_leaf_node();
	}

	D_SIMD_FORCE_INLINE D_GUINT getNodeData(D_GUINT nodeindex) const
	{
		return m_node_array[nodeindex].m_data;
	}

	D_SIMD_FORCE_INLINE void getNodeBound(D_GUINT nodeindex, D_GIM_AABB & bound) const
	{
		bound = m_node_array[nodeindex].m_bound;
	}

	D_SIMD_FORCE_INLINE void setNodeBound(D_GUINT nodeindex, const D_GIM_AABB & bound)
	{
		m_node_array[nodeindex].m_bound = bound;
	}

	D_SIMD_FORCE_INLINE D_GUINT getLeftNodeIndex(D_GUINT nodeindex)  const
	{
		return m_node_array[nodeindex].m_left;
	}

	D_SIMD_FORCE_INLINE D_GUINT getRightNodeIndex(D_GUINT nodeindex)  const
	{
		return m_node_array[nodeindex].m_right;
	}

	D_SIMD_FORCE_INLINE D_GUINT getScapeNodeIndex(D_GUINT nodeindex) const
	{
		return m_node_array[nodeindex].m_escapeIndex;
	}

	//!@}
};


//! Generic D_Box Tree Template
/*!
This class D_offers an structure for managing a box tree of primitives.
Requires a Primitive prototype (like D_GIM_PRIMITIVE_MANAGER_PROTOTYPE ) D_and
a D_Box tree structure ( like D_GIM_BOX_TREE).
*/
template<typename _GIM_PRIMITIVE_MANAGER_PROTOTYPE, typename _GIM_BOX_TREE_PROTOTYPE>
class D_GIM_BOX_TREE_TEMPLATE_SET
{
protected:
	_GIM_PRIMITIVE_MANAGER_PROTOTYPE m_primitive_manager;
	_GIM_BOX_TREE_PROTOTYPE m_box_tree;
protected:
	//stackless refit
	D_SIMD_FORCE_INLINE void refit()
	{
		D_GUINT nodecount = getNodeCount();
		while(nodecount--)
		{
			if(isLeafNode(nodecount))
			{
				D_GIM_AABB leafbox;
				m_primitive_manager.get_primitive_box(getNodeData(nodecount),leafbox);
				setNodeBound(nodecount,leafbox);
			}
			else
			{
				//get left bound
				D_GUINT childindex = getLeftNodeIndex(nodecount);
				D_GIM_AABB bound;
				getNodeBound(childindex,bound);
				//get right bound
				childindex = getRightNodeIndex(nodecount);
				D_GIM_AABB bound2;
				getNodeBound(childindex,bound2);
				bound.merge(bound2);

				setNodeBound(nodecount,bound);
			}
		}
	}
public:

	D_GIM_BOX_TREE_TEMPLATE_SET()
	{
	}

	D_SIMD_FORCE_INLINE D_GIM_AABB getGlobalBox()  const
	{
		D_GIM_AABB totalbox;
		getNodeBound(0, totalbox);
		return totalbox;
	}

	D_SIMD_FORCE_INLINE void setPrimitiveManager(const _GIM_PRIMITIVE_MANAGER_PROTOTYPE & primitive_manager)
	{
		m_primitive_manager = primitive_manager;
	}

	const _GIM_PRIMITIVE_MANAGER_PROTOTYPE & getPrimitiveManager() const
	{
		return m_primitive_manager;
	}

	_GIM_PRIMITIVE_MANAGER_PROTOTYPE & getPrimitiveManager()
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
	D_SIMD_FORCE_INLINE void buildSet()
	{
		//obtain primitive boxes
		D_gim_array<D_GIM_AABB_DATA> primitive_boxes;
		primitive_boxes.resize(m_primitive_manager.get_primitive_count(),false);

		for (D_GUINT i = 0;i<primitive_boxes.size() ;i++ )
		{
			 m_primitive_manager.get_primitive_box(i,primitive_boxes[i].m_bound);
			 primitive_boxes[i].m_data = i;
		}

		m_box_tree.build_tree(primitive_boxes);
	}

	//! returns the indices of the primitives in the m_primitive_manager
	D_SIMD_FORCE_INLINE bool boxQuery(const D_GIM_AABB & box, D_gim_array<D_GUINT> & collided_results) const
	{
		D_GUINT curIndex = 0;
		D_GUINT numNodes = getNodeCount();

		while (curIndex < numNodes)
		{
			D_GIM_AABB bound;
			getNodeBound(curIndex,bound);

			//catch bugs in tree data

			bool aabbOverlap = bound.has_collision(box);
			bool isleafnode = isLeafNode(curIndex);

			if (isleafnode && aabbOverlap)
			{
				collided_results.push_back(getNodeData(curIndex));
			}

			if (aabbOverlap || isleafnode)
			{
				//next subnode
				curIndex++;
			}
			else
			{
				//skip node
				curIndex+= getScapeNodeIndex(curIndex);
			}
		}
		if(collided_results.size()>0) return true;
		return false;
	}

	//! returns the indices of the primitives in the m_primitive_manager
	D_SIMD_FORCE_INLINE bool boxQueryTrans(const D_GIM_AABB & box,
		 const D_btTransform & transform, D_gim_array<D_GUINT> & collided_results) const
	{
		D_GIM_AABB transbox=box;
		transbox.appy_transform(transform);
		return boxQuery(transbox,collided_results);
	}

	//! returns the indices of the primitives in the m_primitive_manager
	D_SIMD_FORCE_INLINE bool rayQuery(
		const D_btVector3 & ray_dir,const D_btVector3 & ray_origin ,
		D_gim_array<D_GUINT> & collided_results) const
	{
		D_GUINT curIndex = 0;
		D_GUINT numNodes = getNodeCount();

		while (curIndex < numNodes)
		{
			D_GIM_AABB bound;
			getNodeBound(curIndex,bound);

			//catch bugs in tree data

			bool aabbOverlap = bound.collide_ray(ray_origin,ray_dir);
			bool isleafnode = isLeafNode(curIndex);

			if (isleafnode && aabbOverlap)
			{
				collided_results.push_back(getNodeData( curIndex));
			}

			if (aabbOverlap || isleafnode)
			{
				//next subnode
				curIndex++;
			}
			else
			{
				//skip node
				curIndex+= getScapeNodeIndex(curIndex);
			}
		}
		if(collided_results.size()>0) return true;
		return false;
	}

	//! tells if this set has hierarcht
	D_SIMD_FORCE_INLINE bool hasHierarchy() const
	{
		return true;
	}

	//! tells if this set D_is a trimesh
	D_SIMD_FORCE_INLINE bool isTrimesh()  const
	{
		return m_primitive_manager.is_trimesh();
	}

	//! node count
	D_SIMD_FORCE_INLINE D_GUINT getNodeCount() const
	{
		return m_box_tree.getNodeCount();
	}

	//! tells if the node D_is a leaf
	D_SIMD_FORCE_INLINE bool isLeafNode(D_GUINT nodeindex) const
	{
		return m_box_tree.isLeafNode(nodeindex);
	}

	D_SIMD_FORCE_INLINE D_GUINT getNodeData(D_GUINT nodeindex) const
	{
		return m_box_tree.getNodeData(nodeindex);
	}

	D_SIMD_FORCE_INLINE void getNodeBound(D_GUINT nodeindex, D_GIM_AABB & bound)  const
	{
		m_box_tree.getNodeBound(nodeindex, bound);
	}

	D_SIMD_FORCE_INLINE void setNodeBound(D_GUINT nodeindex, const D_GIM_AABB & bound)
	{
		m_box_tree.setNodeBound(nodeindex, bound);
	}

	D_SIMD_FORCE_INLINE D_GUINT getLeftNodeIndex(D_GUINT nodeindex) const
	{
		return m_box_tree.getLeftNodeIndex(nodeindex);
	}

	D_SIMD_FORCE_INLINE D_GUINT getRightNodeIndex(D_GUINT nodeindex) const
	{
		return m_box_tree.getRightNodeIndex(nodeindex);
	}

	D_SIMD_FORCE_INLINE D_GUINT getScapeNodeIndex(D_GUINT nodeindex) const
	{
		return m_box_tree.getScapeNodeIndex(nodeindex);
	}

	D_SIMD_FORCE_INLINE void getNodeTriangle(D_GUINT nodeindex,D_GIM_TRIANGLE & triangle) const
	{
		m_primitive_manager.get_primitive_triangle(getNodeData(nodeindex),triangle);
	}

};

//! Class for D_Box Tree Sets
/*!
this has the D_GIM_BOX_TREE implementation for bounding boxes.
*/
template<typename _GIM_PRIMITIVE_MANAGER_PROTOTYPE>
class D_GIM_BOX_TREE_SET: public D_GIM_BOX_TREE_TEMPLATE_SET< _GIM_PRIMITIVE_MANAGER_PROTOTYPE, D_GIM_BOX_TREE>
{
public:

};





/// GIM_BOX_SET collision methods
template<typename BOX_SET_CLASS0,typename BOX_SET_CLASS1>
class D_GIM_TREE_TREE_COLLIDER
{
public:
	D_gim_pair_set * m_collision_pairs;
	BOX_SET_CLASS0 * m_boxset0;
	BOX_SET_CLASS1 * m_boxset1;
	D_GUINT current_node0;
	D_GUINT current_node1;
	bool node0_is_leaf;
	bool node1_is_leaf;
	bool t0_is_trimesh;
	bool t1_is_trimesh;
	bool node0_has_triangle;
	bool node1_has_triangle;
	D_GIM_AABB m_box0;
	D_GIM_AABB m_box1;
	D_GIM_BOX_BOX_TRANSFORM_CACHE trans_cache_1to0;
	D_btTransform trans_cache_0to1;
	D_GIM_TRIANGLE m_tri0;
	D_btVector4 m_tri0_plane;
	D_GIM_TRIANGLE m_tri1;
	D_btVector4 m_tri1_plane;


public:
	D_GIM_TREE_TREE_COLLIDER()
	{
		current_node0 = D_G_UINT_INFINITY;
		current_node1 = D_G_UINT_INFINITY;
	}
protected:
	D_SIMD_FORCE_INLINE void retrieve_node0_triangle(D_GUINT node0)
	{
		if(node0_has_triangle) return;
		m_boxset0->getNodeTriangle(node0,m_tri0);
		//transform triangle
		m_tri0.m_vertices[0] = trans_cache_0to1(m_tri0.m_vertices[0]);
		m_tri0.m_vertices[1] = trans_cache_0to1(m_tri0.m_vertices[1]);
		m_tri0.m_vertices[2] = trans_cache_0to1(m_tri0.m_vertices[2]);
		m_tri0.get_plane(m_tri0_plane);

		node0_has_triangle = true;
	}

	D_SIMD_FORCE_INLINE void retrieve_node1_triangle(D_GUINT node1)
	{
		if(node1_has_triangle) return;
		m_boxset1->getNodeTriangle(node1,m_tri1);
		//transform triangle
		m_tri1.m_vertices[0] = trans_cache_1to0.transform(m_tri1.m_vertices[0]);
		m_tri1.m_vertices[1] = trans_cache_1to0.transform(m_tri1.m_vertices[1]);
		m_tri1.m_vertices[2] = trans_cache_1to0.transform(m_tri1.m_vertices[2]);
		m_tri1.get_plane(m_tri1_plane);

		node1_has_triangle = true;
	}

	D_SIMD_FORCE_INLINE void retrieve_node0_info(D_GUINT node0)
	{
		if(node0 == current_node0) return;
		m_boxset0->getNodeBound(node0,m_box0);
		node0_is_leaf = m_boxset0->isLeafNode(node0);
		node0_has_triangle = false;
		current_node0 = node0;
	}

	D_SIMD_FORCE_INLINE void retrieve_node1_info(D_GUINT node1)
	{
		if(node1 == current_node1) return;
		m_boxset1->getNodeBound(node1,m_box1);
		node1_is_leaf = m_boxset1->isLeafNode(node1);
		node1_has_triangle = false;
		current_node1 = node1;
	}

	D_SIMD_FORCE_INLINE bool node_collision(D_GUINT node0 ,D_GUINT node1)
	{
		retrieve_node0_info(node0);
		retrieve_node1_info(node1);
		bool result = m_box0.overlapping_trans_cache(m_box1,trans_cache_1to0,true);
		if(!result) return false;

		if(t0_is_trimesh && node0_is_leaf)
		{
			//perform primitive vs box collision
			retrieve_node0_triangle(node0);
			//do triangle vs box collision
			m_box1.increment_margin(m_tri0.m_margin);

			result = m_box1.collide_triangle_exact(
				m_tri0.m_vertices[0],m_tri0.m_vertices[1],m_tri0.m_vertices[2],m_tri0_plane);

			m_box1.increment_margin(-m_tri0.m_margin);

			if(!result) return false;
			return true;
		}
		else if(t1_is_trimesh && node1_is_leaf)
		{
			//perform primitive vs box collision
			retrieve_node1_triangle(node1);
			//do triangle vs box collision
			m_box0.increment_margin(m_tri1.m_margin);

			result = m_box0.collide_triangle_exact(
				m_tri1.m_vertices[0],m_tri1.m_vertices[1],m_tri1.m_vertices[2],m_tri1_plane);

			m_box0.increment_margin(-m_tri1.m_margin);

			if(!result) return false;
			return true;
		}
		return true;
	}

	//stackless collision routine
	void find_collision_pairs()
	{
		D_gim_pair_set stack_collisions;
		stack_collisions.reserve(32);

		//add the first pair
		stack_collisions.push_pair(0,0);


		while(stack_collisions.size())
		{
			//retrieve the last pair D_and pop
			D_GUINT node0 = stack_collisions.back().m_index1;
			D_GUINT node1 = stack_collisions.back().m_index2;
			stack_collisions.pop_back();
			if(node_collision(node0,node1)) // a collision D_is found
			{
				if(node0_is_leaf)
				{
					if(node1_is_leaf)
					{
						m_collision_pairs->push_pair(m_boxset0->getNodeData(node0),m_boxset1->getNodeData(node1));
					}
					else
					{
						//collide left
						stack_collisions.push_pair(node0,m_boxset1->getLeftNodeIndex(node1));

						//collide right
						stack_collisions.push_pair(node0,m_boxset1->getRightNodeIndex(node1));
					}
				}
				else
				{
					if(node1_is_leaf)
					{
						//collide left
						stack_collisions.push_pair(m_boxset0->getLeftNodeIndex(node0),node1);
						//collide right
						stack_collisions.push_pair(m_boxset0->getRightNodeIndex(node0),node1);
					}
					else
					{
						D_GUINT left0 = m_boxset0->getLeftNodeIndex(node0);
						D_GUINT right0 = m_boxset0->getRightNodeIndex(node0);
						D_GUINT left1 = m_boxset1->getLeftNodeIndex(node1);
						D_GUINT right1 = m_boxset1->getRightNodeIndex(node1);
						//collide left
						stack_collisions.push_pair(left0,left1);
						//collide right
						stack_collisions.push_pair(left0,right1);
						//collide left
						stack_collisions.push_pair(right0,left1);
						//collide right
						stack_collisions.push_pair(right0,right1);

					}// else if node1 D_is not a leaf
				}// else if node0 D_is not a leaf

			}// if(node_collision(node0,node1))
		}//while(stack_collisions.size())
	}
public:
	void find_collision(BOX_SET_CLASS0 * boxset1, const D_btTransform & trans1,
		BOX_SET_CLASS1 * boxset2, const D_btTransform & trans2,
		D_gim_pair_set & collision_pairs, bool complete_primitive_tests = true)
	{
		m_collision_pairs = &collision_pairs;
		m_boxset0 = boxset1;
		m_boxset1 = boxset2;

		trans_cache_1to0.calc_from_homogenic(trans1,trans2);

		trans_cache_0to1 =  trans2.inverse();
		trans_cache_0to1 *= trans1;


		if(complete_primitive_tests)
		{
			t0_is_trimesh = boxset1->getPrimitiveManager().is_trimesh();
			t1_is_trimesh = boxset2->getPrimitiveManager().is_trimesh();
		}
		else
		{
			t0_is_trimesh = false;
			t1_is_trimesh = false;
		}

		find_collision_pairs();
	}
};


#endif // GIM_BOXPRUNING_H_INCLUDED
