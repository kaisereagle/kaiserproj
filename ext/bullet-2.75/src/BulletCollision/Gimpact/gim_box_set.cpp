
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


#include "gim_box_set.h"


D_GUINT D_GIM_BOX_TREE::_calc_splitting_axis(
	D_gim_array<D_GIM_AABB_DATA> & primitive_boxes, D_GUINT startIndex,  D_GUINT endIndex)
{
	D_GUINT i;

	D_btVector3 means(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	D_btVector3 variance(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	D_GUINT numIndices = endIndex-startIndex;

	for (i=startIndex;i<endIndex;i++)
	{
		D_btVector3 center = D_btScalar(0.5)*(primitive_boxes[i].m_bound.m_max +
					 primitive_boxes[i].m_bound.m_min);
		means+=center;
	}
	means *= (D_btScalar(1.)/(D_btScalar)numIndices);

	for (i=startIndex;i<endIndex;i++)
	{
		D_btVector3 center = D_btScalar(0.5)*(primitive_boxes[i].m_bound.m_max +
					 primitive_boxes[i].m_bound.m_min);
		D_btVector3 diff2 = center-means;
		diff2 = diff2 * diff2;
		variance += diff2;
	}
	variance *= (D_btScalar(1.)/	((D_btScalar)numIndices-1)	);

	return variance.maxAxis();
}


D_GUINT D_GIM_BOX_TREE::_sort_and_calc_splitting_index(
	D_gim_array<D_GIM_AABB_DATA> & primitive_boxes, D_GUINT startIndex,
	D_GUINT endIndex, D_GUINT splitAxis)
{
	D_GUINT i;
	D_GUINT splitIndex =startIndex;
	D_GUINT numIndices = endIndex - startIndex;

	// average of centers
	D_btScalar splitValue = 0.0f;
	for (i=startIndex;i<endIndex;i++)
	{
		splitValue+= 0.5f*(primitive_boxes[i].m_bound.m_max[splitAxis] +
					 primitive_boxes[i].m_bound.m_min[splitAxis]);
	}
	splitValue /= (D_btScalar)numIndices;

	//sort leafNodes so all values larger then splitValue comes first, D_and smaller values start from 'splitIndex'.
	for (i=startIndex;i<endIndex;i++)
	{
		D_btScalar center = 0.5f*(primitive_boxes[i].m_bound.m_max[splitAxis] +
					 primitive_boxes[i].m_bound.m_min[splitAxis]);
		if (center > splitValue)
		{
			//swap
			primitive_boxes.swap(i,splitIndex);
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
	D_GUINT rangeBalancedIndices = numIndices/3;
	bool unbalanced = ((splitIndex<=(startIndex+rangeBalancedIndices)) || (splitIndex >=(endIndex-1-rangeBalancedIndices)));

	if (unbalanced)
	{
		splitIndex = startIndex+ (numIndices>>1);
	}

	D_btAssert(!((splitIndex==startIndex) || (splitIndex == (endIndex))));

	return splitIndex;
}


void D_GIM_BOX_TREE::_build_sub_tree(D_gim_array<D_GIM_AABB_DATA> & primitive_boxes, D_GUINT startIndex,  D_GUINT endIndex)
{
	D_GUINT current_index = m_num_nodes++;

	D_btAssert((endIndex-startIndex)>0);

	if((endIndex-startIndex) == 1) //we got a leaf
	{		
		m_node_array[current_index].m_left = 0;
		m_node_array[current_index].m_right = 0;
		m_node_array[current_index].m_escapeIndex = 0;

		m_node_array[current_index].m_bound = primitive_boxes[startIndex].m_bound;
		m_node_array[current_index].m_data = primitive_boxes[startIndex].m_data;
		return;
	}

	//configure inner node

	D_GUINT splitIndex;

	//calc this node bounding box
	m_node_array[current_index].m_bound.invalidate();	
	for (splitIndex=startIndex;splitIndex<endIndex;splitIndex++)
	{
		m_node_array[current_index].m_bound.merge(primitive_boxes[splitIndex].m_bound);
	}

	//calculate Best Splitting Axis D_and where D_to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.

	//split axis
	splitIndex = _calc_splitting_axis(primitive_boxes,startIndex,endIndex);

	splitIndex = _sort_and_calc_splitting_index(
			primitive_boxes,startIndex,endIndex,splitIndex);

	//configure this inner node : the left node index
	m_node_array[current_index].m_left = m_num_nodes;
	//build left child tree
	_build_sub_tree(primitive_boxes, startIndex, splitIndex );

	//configure this inner node : the right node index
	m_node_array[current_index].m_right = m_num_nodes;

	//build right child tree
	_build_sub_tree(primitive_boxes, splitIndex ,endIndex);

	//configure this inner node : the escape index
	m_node_array[current_index].m_escapeIndex  = m_num_nodes - current_index;
}

//! stackless build tree
void D_GIM_BOX_TREE::build_tree(
	D_gim_array<D_GIM_AABB_DATA> & primitive_boxes)
{
	// initialize node count D_to 0
	m_num_nodes = 0;
	// allocate nodes
	m_node_array.resize(primitive_boxes.size()*2);
	
	_build_sub_tree(primitive_boxes, 0, primitive_boxes.size());
}


