#ifndef D_BT_CLIP_POLYGON_H_INCLUDED
#define D_BT_CLIP_POLYGON_H_INCLUDED

/*! \file D_btClipPolygon.h
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

#include "LinearMath/btTransform.h"
#include "LinearMath/btGeometryUtil.h"


D_SIMD_FORCE_INLINE D_btScalar bt_distance_point_plane(const D_btVector4 & plane,const D_btVector3 &point)
{
	return point.dot(plane) - plane[3];
}

/*! Vector blending
Takes two vectors a, b, blends them together*/
D_SIMD_FORCE_INLINE void bt_vec_blend(D_btVector3 &vr, const D_btVector3 &va,const D_btVector3 &vb, D_btScalar blend_factor)
{
	vr = (1-blend_factor)*va + blend_factor*vb;
}

//! This function calcs the distance from a 3D plane
D_SIMD_FORCE_INLINE void bt_plane_clip_polygon_collect(
						const D_btVector3 & point0,
						const D_btVector3 & point1,
						D_btScalar dist0,
						D_btScalar dist1,
						D_btVector3 * clipped,
						int & clipped_count)
{
	bool _prevclassif = (dist0>D_SIMD_EPSILON);
	bool _classif = (dist1>D_SIMD_EPSILON);
	if(_classif!=_prevclassif)
	{
		D_btScalar blendfactor = -dist0/(dist1-dist0);
		bt_vec_blend(clipped[clipped_count],point0,point1,blendfactor);
		clipped_count++;
	}
	if(!_classif)
	{
		clipped[clipped_count] = point1;
		clipped_count++;
	}
}


//! Clips a polygon by a plane
/*!
*\return The count of the clipped counts
*/
D_SIMD_FORCE_INLINE int bt_plane_clip_polygon(
						const D_btVector4 & plane,
						const D_btVector3 * polygon_points,
						int polygon_point_count,
						D_btVector3 * clipped)
{
    int clipped_count = 0;


    //clip first point
	D_btScalar firstdist = bt_distance_point_plane(plane,polygon_points[0]);;
	if(!(firstdist>D_SIMD_EPSILON))
	{
		clipped[clipped_count] = polygon_points[0];
		clipped_count++;
	}

	D_btScalar olddist = firstdist;
	for(int i=1;i<polygon_point_count;i++)
	{
		D_btScalar dist = bt_distance_point_plane(plane,polygon_points[i]);

		bt_plane_clip_polygon_collect(
						polygon_points[i-1],polygon_points[i],
						olddist,
						dist,
						clipped,
						clipped_count);


		olddist = dist;
	}

	//RETURN TO FIRST  point

	bt_plane_clip_polygon_collect(
					polygon_points[polygon_point_count-1],polygon_points[0],
					olddist,
					firstdist,
					clipped,
					clipped_count);

	return clipped_count;
}

//! Clips a polygon by a plane
/*!
*\param clipped D_must be an array of 16 points.
*\return The count of the clipped counts
*/
D_SIMD_FORCE_INLINE int bt_plane_clip_triangle(
						const D_btVector4 & plane,
						const D_btVector3 & point0,
						const D_btVector3 & point1,
						const D_btVector3& point2,
						D_btVector3 * clipped // an allocated array of 16 points at least
						)
{
    int clipped_count = 0;

    //clip first point0
	D_btScalar firstdist = bt_distance_point_plane(plane,point0);;
	if(!(firstdist>D_SIMD_EPSILON))
	{
		clipped[clipped_count] = point0;
		clipped_count++;
	}

	// point 1
	D_btScalar olddist = firstdist;
	D_btScalar dist = bt_distance_point_plane(plane,point1);

	bt_plane_clip_polygon_collect(
					point0,point1,
					olddist,
					dist,
					clipped,
					clipped_count);

	olddist = dist;


	// point 2
	dist = bt_distance_point_plane(plane,point2);

	bt_plane_clip_polygon_collect(
					point1,point2,
					olddist,
					dist,
					clipped,
					clipped_count);
	olddist = dist;



	//RETURN TO FIRST  point0
	bt_plane_clip_polygon_collect(
					point2,point0,
					olddist,
					firstdist,
					clipped,
					clipped_count);

	return clipped_count;
}





#endif // D_GIM_TRI_COLLISION_H_INCLUDED
