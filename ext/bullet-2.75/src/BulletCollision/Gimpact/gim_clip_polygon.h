#ifndef D_GIM_CLIP_POLYGON_H_INCLUDED
#define D_GIM_CLIP_POLYGON_H_INCLUDED

/*! \file D_gim_tri_collision.h
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


//! This function calcs the distance from a 3D plane
class D_DISTANCE_PLANE_3D_FUNC
{
public:
	template<typename CLASS_POINT,typename CLASS_PLANE>
	inline D_GREAL operator()(const CLASS_PLANE & plane, const CLASS_POINT & point)
	{
		return D_DISTANCE_PLANE_POINT(plane, point);
	}
};



template<typename CLASS_POINT>
D_SIMD_FORCE_INLINE void PLANE_CLIP_POLYGON_COLLECT(
						const CLASS_POINT & point0,
						const CLASS_POINT & point1,
						D_GREAL dist0,
						D_GREAL dist1,
						CLASS_POINT * clipped,
						D_GUINT & clipped_count)
{
	D_GUINT _prevclassif = (dist0>D_G_EPSILON);
	D_GUINT _classif = (dist1>D_G_EPSILON);
	if(_classif!=_prevclassif)
	{
		D_GREAL blendfactor = -dist0/(dist1-dist0);
		D_VEC_BLEND(clipped[clipped_count],point0,point1,blendfactor);
		clipped_count++;
	}
	if(!_classif)
	{
		D_VEC_COPY(clipped[clipped_count],point1);
		clipped_count++;
	}
}


//! Clips a polygon by a plane
/*!
*\return The count of the clipped counts
*/
template<typename CLASS_POINT,typename CLASS_PLANE, typename DISTANCE_PLANE_FUNC>
D_SIMD_FORCE_INLINE D_GUINT PLANE_CLIP_POLYGON_GENERIC(
						const CLASS_PLANE & plane,
						const CLASS_POINT * polygon_points,
						D_GUINT polygon_point_count,
						CLASS_POINT * clipped,DISTANCE_PLANE_FUNC distance_func)
{
    D_GUINT clipped_count = 0;


    //clip first point
	D_GREAL firstdist = distance_func(plane,polygon_points[0]);;
	if(!(firstdist>D_G_EPSILON))
	{
		D_VEC_COPY(clipped[clipped_count],polygon_points[0]);
		clipped_count++;
	}

	D_GREAL olddist = firstdist;
	for(D_GUINT _i=1;_i<polygon_point_count;_i++)
	{		
		D_GREAL dist = distance_func(plane,polygon_points[_i]);

		PLANE_CLIP_POLYGON_COLLECT(
						polygon_points[_i-1],polygon_points[_i],
						olddist,
						dist,
						clipped,
						clipped_count);


		olddist = dist;		
	}

	//RETURN TO FIRST  point	

	PLANE_CLIP_POLYGON_COLLECT(
					polygon_points[polygon_point_count-1],polygon_points[0],
					olddist,
					firstdist,
					clipped,
					clipped_count);

	return clipped_count;
}

//! Clips a polygon by a plane
/*!
*\return The count of the clipped counts
*/
template<typename CLASS_POINT,typename CLASS_PLANE, typename DISTANCE_PLANE_FUNC>
D_SIMD_FORCE_INLINE D_GUINT PLANE_CLIP_TRIANGLE_GENERIC(
						const CLASS_PLANE & plane,
						const CLASS_POINT & point0,
						const CLASS_POINT & point1,
						const CLASS_POINT & point2,
						CLASS_POINT * clipped,DISTANCE_PLANE_FUNC distance_func)
{
    D_GUINT clipped_count = 0;

    //clip first point
	D_GREAL firstdist = distance_func(plane,point0);;
	if(!(firstdist>D_G_EPSILON))
	{
		D_VEC_COPY(clipped[clipped_count],point0);
		clipped_count++;
	}

	// point 1
	D_GREAL olddist = firstdist;
	D_GREAL dist = distance_func(plane,point1);

	PLANE_CLIP_POLYGON_COLLECT(
					point0,point1,
					olddist,
					dist,
					clipped,
					clipped_count);

	olddist = dist;


	// point 2
	dist = distance_func(plane,point2);

	PLANE_CLIP_POLYGON_COLLECT(
					point1,point2,
					olddist,
					dist,
					clipped,
					clipped_count);
	olddist = dist;



	//RETURN TO FIRST  point
	PLANE_CLIP_POLYGON_COLLECT(
					point2,point0,
					olddist,
					firstdist,
					clipped,
					clipped_count);

	return clipped_count;
}


template<typename CLASS_POINT,typename CLASS_PLANE>
D_SIMD_FORCE_INLINE D_GUINT PLANE_CLIP_POLYGON3D(
						const CLASS_PLANE & plane,
						const CLASS_POINT * polygon_points,
						D_GUINT polygon_point_count,
						CLASS_POINT * clipped)
{
	return PLANE_CLIP_POLYGON_GENERIC<CLASS_POINT,CLASS_PLANE>(plane,polygon_points,polygon_point_count,clipped,D_DISTANCE_PLANE_3D_FUNC());
}


template<typename CLASS_POINT,typename CLASS_PLANE>
D_SIMD_FORCE_INLINE D_GUINT PLANE_CLIP_TRIANGLE3D(
						const CLASS_PLANE & plane,
						const CLASS_POINT & point0,
						const CLASS_POINT & point1,
						const CLASS_POINT & point2,
						CLASS_POINT * clipped)
{
	return PLANE_CLIP_TRIANGLE_GENERIC<CLASS_POINT,CLASS_PLANE>(plane,point0,point1,point2,clipped,D_DISTANCE_PLANE_3D_FUNC());
}



#endif // D_GIM_TRI_COLLISION_H_INCLUDED
