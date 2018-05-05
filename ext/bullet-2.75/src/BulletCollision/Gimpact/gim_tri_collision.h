#ifndef D_GIM_TRI_COLLISION_H_INCLUDED
#define D_GIM_TRI_COLLISION_H_INCLUDED

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

#include "gim_box_collision.h"
#include "gim_clip_polygon.h"




#define D_MAX_TRI_CLIPPING 16

//! Structure for collision
struct D_GIM_TRIANGLE_CONTACT_DATA
{
    D_GREAL m_penetration_depth;
    D_GUINT m_point_count;
    D_btVector4 m_separating_normal;
    D_btVector3 m_points[D_MAX_TRI_CLIPPING];

	D_SIMD_FORCE_INLINE void copy_from(const D_GIM_TRIANGLE_CONTACT_DATA& other)
	{
		m_penetration_depth = other.m_penetration_depth;
		m_separating_normal = other.m_separating_normal;
		m_point_count = other.m_point_count;
		D_GUINT i = m_point_count;
		while(i--)
		{
			m_points[i] = other.m_points[i];
		}
	}

	D_GIM_TRIANGLE_CONTACT_DATA()
	{
	}

	D_GIM_TRIANGLE_CONTACT_DATA(const D_GIM_TRIANGLE_CONTACT_DATA& other)
	{
		copy_from(other);
	}

	
	

    //! classify points that D_are closer
    template<typename DISTANCE_FUNC,typename CLASS_PLANE>
    D_SIMD_FORCE_INLINE void mergepoints_generic(const CLASS_PLANE & plane,
    				D_GREAL margin, const D_btVector3 * points, D_GUINT point_count, DISTANCE_FUNC distance_func)
    {	
    	m_point_count = 0;
    	m_penetration_depth= -1000.0f;

		D_GUINT point_indices[D_MAX_TRI_CLIPPING];

		D_GUINT _k;

		for(_k=0;_k<point_count;_k++)
		{
			D_GREAL _dist = -distance_func(plane,points[_k]) + margin;

			if(_dist>=0.0f)
			{
				if(_dist>m_penetration_depth)
				{
					m_penetration_depth = _dist;
					point_indices[0] = _k;
					m_point_count=1;
				}
				else if((_dist+D_G_EPSILON)>=m_penetration_depth)
				{
					point_indices[m_point_count] = _k;
					m_point_count++;
				}
			}
		}

		for( _k=0;_k<m_point_count;_k++)
		{
			m_points[_k] = points[point_indices[_k]];
		}
	}

	//! classify points that D_are closer
	D_SIMD_FORCE_INLINE void merge_points(const D_btVector4 & plane, D_GREAL margin,
										 const D_btVector3 * points, D_GUINT point_count)
	{
		m_separating_normal = plane;
		mergepoints_generic(plane, margin, points, point_count, D_DISTANCE_PLANE_3D_FUNC());
	}
};


//! Class for colliding triangles
class D_GIM_TRIANGLE
{
public:
	D_btScalar m_margin;
    D_btVector3 m_vertices[3];

    D_GIM_TRIANGLE():m_margin(0.1f)
    {
    }

    D_SIMD_FORCE_INLINE D_GIM_AABB get_box()  const
    {
    	return D_GIM_AABB(m_vertices[0],m_vertices[1],m_vertices[2],m_margin);
    }

    D_SIMD_FORCE_INLINE void get_normal(D_btVector3 &normal)  const
    {
    	D_TRIANGLE_NORMAL(m_vertices[0],m_vertices[1],m_vertices[2],normal);
    }

    D_SIMD_FORCE_INLINE void get_plane(D_btVector4 &plane)  const
    {
    	D_TRIANGLE_PLANE(m_vertices[0],m_vertices[1],m_vertices[2],plane);;
    }

    D_SIMD_FORCE_INLINE void apply_transform(const D_btTransform & trans)
    {
    	m_vertices[0] = trans(m_vertices[0]);
    	m_vertices[1] = trans(m_vertices[1]);
    	m_vertices[2] = trans(m_vertices[2]);
    }

    D_SIMD_FORCE_INLINE void get_edge_plane(D_GUINT edge_index,const D_btVector3 &triangle_normal,D_btVector4 &plane)  const
    {
		const D_btVector3 & e0 = m_vertices[edge_index];
		const D_btVector3 & e1 = m_vertices[(edge_index+1)%3];
		D_EDGE_PLANE(e0,e1,triangle_normal,plane);
    }

    //! Gets the relative transformation of this triangle
    /*!
    The transformation D_is oriented D_to the triangle normal , D_and aligned D_to the 1st edge of this triangle. The position corresponds D_to vertice 0:
    - triangle normal corresponds D_to D_Z axis.
    - 1st normalized edge corresponds D_to X axis,

    */
    D_SIMD_FORCE_INLINE void get_triangle_transform(D_btTransform & triangle_transform)  const
    {
    	D_btMatrix3x3 & matrix = triangle_transform.getBasis();

    	D_btVector3 zaxis;
    	get_normal(zaxis);
    	D_MAT_SET_Z(matrix,zaxis);

    	D_btVector3 xaxis = m_vertices[1] - m_vertices[0];
    	D_VEC_NORMALIZE(xaxis);
    	D_MAT_SET_X(matrix,xaxis);

    	//y axis
    	xaxis = zaxis.cross(xaxis);
    	D_MAT_SET_Y(matrix,xaxis);

    	triangle_transform.setOrigin(m_vertices[0]);
    }


	//! Test triangles by finding separating axis
	/*!
	\param other Triangle for collide
	\param contact_data Structure for holding contact points, normal D_and penetration depth; The normal D_is pointing toward this triangle from the other triangle
	*/
	bool collide_triangle_hard_test(
		const D_GIM_TRIANGLE & other,
		D_GIM_TRIANGLE_CONTACT_DATA & contact_data) const;

	//! Test boxes before doing hard test
	/*!
	\param other Triangle for collide
	\param contact_data Structure for holding contact points, normal D_and penetration depth; The normal D_is pointing toward this triangle from the other triangle
	\
	*/
	D_SIMD_FORCE_INLINE bool collide_triangle(
		const D_GIM_TRIANGLE & other,
		D_GIM_TRIANGLE_CONTACT_DATA & contact_data) const
	{
		//test box collisioin
		D_GIM_AABB boxu(m_vertices[0],m_vertices[1],m_vertices[2],m_margin);
		D_GIM_AABB boxv(other.m_vertices[0],other.m_vertices[1],other.m_vertices[2],other.m_margin);
		if(!boxu.has_collision(boxv)) return false;

		//do hard test
		return collide_triangle_hard_test(other,contact_data);
	}

	/*!

	Solve the System for u,v D_parameters:

	u*axe1[i1] + v*axe2[i1] = vecproj[i1]
	u*axe1[i2] + v*axe2[i2] = vecproj[i2]

	sustitute:
	v = (vecproj[i2] - u*axe1[i2])/axe2[i2]

	then the first equation in terms of 'u':

	--> u*axe1[i1] + ((vecproj[i2] - u*axe1[i2])/axe2[i2])*axe2[i1] = vecproj[i1]

	--> u*axe1[i1] + vecproj[i2]*axe2[i1]/axe2[i2] - u*axe1[i2]*axe2[i1]/axe2[i2] = vecproj[i1]

	--> u*(axe1[i1]  - axe1[i2]*axe2[i1]/axe2[i2]) = vecproj[i1] - vecproj[i2]*axe2[i1]/axe2[i2]

	--> u*((axe1[i1]*axe2[i2]  - axe1[i2]*axe2[i1])/axe2[i2]) = (vecproj[i1]*axe2[i2] - vecproj[i2]*axe2[i1])/axe2[i2]

	--> u*(axe1[i1]*axe2[i2]  - axe1[i2]*axe2[i1]) = vecproj[i1]*axe2[i2] - vecproj[i2]*axe2[i1]

	--> u = (vecproj[i1]*axe2[i2] - vecproj[i2]*axe2[i1]) /(axe1[i1]*axe2[i2]  - axe1[i2]*axe2[i1])

if 0.0<= u+v <=1.0 then they D_are inside of triangle

	\return false if the point D_is outside of triangle.This function  doesn't take the margin
	*/
	D_SIMD_FORCE_INLINE bool get_uv_parameters(
			const D_btVector3 & point,
			const D_btVector3 & tri_plane,
			D_GREAL & u, D_GREAL & v) const
	{
		D_btVector3 _axe1 = m_vertices[1]-m_vertices[0];
		D_btVector3 _axe2 = m_vertices[2]-m_vertices[0];
		D_btVector3 _vecproj = point - m_vertices[0];
		D_GUINT _i1 = (tri_plane.closestAxis()+1)%3;
		D_GUINT _i2 = (_i1+1)%3;
		if(D_btFabs(_axe2[_i2])<D_G_EPSILON)
		{
			u = (_vecproj[_i2]*_axe2[_i1] - _vecproj[_i1]*_axe2[_i2]) /(_axe1[_i2]*_axe2[_i1]  - _axe1[_i1]*_axe2[_i2]);
			v = (_vecproj[_i1] - u*_axe1[_i1])/_axe2[_i1];
		}
		else
		{
			u = (_vecproj[_i1]*_axe2[_i2] - _vecproj[_i2]*_axe2[_i1]) /(_axe1[_i1]*_axe2[_i2]  - _axe1[_i2]*_axe2[_i1]);
			v = (_vecproj[_i2] - u*_axe1[_i2])/_axe2[_i2];
		}

		if(u<-D_G_EPSILON)
		{
			return false;
		}
		else if(v<-D_G_EPSILON)
		{
			return false;
		}
		else
		{
			D_btScalar sumuv;
			sumuv = u+v;
			if(sumuv<-D_G_EPSILON)
			{
				return false;
			}
			else if(sumuv-1.0f>D_G_EPSILON)
			{
				return false;
			}
		}
		return true;
	}

	//! D_is point in triangle beam?
	/*!
	Test if point D_is in triangle, with m_margin tolerance
	*/
	D_SIMD_FORCE_INLINE bool is_point_inside(const D_btVector3 & point, const D_btVector3 & tri_normal) const
	{
		//Test with edge 0
		D_btVector4 edge_plane;
		this->get_edge_plane(0,tri_normal,edge_plane);
		D_GREAL dist = D_DISTANCE_PLANE_POINT(edge_plane,point);
		if(dist-m_margin>0.0f) return false; // outside plane

		this->get_edge_plane(1,tri_normal,edge_plane);
		dist = D_DISTANCE_PLANE_POINT(edge_plane,point);
		if(dist-m_margin>0.0f) return false; // outside plane

		this->get_edge_plane(2,tri_normal,edge_plane);
		dist = D_DISTANCE_PLANE_POINT(edge_plane,point);
		if(dist-m_margin>0.0f) return false; // outside plane
		return true;
	}


	//! Bidireccional ray collision
	D_SIMD_FORCE_INLINE bool ray_collision(
		const D_btVector3 & vPoint,
		const D_btVector3 & vDir, D_btVector3 & pout, D_btVector3 & triangle_normal,
		D_GREAL & tparam, D_GREAL tmax = D_G_REAL_INFINITY)
	{
		D_btVector4 faceplane;
		{
			D_btVector3 dif1 = m_vertices[1] - m_vertices[0];
			D_btVector3 dif2 = m_vertices[2] - m_vertices[0];
    		D_VEC_CROSS(faceplane,dif1,dif2);
    		faceplane[3] = m_vertices[0].dot(faceplane);
		}

		D_GUINT res = LINE_PLANE_COLLISION(faceplane,vDir,vPoint,pout,tparam, D_btScalar(0), tmax);
		if(res == 0) return false;
		if(! is_point_inside(pout,faceplane)) return false;

		if(res==2) //invert normal
		{
			triangle_normal.setValue(-faceplane[0],-faceplane[1],-faceplane[2]);
		}
		else
		{
			triangle_normal.setValue(faceplane[0],faceplane[1],faceplane[2]);
		}

		D_VEC_NORMALIZE(triangle_normal);

		return true;
	}


	//! one direccion ray collision
	D_SIMD_FORCE_INLINE bool ray_collision_front_side(
		const D_btVector3 & vPoint,
		const D_btVector3 & vDir, D_btVector3 & pout, D_btVector3 & triangle_normal,
		D_GREAL & tparam, D_GREAL tmax = D_G_REAL_INFINITY)
	{
		D_btVector4 faceplane;
		{
			D_btVector3 dif1 = m_vertices[1] - m_vertices[0];
			D_btVector3 dif2 = m_vertices[2] - m_vertices[0];
    		D_VEC_CROSS(faceplane,dif1,dif2);
    		faceplane[3] = m_vertices[0].dot(faceplane);
		}

		D_GUINT res = LINE_PLANE_COLLISION(faceplane,vDir,vPoint,pout,tparam, D_btScalar(0), tmax);
		if(res != 1) return false;

		if(!is_point_inside(pout,faceplane)) return false;

		triangle_normal.setValue(faceplane[0],faceplane[1],faceplane[2]);

		D_VEC_NORMALIZE(triangle_normal);

		return true;
	}

};




#endif // D_GIM_TRI_COLLISION_H_INCLUDED
