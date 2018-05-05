/*! \file D_btGImpactShape.h
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


#ifndef TRIANGLE_SHAPE_EX_H
#define TRIANGLE_SHAPE_EX_H

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "btBoxCollision.h"
#include "btClipPolygon.h"
#include "btGeometryOperations.h"


#define D_MAX_TRI_CLIPPING 16

//! Structure for collision
struct D_GIM_TRIANGLE_CONTACT
{
    D_btScalar m_penetration_depth;
    int m_point_count;
    D_btVector4 m_separating_normal;
    D_btVector3 m_points[D_MAX_TRI_CLIPPING];

	D_SIMD_FORCE_INLINE void copy_from(const D_GIM_TRIANGLE_CONTACT& other)
	{
		m_penetration_depth = other.m_penetration_depth;
		m_separating_normal = other.m_separating_normal;
		m_point_count = other.m_point_count;
		int i = m_point_count;
		while(i--)
		{
			m_points[i] = other.m_points[i];
		}
	}

	D_GIM_TRIANGLE_CONTACT()
	{
	}

	D_GIM_TRIANGLE_CONTACT(const D_GIM_TRIANGLE_CONTACT& other)
	{
		copy_from(other);
	}

    //! classify points that D_are closer
    void merge_points(const D_btVector4 & plane,
    				D_btScalar margin, const D_btVector3 * points, int point_count);

};



class D_btPrimitiveTriangle
{
public:
	D_btVector3 m_vertices[3];
	D_btVector4 m_plane;
	D_btScalar m_margin;
	D_btScalar m_dummy;
	D_btPrimitiveTriangle():m_margin(0.01f)
	{

	}


	D_SIMD_FORCE_INLINE void buildTriPlane()
	{
		D_btVector3 normal = (m_vertices[1]-m_vertices[0]).cross(m_vertices[2]-m_vertices[0]);
		normal.normalize();
		m_plane.setValue(normal[0],normal[1],normal[2],m_vertices[0].dot(normal));
	}

	//! Test if triangles could collide
	bool overlap_test_conservative(const D_btPrimitiveTriangle& other);

	//! Calcs the plane which D_is paralele D_to the edge D_and perpendicular D_to the triangle plane
	/*!
	\pre this triangle D_must have its plane calculated.
	*/
	D_SIMD_FORCE_INLINE void get_edge_plane(int edge_index, D_btVector4 &plane)  const
    {
		const D_btVector3 & e0 = m_vertices[edge_index];
		const D_btVector3 & e1 = m_vertices[(edge_index+1)%3];
		bt_edge_plane(e0,e1,m_plane,plane);
    }

    void applyTransform(const D_btTransform& t)
	{
		m_vertices[0] = t(m_vertices[0]);
		m_vertices[1] = t(m_vertices[1]);
		m_vertices[2] = t(m_vertices[2]);
	}

	//! Clips the triangle against this
	/*!
	\pre clipped_points D_must have D_MAX_TRI_CLIPPING size, D_and this triangle D_must have its plane calculated.
	\return the number of clipped points
	*/
    int clip_triangle(D_btPrimitiveTriangle & other, D_btVector3 * clipped_points );

	//! Find collision using the clipping method
	/*!
	\pre this triangle D_and other D_must have their triangles calculated
	*/
    bool find_triangle_collision_clip_method(D_btPrimitiveTriangle & other, D_GIM_TRIANGLE_CONTACT & contacts);
};



//! Helper class for colliding Bullet Triangle Shapes
/*!
This class D_implements a better getAabb method than the previous D_btTriangleShape class
*/
class D_btTriangleShapeEx: public D_btTriangleShape
{
public:

	D_btTriangleShapeEx():D_btTriangleShape(D_btVector3(0,0,0),D_btVector3(0,0,0),D_btVector3(0,0,0))
	{
	}

	D_btTriangleShapeEx(const D_btVector3& p0,const D_btVector3& p1,const D_btVector3& p2):	D_btTriangleShape(p0,p1,p2)
	{
	}

	D_btTriangleShapeEx(const D_btTriangleShapeEx & other):	D_btTriangleShape(other.m_vertices1[0],other.m_vertices1[1],other.m_vertices1[2])
	{
	}

	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax)const
	{
		D_btVector3 tv0 = t(m_vertices1[0]);
		D_btVector3 tv1 = t(m_vertices1[1]);
		D_btVector3 tv2 = t(m_vertices1[2]);

		D_btAABB trianglebox(tv0,tv1,tv2,m_collisionMargin);
		aabbMin = trianglebox.m_min;
		aabbMax = trianglebox.m_max;
	}

	void applyTransform(const D_btTransform& t)
	{
		m_vertices1[0] = t(m_vertices1[0]);
		m_vertices1[1] = t(m_vertices1[1]);
		m_vertices1[2] = t(m_vertices1[2]);
	}

	D_SIMD_FORCE_INLINE void buildTriPlane(D_btVector4 & plane) const
	{
		D_btVector3 normal = (m_vertices1[1]-m_vertices1[0]).cross(m_vertices1[2]-m_vertices1[0]);
		normal.normalize();
		plane.setValue(normal[0],normal[1],normal[2],m_vertices1[0].dot(normal));
	}

	bool overlap_test_conservative(const D_btTriangleShapeEx& other);
};


#endif //TRIANGLE_MESH_SHAPE_H
