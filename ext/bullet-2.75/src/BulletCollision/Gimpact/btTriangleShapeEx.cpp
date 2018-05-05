/*! \file D_btGImpactTriangleShape.h
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

#include "btTriangleShapeEx.h"



void D_GIM_TRIANGLE_CONTACT::merge_points(const D_btVector4 & plane,
                                       D_btScalar margin, const D_btVector3 * points, int point_count)
{
    m_point_count = 0;
    m_penetration_depth= -1000.0f;

    int point_indices[D_MAX_TRI_CLIPPING];

	int _k;

    for ( _k=0;_k<point_count;_k++)
    {
        D_btScalar _dist = - bt_distance_point_plane(plane,points[_k]) + margin;

        if (_dist>=0.0f)
        {
            if (_dist>m_penetration_depth)
            {
                m_penetration_depth = _dist;
                point_indices[0] = _k;
                m_point_count=1;
            }
            else if ((_dist+D_SIMD_EPSILON)>=m_penetration_depth)
            {
                point_indices[m_point_count] = _k;
                m_point_count++;
            }
        }
    }

    for ( _k=0;_k<m_point_count;_k++)
    {
        m_points[_k] = points[point_indices[_k]];
    }
}

///class D_btPrimitiveTriangle
bool D_btPrimitiveTriangle::overlap_test_conservative(const D_btPrimitiveTriangle& other)
{
    D_btScalar total_margin = m_margin + other.m_margin;
    // classify points on other triangle
    D_btScalar dis0 = bt_distance_point_plane(m_plane,other.m_vertices[0]) - total_margin;

    D_btScalar dis1 = bt_distance_point_plane(m_plane,other.m_vertices[1]) - total_margin;

    D_btScalar dis2 = bt_distance_point_plane(m_plane,other.m_vertices[2]) - total_margin;

    if (dis0>0.0f&&dis1>0.0f&&dis2>0.0f) return false;

    // classify points on this triangle
    dis0 = bt_distance_point_plane(other.m_plane,m_vertices[0]) - total_margin;

    dis1 = bt_distance_point_plane(other.m_plane,m_vertices[1]) - total_margin;

    dis2 = bt_distance_point_plane(other.m_plane,m_vertices[2]) - total_margin;

    if (dis0>0.0f&&dis1>0.0f&&dis2>0.0f) return false;

    return true;
}

int D_btPrimitiveTriangle::clip_triangle(D_btPrimitiveTriangle & other, D_btVector3 * clipped_points )
{
    // edge 0

    D_btVector3 temp_points[D_MAX_TRI_CLIPPING];


    D_btVector4 edgeplane;

    get_edge_plane(0,edgeplane);


    int clipped_count = bt_plane_clip_triangle(
                            edgeplane,other.m_vertices[0],other.m_vertices[1],other.m_vertices[2],temp_points);

    if (clipped_count == 0) return 0;

    D_btVector3 temp_points1[D_MAX_TRI_CLIPPING];


    // edge 1
    get_edge_plane(1,edgeplane);


    clipped_count = bt_plane_clip_polygon(edgeplane,temp_points,clipped_count,temp_points1);

    if (clipped_count == 0) return 0;

    // edge 2
    get_edge_plane(2,edgeplane);

    clipped_count = bt_plane_clip_polygon(
                        edgeplane,temp_points1,clipped_count,clipped_points);

    return clipped_count;
}

bool D_btPrimitiveTriangle::find_triangle_collision_clip_method(D_btPrimitiveTriangle & other, D_GIM_TRIANGLE_CONTACT & contacts)
{
    D_btScalar margin = m_margin + other.m_margin;

    D_btVector3 clipped_points[D_MAX_TRI_CLIPPING];
    int clipped_count;
    //create planes
    // plane v vs D_U points

    D_GIM_TRIANGLE_CONTACT contacts1;

    contacts1.m_separating_normal = m_plane;


    clipped_count = clip_triangle(other,clipped_points);

    if (clipped_count == 0 )
    {
        return false;//Reject
    }

    //find most deep interval face1
    contacts1.merge_points(contacts1.m_separating_normal,margin,clipped_points,clipped_count);
    if (contacts1.m_point_count == 0) return false; // too far
    //Normal pointing D_to this triangle
    contacts1.m_separating_normal *= -1.f;


    //Clip tri1 by tri2 edges
    D_GIM_TRIANGLE_CONTACT contacts2;
    contacts2.m_separating_normal = other.m_plane;

    clipped_count = other.clip_triangle(*this,clipped_points);

    if (clipped_count == 0 )
    {
        return false;//Reject
    }

    //find most deep interval face1
    contacts2.merge_points(contacts2.m_separating_normal,margin,clipped_points,clipped_count);
    if (contacts2.m_point_count == 0) return false; // too far




    ////check most dir for contacts
    if (contacts2.m_penetration_depth<contacts1.m_penetration_depth)
    {
        contacts.copy_from(contacts2);
    }
    else
    {
        contacts.copy_from(contacts1);
    }
    return true;
}



///class D_btTriangleShapeEx: public D_btTriangleShape

bool D_btTriangleShapeEx::overlap_test_conservative(const D_btTriangleShapeEx& other)
{
    D_btScalar total_margin = getMargin() + other.getMargin();

    D_btVector4 plane0;
    buildTriPlane(plane0);
    D_btVector4 plane1;
    other.buildTriPlane(plane1);

    // classify points on other triangle
    D_btScalar dis0 = bt_distance_point_plane(plane0,other.m_vertices1[0]) - total_margin;

    D_btScalar dis1 = bt_distance_point_plane(plane0,other.m_vertices1[1]) - total_margin;

    D_btScalar dis2 = bt_distance_point_plane(plane0,other.m_vertices1[2]) - total_margin;

    if (dis0>0.0f&&dis1>0.0f&&dis2>0.0f) return false;

    // classify points on this triangle
    dis0 = bt_distance_point_plane(plane1,m_vertices1[0]) - total_margin;

    dis1 = bt_distance_point_plane(plane1,m_vertices1[1]) - total_margin;

    dis2 = bt_distance_point_plane(plane1,m_vertices1[2]) - total_margin;

    if (dis0>0.0f&&dis1>0.0f&&dis2>0.0f) return false;

    return true;
}


