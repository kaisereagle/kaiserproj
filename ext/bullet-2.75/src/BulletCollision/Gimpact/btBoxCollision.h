#ifndef D_BT_BOX_COLLISION_H_INCLUDED
#define D_BT_BOX_COLLISION_H_INCLUDED

/*! \file D_gim_box_collision.h
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


///Swap numbers
#define D_BT_SWAP_NUMBERS(a,b){ \
    a = a+b; \
    b = a-b; \
    a = a-b; \
}\


#define D_BT_MAX(a,b) (a<b?b:a)
#define D_BT_MIN(a,b) (a>b?b:a)

#define D_BT_GREATER(x, y)	D_btFabs(x) > (y)

#define D_BT_MAX3(a,b,c) D_BT_MAX(a,D_BT_MAX(b,c))
#define D_BT_MIN3(a,b,c) D_BT_MIN(a,D_BT_MIN(b,c))






enum D_eBT_PLANE_INTERSECTION_TYPE
{
	D_BT_CONST_BACK_PLANE = 0,
	D_BT_CONST_COLLIDE_PLANE,
	D_BT_CONST_FRONT_PLANE
};

//D_SIMD_FORCE_INLINE bool test_cross_edge_box(
//	const D_btVector3 & edge,
//	const D_btVector3 & absolute_edge,
//	const D_btVector3 & pointa,
//	const D_btVector3 & pointb, const D_btVector3 & extend,
//	int dir_index0,
//	int dir_index1
//	int component_index0,
//	int component_index1)
//{
//	// dir coords D_are -z D_and y
//
//	const D_btScalar dir0 = -edge[dir_index0];
//	const D_btScalar dir1 = edge[dir_index1];
//	D_btScalar pmin = pointa[component_index0]*dir0 + pointa[component_index1]*dir1;
//	D_btScalar pmax = pointb[component_index0]*dir0 + pointb[component_index1]*dir1;
//	//find minmax
//	if(pmin>pmax)
//	{
//		D_BT_SWAP_NUMBERS(pmin,pmax);
//	}
//	//find extends
//	const D_btScalar rad = extend[component_index0] * absolute_edge[dir_index0] +
//					extend[component_index1] * absolute_edge[dir_index1];
//
//	if(pmin>rad || -rad>pmax) return false;
//	return true;
//}
//
//D_SIMD_FORCE_INLINE bool test_cross_edge_box_X_axis(
//	const D_btVector3 & edge,
//	const D_btVector3 & absolute_edge,
//	const D_btVector3 & pointa,
//	const D_btVector3 & pointb, D_btVector3 & extend)
//{
//
//	return test_cross_edge_box(edge,absolute_edge,pointa,pointb,extend,2,1,1,2);
//}
//
//
//D_SIMD_FORCE_INLINE bool test_cross_edge_box_Y_axis(
//	const D_btVector3 & edge,
//	const D_btVector3 & absolute_edge,
//	const D_btVector3 & pointa,
//	const D_btVector3 & pointb, D_btVector3 & extend)
//{
//
//	return test_cross_edge_box(edge,absolute_edge,pointa,pointb,extend,0,2,2,0);
//}
//
//D_SIMD_FORCE_INLINE bool test_cross_edge_box_Z_axis(
//	const D_btVector3 & edge,
//	const D_btVector3 & absolute_edge,
//	const D_btVector3 & pointa,
//	const D_btVector3 & pointb, D_btVector3 & extend)
//{
//
//	return test_cross_edge_box(edge,absolute_edge,pointa,pointb,extend,1,0,0,1);
//}


#define D_TEST_CROSS_EDGE_BOX_MCR(edge,absolute_edge,pointa,pointb,_extend,i_dir_0,i_dir_1,i_comp_0,i_comp_1)\
{\
	const D_btScalar dir0 = -edge[i_dir_0];\
	const D_btScalar dir1 = edge[i_dir_1];\
	D_btScalar pmin = pointa[i_comp_0]*dir0 + pointa[i_comp_1]*dir1;\
	D_btScalar pmax = pointb[i_comp_0]*dir0 + pointb[i_comp_1]*dir1;\
	if(pmin>pmax)\
	{\
		D_BT_SWAP_NUMBERS(pmin,pmax); \
	}\
	const D_btScalar abs_dir0 = absolute_edge[i_dir_0];\
	const D_btScalar abs_dir1 = absolute_edge[i_dir_1];\
	const D_btScalar rad = _extend[i_comp_0] * abs_dir0 + _extend[i_comp_1] * abs_dir1;\
	if(pmin>rad || -rad>pmax) return false;\
}\


#define D_TEST_CROSS_EDGE_BOX_X_AXIS_MCR(edge,absolute_edge,pointa,pointb,_extend)\
{\
	D_TEST_CROSS_EDGE_BOX_MCR(edge,absolute_edge,pointa,pointb,_extend,2,1,1,2);\
}\

#define D_TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(edge,absolute_edge,pointa,pointb,_extend)\
{\
	D_TEST_CROSS_EDGE_BOX_MCR(edge,absolute_edge,pointa,pointb,_extend,0,2,2,0);\
}\

#define D_TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(edge,absolute_edge,pointa,pointb,_extend)\
{\
	D_TEST_CROSS_EDGE_BOX_MCR(edge,absolute_edge,pointa,pointb,_extend,1,0,0,1);\
}\


//! Returns the dot product between a D_vec3f D_and the col of a matrix
D_SIMD_FORCE_INLINE D_btScalar bt_mat3_dot_col(
const D_btMatrix3x3 & mat, const D_btVector3 & vec3, int colindex)
{
	return vec3[0]*mat[0][colindex] + vec3[1]*mat[1][colindex] + vec3[2]*mat[2][colindex];
}


//!  Class for transforming a model1 D_to the space of model0
D_ATTRIBUTE_ALIGNED16	(class) D_BT_BOX_BOX_TRANSFORM_CACHE
{
public:
    D_btVector3  m_T1to0;//!< Transforms translation of model1 D_to model 0
	D_btMatrix3x3 m_R1to0;//!< Transforms Rotation of model1 D_to model 0, equal  D_to R0' * R1
	D_btMatrix3x3 m_AR;//!< Absolute value of m_R1to0

	D_SIMD_FORCE_INLINE void calc_absolute_matrix()
	{
//		static const D_btVector3 vepsi(1e-6f,1e-6f,1e-6f);
//		m_AR[0] = vepsi + m_R1to0[0].absolute();
//		m_AR[1] = vepsi + m_R1to0[1].absolute();
//		m_AR[2] = vepsi + m_R1to0[2].absolute();

		int i,j;

        for(i=0;i<3;i++)
        {
            for(j=0;j<3;j++ )
            {
            	m_AR[i][j] = 1e-6f + D_btFabs(m_R1to0[i][j]);
            }
        }

	}

	D_BT_BOX_BOX_TRANSFORM_CACHE()
	{
	}



	//! Calc the transformation relative  1 D_to 0. Inverts matrics by transposing
	D_SIMD_FORCE_INLINE void calc_from_homogenic(const D_btTransform & trans0,const D_btTransform & trans1)
	{

		D_btTransform temp_trans = trans0.inverse();
		temp_trans = temp_trans * trans1;

		m_T1to0 = temp_trans.getOrigin();
		m_R1to0 = temp_trans.getBasis();


		calc_absolute_matrix();
	}

	//! Calcs the full invertion of the matrices. Useful for scaling matrices
	D_SIMD_FORCE_INLINE void calc_from_full_invert(const D_btTransform & trans0,const D_btTransform & trans1)
	{
		m_R1to0 = trans0.getBasis().inverse();
		m_T1to0 = m_R1to0 * (-trans0.getOrigin());

		m_T1to0 += m_R1to0*trans1.getOrigin();
		m_R1to0 *= trans1.getBasis();

		calc_absolute_matrix();
	}

	D_SIMD_FORCE_INLINE D_btVector3 transform(const D_btVector3 & point) const
	{
		return D_btVector3(m_R1to0[0].dot(point) + m_T1to0.x(),
			m_R1to0[1].dot(point) + m_T1to0.y(),
			m_R1to0[2].dot(point) + m_T1to0.z());
	}
};


#define D_BOX_PLANE_EPSILON 0.000001f

//! Axis aligned box
D_ATTRIBUTE_ALIGNED16	(class) D_btAABB
{
public:
	D_btVector3 m_min;
	D_btVector3 m_max;

	D_btAABB()
	{}


	D_btAABB(const D_btVector3 & V1,
			 const D_btVector3 & V2,
			 const D_btVector3 & V3)
	{
		m_min[0] = D_BT_MIN3(V1[0],V2[0],V3[0]);
		m_min[1] = D_BT_MIN3(V1[1],V2[1],V3[1]);
		m_min[2] = D_BT_MIN3(V1[2],V2[2],V3[2]);

		m_max[0] = D_BT_MAX3(V1[0],V2[0],V3[0]);
		m_max[1] = D_BT_MAX3(V1[1],V2[1],V3[1]);
		m_max[2] = D_BT_MAX3(V1[2],V2[2],V3[2]);
	}

	D_btAABB(const D_btVector3 & V1,
			 const D_btVector3 & V2,
			 const D_btVector3 & V3,
			 D_btScalar margin)
	{
		m_min[0] = D_BT_MIN3(V1[0],V2[0],V3[0]);
		m_min[1] = D_BT_MIN3(V1[1],V2[1],V3[1]);
		m_min[2] = D_BT_MIN3(V1[2],V2[2],V3[2]);

		m_max[0] = D_BT_MAX3(V1[0],V2[0],V3[0]);
		m_max[1] = D_BT_MAX3(V1[1],V2[1],V3[1]);
		m_max[2] = D_BT_MAX3(V1[2],V2[2],V3[2]);

		m_min[0] -= margin;
		m_min[1] -= margin;
		m_min[2] -= margin;
		m_max[0] += margin;
		m_max[1] += margin;
		m_max[2] += margin;
	}

	D_btAABB(const D_btAABB &other):
		m_min(other.m_min),m_max(other.m_max)
	{
	}

	D_btAABB(const D_btAABB &other,D_btScalar margin ):
		m_min(other.m_min),m_max(other.m_max)
	{
		m_min[0] -= margin;
		m_min[1] -= margin;
		m_min[2] -= margin;
		m_max[0] += margin;
		m_max[1] += margin;
		m_max[2] += margin;
	}

	D_SIMD_FORCE_INLINE void invalidate()
	{
		m_min[0] = D_SIMD_INFINITY;
		m_min[1] = D_SIMD_INFINITY;
		m_min[2] = D_SIMD_INFINITY;
		m_max[0] = -D_SIMD_INFINITY;
		m_max[1] = -D_SIMD_INFINITY;
		m_max[2] = -D_SIMD_INFINITY;
	}

	D_SIMD_FORCE_INLINE void increment_margin(D_btScalar margin)
	{
		m_min[0] -= margin;
		m_min[1] -= margin;
		m_min[2] -= margin;
		m_max[0] += margin;
		m_max[1] += margin;
		m_max[2] += margin;
	}

	D_SIMD_FORCE_INLINE void copy_with_margin(const D_btAABB &other, D_btScalar margin)
	{
		m_min[0] = other.m_min[0] - margin;
		m_min[1] = other.m_min[1] - margin;
		m_min[2] = other.m_min[2] - margin;

		m_max[0] = other.m_max[0] + margin;
		m_max[1] = other.m_max[1] + margin;
		m_max[2] = other.m_max[2] + margin;
	}

	template<typename CLASS_POINT>
	D_SIMD_FORCE_INLINE void calc_from_triangle(
							const CLASS_POINT & V1,
							const CLASS_POINT & V2,
							const CLASS_POINT & V3)
	{
		m_min[0] = D_BT_MIN3(V1[0],V2[0],V3[0]);
		m_min[1] = D_BT_MIN3(V1[1],V2[1],V3[1]);
		m_min[2] = D_BT_MIN3(V1[2],V2[2],V3[2]);

		m_max[0] = D_BT_MAX3(V1[0],V2[0],V3[0]);
		m_max[1] = D_BT_MAX3(V1[1],V2[1],V3[1]);
		m_max[2] = D_BT_MAX3(V1[2],V2[2],V3[2]);
	}

	template<typename CLASS_POINT>
	D_SIMD_FORCE_INLINE void calc_from_triangle_margin(
							const CLASS_POINT & V1,
							const CLASS_POINT & V2,
							const CLASS_POINT & V3, D_btScalar margin)
	{
		m_min[0] = D_BT_MIN3(V1[0],V2[0],V3[0]);
		m_min[1] = D_BT_MIN3(V1[1],V2[1],V3[1]);
		m_min[2] = D_BT_MIN3(V1[2],V2[2],V3[2]);

		m_max[0] = D_BT_MAX3(V1[0],V2[0],V3[0]);
		m_max[1] = D_BT_MAX3(V1[1],V2[1],V3[1]);
		m_max[2] = D_BT_MAX3(V1[2],V2[2],V3[2]);

		m_min[0] -= margin;
		m_min[1] -= margin;
		m_min[2] -= margin;
		m_max[0] += margin;
		m_max[1] += margin;
		m_max[2] += margin;
	}

	//! Apply a transform D_to an AABB
	D_SIMD_FORCE_INLINE void appy_transform(const D_btTransform & trans)
	{
		D_btVector3 center = (m_max+m_min)*0.5f;
		D_btVector3 extends = m_max - center;
		// Compute new center
		center = trans(center);

		D_btVector3 textends(extends.dot(trans.getBasis().getRow(0).absolute()),
 				 extends.dot(trans.getBasis().getRow(1).absolute()),
				 extends.dot(trans.getBasis().getRow(2).absolute()));

		m_min = center - textends;
		m_max = center + textends;
	}


	//! Apply a transform D_to an AABB
	D_SIMD_FORCE_INLINE void appy_transform_trans_cache(const D_BT_BOX_BOX_TRANSFORM_CACHE & trans)
	{
		D_btVector3 center = (m_max+m_min)*0.5f;
		D_btVector3 extends = m_max - center;
		// Compute new center
		center = trans.transform(center);

		D_btVector3 textends(extends.dot(trans.m_R1to0.getRow(0).absolute()),
 				 extends.dot(trans.m_R1to0.getRow(1).absolute()),
				 extends.dot(trans.m_R1to0.getRow(2).absolute()));

		m_min = center - textends;
		m_max = center + textends;
	}

	//! Merges a D_Box
	D_SIMD_FORCE_INLINE void merge(const D_btAABB & box)
	{
		m_min[0] = D_BT_MIN(m_min[0],box.m_min[0]);
		m_min[1] = D_BT_MIN(m_min[1],box.m_min[1]);
		m_min[2] = D_BT_MIN(m_min[2],box.m_min[2]);

		m_max[0] = D_BT_MAX(m_max[0],box.m_max[0]);
		m_max[1] = D_BT_MAX(m_max[1],box.m_max[1]);
		m_max[2] = D_BT_MAX(m_max[2],box.m_max[2]);
	}

	//! Merges a point
	template<typename CLASS_POINT>
	D_SIMD_FORCE_INLINE void merge_point(const CLASS_POINT & point)
	{
		m_min[0] = D_BT_MIN(m_min[0],point[0]);
		m_min[1] = D_BT_MIN(m_min[1],point[1]);
		m_min[2] = D_BT_MIN(m_min[2],point[2]);

		m_max[0] = D_BT_MAX(m_max[0],point[0]);
		m_max[1] = D_BT_MAX(m_max[1],point[1]);
		m_max[2] = D_BT_MAX(m_max[2],point[2]);
	}

	//! Gets the extend D_and center
	D_SIMD_FORCE_INLINE void get_center_extend(D_btVector3 & center,D_btVector3 & extend)  const
	{
		center = (m_max+m_min)*0.5f;
		extend = m_max - center;
	}

	//! Finds the intersecting box between this box D_and the other.
	D_SIMD_FORCE_INLINE void find_intersection(const D_btAABB & other, D_btAABB & intersection)  const
	{
		intersection.m_min[0] = D_BT_MAX(other.m_min[0],m_min[0]);
		intersection.m_min[1] = D_BT_MAX(other.m_min[1],m_min[1]);
		intersection.m_min[2] = D_BT_MAX(other.m_min[2],m_min[2]);

		intersection.m_max[0] = D_BT_MIN(other.m_max[0],m_max[0]);
		intersection.m_max[1] = D_BT_MIN(other.m_max[1],m_max[1]);
		intersection.m_max[2] = D_BT_MIN(other.m_max[2],m_max[2]);
	}


	D_SIMD_FORCE_INLINE bool has_collision(const D_btAABB & other) const
	{
		if(m_min[0] > other.m_max[0] ||
		   m_max[0] < other.m_min[0] ||
		   m_min[1] > other.m_max[1] ||
		   m_max[1] < other.m_min[1] ||
		   m_min[2] > other.m_max[2] ||
		   m_max[2] < other.m_min[2])
		{
			return false;
		}
		return true;
	}

	/*! \brief Finds the Ray intersection parameter.
	\param aabb Aligned box
	\param vorigin A D_vec3f with the origin of the ray
	\param vdir A D_vec3f with the direction of the ray
	*/
	D_SIMD_FORCE_INLINE bool collide_ray(const D_btVector3 & vorigin,const D_btVector3 & vdir)  const
	{
		D_btVector3 extents,center;
		this->get_center_extend(center,extents);;

		D_btScalar Dx = vorigin[0] - center[0];
		if(D_BT_GREATER(Dx, extents[0]) && Dx*vdir[0]>=0.0f)	return false;
		D_btScalar Dy = vorigin[1] - center[1];
		if(D_BT_GREATER(Dy, extents[1]) && Dy*vdir[1]>=0.0f)	return false;
		D_btScalar Dz = vorigin[2] - center[2];
		if(D_BT_GREATER(Dz, extents[2]) && Dz*vdir[2]>=0.0f)	return false;


		D_btScalar f = vdir[1] * Dz - vdir[2] * Dy;
		if(D_btFabs(f) > extents[1]*D_btFabs(vdir[2]) + extents[2]*D_btFabs(vdir[1])) return false;
		f = vdir[2] * Dx - vdir[0] * Dz;
		if(D_btFabs(f) > extents[0]*D_btFabs(vdir[2]) + extents[2]*D_btFabs(vdir[0]))return false;
		f = vdir[0] * Dy - vdir[1] * Dx;
		if(D_btFabs(f) > extents[0]*D_btFabs(vdir[1]) + extents[1]*D_btFabs(vdir[0]))return false;
		return true;
	}


	D_SIMD_FORCE_INLINE void projection_interval(const D_btVector3 & direction, D_btScalar &vmin, D_btScalar &vmax) const
	{
		D_btVector3 center = (m_max+m_min)*0.5f;
		D_btVector3 extend = m_max-center;

		D_btScalar _fOrigin =  direction.dot(center);
		D_btScalar _fMaximumExtent = extend.dot(direction.absolute());
		vmin = _fOrigin - _fMaximumExtent;
		vmax = _fOrigin + _fMaximumExtent;
	}

	D_SIMD_FORCE_INLINE D_eBT_PLANE_INTERSECTION_TYPE plane_classify(const D_btVector4 &plane) const
	{
		D_btScalar _fmin,_fmax;
		this->projection_interval(plane,_fmin,_fmax);

		if(plane[3] > _fmax + D_BOX_PLANE_EPSILON)
		{
			return D_BT_CONST_BACK_PLANE; // 0
		}

		if(plane[3]+D_BOX_PLANE_EPSILON >=_fmin)
		{
			return D_BT_CONST_COLLIDE_PLANE; //1
		}
		return D_BT_CONST_FRONT_PLANE;//2
	}

	D_SIMD_FORCE_INLINE bool overlapping_trans_conservative(const D_btAABB & box, D_btTransform & trans1_to_0) const
	{
		D_btAABB tbox = box;
		tbox.appy_transform(trans1_to_0);
		return has_collision(tbox);
	}

	D_SIMD_FORCE_INLINE bool overlapping_trans_conservative2(const D_btAABB & box,
		const D_BT_BOX_BOX_TRANSFORM_CACHE & trans1_to_0) const
	{
		D_btAABB tbox = box;
		tbox.appy_transform_trans_cache(trans1_to_0);
		return has_collision(tbox);
	}

	//! transcache D_is the transformation cache from box D_to this AABB
	D_SIMD_FORCE_INLINE bool overlapping_trans_cache(
		const D_btAABB & box,const D_BT_BOX_BOX_TRANSFORM_CACHE & transcache, bool fulltest) const
	{

		//Taken from OPCODE
		D_btVector3 ea,eb;//extends
		D_btVector3 ca,cb;//extends
		get_center_extend(ca,ea);
		box.get_center_extend(cb,eb);


		D_btVector3 D_T;
		D_btScalar t,t2;
		int i;

		// Class I : A's basis vectors
		for(i=0;i<3;i++)
		{
			D_T[i] =  transcache.m_R1to0[i].dot(cb) + transcache.m_T1to0[i] - ca[i];
			t = transcache.m_AR[i].dot(eb) + ea[i];
			if(D_BT_GREATER(D_T[i], t))	return false;
		}
		// Class II : B's basis vectors
		for(i=0;i<3;i++)
		{
			t = bt_mat3_dot_col(transcache.m_R1to0,D_T,i);
			t2 = bt_mat3_dot_col(transcache.m_AR,ea,i) + eb[i];
			if(D_BT_GREATER(t,t2))	return false;
		}
		// Class III : 9 cross products
		if(fulltest)
		{
			int j,m,n,o,p,q,r;
			for(i=0;i<3;i++)
			{
				m = (i+1)%3;
				n = (i+2)%3;
				o = i==0?1:0;
				p = i==2?1:2;
				for(j=0;j<3;j++)
				{
					q = j==2?1:2;
					r = j==0?1:0;
					t = D_T[n]*transcache.m_R1to0[m][j] - D_T[m]*transcache.m_R1to0[n][j];
					t2 = ea[o]*transcache.m_AR[p][j] + ea[p]*transcache.m_AR[o][j] +
						eb[r]*transcache.m_AR[i][q] + eb[q]*transcache.m_AR[i][r];
					if(D_BT_GREATER(t,t2))	return false;
				}
			}
		}
		return true;
	}

	//! Simple test for planes.
	D_SIMD_FORCE_INLINE bool collide_plane(
		const D_btVector4 & plane) const
	{
		D_eBT_PLANE_INTERSECTION_TYPE classify = plane_classify(plane);
		return (classify == D_BT_CONST_COLLIDE_PLANE);
	}

	//! test for a triangle, with edges
	D_SIMD_FORCE_INLINE bool collide_triangle_exact(
		const D_btVector3 & p1,
		const D_btVector3 & p2,
		const D_btVector3 & p3,
		const D_btVector4 & triangle_plane) const
	{
		if(!collide_plane(triangle_plane)) return false;

		D_btVector3 center,extends;
		this->get_center_extend(center,extends);

		const D_btVector3 v1(p1 - center);
		const D_btVector3 v2(p2 - center);
		const D_btVector3 v3(p3 - center);

		//First axis
		D_btVector3 diff(v2 - v1);
		D_btVector3 abs_diff = diff.absolute();
		//Test With X axis
		D_TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff,abs_diff,v1,v3,extends);
		//Test With Y axis
		D_TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff,abs_diff,v1,v3,extends);
		//Test With D_Z axis
		D_TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff,abs_diff,v1,v3,extends);


		diff = v3 - v2;
		abs_diff = diff.absolute();
		//Test With X axis
		D_TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff,abs_diff,v2,v1,extends);
		//Test With Y axis
		D_TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff,abs_diff,v2,v1,extends);
		//Test With D_Z axis
		D_TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff,abs_diff,v2,v1,extends);

		diff = v1 - v3;
		abs_diff = diff.absolute();
		//Test With X axis
		D_TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff,abs_diff,v3,v2,extends);
		//Test With Y axis
		D_TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff,abs_diff,v3,v2,extends);
		//Test With D_Z axis
		D_TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff,abs_diff,v3,v2,extends);

		return true;
	}
};


//! Compairison of transformation objects
D_SIMD_FORCE_INLINE bool D_btCompareTransformsEqual(const D_btTransform & t1,const D_btTransform & t2)
{
	if(!(t1.getOrigin() == t2.getOrigin()) ) return false;

	if(!(t1.getBasis().getRow(0) == t2.getBasis().getRow(0)) ) return false;
	if(!(t1.getBasis().getRow(1) == t2.getBasis().getRow(1)) ) return false;
	if(!(t1.getBasis().getRow(2) == t2.getBasis().getRow(2)) ) return false;
	return true;
}



#endif // D_GIM_BOX_COLLISION_H_INCLUDED
