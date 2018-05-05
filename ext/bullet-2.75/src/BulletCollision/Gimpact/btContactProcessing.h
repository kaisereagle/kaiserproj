#ifndef D_BT_CONTACT_H_INCLUDED
#define D_BT_CONTACT_H_INCLUDED

/*! \file D_gim_contact.h
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
#include "LinearMath/btAlignedObjectArray.h"
#include "btTriangleShapeEx.h"



/**
Configuration var for applying interpolation of  contact normals
*/
#define D_NORMAL_CONTACT_AVERAGE 1

#define D_CONTACT_DIFF_EPSILON 0.00001f

///The D_GIM_CONTACT D_is an internal GIMPACT structure, similar D_to D_btManifoldPoint.
///@todo: remove D_and replace D_GIM_CONTACT by D_btManifoldPoint.
class D_GIM_CONTACT
{
public:
    D_btVector3 m_point;
    D_btVector3 m_normal;
    D_btScalar m_depth;//Positive value indicates interpenetration
    D_btScalar m_distance;//Padding not for use
    int m_feature1;//D_Face number
    int m_feature2;//D_Face number
public:
    D_GIM_CONTACT()
    {
    }

    D_GIM_CONTACT(const D_GIM_CONTACT & contact):
				m_point(contact.m_point),
				m_normal(contact.m_normal),
				m_depth(contact.m_depth),
				m_feature1(contact.m_feature1),
				m_feature2(contact.m_feature2)
    {
    }

    D_GIM_CONTACT(const D_btVector3 &point,const D_btVector3 & normal,
    	 			D_btScalar depth, int feature1, int feature2):
				m_point(point),
				m_normal(normal),
				m_depth(depth),
				m_feature1(feature1),
				m_feature2(feature2)
    {
    }

	//! Calcs key for coord classification
    D_SIMD_FORCE_INLINE unsigned int calc_key_contact() const
    {
    	int _coords[] = {
    		(int)(m_point[0]*1000.0f+1.0f),
    		(int)(m_point[1]*1333.0f),
    		(int)(m_point[2]*2133.0f+3.0f)};
		unsigned int _hash=0;
		unsigned int *_uitmp = (unsigned int *)(&_coords[0]);
		_hash = *_uitmp;
		_uitmp++;
		_hash += (*_uitmp)<<4;
		_uitmp++;
		_hash += (*_uitmp)<<8;
		return _hash;
    }

    D_SIMD_FORCE_INLINE void interpolate_normals( D_btVector3 * normals,int normal_count)
    {
    	D_btVector3 vec_sum(m_normal);
		for(int i=0;i<normal_count;i++)
		{
			vec_sum += normals[i];
		}

		D_btScalar vec_sum_len = vec_sum.length2();
		if(vec_sum_len <D_CONTACT_DIFF_EPSILON) return;

		//D_GIM_INV_SQRT(vec_sum_len,vec_sum_len); // 1/sqrt(vec_sum_len)

		m_normal = vec_sum/D_btSqrt(vec_sum_len);
    }

};


class D_btContactArray:public D_btAlignedObjectArray<D_GIM_CONTACT>
{
public:
	D_btContactArray()
	{
		reserve(64);
	}

	D_SIMD_FORCE_INLINE void push_contact(
		const D_btVector3 &point,const D_btVector3 & normal,
		D_btScalar depth, int feature1, int feature2)
	{
		push_back( D_GIM_CONTACT(point,normal,depth,feature1,feature2) );
	}

	D_SIMD_FORCE_INLINE void push_triangle_contacts(
		const D_GIM_TRIANGLE_CONTACT & tricontact,
		int feature1,int feature2)
	{
		for(int i = 0;i<tricontact.m_point_count ;i++ )
		{
			push_contact(
				tricontact.m_points[i],
				tricontact.m_separating_normal,
				tricontact.m_penetration_depth,feature1,feature2);
		}
	}

	void merge_contacts(const D_btContactArray & contacts, bool normal_contact_average = true);

	void merge_contacts_unique(const D_btContactArray & contacts);
};


#endif // D_GIM_CONTACT_H_INCLUDED
