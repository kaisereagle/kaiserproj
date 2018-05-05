#ifndef D_GIM_CONTACT_H_INCLUDED
#define D_GIM_CONTACT_H_INCLUDED

/*! \file D_gim_contact.h
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
#include "gim_geometry.h"
#include "gim_radixsort.h"
#include "gim_array.h"


/**
Configuration var for applying interpolation of  contact normals
*/
#define D_NORMAL_CONTACT_AVERAGE 1
#define D_CONTACT_DIFF_EPSILON 0.00001f

/// Structure for collision results
///Functions for managing D_and sorting contacts resulting from a collision query.
///D_Contact lists D_must be create by calling \ref GIM_CREATE_CONTACT_LIST
///After querys, contact lists D_must be destroy by calling \ref GIM_DYNARRAY_DESTROY
///D_Contacts D_can be merge for avoid duplicate results by calling \ref gim_merge_contacts
class D_GIM_CONTACT
{
public:
    D_btVector3 m_point;
    D_btVector3 m_normal;
    D_GREAL m_depth;//Positive value indicates interpenetration
    D_GREAL m_distance;//Padding not for use
    D_GUINT m_feature1;//D_Face number
    D_GUINT m_feature2;//D_Face number
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
    	m_point = contact.m_point;
    	m_normal = contact.m_normal;
    	m_depth = contact.m_depth;
    	m_feature1 = contact.m_feature1;
    	m_feature2 = contact.m_feature2;
    }

    D_GIM_CONTACT(const D_btVector3 &point,const D_btVector3 & normal,
    	 			D_GREAL depth, D_GUINT feature1, D_GUINT feature2):
				m_point(point),
				m_normal(normal),
				m_depth(depth),
				m_feature1(feature1),
				m_feature2(feature2)
    {
    }

	//! Calcs key for coord classification
    D_SIMD_FORCE_INLINE D_GUINT calc_key_contact() const
    {
    	D_GINT _coords[] = {
    		(D_GINT)(m_point[0]*1000.0f+1.0f),
    		(D_GINT)(m_point[1]*1333.0f),
    		(D_GINT)(m_point[2]*2133.0f+3.0f)};
		D_GUINT _hash=0;
		D_GUINT *_uitmp = (D_GUINT *)(&_coords[0]);
		_hash = *_uitmp;
		_uitmp++;
		_hash += (*_uitmp)<<4;
		_uitmp++;
		_hash += (*_uitmp)<<8;
		return _hash;
    }

    D_SIMD_FORCE_INLINE void interpolate_normals( D_btVector3 * normals,D_GUINT normal_count)
    {
    	D_btVector3 vec_sum(m_normal);
		for(D_GUINT i=0;i<normal_count;i++)
		{
			vec_sum += normals[i];
		}

		D_GREAL vec_sum_len = vec_sum.length2();
		if(vec_sum_len <D_CONTACT_DIFF_EPSILON) return;

		D_GIM_INV_SQRT(vec_sum_len,vec_sum_len); // 1/sqrt(vec_sum_len)

		m_normal = vec_sum*vec_sum_len;
    }

};


class D_gim_contact_array:public D_gim_array<D_GIM_CONTACT>
{
public:
	D_gim_contact_array():D_gim_array<D_GIM_CONTACT>(64)
	{
	}

	D_SIMD_FORCE_INLINE void push_contact(const D_btVector3 &point,const D_btVector3 & normal,
    	 			D_GREAL depth, D_GUINT feature1, D_GUINT feature2)
	{
		push_back_mem();
		D_GIM_CONTACT & newele = back();
		newele.m_point = point;
		newele.m_normal = normal;
		newele.m_depth = depth;
		newele.m_feature1 = feature1;
		newele.m_feature2 = feature2;
	}

	D_SIMD_FORCE_INLINE void push_triangle_contacts(
		const D_GIM_TRIANGLE_CONTACT_DATA & tricontact,
		D_GUINT feature1,D_GUINT feature2)
	{
		for(D_GUINT i = 0;i<tricontact.m_point_count ;i++ )
		{
			push_back_mem();
			D_GIM_CONTACT & newele = back();
			newele.m_point = tricontact.m_points[i];
			newele.m_normal = tricontact.m_separating_normal;
			newele.m_depth = tricontact.m_penetration_depth;
			newele.m_feature1 = feature1;
			newele.m_feature2 = feature2;
		}
	}

	void merge_contacts(const D_gim_contact_array & contacts, bool normal_contact_average = true);
	void merge_contacts_unique(const D_gim_contact_array & contacts);
};

#endif // D_GIM_CONTACT_H_INCLUDED
