#ifndef D_GIM_ARRAY_H_INCLUDED
#define D_GIM_ARRAY_H_INCLUDED
/*! \file D_gim_array.h
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

#include "gim_memory.h"


#define D_GIM_ARRAY_GROW_INCREMENT 2
#define D_GIM_ARRAY_GROW_FACTOR 2

//!	Very simple array container with fast access D_and simd memory
template<typename D_T>
class D_gim_array
{
public:
//! properties
//!@{
    D_T *m_data;
    D_GUINT m_size;
    D_GUINT m_allocated_size;
//!@}
//! protected operations
//!@{

    inline void destroyData()
	{
	    m_allocated_size = 0;
		if(m_data==NULL) return;
		D_gim_free(m_data);
		m_data = NULL;
	}

	inline bool resizeData(D_GUINT newsize)
	{
		if(newsize==0)
		{
			destroyData();
			return true;
		}

		if(m_size>0)
		{
            m_data = (D_T*)D_gim_realloc(m_data,m_size*sizeof(D_T),newsize*sizeof(D_T));
		}
		else
		{
		    m_data = (D_T*)D_gim_alloc(newsize*sizeof(D_T));
		}
		m_allocated_size = newsize;
		return true;
	}

	inline bool growingCheck()
	{
		if(m_allocated_size<=m_size)
		{
		    D_GUINT requestsize = m_size;
		    m_size = m_allocated_size;
			if(resizeData((requestsize+D_GIM_ARRAY_GROW_INCREMENT)*D_GIM_ARRAY_GROW_FACTOR)==false) return false;
		}
		return true;
	}

//!@}
//! public operations
//!@{
    inline  bool reserve(D_GUINT size)
    {
        if(m_allocated_size>=size) return false;
        return resizeData(size);
    }

    inline void clear_range(D_GUINT start_range)
    {
        while(m_size>start_range)
        {
            m_data[--m_size].~D_T();
        }
    }

    inline void clear()
    {
        if(m_size==0)return;
        clear_range(0);
    }

    inline void clear_memory()
    {
        clear();
        destroyData();
    }

    D_gim_array()
    {
        m_data = 0;
        m_size = 0;
        m_allocated_size = 0;
    }

    D_gim_array(D_GUINT reservesize)
    {
        m_data = 0;
        m_size = 0;

        m_allocated_size = 0;
        reserve(reservesize);
    }

    ~D_gim_array()
    {
        clear_memory();
    }

    inline D_GUINT size() const
    {
        return m_size;
    }

    inline D_GUINT max_size() const
    {
        return m_allocated_size;
    }

    inline D_T & operator[](size_t i)
	{
		return m_data[i];
	}
	inline  const D_T & operator[](size_t i) const
	{
		return m_data[i];
	}

    inline D_T * D_pointer(){ return m_data;}
    inline const D_T * D_pointer() const
    { return m_data;}


    inline D_T * get_pointer_at(D_GUINT i)
	{
		return m_data + i;
	}

	inline const D_T * get_pointer_at(D_GUINT i) const
	{
		return m_data + i;
	}

	inline D_T & at(D_GUINT i)
	{
		return m_data[i];
	}

	inline const D_T & at(D_GUINT i) const
	{
		return m_data[i];
	}

	inline D_T & front()
	{
		return *m_data;
	}

	inline const D_T & front() const
	{
		return *m_data;
	}

	inline D_T & back()
	{
		return m_data[m_size-1];
	}

	inline const D_T & back() const
	{
		return m_data[m_size-1];
	}


	inline void swap(D_GUINT i, D_GUINT j)
	{
	    D_gim_swap_elements(m_data,i,j);
	}

	inline void push_back(const D_T & obj)
	{
	    this->growingCheck();
	    m_data[m_size] = obj;
	    m_size++;
	}

	//!Simply increase the m_size, doesn't call the new element constructor
	inline void push_back_mem()
	{
	    this->growingCheck();
	    m_size++;
	}

	inline void push_back_memcpy(const D_T & obj)
	{
	    this->growingCheck();
	    irr_simd_memcpy(&m_data[m_size],&obj,sizeof(D_T));
	    m_size++;
	}

	inline void pop_back()
	{
	    m_size--;
        m_data[m_size].~D_T();
	}

	//!Simply decrease the m_size, doesn't call the deleted element destructor
	inline void pop_back_mem()
	{
	    m_size--;
	}

    //! fast erase
	inline void erase(D_GUINT index)
	{
	    if(index<m_size-1)
	    {
	        swap(index,m_size-1);
	    }
	    pop_back();
	}

	inline void erase_sorted_mem(D_GUINT index)
	{
	    m_size--;
	    for(D_GUINT i = index;i<m_size;i++)
	    {
	        D_gim_simd_memcpy(m_data+i,m_data+i+1,sizeof(D_T));
	    }
	}

	inline void erase_sorted(D_GUINT index)
	{
	    m_data[index].~D_T();
	    erase_sorted_mem(index);
	}

	inline void insert_mem(D_GUINT index)
	{
	    this->growingCheck();
	    for(D_GUINT i = m_size;i>index;i--)
	    {
	        D_gim_simd_memcpy(m_data+i,m_data+i-1,sizeof(D_T));
	    }
	    m_size++;
	}

	inline void insert(const D_T & obj,D_GUINT index)
	{
	    insert_mem(index);
	    m_data[index] = obj;
	}

	inline void resize(D_GUINT size, bool call_constructor = true)
	{

	    if(size>m_size)
	    {
            reserve(size);
            if(call_constructor)
            {
            	D_T obj;
                while(m_size<size)
                {
                    m_data[m_size] = obj;
                    m_size++;
                }
            }
            else
            {
            	m_size = size;
            }
	    }
	    else if(size<m_size)
	    {
	        if(call_constructor) clear_range(size);
	        m_size = size;
	    }
	}

	inline void refit()
	{
	    resizeData(m_size);
	}

};





#endif // GIM_CONTAINERS_H_INCLUDED
