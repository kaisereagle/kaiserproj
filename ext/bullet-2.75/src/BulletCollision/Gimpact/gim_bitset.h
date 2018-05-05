#ifndef D_GIM_BITSET_H_INCLUDED
#define D_GIM_BITSET_H_INCLUDED
/*! \file D_gim_bitset.h
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

#include "gim_array.h"


#define D_GUINT_BIT_COUNT 32
#define D_GUINT_EXPONENT 5

class D_gim_bitset
{
public:
    D_gim_array<D_GUINT> m_container;

    D_gim_bitset()
    {

    }

    D_gim_bitset(D_GUINT bits_count)
    {
        resize(bits_count);
    }

    ~D_gim_bitset()
    {
    }

	inline bool resize(D_GUINT newsize)
	{
		D_GUINT oldsize = m_container.size();
		m_container.resize(newsize/D_GUINT_BIT_COUNT + 1,false);
		while(oldsize<m_container.size())
		{
			m_container[oldsize] = 0;
		}
		return true;
	}

	inline D_GUINT size()
	{
		return m_container.size()*D_GUINT_BIT_COUNT;
	}

	inline void set_all()
	{
		for(D_GUINT i = 0;i<m_container.size();++i)
		{
			m_container[i] = 0xffffffff;
		}
	}

	inline void clear_all()
	{
	    for(D_GUINT i = 0;i<m_container.size();++i)
		{
			m_container[i] = 0;
		}
	}

	inline void set(D_GUINT bit_index)
	{
		if(bit_index>=size())
		{
			resize(bit_index);
		}
		m_container[bit_index >> D_GUINT_EXPONENT] |= (1 << (bit_index & (D_GUINT_BIT_COUNT-1)));
	}

	///Return 0 or 1
	inline char get(D_GUINT bit_index)
	{
		if(bit_index>=size())
		{
			return 0;
		}
		char value = m_container[bit_index >> D_GUINT_EXPONENT] &
					 (1 << (bit_index & (D_GUINT_BIT_COUNT-1)));
		return value;
	}

	inline void clear(D_GUINT bit_index)
	{
	    m_container[bit_index >> D_GUINT_EXPONENT] &= ~(1 << (bit_index & (D_GUINT_BIT_COUNT-1)));
	}
};





#endif // GIM_CONTAINERS_H_INCLUDED
