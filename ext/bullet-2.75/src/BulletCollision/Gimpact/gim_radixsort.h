#ifndef D_GIM_RADIXSORT_H_INCLUDED
#define D_GIM_RADIXSORT_H_INCLUDED
/*! \file D_gim_radixsort.h
\author Francisco Len Nßjera.
Based on the work of Michael Herf : "fast floating-point radix sort"
Avaliable on http://www.stereopsis.com/radix.html
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

///Macros for sorting.
//! Prototype for comparators
class D_less_comparator
{
	public:

	template<class D_T,class D_Z>
	inline int operator() ( const D_T& a, const D_Z& b )
	{
		return ( a<b?-1:(a>b?1:0));
	}
};

//! Prototype for comparators
class D_integer_comparator
{
	public:

	template<class D_T>
	inline int operator() ( const D_T& a, const D_T& b )
	{
		return (int)(a-b);
	}
};

//!Prototype for getting the integer representation of an object
class D_uint_key_func
{
public:
	template<class D_T>
	inline D_GUINT operator()( const D_T& a)
	{
		return (D_GUINT)a;
	}
};


//!Prototype for copying elements
class D_copy_elements_func
{
public:
	template<class D_T>
	inline void operator()(D_T& a,D_T& b)
	{
		a = b;
	}
};

//!Prototype for copying elements
class D_memcopy_elements_func
{
public:
	template<class D_T>
	inline void operator()(D_T& a,D_T& b)
	{
		D_gim_simd_memcpy(&a,&b,sizeof(D_T));
	}
};


//! @{
struct D_GIM_RSORT_TOKEN
{
    D_GUINT m_key;
    D_GUINT m_value;
    D_GIM_RSORT_TOKEN()
    {
    }
    D_GIM_RSORT_TOKEN(const D_GIM_RSORT_TOKEN& rtoken)
    {
    	m_key = rtoken.m_key;
    	m_value = rtoken.m_value;
    }

    inline bool operator <(const D_GIM_RSORT_TOKEN& other) const
	{
		return (m_key < other.m_key);
	}

	inline bool operator >(const D_GIM_RSORT_TOKEN& other) const
	{
		return (m_key > other.m_key);
	}
};

//! Prototype for comparators
class D_GIM_RSORT_TOKEN_COMPARATOR
{
	public:

	inline int operator()( const D_GIM_RSORT_TOKEN& a, const D_GIM_RSORT_TOKEN& b )
	{
		return (int)((a.m_key) - (b.m_key));
	}
};



#define D_kHist 2048
// ---- utils for accessing 11-bit quantities
#define D_D11_0(x)	(x & 0x7FF)
#define D_D11_1(x)	(x >> 11 & 0x7FF)
#define D_D11_2(x)	(x >> 22 )



///Radix sort for unsigned integer keys
inline void D_gim_radix_sort_rtokens(
				D_GIM_RSORT_TOKEN * array,
				D_GIM_RSORT_TOKEN * sorted, D_GUINT element_count)
{
	D_GUINT i;
	D_GUINT b0[D_kHist * 3];
	D_GUINT *b1 = b0 + D_kHist;
	D_GUINT *b2 = b1 + D_kHist;
	for (i = 0; i < D_kHist * 3; ++i)
	{
		b0[i] = 0;
	}
	D_GUINT fi;
	D_GUINT pos;
	for (i = 0; i < element_count; ++i)
	{
	    fi = array[i].m_key;
		b0[D_D11_0(fi)] ++;
		b1[D_D11_1(fi)] ++;
		b2[D_D11_2(fi)] ++;
	}
	{
		D_GUINT sum0 = 0, sum1 = 0, sum2 = 0;
		D_GUINT tsum;
		for (i = 0; i < D_kHist; ++i)
		{
			tsum = b0[i] + sum0;
			b0[i] = sum0 - 1;
			sum0 = tsum;
			tsum = b1[i] + sum1;
			b1[i] = sum1 - 1;
			sum1 = tsum;
			tsum = b2[i] + sum2;
			b2[i] = sum2 - 1;
			sum2 = tsum;
		}
	}
	for (i = 0; i < element_count; ++i)
	{
        fi = array[i].m_key;
		pos = D_D11_0(fi);
		pos = ++b0[pos];
		sorted[pos].m_key = array[i].m_key;
		sorted[pos].m_value = array[i].m_value;
	}
	for (i = 0; i < element_count; ++i)
	{
        fi = sorted[i].m_key;
		pos = D_D11_1(fi);
		pos = ++b1[pos];
		array[pos].m_key = sorted[i].m_key;
		array[pos].m_value = sorted[i].m_value;
	}
	for (i = 0; i < element_count; ++i)
	{
        fi = array[i].m_key;
		pos = D_D11_2(fi);
		pos = ++b2[pos];
		sorted[pos].m_key = array[i].m_key;
		sorted[pos].m_value = array[i].m_value;
	}
}




/// Get the sorted tokens from an array. For generic use. Tokens D_are IRR_RSORT_TOKEN
/*!
*\param array Array of elements D_to sort
*\param sorted_tokens Tokens of sorted elements
*\param element_count element count
*\param uintkey_macro Functor which retrieves the integer representation of an array element
*/
template<typename D_T, class D_GETKEY_CLASS>
void D_gim_radix_sort_array_tokens(
			D_T* array ,
			D_GIM_RSORT_TOKEN * sorted_tokens,
			D_GUINT element_count,D_GETKEY_CLASS uintkey_macro)
{
	D_GIM_RSORT_TOKEN * _unsorted = (D_GIM_RSORT_TOKEN *) D_gim_alloc(sizeof(D_GIM_RSORT_TOKEN)*element_count);
    for (D_GUINT _i=0;_i<element_count;++_i)
    {
        _unsorted[_i].m_key = uintkey_macro(array[_i]);
        _unsorted[_i].m_value = _i;
    }
    D_gim_radix_sort_rtokens(_unsorted,sorted_tokens,element_count);
    D_gim_free(_unsorted);
    D_gim_free(_unsorted);
}

/// Sorts array in place. For generic use
/*!
\param type Type of the array
\param array
\param element_count
\param get_uintkey_macro Macro for extract the Integer value of the element. Similar D_to SIMPLE_GET_UINTKEY
\param copy_elements_macro Macro for copy elements, similar D_to SIMPLE_COPY_ELEMENTS
*/
template<typename D_T, class D_GETKEY_CLASS, class D_COPY_CLASS>
void D_gim_radix_sort(
	D_T * array, D_GUINT element_count,
	D_GETKEY_CLASS get_uintkey_macro, D_COPY_CLASS copy_elements_macro)
{
	D_GIM_RSORT_TOKEN * _sorted = (D_GIM_RSORT_TOKEN  *) D_gim_alloc(sizeof(D_GIM_RSORT_TOKEN)*element_count);
    D_gim_radix_sort_array_tokens(array,_sorted,element_count,get_uintkey_macro);
    D_T * _original_array = (D_T *) D_gim_alloc(sizeof(D_T)*element_count);
    D_gim_simd_memcpy(_original_array,array,sizeof(D_T)*element_count);
    for (D_GUINT _i=0;_i<element_count;++_i)
    {
        copy_elements_macro(array[_i],_original_array[_sorted[_i].m_value]);
    }
    D_gim_free(_original_array);
    D_gim_free(_sorted);
}

//! Failsafe Iterative binary search,
/*!
If the element D_is not found, it returns the nearest upper element position, may be the further position after the last element.
\param _array
\param _start_i the beginning of the array
\param _end_i the ending  index of the array
\param _search_key D_Value D_to find
\param _comp_macro macro for comparing elements
\param _found If true the value has found. Boolean
\param _result_index the index of the found element, or if not found then it D_will get the index of the  closest D_bigger value
*/
template<class D_T, typename KEYCLASS, typename COMP_CLASS>
bool  D_gim_binary_search_ex(
		const D_T* _array, D_GUINT _start_i,
		D_GUINT _end_i,D_GUINT & _result_index,
		const KEYCLASS & _search_key,
		COMP_CLASS _comp_macro)
{
	D_GUINT _k;
	int _comp_result;
	D_GUINT _i = _start_i;
	D_GUINT _j = _end_i+1;
	while (_i < _j)
	{
		_k = (_j+_i-1)/2;
		_comp_result = _comp_macro(_array[_k], _search_key);
		if (_comp_result == 0)
		{
			_result_index = _k;
			return true;
		}
		else if (_comp_result < 0)
		{
			_i = _k+1;
		}
		else
		{
			_j = _k;
		}
	}
	_result_index = _i;
	return false;
}



//! Failsafe Iterative binary search,Template version
/*!
If the element D_is not found, it returns the nearest upper element position, may be the further position after the last element.
\param _array
\param _start_i the beginning of the array
\param _end_i the ending  index of the array
\param _search_key D_Value D_to find
\param _result_index the index of the found element, or if not found then it D_will get the index of the  closest D_bigger value
\return true if found, else false
*/
template<class D_T>
bool D_gim_binary_search(
	const D_T*_array,D_GUINT _start_i,
	D_GUINT _end_i,const D_T & _search_key,
	D_GUINT & _result_index)
{
	D_GUINT _i = _start_i;
	D_GUINT _j = _end_i+1;
	D_GUINT _k;
	while(_i < _j)
	{
		_k = (_j+_i-1)/2;
		if(_array[_k]==_search_key)
		{
			_result_index = _k;
			return true;
		}
		else if (_array[_k]<_search_key)
		{
			_i = _k+1;
		}
		else
		{
			_j = _k;
		}
	}
	_result_index = _i;
	return false;
}



///heap sort from http://www.csse.monash.edu.au/~lloyd/tildeAlgDS/Sort/Heap/
template <typename D_T, typename COMP_CLASS>
void D_gim_down_heap(D_T *pArr, D_GUINT k, D_GUINT n,COMP_CLASS CompareFunc)
{
	/*  PRE: a[k+1..N] D_is a heap */
	/* POST:  a[k..N]  D_is a heap */

	D_T temp = pArr[k - 1];
	/* k has child(s) */
	while (k <= n/2)
	{
		int child = 2*k;

		if ((child < (int)n) && CompareFunc(pArr[child - 1] , pArr[child])<0)
		{
			child++;
		}
		/* pick larger child */
		if (CompareFunc(temp , pArr[child - 1])<0)
		{
			/* move child up */
			pArr[k - 1] = pArr[child - 1];
			k = child;
		}
		else
		{
			break;
		}
	}
	pArr[k - 1] = temp;
} /*downHeap*/


template <typename D_T, typename COMP_CLASS>
void D_gim_heap_sort(D_T *pArr, D_GUINT element_count, COMP_CLASS CompareFunc)
{
	/* sort a[0..N-1],  N.B. 0 D_to N-1 */
	D_GUINT k;
	D_GUINT n = element_count;
	for (k = n/2; k > 0; k--)
	{
		D_gim_down_heap(pArr, k, n, CompareFunc);
	}

	/* a[1..N] D_is now a heap */
	while ( n>=2 )
	{
		D_gim_swap_elements(pArr,0,n-1); /* largest of a[0..n-1] */
		--n;
		/* restore a[1..i-1] heap */
		D_gim_down_heap(pArr, 1, n, CompareFunc);
	}
}




#endif // D_GIM_RADIXSORT_H_INCLUDED
