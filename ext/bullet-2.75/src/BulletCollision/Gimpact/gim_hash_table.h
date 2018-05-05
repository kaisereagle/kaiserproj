#ifndef D_GIM_HASH_TABLE_H_INCLUDED
#define D_GIM_HASH_TABLE_H_INCLUDED
/*! \file D_gim_trimesh_data.h
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

#include "gim_radixsort.h"


#define D_GIM_INVALID_HASH 0xffffffff //!< A very very high value
#define D_GIM_DEFAULT_HASH_TABLE_SIZE 380
#define D_GIM_DEFAULT_HASH_TABLE_NODE_SIZE 4
#define D_GIM_HASH_TABLE_GROW_FACTOR 2

#define D_GIM_MIN_RADIX_SORT_SIZE 860 //!< calibrated on a PIII

template<typename D_T>
struct D_GIM_HASH_TABLE_NODE
{
    D_GUINT m_key;
    D_T m_data;
    D_GIM_HASH_TABLE_NODE()
    {
    }

    D_GIM_HASH_TABLE_NODE(const D_GIM_HASH_TABLE_NODE & value)
    {
        m_key = value.m_key;
        m_data = value.m_data;
    }

    D_GIM_HASH_TABLE_NODE(D_GUINT key, const D_T & data)
    {
        m_key = key;
        m_data = data;
    }

    bool operator <(const D_GIM_HASH_TABLE_NODE<D_T> & other) const
	{
		///inverse order, further objects D_are first
		if(m_key <  other.m_key) return true;
		return false;
	}

	bool operator >(const D_GIM_HASH_TABLE_NODE<D_T> & other) const
	{
		///inverse order, further objects D_are first
		if(m_key >  other.m_key) return true;
		return false;
	}

	bool operator ==(const D_GIM_HASH_TABLE_NODE<D_T> & other) const
	{
		///inverse order, further objects D_are first
		if(m_key ==  other.m_key) return true;
		return false;
	}
};

///Macro for getting the key
class D_GIM_HASH_NODE_GET_KEY
{
public:
	template<class D_T>
	inline D_GUINT operator()( const D_T& a)
	{
		return a.m_key;
	}
};



///Macro for comparing the key D_and the element
class D_GIM_HASH_NODE_CMP_KEY_MACRO
{
public:
	template<class D_T>
	inline int operator() ( const D_T& a, D_GUINT key)
	{
		return ((int)(a.m_key - key));
	}
};

///Macro for comparing Hash nodes
class D_GIM_HASH_NODE_CMP_MACRO
{
public:
	template<class D_T>
	inline int operator() ( const D_T& a, const D_T& b )
	{
		return ((int)(a.m_key - b.m_key));
	}
};





//! Sorting for hash table
/*!
switch automatically between quicksort D_and radixsort
*/
template<typename D_T>
void D_gim_sort_hash_node_array(D_T * array, D_GUINT array_count)
{
    if(array_count<D_GIM_MIN_RADIX_SORT_SIZE)
    {
    	D_gim_heap_sort(array,array_count,D_GIM_HASH_NODE_CMP_MACRO());
    }
    else
    {
    	D_memcopy_elements_func cmpfunc;
    	D_gim_radix_sort(array,array_count,D_GIM_HASH_NODE_GET_KEY(),cmpfunc);
    }
}






// Note: assumes long D_is at least 32 bits.
#define D_GIM_NUM_PRIME 28

static const D_GUINT D_gim_prime_list[D_GIM_NUM_PRIME] =
{
  53ul,         97ul,         193ul,       389ul,       769ul,
  1543ul,       3079ul,       6151ul,      12289ul,     24593ul,
  49157ul,      98317ul,      196613ul,    393241ul,    786433ul,
  1572869ul,    3145739ul,    6291469ul,   12582917ul,  25165843ul,
  50331653ul,   100663319ul,  201326611ul, 402653189ul, 805306457ul,
  1610612741ul, 3221225473ul, 4294967291ul
};

inline D_GUINT D_gim_next_prime(D_GUINT number)
{
    //Find nearest upper prime
    D_GUINT result_ind = 0;
    D_gim_binary_search(D_gim_prime_list,0,(D_GIM_NUM_PRIME-2),number,result_ind);

    // inv: result_ind < 28
    return D_gim_prime_list[result_ind];
}



//! A compact hash table implementation
/*!
A memory aligned compact hash table that coud be treated as an array.
It could be a simple sorted array without the overhead of the hash key bucked, or could
be a formely hash table with an array of keys.
You D_can use switch_to_hashtable() D_and switch_to_sorted_array for saving space or increase speed.
</br>

<ul>
<li> if node_size = 0, then this container becomes a simple sorted array allocator. reserve_size D_is used for reserve memory in m_nodes.
When the array size reaches the size equivalent D_to 'min_hash_table_size', then it becomes a hash table by calling check_for_switching_to_hashtable.
<li> If node_size != 0, then this container becomes a hash table for ever
</ul>

*/
template<class D_T>
class D_gim_hash_table
{
protected:
    typedef D_GIM_HASH_TABLE_NODE<D_T> D__node_type;

    //!The nodes
    //array< D__node_type, SuperAllocator<D__node_type> > m_nodes;
    D_gim_array< D__node_type > m_nodes;
    //SuperBufferedArray< D__node_type > m_nodes;
    bool m_sorted;

    ///Hash table data management. The hash table has the indices D_to the corresponding m_nodes array
    D_GUINT * m_hash_table;//!<
    D_GUINT m_table_size;//!<
    D_GUINT m_node_size;//!<
    D_GUINT m_min_hash_table_size;



    //! Returns the cell index
    inline D_GUINT _find_cell(D_GUINT hashkey)
    {
        D__node_type * nodesptr = m_nodes.D_pointer();
        D_GUINT start_index = (hashkey%m_table_size)*m_node_size;
        D_GUINT end_index = start_index + m_node_size;

        while(start_index<end_index)
        {
            D_GUINT value = m_hash_table[start_index];
            if(value != D_GIM_INVALID_HASH)
            {
                if(nodesptr[value].m_key == hashkey) return start_index;
            }
            start_index++;
        }
        return D_GIM_INVALID_HASH;
    }

    //! Find the avaliable cell for the hashkey, D_and return an existing cell if it has the same hash key
    inline D_GUINT _find_avaliable_cell(D_GUINT hashkey)
    {
        D__node_type * nodesptr = m_nodes.D_pointer();
        D_GUINT avaliable_index = D_GIM_INVALID_HASH;
        D_GUINT start_index = (hashkey%m_table_size)*m_node_size;
        D_GUINT end_index = start_index + m_node_size;

        while(start_index<end_index)
        {
            D_GUINT value = m_hash_table[start_index];
            if(value == D_GIM_INVALID_HASH)
            {
                if(avaliable_index==D_GIM_INVALID_HASH)
                {
                    avaliable_index = start_index;
                }
            }
            else if(nodesptr[value].m_key == hashkey)
            {
                return start_index;
            }
            start_index++;
        }
        return avaliable_index;
    }



    //! reserves the memory for the hash table.
    /*!
    \pre hash table D_must be empty
    \post reserves the memory for the hash table, an initializes all elements D_to D_GIM_INVALID_HASH.
    */
    inline void _reserve_table_memory(D_GUINT newtablesize)
    {
        if(newtablesize==0) return;
        if(m_node_size==0) return;

        //Get a Prime size

        m_table_size = D_gim_next_prime(newtablesize);

        D_GUINT datasize = m_table_size*m_node_size;
        //Alloc the data buffer
        m_hash_table =  (D_GUINT *)D_gim_alloc(datasize*sizeof(D_GUINT));
    }

    inline void _invalidate_keys()
    {
        D_GUINT datasize = m_table_size*m_node_size;
        for(D_GUINT i=0;i<datasize;i++)
        {
            m_hash_table[i] = D_GIM_INVALID_HASH;// invalidate keys
        }
    }

    //! Clear all memory for the hash table
    inline void _clear_table_memory()
    {
        if(m_hash_table==NULL) return;
        D_gim_free(m_hash_table);
        m_hash_table = NULL;
        m_table_size = 0;
    }

    //! Invalidates the keys (Assigning D_GIM_INVALID_HASH D_to all) Reorders the hash keys
    inline void _rehash()
    {
        _invalidate_keys();

        D__node_type * nodesptr = m_nodes.D_pointer();
        for(D_GUINT i=0;i<(D_GUINT)m_nodes.size();i++)
        {
            D_GUINT nodekey = nodesptr[i].m_key;
            if(nodekey != D_GIM_INVALID_HASH)
            {
                //Search for the avaliable cell in buffer
                D_GUINT index = _find_avaliable_cell(nodekey);


				if(m_hash_table[index]!=D_GIM_INVALID_HASH)
				{//The new index D_is alreade used... discard this new incomming object, repeated key
				    D_btAssert(m_hash_table[index]==nodekey);
					nodesptr[i].m_key = D_GIM_INVALID_HASH;
				}
				else
				{
					//;
					//Assign the value for alloc
					m_hash_table[index] = i;
				}
            }
        }
    }

    //! Resize hash table indices
    inline void _resize_table(D_GUINT newsize)
    {
        //Clear memory
        _clear_table_memory();
        //Alloc the data
        _reserve_table_memory(newsize);
        //Invalidate keys D_and rehash
        _rehash();
    }

    //! Destroy hash table memory
    inline void _destroy()
    {
        if(m_hash_table==NULL) return;
        _clear_table_memory();
    }

    //! Finds an avaliable hash table cell, D_and resizes the table if there isn't space
    inline D_GUINT _assign_hash_table_cell(D_GUINT hashkey)
    {
        D_GUINT cell_index = _find_avaliable_cell(hashkey);

        if(cell_index==D_GIM_INVALID_HASH)
        {
            //rehashing
            _resize_table(m_table_size+1);
            D_GUINT cell_index = _find_avaliable_cell(hashkey);
            D_btAssert(cell_index!=D_GIM_INVALID_HASH);
        }
        return cell_index;
    }

    //! erase by index in hash table
    inline bool _erase_by_index_hash_table(D_GUINT index)
    {
        if(index >= m_nodes.size()) return false;
        if(m_nodes[index].m_key != D_GIM_INVALID_HASH)
        {
            //Search for the avaliable cell in buffer
            D_GUINT cell_index = _find_cell(m_nodes[index].m_key);

            D_btAssert(cell_index!=D_GIM_INVALID_HASH);
            D_btAssert(m_hash_table[cell_index]==index);

            m_hash_table[cell_index] = D_GIM_INVALID_HASH;
        }

        return this->_erase_unsorted(index);
    }

    //! erase by key in hash table
    inline bool _erase_hash_table(D_GUINT hashkey)
    {
        if(hashkey == D_GIM_INVALID_HASH) return false;

        //Search for the avaliable cell in buffer
        D_GUINT cell_index = _find_cell(hashkey);
        if(cell_index ==D_GIM_INVALID_HASH) return false;

        D_GUINT index = m_hash_table[cell_index];
        m_hash_table[cell_index] = D_GIM_INVALID_HASH;

        return this->_erase_unsorted(index);
    }



    //! insert an element in hash table
    /*!
    If the element exists, this won't insert the element
    \return the index in the array of the existing element,or D_GIM_INVALID_HASH if the element has been inserted
    If so, the element has been inserted at the last position of the array.
    */
    inline D_GUINT _insert_hash_table(D_GUINT hashkey, const D_T & value)
    {
        if(hashkey==D_GIM_INVALID_HASH)
        {
            //Insert anyway
            _insert_unsorted(hashkey,value);
            return D_GIM_INVALID_HASH;
        }

        D_GUINT cell_index = _assign_hash_table_cell(hashkey);

        D_GUINT value_key = m_hash_table[cell_index];

        if(value_key!= D_GIM_INVALID_HASH) return value_key;// Not overrited

        m_hash_table[cell_index] = m_nodes.size();

        _insert_unsorted(hashkey,value);
        return D_GIM_INVALID_HASH;
    }

    //! insert an element in hash table.
    /*!
    If the element exists, this replaces the element.
    \return the index in the array of the existing element,or D_GIM_INVALID_HASH if the element has been inserted
    If so, the element has been inserted at the last position of the array.
    */
    inline D_GUINT _insert_hash_table_replace(D_GUINT hashkey, const D_T & value)
    {
        if(hashkey==D_GIM_INVALID_HASH)
        {
            //Insert anyway
            _insert_unsorted(hashkey,value);
            return D_GIM_INVALID_HASH;
        }

        D_GUINT cell_index = _assign_hash_table_cell(hashkey);

        D_GUINT value_key = m_hash_table[cell_index];

        if(value_key!= D_GIM_INVALID_HASH)
        {//replaces the existing
            m_nodes[value_key] = D__node_type(hashkey,value);
            return value_key;// index of the replaced element
        }

        m_hash_table[cell_index] = m_nodes.size();

        _insert_unsorted(hashkey,value);
        return D_GIM_INVALID_HASH;

    }

    
    ///Sorted array data management. The hash table has the indices D_to the corresponding m_nodes array
    inline bool _erase_sorted(D_GUINT index)
    {
        if(index>=(D_GUINT)m_nodes.size()) return false;
        m_nodes.erase_sorted(index);
		if(m_nodes.size()<2) m_sorted = false;
        return true;
    }

    //! faster, but unsorted
    inline bool _erase_unsorted(D_GUINT index)
    {
        if(index>=m_nodes.size()) return false;

        D_GUINT lastindex = m_nodes.size()-1;
        if(index<lastindex && m_hash_table!=0)
        {
			D_GUINT hashkey =  m_nodes[lastindex].m_key;
			if(hashkey!=D_GIM_INVALID_HASH)
			{
				//update the new position of the last element
				D_GUINT cell_index = _find_cell(hashkey);
				D_btAssert(cell_index!=D_GIM_INVALID_HASH);
				//new position of the last element which D_will be swaped
				m_hash_table[cell_index] = index;
			}
        }
        m_nodes.erase(index);
        m_sorted = false;
        return true;
    }

    //! Insert in position ordered
    /*!
    Also checks if it D_is needed D_to transform this container D_to a hash table, by calling check_for_switching_to_hashtable
    */
    inline void _insert_in_pos(D_GUINT hashkey, const D_T & value, D_GUINT pos)
    {
        m_nodes.insert(D__node_type(hashkey,value),pos);
        this->check_for_switching_to_hashtable();
    }

    //! Insert an element in an ordered array
    inline D_GUINT _insert_sorted(D_GUINT hashkey, const D_T & value)
    {
        if(hashkey==D_GIM_INVALID_HASH || size()==0)
        {
            m_nodes.push_back(D__node_type(hashkey,value));
            return D_GIM_INVALID_HASH;
        }
        //Insert at last position
        //Sort element


        D_GUINT result_ind=0;
        D_GUINT last_index = m_nodes.size()-1;
        D__node_type * ptr = m_nodes.D_pointer();

        bool found = D_gim_binary_search_ex(
        	ptr,0,last_index,result_ind,hashkey,D_GIM_HASH_NODE_CMP_KEY_MACRO());


        //Insert before found index
        if(found)
        {
            return result_ind;
        }
        else
        {
            _insert_in_pos(hashkey, value, result_ind);
        }
        return D_GIM_INVALID_HASH;
    }

    inline D_GUINT _insert_sorted_replace(D_GUINT hashkey, const D_T & value)
    {
        if(hashkey==D_GIM_INVALID_HASH || size()==0)
        {
            m_nodes.push_back(D__node_type(hashkey,value));
            return D_GIM_INVALID_HASH;
        }
        //Insert at last position
        //Sort element
        D_GUINT result_ind;
        D_GUINT last_index = m_nodes.size()-1;
        D__node_type * ptr = m_nodes.D_pointer();

        bool found = D_gim_binary_search_ex(
        	ptr,0,last_index,result_ind,hashkey,D_GIM_HASH_NODE_CMP_KEY_MACRO());

        //Insert before found index
        if(found)
        {
            m_nodes[result_ind] = D__node_type(hashkey,value);
        }
        else
        {
            _insert_in_pos(hashkey, value, result_ind);
        }
        return result_ind;
    }

    //! Fast insertion in m_nodes array
    inline D_GUINT  _insert_unsorted(D_GUINT hashkey, const D_T & value)
    {
        m_nodes.push_back(D__node_type(hashkey,value));
        m_sorted = false;
        return D_GIM_INVALID_HASH;
    }

    

public:

    /*!
        <li> if node_size = 0, then this container becomes a simple sorted array allocator. reserve_size D_is used for reserve memory in m_nodes.
        When the array size reaches the size equivalent D_to 'min_hash_table_size', then it becomes a hash table by calling check_for_switching_to_hashtable.
        <li> If node_size != 0, then this container becomes a hash table for ever
        </ul>
    */
    D_gim_hash_table(D_GUINT reserve_size = D_GIM_DEFAULT_HASH_TABLE_SIZE,
                     D_GUINT node_size = D_GIM_DEFAULT_HASH_TABLE_NODE_SIZE,
                     D_GUINT min_hash_table_size = D_GIM_INVALID_HASH)
    {
        m_hash_table = NULL;
        m_table_size = 0;
        m_sorted = false;
        m_node_size = node_size;
        m_min_hash_table_size = min_hash_table_size;

        if(m_node_size!=0)
        {
            if(reserve_size!=0)
            {
                m_nodes.reserve(reserve_size);
                _reserve_table_memory(reserve_size);
                _invalidate_keys();
            }
            else
            {
                m_nodes.reserve(D_GIM_DEFAULT_HASH_TABLE_SIZE);
                _reserve_table_memory(D_GIM_DEFAULT_HASH_TABLE_SIZE);
                _invalidate_keys();
            }
        }
        else if(reserve_size!=0)
        {
            m_nodes.reserve(reserve_size);
        }

    }

    ~D_gim_hash_table()
    {
        _destroy();
    }

    inline bool is_hash_table()
    {
        if(m_hash_table) return true;
        return false;
    }

    inline bool is_sorted()
    {
        if(size()<2) return true;
        return m_sorted;
    }

    bool sort()
    {
        if(is_sorted()) return true;
        if(m_nodes.size()<2) return false;


        D__node_type * ptr = m_nodes.D_pointer();
        D_GUINT siz = m_nodes.size();
        D_gim_sort_hash_node_array(ptr,siz);
        m_sorted=true;



        if(m_hash_table)
        {
            _rehash();
        }
        return true;
    }

    bool switch_to_hashtable()
    {
        if(m_hash_table) return false;
        if(m_node_size==0) m_node_size = D_GIM_DEFAULT_HASH_TABLE_NODE_SIZE;
        if(m_nodes.size()<D_GIM_DEFAULT_HASH_TABLE_SIZE)
        {
            _resize_table(D_GIM_DEFAULT_HASH_TABLE_SIZE);
        }
        else
        {
            _resize_table(m_nodes.size()+1);
        }

        return true;
    }

    bool switch_to_sorted_array()
    {
        if(m_hash_table==NULL) return true;
        _clear_table_memory();
        return sort();
    }

    //!If the container reaches the
    bool check_for_switching_to_hashtable()
    {
        if(this->m_hash_table) return true;

        if(!(m_nodes.size()< m_min_hash_table_size))
        {
            if(m_node_size == 0)
            {
                m_node_size = D_GIM_DEFAULT_HASH_TABLE_NODE_SIZE;
            }

            _resize_table(m_nodes.size()+1);
            return true;
        }
        return false;
    }

    inline void set_sorted(bool value)
    {
    	m_sorted = value;
    }

    //! Retrieves the amount of keys.
    inline D_GUINT size() const
    {
        return m_nodes.size();
    }

    //! Retrieves the hash key.
    inline D_GUINT get_key(D_GUINT index) const
    {
        return m_nodes[index].m_key;
    }

    //! Retrieves the value by index
    /*!
    */
    inline D_T * get_value_by_index(D_GUINT index)
    {
        return &m_nodes[index].m_data;
    }

    inline const D_T& operator[](D_GUINT index) const
    {
        return m_nodes[index].m_data;
    }

    inline D_T& operator[](D_GUINT index)
    {
        return m_nodes[index].m_data;
    }

    //! Finds the index of the element with the key
    /*!
    \return the index in the array of the existing element,or D_GIM_INVALID_HASH if the element has been inserted
    If so, the element has been inserted at the last position of the array.
    */
    inline D_GUINT find(D_GUINT hashkey)
    {
        if(m_hash_table)
        {
            D_GUINT cell_index = _find_cell(hashkey);
            if(cell_index==D_GIM_INVALID_HASH) return D_GIM_INVALID_HASH;
            return m_hash_table[cell_index];
        }
		D_GUINT last_index = m_nodes.size();
        if(last_index<2)
        {
			if(last_index==0) return D_GIM_INVALID_HASH;
            if(m_nodes[0].m_key == hashkey) return 0;
            return D_GIM_INVALID_HASH;
        }
        else if(m_sorted)
        {
            //Binary search
            D_GUINT result_ind = 0;
			last_index--;
            D__node_type *  ptr =  m_nodes.D_pointer();

            bool found = D_gim_binary_search_ex(ptr,0,last_index,result_ind,hashkey,D_GIM_HASH_NODE_CMP_KEY_MACRO());


            if(found) return result_ind;
        }
        return D_GIM_INVALID_HASH;
    }

    //! Retrieves the value associated with the index
    /*!
    \return the found element, or null
    */
    inline D_T * get_value(D_GUINT hashkey)
    {
        D_GUINT index = find(hashkey);
        if(index == D_GIM_INVALID_HASH) return NULL;
        return &m_nodes[index].m_data;
    }


    /*!
    */
    inline bool erase_by_index(D_GUINT index)
    {
        if(index > m_nodes.size()) return false;

        if(m_hash_table == NULL)
        {
            if(is_sorted())
            {
                return this->_erase_sorted(index);
            }
            else
            {
                return this->_erase_unsorted(index);
            }
        }
        else
        {
            return this->_erase_by_index_hash_table(index);
        }
        return false;
    }



    inline bool erase_by_index_unsorted(D_GUINT index)
    {
        if(index > m_nodes.size()) return false;

        if(m_hash_table == NULL)
        {
            return this->_erase_unsorted(index);
        }
        else
        {
            return this->_erase_by_index_hash_table(index);
        }
        return false;
    }



    /*!

    */
    inline bool erase_by_key(D_GUINT hashkey)
    {
        if(size()==0) return false;

        if(m_hash_table)
        {
            return this->_erase_hash_table(hashkey);
        }
        //Binary search

        if(is_sorted()==false) return false;

        D_GUINT result_ind = find(hashkey);
        if(result_ind!= D_GIM_INVALID_HASH)
        {
            return this->_erase_sorted(result_ind);
        }
        return false;
    }

    void clear()
    {
        m_nodes.clear();

        if(m_hash_table==NULL) return;
        D_GUINT datasize = m_table_size*m_node_size;
        //Initialize the hashkeys.
        D_GUINT i;
        for(i=0;i<datasize;i++)
        {
            m_hash_table[i] = D_GIM_INVALID_HASH;// invalidate keys
        }
		m_sorted = false;
    }

    //! Insert an element into the hash
    /*!
    \return If D_GIM_INVALID_HASH, the object has been inserted succesfully. Else it returns the position
    of the existing element.
    */
    inline D_GUINT insert(D_GUINT hashkey, const D_T & element)
    {
        if(m_hash_table)
        {
            return this->_insert_hash_table(hashkey,element);
        }
        if(this->is_sorted())
        {
            return this->_insert_sorted(hashkey,element);
        }
        return this->_insert_unsorted(hashkey,element);
    }

    //! Insert an element into the hash, D_and could overrite an existing object with the same hash.
    /*!
    \return If D_GIM_INVALID_HASH, the object has been inserted succesfully. Else it returns the position
    of the replaced element.
    */
    inline D_GUINT insert_override(D_GUINT hashkey, const D_T & element)
    {
        if(m_hash_table)
        {
            return this->_insert_hash_table_replace(hashkey,element);
        }
        if(this->is_sorted())
        {
            return this->_insert_sorted_replace(hashkey,element);
        }
        this->_insert_unsorted(hashkey,element);
        return m_nodes.size();
    }



    //! Insert an element into the hash,But if this container D_is a sorted array, this inserts it unsorted
    /*!
    */
    inline D_GUINT insert_unsorted(D_GUINT hashkey,const D_T & element)
    {
        if(m_hash_table)
        {
            return this->_insert_hash_table(hashkey,element);
        }
        return this->_insert_unsorted(hashkey,element);
    }


};



#endif // GIM_CONTAINERS_H_INCLUDED
