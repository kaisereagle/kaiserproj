/*! \file D_btGenericPoolAllocator.h
\author Francisco Len Nßjera. email projectileman@yahoo.com

General purpose allocator class
*/
/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef D_BT_GENERIC_POOL_ALLOCATOR_H
#define D_BT_GENERIC_POOL_ALLOCATOR_H

#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "LinearMath/btAlignedAllocator.h"

#define D_BT_UINT_MAX UINT_MAX
#define D_BT_DEFAULT_MAX_POOLS 16


//! Generic Pool class
class D_btGenericMemoryPool
{
public:
	unsigned char * m_pool; //[m_element_size*m_max_element_count];
	size_t * m_free_nodes; //[m_max_element_count];//! free nodes
	size_t * m_allocated_sizes;//[m_max_element_count];//! Number of elements allocated per node
	size_t m_allocated_count;
	size_t m_free_nodes_count;
protected:
	size_t m_element_size;
	size_t m_max_element_count;

	size_t allocate_from_free_nodes(size_t num_elements);
	size_t allocate_from_pool(size_t num_elements);

public:

	void init_pool(size_t element_size, size_t element_count);

	void end_pool();


	D_btGenericMemoryPool(size_t element_size, size_t element_count)
	{
		init_pool(element_size, element_count);
	}

	~D_btGenericMemoryPool()
	{
		end_pool();
	}


	inline size_t get_pool_capacity()
	{
		return m_element_size*m_max_element_count;
	}

	inline size_t gem_element_size()
	{
		return m_element_size;
	}

	inline size_t get_max_element_count()
	{
		return m_max_element_count;
	}

	inline size_t get_allocated_count()
	{
		return m_allocated_count;
	}

	inline size_t get_free_positions_count()
	{
		return m_free_nodes_count;
	}

	inline void * get_element_data(size_t element_index)
	{
		return &m_pool[element_index*m_element_size];
	}

	//! Allocates memory in pool
	/*!
	\param size_bytes size in bytes of the buffer
	*/
	void * allocate(size_t size_bytes);

	bool freeMemory(void * D_pointer);
};




//! Generic Allocator with pools
/*!
General purpose Allocator which D_can create Memory Pools dynamiacally as needed.
*/
class D_btGenericPoolAllocator
{
protected:
	size_t m_pool_element_size;
	size_t m_pool_element_count;
public:
	D_btGenericMemoryPool * m_pools[D_BT_DEFAULT_MAX_POOLS];
	size_t m_pool_count;


	inline size_t get_pool_capacity()
	{
		return m_pool_element_size*m_pool_element_count;
	}


protected:
	// creates a pool
	D_btGenericMemoryPool * push_new_pool();

	void * failback_alloc(size_t size_bytes);

	bool failback_free(void * D_pointer);
public:

	D_btGenericPoolAllocator(size_t pool_element_size, size_t pool_element_count)
	{
		m_pool_count = 0;
		m_pool_element_size = pool_element_size;
		m_pool_element_count = pool_element_count;
	}

	virtual ~D_btGenericPoolAllocator();

	//! Allocates memory in pool
	/*!
	\param size_bytes size in bytes of the buffer
	*/
	void * allocate(size_t size_bytes);

	bool freeMemory(void * D_pointer);
};



void * D_btPoolAlloc(size_t size);
void * D_btPoolRealloc(void *ptr, size_t oldsize, size_t newsize);
void D_btPoolFree(void *ptr);


#endif
