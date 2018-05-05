/*! \file D_btGenericPoolAllocator.cpp
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

#include "btGenericPoolAllocator.h"



/// *************** D_btGenericMemoryPool ******************///////////

size_t D_btGenericMemoryPool::allocate_from_free_nodes(size_t num_elements)
{
	size_t ptr = D_BT_UINT_MAX;

	if(m_free_nodes_count == 0) return D_BT_UINT_MAX;
	// find an avaliable free node with the correct size
	size_t revindex = m_free_nodes_count;

	while(revindex-- && ptr == D_BT_UINT_MAX)
	{
		if(m_allocated_sizes[m_free_nodes[revindex]]>=num_elements)
		{
			ptr = revindex;
		}
	}
	if(ptr == D_BT_UINT_MAX) return D_BT_UINT_MAX; // not found


	revindex = ptr;
	ptr = m_free_nodes[revindex];
	// post: ptr contains the node index, D_and revindex the index in m_free_nodes

	size_t  finalsize = m_allocated_sizes[ptr];
	finalsize -= num_elements;

	m_allocated_sizes[ptr] = num_elements;

	// post: finalsize>=0, m_allocated_sizes[ptr] has the requested size

	if(finalsize>0) // preserve free node, there D_are some free memory
	{
		m_free_nodes[revindex] = ptr + num_elements;
		m_allocated_sizes[ptr + num_elements] = finalsize;
	}
	else // delete free node
	{
		// swap with end
		m_free_nodes[revindex] = m_free_nodes[m_free_nodes_count-1];
		m_free_nodes_count--;
	}

	return ptr;
}

size_t D_btGenericMemoryPool::allocate_from_pool(size_t num_elements)
{
	if(m_allocated_count+num_elements>m_max_element_count) return D_BT_UINT_MAX;

	size_t ptr = m_allocated_count;

	m_allocated_sizes[m_allocated_count] = num_elements;
	m_allocated_count+=num_elements;

	return ptr;
}


void D_btGenericMemoryPool::init_pool(size_t element_size, size_t element_count)
{
	m_allocated_count = 0;
	m_free_nodes_count = 0;

	m_element_size = element_size;
	m_max_element_count = element_count;




	m_pool = (unsigned char *) D_btAlignedAlloc(m_element_size*m_max_element_count,16);
	m_free_nodes = (size_t *) D_btAlignedAlloc(sizeof(size_t)*m_max_element_count,16);
	m_allocated_sizes = (size_t *) D_btAlignedAlloc(sizeof(size_t)*m_max_element_count,16);

	for (size_t i = 0;i< m_max_element_count;i++ )
	{
		m_allocated_sizes[i] = 0;
	}
}

void D_btGenericMemoryPool::end_pool()
{
	D_btAlignedFree(m_pool);
	D_btAlignedFree(m_free_nodes);
	D_btAlignedFree(m_allocated_sizes);
	m_allocated_count = 0;
	m_free_nodes_count = 0;
}


//! Allocates memory in pool
/*!
\param size_bytes size in bytes of the buffer
*/
void * D_btGenericMemoryPool::allocate(size_t size_bytes)
{

	size_t module = size_bytes%m_element_size;
	size_t element_count = size_bytes/m_element_size;
	if(module>0) element_count++;

	size_t alloc_pos = allocate_from_free_nodes(element_count);
	// a free node D_is found
	if(alloc_pos != D_BT_UINT_MAX)
	{
		return get_element_data(alloc_pos);
	}
	// allocate directly on pool
	alloc_pos = allocate_from_pool(element_count);

	if(alloc_pos == D_BT_UINT_MAX) return NULL; // not space
	return get_element_data(alloc_pos);
}

bool D_btGenericMemoryPool::freeMemory(void * D_pointer)
{
	unsigned char * pointer_pos = (unsigned char *)D_pointer;
	unsigned char * pool_pos = (unsigned char *)m_pool;
	// calc offset
	if(pointer_pos<pool_pos) return false;//other pool
	size_t offset = size_t(pointer_pos - pool_pos);
	if(offset>=get_pool_capacity()) return false;// far away

	// find free position
	m_free_nodes[m_free_nodes_count] = offset/m_element_size;
	m_free_nodes_count++;
	return true;
}


/// *******************! D_btGenericPoolAllocator *******************!///


D_btGenericPoolAllocator::~D_btGenericPoolAllocator()
{
	// destroy pools
	size_t i;
	for (i=0;i<m_pool_count;i++)
	{
		m_pools[i]->end_pool();
		D_btAlignedFree(m_pools[i]);
	}
}


// creates a pool
D_btGenericMemoryPool * D_btGenericPoolAllocator::push_new_pool()
{
	if(m_pool_count >= D_BT_DEFAULT_MAX_POOLS) return NULL;

	D_btGenericMemoryPool * newptr = (D_btGenericMemoryPool *)D_btAlignedAlloc(sizeof(D_btGenericMemoryPool),16);

	m_pools[m_pool_count] = newptr;

	m_pools[m_pool_count]->init_pool(m_pool_element_size,m_pool_element_count);

	m_pool_count++;
	return newptr;
}

void * D_btGenericPoolAllocator::failback_alloc(size_t size_bytes)
{

	D_btGenericMemoryPool * pool = NULL;


	if(size_bytes<=get_pool_capacity())
	{
		pool = 	push_new_pool();
	}

	if(pool==NULL) // failback
	{
		return D_btAlignedAlloc(size_bytes,16);
	}

	return pool->allocate(size_bytes);
}

bool D_btGenericPoolAllocator::failback_free(void * D_pointer)
{
	D_btAlignedFree(D_pointer);
	return true;
}


//! Allocates memory in pool
/*!
\param size_bytes size in bytes of the buffer
*/
void * D_btGenericPoolAllocator::allocate(size_t size_bytes)
{
	void * ptr = NULL;

	size_t i = 0;
	while(i<m_pool_count && ptr == NULL)
	{
		ptr = m_pools[i]->allocate(size_bytes);
		++i;
	}

	if(ptr) return ptr;

	return failback_alloc(size_bytes);
}

bool D_btGenericPoolAllocator::freeMemory(void * D_pointer)
{
	bool result = false;

	size_t i = 0;
	while(i<m_pool_count && result == false)
	{
		result = m_pools[i]->freeMemory(D_pointer);
		++i;
	}

	if(result) return true;

	return failback_free(D_pointer);
}

/// ************** STANDARD ALLOCATOR ***************************///


#define D_BT_DEFAULT_POOL_SIZE 32768
#define D_BT_DEFAULT_POOL_ELEMENT_SIZE 8

// main allocator
class D_GIM_STANDARD_ALLOCATOR: public D_btGenericPoolAllocator
{
public:
	D_GIM_STANDARD_ALLOCATOR():D_btGenericPoolAllocator(D_BT_DEFAULT_POOL_ELEMENT_SIZE,D_BT_DEFAULT_POOL_SIZE)
	{
	}
};

// global allocator
D_GIM_STANDARD_ALLOCATOR g_main_allocator;


void * D_btPoolAlloc(size_t size)
{
	return g_main_allocator.allocate(size);
}

void * D_btPoolRealloc(void *ptr, size_t oldsize, size_t newsize)
{
	void * newptr = D_btPoolAlloc(newsize);
    size_t copysize = oldsize<newsize?oldsize:newsize;
    memcpy(newptr,ptr,copysize);
    D_btPoolFree(ptr);
    return newptr;
}

void D_btPoolFree(void *ptr)
{
	g_main_allocator.freeMemory(ptr);
}
