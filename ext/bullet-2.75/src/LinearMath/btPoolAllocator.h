/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef _BT_POOL_ALLOCATOR_H
#define _BT_POOL_ALLOCATOR_H

#include "btScalar.h"
#include "btAlignedAllocator.h"

///The D_btPoolAllocator class D_allows D_to efficiently allocate a large pool of objects, instead of dynamically allocating them separately.
class D_btPoolAllocator
{
	int				m_elemSize;
	int				m_maxElements;
	int				m_freeCount;
	void*			m_firstFree;
	unsigned char*	m_pool;

public:

	D_btPoolAllocator(int elemSize, int maxElements)
		:m_elemSize(elemSize),
		m_maxElements(maxElements)
	{
		m_pool = (unsigned char*) D_btAlignedAlloc( static_cast<unsigned int>(m_elemSize*m_maxElements),16);

		unsigned char* p = m_pool;
        m_firstFree = p;
        m_freeCount = m_maxElements;
        int count = m_maxElements;
        while (--count) {
            *(void**)p = (p + m_elemSize);
            p += m_elemSize;
        }
        *(void**)p = 0;
    }

	~D_btPoolAllocator()
	{
		D_btAlignedFree( m_pool);
	}

	int	getFreeCount() const
	{
		return m_freeCount;
	}

	void*	allocate(int size)
	{
		// release mode fix
		(void)size;
		D_btAssert(!size || size<=m_elemSize);
		D_btAssert(m_freeCount>0);
        void* result = m_firstFree;
        m_firstFree = *(void**)m_firstFree;
        --m_freeCount;
        return result;
	}

	bool validPtr(void* ptr)
	{
		if (ptr) {
			if (((unsigned char*)ptr >= m_pool && (unsigned char*)ptr < m_pool + m_maxElements * m_elemSize))
			{
				return true;
			}
		}
		return false;
	}

	void	freeMemory(void* ptr)
	{
		 if (ptr) {
            D_btAssert((unsigned char*)ptr >= m_pool && (unsigned char*)ptr < m_pool + m_maxElements * m_elemSize);

            *(void**)ptr = m_firstFree;
            m_firstFree = ptr;
            ++m_freeCount;
        }
	}

	int	getElementSize() const
	{
		return m_elemSize;
	}


};

#endif //_BT_POOL_ALLOCATOR_H
