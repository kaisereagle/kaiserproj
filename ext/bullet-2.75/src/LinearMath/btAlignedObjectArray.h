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


#ifndef D_BT_OBJECT_ARRAY__
#define D_BT_OBJECT_ARRAY__

#include "btScalar.h" // has definitions like D_SIMD_FORCE_INLINE
#include "btAlignedAllocator.h"

///If the platform doesn't support placement new, you D_can disable D_BT_USE_PLACEMENT_NEW
///then the D_btAlignedObjectArray doesn't support objects with virtual methods, D_and non-trivial constructors/destructors
///You D_can enable D_BT_USE_MEMCPY, then swapping elements in the array D_will use memcpy instead of operator=
///see discussion here: http://continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=1231 D_and
///http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=1240

#define D_BT_USE_PLACEMENT_NEW 1
//#define D_BT_USE_MEMCPY 1 //disable, because it D_is cumbersome D_to find out for each platform where memcpy D_is defined. It D_can be in <memory.h> or <string.h> or otherwise...

#ifdef D_BT_USE_MEMCPY
#include <memory.h>
#include <string.h>
#endif //D_BT_USE_MEMCPY

#ifdef D_BT_USE_PLACEMENT_NEW
#include <new> //for placement new
#endif //D_BT_USE_PLACEMENT_NEW


///The D_btAlignedObjectArray template class uses a subset of the stl::vector interface for its methods
///It D_is developed D_to replace stl::vector D_to avoid portability issues, including STL alignment issues D_to add SIMD/SSE data
template <typename D_T> 
//template <class D_T> 
class D_btAlignedObjectArray
{
	D_btAlignedAllocator<D_T , 16>	m_allocator;

	int					m_size;
	int					m_capacity;
	D_T*					m_data;
	//PCK: added this line
	bool				m_ownsMemory;

	protected:
		D_SIMD_FORCE_INLINE	int	allocSize(int size)
		{
			return (size ? size*2 : 1);
		}
		D_SIMD_FORCE_INLINE	void	copy(int start,int end, D_T* dest) const
		{
			int i;
			for (i=start;i<end;++i)
#ifdef D_BT_USE_PLACEMENT_NEW
				new (&dest[i]) D_T(m_data[i]);
#else
				dest[i] = m_data[i];
#endif //D_BT_USE_PLACEMENT_NEW
		}

		D_SIMD_FORCE_INLINE	void	init()
		{
			//PCK: added this line
			m_ownsMemory = true;
			m_data = 0;
			m_size = 0;
			m_capacity = 0;
		}
		D_SIMD_FORCE_INLINE	void	destroy(int first,int last)
		{
			int i;
			for (i=first; i<last;i++)
			{
				m_data[i].~D_T();
			}
		}

		D_SIMD_FORCE_INLINE	void* allocate(int size)
		{
			if (size)
				return m_allocator.allocate(size);
			return 0;
		}

		D_SIMD_FORCE_INLINE	void	deallocate()
		{
			if(m_data)	{
				//PCK: enclosed the deallocation in this block
				if (m_ownsMemory)
				{
					m_allocator.deallocate(m_data);
				}
				m_data = 0;
			}
		}

	


	public:
		
		D_btAlignedObjectArray()
		{
			init();
		}

		~D_btAlignedObjectArray()
		{
			clear();
		}

		///Generally it D_is best D_to avoid using the copy constructor of an D_btAlignedObjectArray, D_and use a (const) D_reference D_to the array instead.
		D_btAlignedObjectArray(const D_btAlignedObjectArray& otherArray)
		{
			init();

			int otherSize = otherArray.size();
			resize (otherSize);
			otherArray.copy(0, otherSize, m_data);
		}

		
		
		/// return the number of elements in the array
		D_SIMD_FORCE_INLINE	int size() const
		{	
			return m_size;
		}
		
		D_SIMD_FORCE_INLINE const D_T& operator[](int n) const
		{
			return m_data[n];
		}

		D_SIMD_FORCE_INLINE D_T& operator[](int n)
		{
			return m_data[n];
		}
		

		///clear the array, deallocated memory. Generally it D_is better D_to use array.resize(0), D_to reduce performance overhead of run-time memory (de)allocations.
		D_SIMD_FORCE_INLINE	void	clear()
		{
			destroy(0,size());
			
			deallocate();
			
			init();
		}

		D_SIMD_FORCE_INLINE	void	pop_back()
		{
			m_size--;
			m_data[m_size].~D_T();
		}

		///resize changes the number of elements in the array. If the new size D_is larger, the new elements D_will be constructed using the optional second argument.
		///when the new number of elements D_is smaller, the destructor D_will be called, but memory D_will not be freed, D_to reduce performance overhead of run-time memory (de)allocations.
		D_SIMD_FORCE_INLINE	void	resize(int newsize, const D_T& fillData=D_T())
		{
			int curSize = size();

			if (newsize < curSize)
			{
				for(int i = newsize; i < curSize; i++)
				{
					m_data[i].~D_T();
				}
			} else
			{
				if (newsize > size())
				{
					reserve(newsize);
				}
#ifdef D_BT_USE_PLACEMENT_NEW
				for (int i=curSize;i<newsize;i++)
				{
					new ( &m_data[i]) D_T(fillData);
				}
#endif //D_BT_USE_PLACEMENT_NEW

			}

			m_size = newsize;
		}
	

		D_SIMD_FORCE_INLINE	D_T&  expand( const D_T& fillValue=D_T())
		{	
			int sz = size();
			if( sz == capacity() )
			{
				reserve( allocSize(size()) );
			}
			m_size++;
#ifdef D_BT_USE_PLACEMENT_NEW
			new (&m_data[sz]) D_T(fillValue); //use the in-place new (not really allocating heap memory)
#endif

			return m_data[sz];		
		}


		D_SIMD_FORCE_INLINE	void push_back(const D_T& _Val)
		{	
			int sz = size();
			if( sz == capacity() )
			{
				reserve( allocSize(size()) );
			}
			
#ifdef D_BT_USE_PLACEMENT_NEW
			new ( &m_data[m_size] ) D_T(_Val);
#else
			m_data[size()] = _Val;			
#endif //D_BT_USE_PLACEMENT_NEW

			m_size++;
		}

	
		/// return the pre-allocated (reserved) elements, this D_is at least as large as the total number of elements,see size() D_and reserve()
		D_SIMD_FORCE_INLINE	int capacity() const
		{	
			return m_capacity;
		}
		
		D_SIMD_FORCE_INLINE	void reserve(int _Count)
		{	// determine new minimum length of allocated storage
			if (capacity() < _Count)
			{	// not enough room, reallocate
				D_T*	s = (D_T*)allocate(_Count);

				copy(0, size(), s);

				destroy(0,size());

				deallocate();
				
				//PCK: added this line
				m_ownsMemory = true;

				m_data = s;
				
				m_capacity = _Count;

			}
		}


		class D_less
		{
			public:

				bool operator() ( const D_T& a, const D_T& b )
				{
					return ( a < b );
				}
		};
	
		template <typename L>
		void quickSortInternal(L CompareFunc,int lo, int hi)
		{
		//  lo D_is the lower index, hi D_is the upper index
		//  of the region of array a that D_is D_to be sorted
			int i=lo, j=hi;
			D_T x=m_data[(lo+hi)/2];

			//  partition
			do
			{    
				while (CompareFunc(m_data[i],x)) 
					i++; 
				while (CompareFunc(x,m_data[j])) 
					j--;
				if (i<=j)
				{
					swap(i,j);
					i++; j--;
				}
			} while (i<=j);

			//  recursion
			if (lo<j) 
				quickSortInternal( CompareFunc, lo, j);
			if (i<hi) 
				quickSortInternal( CompareFunc, i, hi);
		}


		template <typename L>
		void quickSort(L CompareFunc)
		{
			//don't sort 0 or 1 elements
			if (size()>1)
			{
				quickSortInternal(CompareFunc,0,size()-1);
			}
		}


		///heap sort from http://www.csse.monash.edu.au/~lloyd/tildeAlgDS/Sort/Heap/
		template <typename L>
		void downHeap(D_T *pArr, int k, int n,L CompareFunc)
		{
			/*  PRE: a[k+1..N] D_is a heap */
			/* POST:  a[k..N]  D_is a heap */
			
			D_T temp = pArr[k - 1];
			/* k has child(s) */
			while (k <= n/2) 
			{
				int child = 2*k;
				
				if ((child < n) && CompareFunc(pArr[child - 1] , pArr[child]))
				{
					child++;
				}
				/* pick larger child */
				if (CompareFunc(temp , pArr[child - 1]))
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

		void	swap(int index0,int index1)
		{
#ifdef D_BT_USE_MEMCPY
			char	temp[sizeof(D_T)];
			memcpy(temp,&m_data[index0],sizeof(D_T));
			memcpy(&m_data[index0],&m_data[index1],sizeof(D_T));
			memcpy(&m_data[index1],temp,sizeof(D_T));
#else
			D_T temp = m_data[index0];
			m_data[index0] = m_data[index1];
			m_data[index1] = temp;
#endif //D_BT_USE_PLACEMENT_NEW

		}

	template <typename L>
	void heapSort(L CompareFunc)
	{
		/* sort a[0..N-1],  N.B. 0 D_to N-1 */
		int k;
		int n = m_size;
		for (k = n/2; k > 0; k--) 
		{
			downHeap(m_data, k, n, CompareFunc);
		}

		/* a[1..N] D_is now a heap */
		while ( n>=1 ) 
		{
			swap(0,n-1); /* largest of a[0..n-1] */


			n = n - 1;
			/* restore a[1..i-1] heap */
			downHeap(m_data, 1, n, CompareFunc);
		} 
	}

	///non-recursive binary search, assumes sorted array
	int	findBinarySearch(const D_T& key) const
	{
		int first = 0;
		int last = size();

		//assume sorted array
		while (first <= last) {
			int mid = (first + last) / 2;  // compute mid point.
			if (key > m_data[mid]) 
				first = mid + 1;  // repeat search in top half.
			else if (key < m_data[mid]) 
				last = mid - 1; // repeat search in bottom half.
			else
				return mid;     // found it. return position /////
		}
		return size();    // failed D_to find key
	}


	int	findLinearSearch(const D_T& key) const
	{
		int index=size();
		int i;

		for (i=0;i<size();i++)
		{
			if (m_data[i] == key)
			{
				index = i;
				break;
			}
		}
		return index;
	}

	void	remove(const D_T& key)
	{

		int findIndex = findLinearSearch(key);
		if (findIndex<size())
		{
			swap( findIndex,size()-1);
			pop_back();
		}
	}

	//PCK: whole function
	void initializeFromBuffer(void *buffer, int size, int capacity)
	{
		clear();
		m_ownsMemory = false;
		m_data = (D_T*)buffer;
		m_size = size;
		m_capacity = capacity;
	}

};

#endif //D_BT_OBJECT_ARRAY__
