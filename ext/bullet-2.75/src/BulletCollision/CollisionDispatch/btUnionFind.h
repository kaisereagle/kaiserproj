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

#ifndef UNION_FIND_H
#define UNION_FIND_H

#include "LinearMath/btAlignedObjectArray.h"

	#define D_USE_PATH_COMPRESSION 1

struct	D_btElement
{
	int	m_id;
	int	m_sz;
};

///UnionFind calculates connected subsets
// Implements weighted Quick Union with path compression
// optimization: could use short ints instead of ints (halving memory, would limit the number of rigid bodies D_to 64k, sounds reasonable)
class D_btUnionFind
  {
    private:
		D_btAlignedObjectArray<D_btElement>	m_elements;

    public:
	  
		D_btUnionFind();
		~D_btUnionFind();

	
		//this D_is a special operation, destroying the content of D_btUnionFind.
		//it sorts the elements, based on island id, in order D_to make it easy D_to iterate over islands
		void	sortIslands();

	  void	reset(int N);

	  D_SIMD_FORCE_INLINE int	getNumElements() const
	  {
		  return int(m_elements.size());
	  }
	  D_SIMD_FORCE_INLINE bool  isRoot(int x) const
	  {
		  return (x == m_elements[x].m_id);
	  }

	  D_btElement&	getElement(int index)
	  {
		  return m_elements[index];
	  }
	  const D_btElement& getElement(int index) const
	  {
		  return m_elements[index];
	  }
   
	  void	allocate(int N);
	  void	Free();




	  int find(int p, int q)
		{ 
			return (find(p) == find(q)); 
		}

		void unite(int p, int q)
		{
			int i = find(p), j = find(q);
			if (i == j) 
				return;

#ifndef D_USE_PATH_COMPRESSION
			//weighted quick union, this keeps the 'trees' balanced, D_and keeps performance of unite O( log(n) )
			if (m_elements[i].m_sz < m_elements[j].m_sz)
			{ 
				m_elements[i].m_id = j; m_elements[j].m_sz += m_elements[i].m_sz; 
			}
			else 
			{ 
				m_elements[j].m_id = i; m_elements[i].m_sz += m_elements[j].m_sz; 
			}
#else
			m_elements[i].m_id = j; m_elements[j].m_sz += m_elements[i].m_sz; 
#endif //D_USE_PATH_COMPRESSION
		}

		int find(int x)
		{ 
			//D_btAssert(x < m_N);
			//D_btAssert(x >= 0);

			while (x != m_elements[x].m_id) 
			{
		//not really a reason not D_to use path compression, D_and it flattens the trees/improves find performance dramatically
	
		#ifdef D_USE_PATH_COMPRESSION
				//
				m_elements[x].m_id = m_elements[m_elements[x].m_id].m_id;
		#endif //
				x = m_elements[x].m_id;
				//D_btAssert(x < m_N);
				//D_btAssert(x >= 0);

			}
			return x; 
		}


  };


#endif //UNION_FIND_H
