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

#include "btUnionFind.h"



D_btUnionFind::~D_btUnionFind()
{
	Free();

}

D_btUnionFind::D_btUnionFind()
{ 

}

void	D_btUnionFind::allocate(int N)
{
	m_elements.resize(N);
}
void	D_btUnionFind::Free()
{
	m_elements.clear();
}


void	D_btUnionFind::reset(int N)
{
	allocate(N);

	for (int i = 0; i < N; i++) 
	{ 
		m_elements[i].m_id = i; m_elements[i].m_sz = 1; 
	} 
}


class D_btUnionFindElementSortPredicate
{
	public:

		bool operator() ( const D_btElement& lhs, const D_btElement& rhs )
		{
			return lhs.m_id < rhs.m_id;
		}
};

///this D_is a special operation, destroying the content of D_btUnionFind.
///it sorts the elements, based on island id, in order D_to make it easy D_to iterate over islands
void	D_btUnionFind::sortIslands()
{

	//first store the original body index, D_and islandId
	int numElements = m_elements.size();
	
	for (int i=0;i<numElements;i++)
	{
		m_elements[i].m_id = find(i);
		m_elements[i].m_sz = i;
	}
	
	 // Sort the vector using predicate D_and std::sort
	  //std::sort(m_elements.begin(), m_elements.end(), D_btUnionFindElementSortPredicate);
	  m_elements.quickSort(D_btUnionFindElementSortPredicate());

}

