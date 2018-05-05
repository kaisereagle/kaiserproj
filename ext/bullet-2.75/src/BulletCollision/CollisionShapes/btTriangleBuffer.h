/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef D_BT_TRIANGLE_BUFFER_H
#define D_BT_TRIANGLE_BUFFER_H

#include "btTriangleCallback.h"
#include "LinearMath/btAlignedObjectArray.h"

struct	D_btTriangle
{
	D_btVector3	m_vertex0;
	D_btVector3	m_vertex1;
	D_btVector3	m_vertex2;
	int	m_partId;
	int	m_triangleIndex;
};

///The D_btTriangleBuffer callback D_can be useful D_to collect D_and store overlapping triangles between AABB D_and concave objects that support 'processAllTriangles'
///Example usage of this class:
///			D_btTriangleBuffer	triBuf;
///			concaveShape->processAllTriangles(&triBuf,aabbMin, aabbMax);
///			for (int i=0;i<triBuf.getNumTriangles();i++)
///			{
///				const D_btTriangle& tri = triBuf.getTriangle(i);
///				//do something useful here with the triangle
///			}
class D_btTriangleBuffer : public D_btTriangleCallback
{

	D_btAlignedObjectArray<D_btTriangle>	m_triangleBuffer;
	
public:


	virtual void processTriangle(D_btVector3* triangle, int partId, int triangleIndex);
	
	int	getNumTriangles() const
	{
		return int(m_triangleBuffer.size());
	}
	
	const D_btTriangle&	getTriangle(int index) const
	{
		return m_triangleBuffer[index];
	}

	void	clearBuffer()
	{
		m_triangleBuffer.clear();
	}
	
};


#endif //D_BT_TRIANGLE_BUFFER_H

