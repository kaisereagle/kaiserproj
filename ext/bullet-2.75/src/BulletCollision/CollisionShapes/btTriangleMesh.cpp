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


#include "btTriangleMesh.h"



D_btTriangleMesh::D_btTriangleMesh (bool use32bitIndices,bool use4componentVertices)
:m_use32bitIndices(use32bitIndices),
m_use4componentVertices(use4componentVertices),
m_weldingThreshold(0.0)
{
	D_btIndexedMesh meshIndex;
	meshIndex.m_numTriangles = 0;
	meshIndex.m_numVertices = 0;
	meshIndex.m_indexType = D_PHY_INTEGER;
	meshIndex.m_triangleIndexBase = 0;
	meshIndex.m_triangleIndexStride = 3*sizeof(int);
	meshIndex.m_vertexBase = 0;
	meshIndex.m_vertexStride = sizeof(D_btVector3);
	m_indexedMeshes.push_back(meshIndex);

	if (m_use32bitIndices)
	{
		m_indexedMeshes[0].m_numTriangles = m_32bitIndices.size()/3;
		m_indexedMeshes[0].m_triangleIndexBase = 0;
		m_indexedMeshes[0].m_indexType = D_PHY_INTEGER;
		m_indexedMeshes[0].m_triangleIndexStride = 3*sizeof(int);
	} else
	{
		m_indexedMeshes[0].m_numTriangles = m_16bitIndices.size()/3;
		m_indexedMeshes[0].m_triangleIndexBase = 0;
		m_indexedMeshes[0].m_indexType = D_PHY_SHORT;
		m_indexedMeshes[0].m_triangleIndexStride = 3*sizeof(short int);
	}

	if (m_use4componentVertices)
	{
		m_indexedMeshes[0].m_numVertices = m_4componentVertices.size();
		m_indexedMeshes[0].m_vertexBase = 0;
		m_indexedMeshes[0].m_vertexStride = sizeof(D_btVector3);
	} else
	{
		m_indexedMeshes[0].m_numVertices = m_3componentVertices.size()/3;
		m_indexedMeshes[0].m_vertexBase = 0;
		m_indexedMeshes[0].m_vertexStride = 3*sizeof(D_btScalar);
	}


}

void	D_btTriangleMesh::addIndex(int index)
{
	if (m_use32bitIndices)
	{
		m_32bitIndices.push_back(index);
		m_indexedMeshes[0].m_triangleIndexBase = (unsigned char*) &m_32bitIndices[0];
	} else
	{
		m_16bitIndices.push_back(index);
		m_indexedMeshes[0].m_triangleIndexBase = (unsigned char*) &m_16bitIndices[0];
	}
}


int	D_btTriangleMesh::findOrAddVertex(const D_btVector3& vertex, bool removeDuplicateVertices)
{
	//return index of new/existing vertex
	///@todo: could use acceleration structure for this
	if (m_use4componentVertices)
	{
		if (removeDuplicateVertices)
			{
			for (int i=0;i< m_4componentVertices.size();i++)
			{
				if ((m_4componentVertices[i]-vertex).length2() <= m_weldingThreshold)
				{
					return i;
				}
			}
		}
		m_indexedMeshes[0].m_numVertices++;
		m_4componentVertices.push_back(vertex);
		m_indexedMeshes[0].m_vertexBase = (unsigned char*)&m_4componentVertices[0];

		return m_4componentVertices.size()-1;
		
	} else
	{
		
		if (removeDuplicateVertices)
		{
			for (int i=0;i< m_3componentVertices.size();i+=3)
			{
				D_btVector3 vtx(m_3componentVertices[i],m_3componentVertices[i+1],m_3componentVertices[i+2]);
				if ((vtx-vertex).length2() <= m_weldingThreshold)
				{
					return i/3;
				}
			}
	}
		m_3componentVertices.push_back((float)vertex.getX());
		m_3componentVertices.push_back((float)vertex.getY());
		m_3componentVertices.push_back((float)vertex.getZ());
		m_indexedMeshes[0].m_numVertices++;
		m_indexedMeshes[0].m_vertexBase = (unsigned char*)&m_3componentVertices[0];
		return (m_3componentVertices.size()/3)-1;
	}

}
		
void	D_btTriangleMesh::addTriangle(const D_btVector3& vertex0,const D_btVector3& vertex1,const D_btVector3& vertex2,bool removeDuplicateVertices)
{
	m_indexedMeshes[0].m_numTriangles++;
	addIndex(findOrAddVertex(vertex0,removeDuplicateVertices));
	addIndex(findOrAddVertex(vertex1,removeDuplicateVertices));
	addIndex(findOrAddVertex(vertex2,removeDuplicateVertices));
}

int D_btTriangleMesh::getNumTriangles() const
{
	if (m_use32bitIndices)
	{
		return m_32bitIndices.size() / 3;
	}
	return m_16bitIndices.size() / 3;
}
