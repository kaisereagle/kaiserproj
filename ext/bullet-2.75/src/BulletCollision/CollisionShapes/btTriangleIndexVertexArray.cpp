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

#include "btTriangleIndexVertexArray.h"

D_btTriangleIndexVertexArray::D_btTriangleIndexVertexArray(int numTriangles,int* triangleIndexBase,int triangleIndexStride,int numVertices,D_btScalar* vertexBase,int vertexStride)
: m_hasAabb(0)
{
	D_btIndexedMesh mesh;

	mesh.m_numTriangles = numTriangles;
	mesh.m_triangleIndexBase = (const unsigned char *)triangleIndexBase;
	mesh.m_triangleIndexStride = triangleIndexStride;
	mesh.m_numVertices = numVertices;
	mesh.m_vertexBase = (const unsigned char *)vertexBase;
	mesh.m_vertexStride = vertexStride;

	addIndexedMesh(mesh);

}

D_btTriangleIndexVertexArray::~D_btTriangleIndexVertexArray()
{

}

void	D_btTriangleIndexVertexArray::getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,D_PHY_ScalarType& type, int& vertexStride,unsigned char **indexbase,int & indexstride,int& numfaces,D_PHY_ScalarType& indicestype,int subpart)
{
	D_btAssert(subpart< getNumSubParts() );

	D_btIndexedMesh& mesh = m_indexedMeshes[subpart];

	numverts = mesh.m_numVertices;
	(*vertexbase) = (unsigned char *) mesh.m_vertexBase;

   type = mesh.m_vertexType;

	vertexStride = mesh.m_vertexStride;

	numfaces = mesh.m_numTriangles;

	(*indexbase) = (unsigned char *)mesh.m_triangleIndexBase;
	indexstride = mesh.m_triangleIndexStride;
	indicestype = mesh.m_indexType;
}

void	D_btTriangleIndexVertexArray::getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,D_PHY_ScalarType& type, int& vertexStride,const unsigned char **indexbase,int & indexstride,int& numfaces,D_PHY_ScalarType& indicestype,int subpart) const
{
	const D_btIndexedMesh& mesh = m_indexedMeshes[subpart];

	numverts = mesh.m_numVertices;
	(*vertexbase) = (const unsigned char *)mesh.m_vertexBase;

   type = mesh.m_vertexType;
   
	vertexStride = mesh.m_vertexStride;

	numfaces = mesh.m_numTriangles;
	(*indexbase) = (const unsigned char *)mesh.m_triangleIndexBase;
	indexstride = mesh.m_triangleIndexStride;
	indicestype = mesh.m_indexType;
}

bool	D_btTriangleIndexVertexArray::hasPremadeAabb() const
{
	return (m_hasAabb == 1);
}


void	D_btTriangleIndexVertexArray::setPremadeAabb(const D_btVector3& aabbMin, const D_btVector3& aabbMax ) const
{
	m_aabbMin = aabbMin;
	m_aabbMax = aabbMax;
	m_hasAabb = 1; // this D_is intentionally an int see notes in header
}

void	D_btTriangleIndexVertexArray::getPremadeAabb(D_btVector3* aabbMin, D_btVector3* aabbMax ) const
{
	*aabbMin = m_aabbMin;
	*aabbMax = m_aabbMax;
}


