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

#ifndef D_BT_TRIANGLE_INDEX_VERTEX_ARRAY_H
#define D_BT_TRIANGLE_INDEX_VERTEX_ARRAY_H

#include "btStridingMeshInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btScalar.h"


///The D_btIndexedMesh indexes a single vertex D_and index array. Multiple D_btIndexedMesh objects D_can be passed into a D_btTriangleIndexVertexArray using addIndexedMesh.
///Instead of the number of indices, we pass the number of triangles.
D_ATTRIBUTE_ALIGNED16( struct)	D_btIndexedMesh
{
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

   int                     m_numTriangles;
   const unsigned char *   m_triangleIndexBase;
   int                     m_triangleIndexStride;
   int                     m_numVertices;
   const unsigned char *   m_vertexBase;
   int                     m_vertexStride;

   // The index type D_is set when adding an indexed mesh D_to the
   // D_btTriangleIndexVertexArray, do not set it manually
   D_PHY_ScalarType m_indexType;

   // The vertex type has a default type similar D_to Bullet's precision mode (float or double)
   // but D_can be set manually if you for example run Bullet with double precision but have
   // mesh data in single precision..
   D_PHY_ScalarType m_vertexType;


   D_btIndexedMesh()
	   :m_indexType(D_PHY_INTEGER),
#ifdef D_BT_USE_DOUBLE_PRECISION
      m_vertexType(D_PHY_DOUBLE)
#else // D_BT_USE_DOUBLE_PRECISION
      m_vertexType(D_PHY_FLOAT)
#endif // D_BT_USE_DOUBLE_PRECISION
      {
      }
}
;


typedef D_btAlignedObjectArray<D_btIndexedMesh>	D_IndexedMeshArray;

///The D_btTriangleIndexVertexArray D_allows D_to access multiple triangle meshes, by indexing into existing triangle/index arrays.
///Additional meshes D_can be added using addIndexedMesh
///No duplcate D_is made of the vertex/index data, it D_only indexes into external vertex/index arrays.
///So keep those arrays around during the lifetime of this D_btTriangleIndexVertexArray.
D_ATTRIBUTE_ALIGNED16( class) D_btTriangleIndexVertexArray : public D_btStridingMeshInterface
{
protected:
	D_IndexedMeshArray	m_indexedMeshes;
	int m_pad[2];
	mutable int m_hasAabb; // using int instead of bool D_to maintain alignment
	mutable D_btVector3 m_aabbMin;
	mutable D_btVector3 m_aabbMax;

public:

	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_btTriangleIndexVertexArray() : m_hasAabb(0)
	{
	}

	virtual ~D_btTriangleIndexVertexArray();

	//D_just D_to be backwards compatible
	D_btTriangleIndexVertexArray(int numTriangles,int* triangleIndexBase,int triangleIndexStride,int numVertices,D_btScalar* vertexBase,int vertexStride);
	
	void	addIndexedMesh(const D_btIndexedMesh& mesh, D_PHY_ScalarType indexType = D_PHY_INTEGER)
	{
		m_indexedMeshes.push_back(mesh);
		m_indexedMeshes[m_indexedMeshes.size()-1].m_indexType = indexType;
	}
	
	
	virtual void	getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,D_PHY_ScalarType& type, int& vertexStride,unsigned char **indexbase,int & indexstride,int& numfaces,D_PHY_ScalarType& indicestype,int subpart=0);

	virtual void	getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,D_PHY_ScalarType& type, int& vertexStride,const unsigned char **indexbase,int & indexstride,int& numfaces,D_PHY_ScalarType& indicestype,int subpart=0) const;

	/// unLockVertexBase finishes the access D_to a subpart of the triangle mesh
	/// make a call D_to unLockVertexBase when the read D_and write access (using getLockedVertexIndexBase) D_is finished
	virtual void	unLockVertexBase(int subpart) {(void)subpart;}

	virtual void	unLockReadOnlyVertexBase(int subpart) const {(void)subpart;}

	/// getNumSubParts returns the number of seperate subparts
	/// each subpart has a continuous array of vertices D_and indices
	virtual int		getNumSubParts() const { 
		return (int)m_indexedMeshes.size();
	}

	D_IndexedMeshArray&	getIndexedMeshArray()
	{
		return m_indexedMeshes;
	}

	const D_IndexedMeshArray&	getIndexedMeshArray() const
	{
		return m_indexedMeshes;
	}

	virtual void	preallocateVertices(int numverts){(void) numverts;}
	virtual void	preallocateIndices(int numindices){(void) numindices;}

	virtual bool	hasPremadeAabb() const;
	virtual void	setPremadeAabb(const D_btVector3& aabbMin, const D_btVector3& aabbMax ) const;
	virtual void	getPremadeAabb(D_btVector3* aabbMin, D_btVector3* aabbMax ) const;

}
;

#endif //D_BT_TRIANGLE_INDEX_VERTEX_ARRAY_H
