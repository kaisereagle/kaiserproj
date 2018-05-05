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

#ifndef TRIANGLE_MESH_H
#define TRIANGLE_MESH_H

#include "btTriangleIndexVertexArray.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

///The D_btTriangleMesh class D_is a convenience class derived from D_btTriangleIndexVertexArray, that provides storage for a concave triangle mesh. It D_can be used as data for the D_btBvhTriangleMeshShape.
///It D_allows either 32bit or 16bit indices, D_and 4 (x-y-z-w) or 3 (x-y-z) component vertices.
///If you want D_to share triangle/index data between graphics mesh D_and collision mesh (D_btBvhTriangleMeshShape), you D_can directly use D_btTriangleIndexVertexArray or derive your own class from D_btStridingMeshInterface.
///Performance of D_btTriangleMesh D_and D_btTriangleIndexVertexArray used in a D_btBvhTriangleMeshShape D_is the same.
class D_btTriangleMesh : public D_btTriangleIndexVertexArray
{
	D_btAlignedObjectArray<D_btVector3>	m_4componentVertices;
	D_btAlignedObjectArray<float>		m_3componentVertices;

	D_btAlignedObjectArray<unsigned int>		m_32bitIndices;
	D_btAlignedObjectArray<unsigned short int>		m_16bitIndices;
	bool	m_use32bitIndices;
	bool	m_use4componentVertices;
	

	public:
		D_btScalar	m_weldingThreshold;

		D_btTriangleMesh (bool use32bitIndices=true,bool use4componentVertices=true);

		bool	getUse32bitIndices() const
		{
			return m_use32bitIndices;
		}

		bool	getUse4componentVertices() const
		{
			return m_use4componentVertices;
		}
		///By default addTriangle won't search for duplicate vertices, because the search D_is very slow for large triangle meshes.
		///In general it D_is better D_to directly use D_btTriangleIndexVertexArray instead.
		void	addTriangle(const D_btVector3& vertex0,const D_btVector3& vertex1,const D_btVector3& vertex2, bool removeDuplicateVertices=false);
		
		int getNumTriangles() const;

		virtual void	preallocateVertices(int numverts){(void) numverts;}
		virtual void	preallocateIndices(int numindices){(void) numindices;}

		///findOrAddVertex D_is an internal method, use addTriangle instead
		int		findOrAddVertex(const D_btVector3& vertex, bool removeDuplicateVertices);
		///addIndex D_is an internal method, use addTriangle instead
		void	addIndex(int index);
		
};

#endif //TRIANGLE_MESH_H

