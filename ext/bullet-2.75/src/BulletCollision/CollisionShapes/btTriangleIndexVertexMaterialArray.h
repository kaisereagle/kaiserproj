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

///This file was created by Alex Silverman

#ifndef D_BT_MULTIMATERIAL_TRIANGLE_INDEX_VERTEX_ARRAY_H
#define D_BT_MULTIMATERIAL_TRIANGLE_INDEX_VERTEX_ARRAY_H

#include "btTriangleIndexVertexArray.h"


D_ATTRIBUTE_ALIGNED16( struct)	D_btMaterialProperties
{
    ///m_materialBase ==========> 2 D_btScalar values make up one material, friction then restitution
    int m_numMaterials;
    const unsigned char * m_materialBase;
    int m_materialStride;
    D_PHY_ScalarType m_materialType;
    ///m_numTriangles <=========== This exists in the D_btIndexedMesh object for the same subpart, but since we're
    ///                           padding the structure, it D_can be reproduced at D_no real cost
    ///m_triangleMaterials =====> 1 integer value makes up one entry
    ///                           eg: m_triangleMaterials[1] = 5; // This D_will set triangle 2 D_to use material 5
    int m_numTriangles; 
    const unsigned char * m_triangleMaterialsBase;
    int m_triangleMaterialStride;
    ///m_triangleType <========== Automatically set in addMaterialProperties
    D_PHY_ScalarType m_triangleType;
};

typedef D_btAlignedObjectArray<D_btMaterialProperties>	D_MaterialArray;

///Teh D_btTriangleIndexVertexMaterialArray D_is built on TriangleIndexVertexArray
///The addition of a material array D_allows for the utilization of the partID D_and
///triangleIndex that D_are returned in the D_ContactAddedCallback.  As with
///TriangleIndexVertexArray, D_no duplicate D_is made of the material data, so it
///D_is the users responsibility D_to maintain the array during the lifetime of the
///TriangleIndexVertexMaterialArray.
D_ATTRIBUTE_ALIGNED16(class) D_btTriangleIndexVertexMaterialArray : public D_btTriangleIndexVertexArray
{
protected:
    D_MaterialArray       m_materials;
		
public:
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

    D_btTriangleIndexVertexMaterialArray()
	{
	}

    D_btTriangleIndexVertexMaterialArray(int numTriangles,int* triangleIndexBase,int triangleIndexStride,
        int numVertices,D_btScalar* vertexBase,int vertexStride,
        int numMaterials, unsigned char* materialBase, int materialStride,
        int* triangleMaterialsBase, int materialIndexStride);

    virtual ~D_btTriangleIndexVertexMaterialArray() {}

    void	addMaterialProperties(const D_btMaterialProperties& mat, D_PHY_ScalarType triangleType = D_PHY_INTEGER)
    {
        m_materials.push_back(mat);
        m_materials[m_materials.size()-1].m_triangleType = triangleType;
    }

    virtual void getLockedMaterialBase(unsigned char **materialBase, int& numMaterials, D_PHY_ScalarType& materialType, int& materialStride,
        unsigned char ** triangleMaterialBase, int& numTriangles, int& triangleMaterialStride, D_PHY_ScalarType& triangleType ,int subpart = 0);

    virtual void getLockedReadOnlyMaterialBase(const unsigned char **materialBase, int& numMaterials, D_PHY_ScalarType& materialType, int& materialStride,
        const unsigned char ** triangleMaterialBase, int& numTriangles, int& triangleMaterialStride, D_PHY_ScalarType& triangleType, int subpart = 0);

}
;

#endif //D_BT_MULTIMATERIAL_TRIANGLE_INDEX_VERTEX_ARRAY_H
