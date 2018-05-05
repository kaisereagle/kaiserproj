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

#include "btTriangleIndexVertexMaterialArray.h"

D_btTriangleIndexVertexMaterialArray::D_btTriangleIndexVertexMaterialArray(int numTriangles,int* triangleIndexBase,int triangleIndexStride,
                                   int numVertices,D_btScalar* vertexBase,int vertexStride,
                                   int numMaterials, unsigned char* materialBase, int materialStride,
                                   int* triangleMaterialsBase, int materialIndexStride) :
D_btTriangleIndexVertexArray(numTriangles, triangleIndexBase, triangleIndexStride, numVertices, vertexBase, vertexStride)
{
    D_btMaterialProperties mat;

    mat.m_numMaterials = numMaterials;
    mat.m_materialBase = materialBase;
    mat.m_materialStride = materialStride;
#ifdef D_BT_USE_DOUBLE_PRECISION
    mat.m_materialType = D_PHY_DOUBLE;
#else
    mat.m_materialType = D_PHY_FLOAT;
#endif

    mat.m_numTriangles = numTriangles;
    mat.m_triangleMaterialsBase = (unsigned char *)triangleMaterialsBase;
    mat.m_triangleMaterialStride = materialIndexStride;
    mat.m_triangleType = D_PHY_INTEGER;

    addMaterialProperties(mat);
}


void D_btTriangleIndexVertexMaterialArray::getLockedMaterialBase(unsigned char **materialBase, int& numMaterials, D_PHY_ScalarType& materialType, int& materialStride,
                                   unsigned char ** triangleMaterialBase, int& numTriangles, int& triangleMaterialStride, D_PHY_ScalarType& triangleType, int subpart)
{
    D_btAssert(subpart< getNumSubParts() );

    D_btMaterialProperties& mats = m_materials[subpart];

    numMaterials = mats.m_numMaterials;
    (*materialBase) = (unsigned char *) mats.m_materialBase;
#ifdef D_BT_USE_DOUBLE_PRECISION
    materialType = D_PHY_DOUBLE;
#else
    materialType = D_PHY_FLOAT;
#endif
    materialStride = mats.m_materialStride;

    numTriangles = mats.m_numTriangles;
    (*triangleMaterialBase) = (unsigned char *)mats.m_triangleMaterialsBase;
    triangleMaterialStride = mats.m_triangleMaterialStride;
    triangleType = mats.m_triangleType;
}

void D_btTriangleIndexVertexMaterialArray::getLockedReadOnlyMaterialBase(const unsigned char **materialBase, int& numMaterials, D_PHY_ScalarType& materialType, int& materialStride,
                                           const unsigned char ** triangleMaterialBase, int& numTriangles, int& triangleMaterialStride, D_PHY_ScalarType& triangleType, int subpart)
{
    D_btMaterialProperties& mats = m_materials[subpart];

    numMaterials = mats.m_numMaterials;
    (*materialBase) = (const unsigned char *) mats.m_materialBase;
#ifdef D_BT_USE_DOUBLE_PRECISION
    materialType = D_PHY_DOUBLE;
#else
    materialType = D_PHY_FLOAT;
#endif
    materialStride = mats.m_materialStride;

    numTriangles = mats.m_numTriangles;
    (*triangleMaterialBase) = (const unsigned char *)mats.m_triangleMaterialsBase;
    triangleMaterialStride = mats.m_triangleMaterialStride;
    triangleType = mats.m_triangleType;
}
