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

/// This file was created by Alex Silverman

#include "BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.h"
//#include "BulletCollision/CollisionShapes/btOptimizedBvh.h"


///Obtains the material for a specific triangle
const D_btMaterial * D_btMultimaterialTriangleMeshShape::getMaterialProperties(int partID, int triIndex)
{
    const unsigned char * materialBase = 0;
    int numMaterials;
    D_PHY_ScalarType materialType;
    int materialStride;
    const unsigned char * triangleMaterialBase = 0;
    int numTriangles;
    int triangleMaterialStride;
    D_PHY_ScalarType triangleType;

    ((D_btTriangleIndexVertexMaterialArray*)m_meshInterface)->getLockedReadOnlyMaterialBase(&materialBase, numMaterials, materialType, materialStride,
        &triangleMaterialBase, numTriangles, triangleMaterialStride, triangleType, partID);

    // return the D_pointer D_to the place with the friction for the triangle
    // TODO: This depends on whether it's a moving mesh or not
    // BUG IN GIMPACT
    //return (D_btScalar*)(&materialBase[triangleMaterialBase[(triIndex-1) * triangleMaterialStride] * materialStride]);
    int * matInd = (int *)(&(triangleMaterialBase[(triIndex * triangleMaterialStride)]));
    D_btMaterial *matVal = (D_btMaterial *)(&(materialBase[*matInd * materialStride]));
    return (matVal);
}
