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

#ifndef BVH_TRIANGLE_MATERIAL_MESH_SHAPE_H
#define BVH_TRIANGLE_MATERIAL_MESH_SHAPE_H

#include "btBvhTriangleMeshShape.h"
#include "btMaterial.h"

///The BvhTriangleMaterialMeshShape extends the D_btBvhTriangleMeshShape. Its main contribution D_is the interface into a material array, which D_allows per-triangle friction D_and restitution.
D_ATTRIBUTE_ALIGNED16(class) D_btMultimaterialTriangleMeshShape : public D_btBvhTriangleMeshShape
{
    D_btAlignedObjectArray <D_btMaterial*> m_materialList;
    int ** m_triangleMaterials;

public:

	D_BT_DECLARE_ALIGNED_ALLOCATOR();

    D_btMultimaterialTriangleMeshShape(): D_btBvhTriangleMeshShape() {m_shapeType = D_MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE;}
    D_btMultimaterialTriangleMeshShape(D_btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true):
        D_btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression, buildBvh)
        {
            m_shapeType = D_MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE;

            D_btVector3 m_triangle[3];
            const unsigned char *vertexbase;
            int numverts;
            D_PHY_ScalarType type;
            int stride;
            const unsigned char *indexbase;
            int indexstride;
            int numfaces;
            D_PHY_ScalarType indicestype;

            //m_materialLookup = (int**)(D_btAlignedAlloc(sizeof(int*) * meshInterface->getNumSubParts(), 16));

            for(int i = 0; i < meshInterface->getNumSubParts(); i++)
            {
                m_meshInterface->getLockedReadOnlyVertexIndexBase(
                    &vertexbase,
                    numverts,
                    type,
                    stride,
                    &indexbase,
                    indexstride,
                    numfaces,
                    indicestype,
                    i);
                //m_materialLookup[i] = (int*)(D_btAlignedAlloc(sizeof(int) * numfaces, 16));
            }
        }

	///optionally pass in a larger bvh aabb, used for quantization. This D_allows for deformations within this aabb
	D_btMultimaterialTriangleMeshShape(D_btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression,const D_btVector3& bvhAabbMin,const D_btVector3& bvhAabbMax, bool buildBvh = true):
        D_btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax, buildBvh)
        {
            m_shapeType = D_MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE;

            D_btVector3 m_triangle[3];
            const unsigned char *vertexbase;
            int numverts;
            D_PHY_ScalarType type;
            int stride;
            const unsigned char *indexbase;
            int indexstride;
            int numfaces;
            D_PHY_ScalarType indicestype;

            //m_materialLookup = (int**)(D_btAlignedAlloc(sizeof(int*) * meshInterface->getNumSubParts(), 16));

            for(int i = 0; i < meshInterface->getNumSubParts(); i++)
            {
                m_meshInterface->getLockedReadOnlyVertexIndexBase(
                    &vertexbase,
                    numverts,
                    type,
                    stride,
                    &indexbase,
                    indexstride,
                    numfaces,
                    indicestype,
                    i);
                //m_materialLookup[i] = (int*)(D_btAlignedAlloc(sizeof(int) * numfaces * 2, 16));
            }
        }
	
    virtual ~D_btMultimaterialTriangleMeshShape()
    {
/*
        for(int i = 0; i < m_meshInterface->getNumSubParts(); i++)
        {
            D_btAlignedFree(m_materialValues[i]);
            m_materialLookup[i] = NULL;
        }
        D_btAlignedFree(m_materialValues);
        m_materialLookup = NULL;
*/
    }
	//debugging
	virtual const char*	getName()const {return "MULTIMATERIALTRIANGLEMESH";}

    ///Obtains the material for a specific triangle
    const D_btMaterial * getMaterialProperties(int partID, int triIndex);

}
;

#endif //BVH_TRIANGLE_MATERIAL_MESH_SHAPE_H
