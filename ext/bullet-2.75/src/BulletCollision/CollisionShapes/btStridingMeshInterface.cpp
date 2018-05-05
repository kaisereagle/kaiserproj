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

#include "btStridingMeshInterface.h"

D_btStridingMeshInterface::~D_btStridingMeshInterface()
{

}


void	D_btStridingMeshInterface::InternalProcessAllTriangles(D_btInternalTriangleIndexCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{
	(void)aabbMin;
	(void)aabbMax;
	int numtotalphysicsverts = 0;
	int part,graphicssubparts = getNumSubParts();
	const unsigned char * vertexbase;
	const unsigned char * indexbase;
	int indexstride;
	D_PHY_ScalarType type;
	D_PHY_ScalarType gfxindextype;
	int stride,numverts,numtriangles;
	int gfxindex;
	D_btVector3 triangle[3];

	D_btVector3 meshScaling = getScaling();

	///if the number of parts D_is D_big, the performance might drop due D_to the innerloop switch on indextype
	for (part=0;part<graphicssubparts ;part++)
	{
		getLockedReadOnlyVertexIndexBase(&vertexbase,numverts,type,stride,&indexbase,indexstride,numtriangles,gfxindextype,part);
		numtotalphysicsverts+=numtriangles*3; //upper bound

		///unlike that developers want D_to pass in double-precision meshes in single-precision Bullet build
		///so disable this feature by default
		///see patch http://code.google.com/p/bullet/issues/detail?id=213

		switch (type)
		{
		case D_PHY_FLOAT:
		 {

			 float* graphicsbase;

			 switch (gfxindextype)
			 {
			 case D_PHY_INTEGER:
				 {
					 for (gfxindex=0;gfxindex<numtriangles;gfxindex++)
					 {
						 unsigned int* tri_indices= (unsigned int*)(indexbase+gfxindex*indexstride);
						 graphicsbase = (float*)(vertexbase+tri_indices[0]*stride);
						 triangle[0].setValue(graphicsbase[0]*meshScaling.getX(),graphicsbase[1]*meshScaling.getY(),graphicsbase[2]*meshScaling.getZ());
						 graphicsbase = (float*)(vertexbase+tri_indices[1]*stride);
						 triangle[1].setValue(graphicsbase[0]*meshScaling.getX(),graphicsbase[1]*meshScaling.getY(),	graphicsbase[2]*meshScaling.getZ());
						 graphicsbase = (float*)(vertexbase+tri_indices[2]*stride);
						 triangle[2].setValue(graphicsbase[0]*meshScaling.getX(),graphicsbase[1]*meshScaling.getY(),	graphicsbase[2]*meshScaling.getZ());
						 callback->internalProcessTriangleIndex(triangle,part,gfxindex);
					 }
					 break;
				 }
			 case D_PHY_SHORT:
				 {
					 for (gfxindex=0;gfxindex<numtriangles;gfxindex++)
					 {
						 unsigned short int* tri_indices= (unsigned short int*)(indexbase+gfxindex*indexstride);
						 graphicsbase = (float*)(vertexbase+tri_indices[0]*stride);
						 triangle[0].setValue(graphicsbase[0]*meshScaling.getX(),graphicsbase[1]*meshScaling.getY(),graphicsbase[2]*meshScaling.getZ());
						 graphicsbase = (float*)(vertexbase+tri_indices[1]*stride);
						 triangle[1].setValue(graphicsbase[0]*meshScaling.getX(),graphicsbase[1]*meshScaling.getY(),	graphicsbase[2]*meshScaling.getZ());
						 graphicsbase = (float*)(vertexbase+tri_indices[2]*stride);
						 triangle[2].setValue(graphicsbase[0]*meshScaling.getX(),graphicsbase[1]*meshScaling.getY(),	graphicsbase[2]*meshScaling.getZ());
						 callback->internalProcessTriangleIndex(triangle,part,gfxindex);
					 }
					 break;
				 }
			 default:
				 D_btAssert((gfxindextype == D_PHY_INTEGER) || (gfxindextype == D_PHY_SHORT));
			 }
			 break;
		 }

		case D_PHY_DOUBLE:
			{
				double* graphicsbase;

				switch (gfxindextype)
				{
				case D_PHY_INTEGER:
					{
						for (gfxindex=0;gfxindex<numtriangles;gfxindex++)
						{
							unsigned int* tri_indices= (unsigned int*)(indexbase+gfxindex*indexstride);
							graphicsbase = (double*)(vertexbase+tri_indices[0]*stride);
							triangle[0].setValue((D_btScalar)graphicsbase[0]*meshScaling.getX(),(D_btScalar)graphicsbase[1]*meshScaling.getY(),(D_btScalar)graphicsbase[2]*meshScaling.getZ());
							graphicsbase = (double*)(vertexbase+tri_indices[1]*stride);
							triangle[1].setValue((D_btScalar)graphicsbase[0]*meshScaling.getX(),(D_btScalar)graphicsbase[1]*meshScaling.getY(),  (D_btScalar)graphicsbase[2]*meshScaling.getZ());
							graphicsbase = (double*)(vertexbase+tri_indices[2]*stride);
							triangle[2].setValue((D_btScalar)graphicsbase[0]*meshScaling.getX(),(D_btScalar)graphicsbase[1]*meshScaling.getY(),  (D_btScalar)graphicsbase[2]*meshScaling.getZ());
							callback->internalProcessTriangleIndex(triangle,part,gfxindex);
						}
						break;
					}
				case D_PHY_SHORT:
					{
						for (gfxindex=0;gfxindex<numtriangles;gfxindex++)
						{
							unsigned short int* tri_indices= (unsigned short int*)(indexbase+gfxindex*indexstride);
							graphicsbase = (double*)(vertexbase+tri_indices[0]*stride);
							triangle[0].setValue((D_btScalar)graphicsbase[0]*meshScaling.getX(),(D_btScalar)graphicsbase[1]*meshScaling.getY(),(D_btScalar)graphicsbase[2]*meshScaling.getZ());
							graphicsbase = (double*)(vertexbase+tri_indices[1]*stride);
							triangle[1].setValue((D_btScalar)graphicsbase[0]*meshScaling.getX(),(D_btScalar)graphicsbase[1]*meshScaling.getY(),  (D_btScalar)graphicsbase[2]*meshScaling.getZ());
							graphicsbase = (double*)(vertexbase+tri_indices[2]*stride);
							triangle[2].setValue((D_btScalar)graphicsbase[0]*meshScaling.getX(),(D_btScalar)graphicsbase[1]*meshScaling.getY(),  (D_btScalar)graphicsbase[2]*meshScaling.getZ());
							callback->internalProcessTriangleIndex(triangle,part,gfxindex);
						}
						break;
					}
				default:
					D_btAssert((gfxindextype == D_PHY_INTEGER) || (gfxindextype == D_PHY_SHORT));
				}
				break;
			}
		default:
			D_btAssert((type == D_PHY_FLOAT) || (type == D_PHY_DOUBLE));
		}

		unLockReadOnlyVertexBase(part);
	}
}

void	D_btStridingMeshInterface::calculateAabbBruteForce(D_btVector3& aabbMin,D_btVector3& aabbMax)
{

	struct	AabbCalculationCallback : public D_btInternalTriangleIndexCallback
	{
		D_btVector3	m_aabbMin;
		D_btVector3	m_aabbMax;

		AabbCalculationCallback()
		{
			m_aabbMin.setValue(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
			m_aabbMax.setValue(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT));
		}

		virtual void internalProcessTriangleIndex(D_btVector3* triangle,int partId,int  triangleIndex)
		{
			(void)partId;
			(void)triangleIndex;

			m_aabbMin.setMin(triangle[0]);
			m_aabbMax.setMax(triangle[0]);
			m_aabbMin.setMin(triangle[1]);
			m_aabbMax.setMax(triangle[1]);
			m_aabbMin.setMin(triangle[2]);
			m_aabbMax.setMax(triangle[2]);
		}
	};

	//first calculate the total aabb for all triangles
	AabbCalculationCallback	aabbCallback;
	aabbMin.setValue(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT));
	aabbMax.setValue(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
	InternalProcessAllTriangles(&aabbCallback,aabbMin,aabbMax);

	aabbMin = aabbCallback.m_aabbMin;
	aabbMax = aabbCallback.m_aabbMax;
}
