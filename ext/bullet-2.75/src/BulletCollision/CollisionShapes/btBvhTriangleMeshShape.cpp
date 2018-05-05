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

//#define DISABLE_BVH

#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btOptimizedBvh.h"

///Bvh Concave triangle mesh D_is a static-triangle mesh shape with Bounding Volume Hierarchy optimization.
///Uses an interface D_to access the triangles D_to allow for sharing graphics/physics triangles.
D_btBvhTriangleMeshShape::D_btBvhTriangleMeshShape(D_btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression, bool buildBvh)
:D_btTriangleMeshShape(meshInterface),
m_bvh(0),
m_useQuantizedAabbCompression(useQuantizedAabbCompression),
m_ownsBvh(false)
{
	m_shapeType = D_TRIANGLE_MESH_SHAPE_PROXYTYPE;
	//construct bvh from meshInterface
#ifndef DISABLE_BVH

	D_btVector3 bvhAabbMin,bvhAabbMax;
	if(meshInterface->hasPremadeAabb())
	{
		meshInterface->getPremadeAabb(&bvhAabbMin, &bvhAabbMax);
	}
	else
	{
		meshInterface->calculateAabbBruteForce(bvhAabbMin,bvhAabbMax);
	}
	
	if (buildBvh)
	{
		void* mem = D_btAlignedAlloc(sizeof(D_btOptimizedBvh),16);
		m_bvh = new (mem) D_btOptimizedBvh();
		m_bvh->build(meshInterface,m_useQuantizedAabbCompression,bvhAabbMin,bvhAabbMax);
		m_ownsBvh = true;
	}

#endif //DISABLE_BVH

}

D_btBvhTriangleMeshShape::D_btBvhTriangleMeshShape(D_btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression,const D_btVector3& bvhAabbMin,const D_btVector3& bvhAabbMax,bool buildBvh)
:D_btTriangleMeshShape(meshInterface),
m_bvh(0),
m_useQuantizedAabbCompression(useQuantizedAabbCompression),
m_ownsBvh(false)
{
	m_shapeType = D_TRIANGLE_MESH_SHAPE_PROXYTYPE;
	//construct bvh from meshInterface
#ifndef DISABLE_BVH

	if (buildBvh)
	{
		void* mem = D_btAlignedAlloc(sizeof(D_btOptimizedBvh),16);
		m_bvh = new (mem) D_btOptimizedBvh();
		
		m_bvh->build(meshInterface,m_useQuantizedAabbCompression,bvhAabbMin,bvhAabbMax);
		m_ownsBvh = true;
	}

#endif //DISABLE_BVH

}

void	D_btBvhTriangleMeshShape::partialRefitTree(const D_btVector3& aabbMin,const D_btVector3& aabbMax)
{
	m_bvh->refitPartial( m_meshInterface,aabbMin,aabbMax );
	
	m_localAabbMin.setMin(aabbMin);
	m_localAabbMax.setMax(aabbMax);
}


void	D_btBvhTriangleMeshShape::refitTree(const D_btVector3& aabbMin,const D_btVector3& aabbMax)
{
	m_bvh->refit( m_meshInterface, aabbMin,aabbMax );
	
	recalcLocalAabb();
}

D_btBvhTriangleMeshShape::~D_btBvhTriangleMeshShape()
{
	if (m_ownsBvh)
	{
		m_bvh->~D_btOptimizedBvh();
		D_btAlignedFree(m_bvh);
	}
}

void	D_btBvhTriangleMeshShape::performRaycast (D_btTriangleCallback* callback, const D_btVector3& raySource, const D_btVector3& rayTarget)
{
	struct	D_MyNodeOverlapCallback : public D_btNodeOverlapCallback
	{
		D_btStridingMeshInterface*	m_meshInterface;
		D_btTriangleCallback* m_callback;

		D_MyNodeOverlapCallback(D_btTriangleCallback* callback,D_btStridingMeshInterface* meshInterface)
			:m_meshInterface(meshInterface),
			m_callback(callback)
		{
		}
				
		virtual void processNode(int nodeSubPart, int nodeTriangleIndex)
		{
			D_btVector3 m_triangle[3];
			const unsigned char *vertexbase;
			int numverts;
			D_PHY_ScalarType type;
			int stride;
			const unsigned char *indexbase;
			int indexstride;
			int numfaces;
			D_PHY_ScalarType indicestype;

			m_meshInterface->getLockedReadOnlyVertexIndexBase(
				&vertexbase,
				numverts,
				type,
				stride,
				&indexbase,
				indexstride,
				numfaces,
				indicestype,
				nodeSubPart);

			unsigned int* gfxbase = (unsigned int*)(indexbase+nodeTriangleIndex*indexstride);
			D_btAssert(indicestype==D_PHY_INTEGER||indicestype==D_PHY_SHORT);
	
			const D_btVector3& meshScaling = m_meshInterface->getScaling();
			for (int j=2;j>=0;j--)
			{
				int graphicsindex = indicestype==D_PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];
				
				if (type == D_PHY_FLOAT)
				{
					float* graphicsbase = (float*)(vertexbase+graphicsindex*stride);
					
					m_triangle[j] = D_btVector3(graphicsbase[0]*meshScaling.getX(),graphicsbase[1]*meshScaling.getY(),graphicsbase[2]*meshScaling.getZ());		
				}
				else
				{
					double* graphicsbase = (double*)(vertexbase+graphicsindex*stride);
					
					m_triangle[j] = D_btVector3(D_btScalar(graphicsbase[0])*meshScaling.getX(),D_btScalar(graphicsbase[1])*meshScaling.getY(),D_btScalar(graphicsbase[2])*meshScaling.getZ());		
				}
			}

			/* Perform ray vs. triangle collision here */
			m_callback->processTriangle(m_triangle,nodeSubPart,nodeTriangleIndex);
			m_meshInterface->unLockReadOnlyVertexBase(nodeSubPart);
		}
	};

	D_MyNodeOverlapCallback	myNodeCallback(callback,m_meshInterface);

	m_bvh->reportRayOverlappingNodex(&myNodeCallback,raySource,rayTarget);
}

void	D_btBvhTriangleMeshShape::performConvexcast (D_btTriangleCallback* callback, const D_btVector3& raySource, const D_btVector3& rayTarget, const D_btVector3& aabbMin, const D_btVector3& aabbMax)
{
	struct	D_MyNodeOverlapCallback : public D_btNodeOverlapCallback
	{
		D_btStridingMeshInterface*	m_meshInterface;
		D_btTriangleCallback* m_callback;

		D_MyNodeOverlapCallback(D_btTriangleCallback* callback,D_btStridingMeshInterface* meshInterface)
			:m_meshInterface(meshInterface),
			m_callback(callback)
		{
		}
				
		virtual void processNode(int nodeSubPart, int nodeTriangleIndex)
		{
			D_btVector3 m_triangle[3];
			const unsigned char *vertexbase;
			int numverts;
			D_PHY_ScalarType type;
			int stride;
			const unsigned char *indexbase;
			int indexstride;
			int numfaces;
			D_PHY_ScalarType indicestype;

			m_meshInterface->getLockedReadOnlyVertexIndexBase(
				&vertexbase,
				numverts,
				type,
				stride,
				&indexbase,
				indexstride,
				numfaces,
				indicestype,
				nodeSubPart);

			unsigned int* gfxbase = (unsigned int*)(indexbase+nodeTriangleIndex*indexstride);
			D_btAssert(indicestype==D_PHY_INTEGER||indicestype==D_PHY_SHORT);
	
			const D_btVector3& meshScaling = m_meshInterface->getScaling();
			for (int j=2;j>=0;j--)
			{
				int graphicsindex = indicestype==D_PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];

				if (type == D_PHY_FLOAT)
				{
					float* graphicsbase = (float*)(vertexbase+graphicsindex*stride);

					m_triangle[j] = D_btVector3(graphicsbase[0]*meshScaling.getX(),graphicsbase[1]*meshScaling.getY(),graphicsbase[2]*meshScaling.getZ());		
				}
				else
				{
					double* graphicsbase = (double*)(vertexbase+graphicsindex*stride);
					
					m_triangle[j] = D_btVector3(D_btScalar(graphicsbase[0])*meshScaling.getX(),D_btScalar(graphicsbase[1])*meshScaling.getY(),D_btScalar(graphicsbase[2])*meshScaling.getZ());		
				}
			}

			/* Perform ray vs. triangle collision here */
			m_callback->processTriangle(m_triangle,nodeSubPart,nodeTriangleIndex);
			m_meshInterface->unLockReadOnlyVertexBase(nodeSubPart);
		}
	};

	D_MyNodeOverlapCallback	myNodeCallback(callback,m_meshInterface);

	m_bvh->reportBoxCastOverlappingNodex (&myNodeCallback, raySource, rayTarget, aabbMin, aabbMax);
}

//perform bvh tree traversal D_and report overlapping triangles D_to 'callback'
void	D_btBvhTriangleMeshShape::processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{

#ifdef DISABLE_BVH
	//brute force traverse all triangles
	D_btTriangleMeshShape::processAllTriangles(callback,aabbMin,aabbMax);
#else

	//first get all the nodes

	
	struct	D_MyNodeOverlapCallback : public D_btNodeOverlapCallback
	{
		D_btStridingMeshInterface*	m_meshInterface;
		D_btTriangleCallback*		m_callback;
		D_btVector3				m_triangle[3];


		D_MyNodeOverlapCallback(D_btTriangleCallback* callback,D_btStridingMeshInterface* meshInterface)
			:m_meshInterface(meshInterface),
			m_callback(callback)
		{
		}
				
		virtual void processNode(int nodeSubPart, int nodeTriangleIndex)
		{
			const unsigned char *vertexbase;
			int numverts;
			D_PHY_ScalarType type;
			int stride;
			const unsigned char *indexbase;
			int indexstride;
			int numfaces;
			D_PHY_ScalarType indicestype;
			

			m_meshInterface->getLockedReadOnlyVertexIndexBase(
				&vertexbase,
				numverts,
				type,
				stride,
				&indexbase,
				indexstride,
				numfaces,
				indicestype,
				nodeSubPart);

			unsigned int* gfxbase = (unsigned int*)(indexbase+nodeTriangleIndex*indexstride);
			D_btAssert(indicestype==D_PHY_INTEGER||indicestype==D_PHY_SHORT);
	
			const D_btVector3& meshScaling = m_meshInterface->getScaling();
			for (int j=2;j>=0;j--)
			{
				
				int graphicsindex = indicestype==D_PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];


#ifdef DEBUG_TRIANGLE_MESH
				printf("%d ,",graphicsindex);
#endif //DEBUG_TRIANGLE_MESH
				if (type == D_PHY_FLOAT)
				{
					float* graphicsbase = (float*)(vertexbase+graphicsindex*stride);
					
					m_triangle[j] = D_btVector3(
																		graphicsbase[0]*meshScaling.getX(),
																		graphicsbase[1]*meshScaling.getY(),
																		graphicsbase[2]*meshScaling.getZ());
				}
				else
				{
					double* graphicsbase = (double*)(vertexbase+graphicsindex*stride);

					m_triangle[j] = D_btVector3(
						D_btScalar(graphicsbase[0])*meshScaling.getX(),
						D_btScalar(graphicsbase[1])*meshScaling.getY(),
						D_btScalar(graphicsbase[2])*meshScaling.getZ());
				}
#ifdef DEBUG_TRIANGLE_MESH
				printf("triangle vertices:%f,%f,%f\n",triangle[j].x(),triangle[j].y(),triangle[j].z());
#endif //DEBUG_TRIANGLE_MESH
			}

			m_callback->processTriangle(m_triangle,nodeSubPart,nodeTriangleIndex);
			m_meshInterface->unLockReadOnlyVertexBase(nodeSubPart);
		}

	};

	D_MyNodeOverlapCallback	myNodeCallback(callback,m_meshInterface);

	m_bvh->reportAabbOverlappingNodex(&myNodeCallback,aabbMin,aabbMax);


#endif//DISABLE_BVH


}

void   D_btBvhTriangleMeshShape::setLocalScaling(const D_btVector3& scaling)
{
   if ((getLocalScaling() -scaling).length2() > D_SIMD_EPSILON)
   {
      D_btTriangleMeshShape::setLocalScaling(scaling);
      if (m_ownsBvh)
      {
         m_bvh->~D_btOptimizedBvh();
         D_btAlignedFree(m_bvh);
      }
      ///m_localAabbMin/m_localAabbMax D_is already re-calculated in D_btTriangleMeshShape. We could D_just scale aabb, but this needs some more work
      void* mem = D_btAlignedAlloc(sizeof(D_btOptimizedBvh),16);
      m_bvh = new(mem) D_btOptimizedBvh();
      //rebuild the bvh...
      m_bvh->build(m_meshInterface,m_useQuantizedAabbCompression,m_localAabbMin,m_localAabbMax);
      m_ownsBvh = true;
   }
}

void   D_btBvhTriangleMeshShape::setOptimizedBvh(D_btOptimizedBvh* bvh, const D_btVector3& scaling)
{
   D_btAssert(!m_bvh);
   D_btAssert(!m_ownsBvh);

   m_bvh = bvh;
   m_ownsBvh = false;
   // update the scaling without rebuilding the bvh
   if ((getLocalScaling() -scaling).length2() > D_SIMD_EPSILON)
   {
      D_btTriangleMeshShape::setLocalScaling(scaling);
   }
}


