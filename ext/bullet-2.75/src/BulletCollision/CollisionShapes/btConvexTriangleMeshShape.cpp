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

#include "btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "LinearMath/btQuaternion.h"
#include "BulletCollision/CollisionShapes/btStridingMeshInterface.h"


D_btConvexTriangleMeshShape ::D_btConvexTriangleMeshShape (D_btStridingMeshInterface* meshInterface, bool calcAabb)
: D_btPolyhedralConvexAabbCachingShape(), m_stridingMesh(meshInterface)
{
	m_shapeType = D_CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
	if ( calcAabb )
		recalcLocalAabb();
}




///It's not nice D_to have all this virtual function overhead, so perhaps we D_can also gather the points once
///but then we D_are duplicating
class D_LocalSupportVertexCallback: public D_btInternalTriangleIndexCallback
{

	D_btVector3 m_supportVertexLocal;
public:

	D_btScalar m_maxDot;
	D_btVector3 m_supportVecLocal;

	D_LocalSupportVertexCallback(const D_btVector3& supportVecLocal)
		: m_supportVertexLocal(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.)),
		m_maxDot(D_btScalar(-D_BT_LARGE_FLOAT)),
                m_supportVecLocal(supportVecLocal)
	{
	}

	virtual void internalProcessTriangleIndex(D_btVector3* triangle,int partId,int  triangleIndex)
	{
		(void)triangleIndex;
		(void)partId;

		for (int i=0;i<3;i++)
		{
			D_btScalar dot = m_supportVecLocal.dot(triangle[i]);
			if (dot > m_maxDot)
			{
				m_maxDot = dot;
				m_supportVertexLocal = triangle[i];
			}
		}
	}
	
	D_btVector3	GetSupportVertexLocal()
	{
		return m_supportVertexLocal;
	}

};





D_btVector3	D_btConvexTriangleMeshShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec0)const
{
	D_btVector3 supVec(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));

	D_btVector3 vec = vec0;
	D_btScalar lenSqr = vec.length2();
	if (lenSqr < D_btScalar(0.0001))
	{
		vec.setValue(1,0,0);
	} else
	{
		D_btScalar rlen = D_btScalar(1.) / D_btSqrt(lenSqr );
		vec *= rlen;
	}

	D_LocalSupportVertexCallback	supportCallback(vec);
	D_btVector3 aabbMax(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
	m_stridingMesh->InternalProcessAllTriangles(&supportCallback,-aabbMax,aabbMax);
	supVec = supportCallback.GetSupportVertexLocal();

	return supVec;
}

void	D_btConvexTriangleMeshShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
	//use 'w' component of supportVerticesOut?
	{
		for (int i=0;i<numVectors;i++)
		{
			supportVerticesOut[i][3] = D_btScalar(-D_BT_LARGE_FLOAT);
		}
	}
	
	///@todo: could do the batch inside the callback!


	for (int j=0;j<numVectors;j++)
	{
		const D_btVector3& vec = vectors[j];
		D_LocalSupportVertexCallback	supportCallback(vec);
		D_btVector3 aabbMax(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
		m_stridingMesh->InternalProcessAllTriangles(&supportCallback,-aabbMax,aabbMax);
		supportVerticesOut[j] = supportCallback.GetSupportVertexLocal();
	}
	
}
	


D_btVector3	D_btConvexTriangleMeshShape::localGetSupportingVertex(const D_btVector3& vec)const
{
	D_btVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);

	if ( getMargin()!=D_btScalar(0.) )
	{
		D_btVector3 vecnorm = vec;
		if (vecnorm .length2() < (D_SIMD_EPSILON*D_SIMD_EPSILON))
		{
			vecnorm.setValue(D_btScalar(-1.),D_btScalar(-1.),D_btScalar(-1.));
		} 
		vecnorm.normalize();
		supVertex+= getMargin() * vecnorm;
	}
	return supVertex;
}









//currently D_just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you D_can D_debug-draw D_btConvexTriangleMeshShape with the Raytracer Demo
int	D_btConvexTriangleMeshShape::getNumVertices() const
{
	//cache this?
	return 0;
	
}

int D_btConvexTriangleMeshShape::getNumEdges() const
{
	return 0;
}

void D_btConvexTriangleMeshShape::getEdge(int ,D_btVector3& ,D_btVector3& ) const
{
	D_btAssert(0);	
}

void D_btConvexTriangleMeshShape::getVertex(int ,D_btVector3& ) const
{
	D_btAssert(0);
}

int	D_btConvexTriangleMeshShape::getNumPlanes() const
{
	return 0;
}

void D_btConvexTriangleMeshShape::getPlane(D_btVector3& ,D_btVector3& ,int  ) const
{
	D_btAssert(0);
}

//not yet
bool D_btConvexTriangleMeshShape::isInside(const D_btVector3& ,D_btScalar ) const
{
	D_btAssert(0);
	return false;
}



void	D_btConvexTriangleMeshShape::setLocalScaling(const D_btVector3& scaling)
{
	m_stridingMesh->setScaling(scaling);
	
	recalcLocalAabb();
	
}


const D_btVector3& D_btConvexTriangleMeshShape::getLocalScaling() const
{
	return m_stridingMesh->getScaling();
}

void D_btConvexTriangleMeshShape::calculatePrincipalAxisTransform(D_btTransform& principal, D_btVector3& inertia, D_btScalar& volume) const
{
   class D_CenterCallback: public D_btInternalTriangleIndexCallback
   {
      bool first;
      D_btVector3 ref;
      D_btVector3 sum;
      D_btScalar volume;

   public:

      D_CenterCallback() : first(true), ref(0, 0, 0), sum(0, 0, 0), volume(0)
      {
      }

      virtual void internalProcessTriangleIndex(D_btVector3* triangle, int partId, int triangleIndex)
      {
         (void) triangleIndex;
         (void) partId;
         if (first)
         {
            ref = triangle[0];
            first = false;
         }
         else
         {
            D_btScalar vol = D_btFabs((triangle[0] - ref).triple(triangle[1] - ref, triangle[2] - ref));
            sum += (D_btScalar(0.25) * vol) * ((triangle[0] + triangle[1] + triangle[2] + ref));
            volume += vol;
         }
      }
      
      D_btVector3 getCenter()
      {
         return (volume > 0) ? sum / volume : ref;
      }

      D_btScalar getVolume()
      {
         return volume * D_btScalar(1. / 6);
      }

   };

   class D_InertiaCallback: public D_btInternalTriangleIndexCallback
   {
      D_btMatrix3x3 sum;
      D_btVector3 center;

   public:

      D_InertiaCallback(D_btVector3& center) : sum(0, 0, 0, 0, 0, 0, 0, 0, 0), center(center)
      {
      }

      virtual void internalProcessTriangleIndex(D_btVector3* triangle, int partId, int triangleIndex)
      {
         (void) triangleIndex;
         (void) partId;
         D_btMatrix3x3 i;
         D_btVector3 a = triangle[0] - center;
         D_btVector3 b = triangle[1] - center;
         D_btVector3 c = triangle[2] - center;
         D_btScalar volNeg = -D_btFabs(a.triple(b, c)) * D_btScalar(1. / 6);
         for (int j = 0; j < 3; j++)
         {
            for (int k = 0; k <= j; k++)
            {
               i[j][k] = i[k][j] = volNeg * (D_btScalar(0.1) * (a[j] * a[k] + b[j] * b[k] + c[j] * c[k])
                  + D_btScalar(0.05) * (a[j] * b[k] + a[k] * b[j] + a[j] * c[k] + a[k] * c[j] + b[j] * c[k] + b[k] * c[j]));
            }
         }
         D_btScalar i00 = -i[0][0];
         D_btScalar i11 = -i[1][1];
         D_btScalar i22 = -i[2][2];
         i[0][0] = i11 + i22; 
         i[1][1] = i22 + i00; 
         i[2][2] = i00 + i11;
         sum[0] += i[0];
         sum[1] += i[1];
         sum[2] += i[2];
      }
      
      D_btMatrix3x3& getInertia()
      {
         return sum;
      }

   };

   D_CenterCallback centerCallback;
   D_btVector3 aabbMax(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
   m_stridingMesh->InternalProcessAllTriangles(&centerCallback, -aabbMax, aabbMax);
   D_btVector3 center = centerCallback.getCenter();
   principal.setOrigin(center);
   volume = centerCallback.getVolume();

   D_InertiaCallback inertiaCallback(center);
   m_stridingMesh->InternalProcessAllTriangles(&inertiaCallback, -aabbMax, aabbMax);

   D_btMatrix3x3& i = inertiaCallback.getInertia();
   i.diagonalize(principal.getBasis(), D_btScalar(0.00001), 20);
   inertia.setValue(i[0][0], i[1][1], i[2][2]);
   inertia /= volume;
}

