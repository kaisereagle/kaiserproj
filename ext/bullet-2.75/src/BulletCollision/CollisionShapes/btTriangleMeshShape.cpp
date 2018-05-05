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

#include "btTriangleMeshShape.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "btStridingMeshInterface.h"
#include "LinearMath/btAabbUtil2.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"


D_btTriangleMeshShape::D_btTriangleMeshShape(D_btStridingMeshInterface* meshInterface)
: D_btConcaveShape (), m_meshInterface(meshInterface)
{
	m_shapeType = D_TRIANGLE_MESH_SHAPE_PROXYTYPE;
	if(meshInterface->hasPremadeAabb())
	{
		meshInterface->getPremadeAabb(&m_localAabbMin, &m_localAabbMax);
	}
	else
	{
		recalcLocalAabb();
	}
}


D_btTriangleMeshShape::~D_btTriangleMeshShape()
{
		
}




void D_btTriangleMeshShape::getAabb(const D_btTransform& trans,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{

	D_btVector3 localHalfExtents = D_btScalar(0.5)*(m_localAabbMax-m_localAabbMin);
	localHalfExtents += D_btVector3(getMargin(),getMargin(),getMargin());
	D_btVector3 localCenter = D_btScalar(0.5)*(m_localAabbMax+m_localAabbMin);
	
	D_btMatrix3x3 abs_b = trans.getBasis().absolute();  

	D_btVector3 center = trans(localCenter);

	D_btVector3 extent = D_btVector3(abs_b[0].dot(localHalfExtents),
		   abs_b[1].dot(localHalfExtents),
		  abs_b[2].dot(localHalfExtents));
	aabbMin = center - extent;
	aabbMax = center + extent;


}

void	D_btTriangleMeshShape::recalcLocalAabb()
{
	for (int i=0;i<3;i++)
	{
		D_btVector3 vec(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
		vec[i] = D_btScalar(1.);
		D_btVector3 tmp = localGetSupportingVertex(vec);
		m_localAabbMax[i] = tmp[i]+m_collisionMargin;
		vec[i] = D_btScalar(-1.);
		tmp = localGetSupportingVertex(vec);
		m_localAabbMin[i] = tmp[i]-m_collisionMargin;
	}
}



class D_SupportVertexCallback : public D_btTriangleCallback
{

	D_btVector3 m_supportVertexLocal;
public:

	D_btTransform	m_worldTrans;
	D_btScalar m_maxDot;
	D_btVector3 m_supportVecLocal;

	D_SupportVertexCallback(const D_btVector3& supportVecWorld,const D_btTransform& trans)
		: m_supportVertexLocal(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.)), m_worldTrans(trans) ,m_maxDot(D_btScalar(-D_BT_LARGE_FLOAT))
		
	{
		m_supportVecLocal = supportVecWorld * m_worldTrans.getBasis();
	}

	virtual void processTriangle( D_btVector3* triangle,int partId, int triangleIndex)
	{
		(void)partId;
		(void)triangleIndex;
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

	D_btVector3 GetSupportVertexWorldSpace()
	{
		return m_worldTrans(m_supportVertexLocal);
	}

	D_btVector3	GetSupportVertexLocal()
	{
		return m_supportVertexLocal;
	}

};

	
void D_btTriangleMeshShape::setLocalScaling(const D_btVector3& scaling)
{
	m_meshInterface->setScaling(scaling);
	recalcLocalAabb();
}

const D_btVector3& D_btTriangleMeshShape::getLocalScaling() const
{
	return m_meshInterface->getScaling();
}






//#define DEBUG_TRIANGLE_MESH



void	D_btTriangleMeshShape::processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{
		struct D_FilteredCallback : public D_btInternalTriangleIndexCallback
	{
		D_btTriangleCallback* m_callback;
		D_btVector3 m_aabbMin;
		D_btVector3 m_aabbMax;

		D_FilteredCallback(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax)
			:m_callback(callback),
			m_aabbMin(aabbMin),
			m_aabbMax(aabbMax)
		{
		}

		virtual void internalProcessTriangleIndex(D_btVector3* triangle,int partId,int triangleIndex)
		{
			if (TestTriangleAgainstAabb2(&triangle[0],m_aabbMin,m_aabbMax))
			{
				//check aabb in triangle-space, before doing this
				m_callback->processTriangle(triangle,partId,triangleIndex);
			}
			
		}

	};

	D_FilteredCallback filterCallback(callback,aabbMin,aabbMax);

	m_meshInterface->InternalProcessAllTriangles(&filterCallback,aabbMin,aabbMax);
}





void	D_btTriangleMeshShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	(void)mass;
	//moving concave objects not supported
	D_btAssert(0);
	inertia.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
}


D_btVector3 D_btTriangleMeshShape::localGetSupportingVertex(const D_btVector3& vec) const
{
	D_btVector3 supportVertex;

	D_btTransform ident;
	ident.setIdentity();

	D_SupportVertexCallback supportCallback(vec,ident);

	D_btVector3 aabbMax(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
	
	processAllTriangles(&supportCallback,-aabbMax,aabbMax);
		
	supportVertex = supportCallback.GetSupportVertexLocal();

	return supportVertex;
}
