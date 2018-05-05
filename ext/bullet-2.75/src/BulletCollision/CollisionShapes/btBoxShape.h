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

#ifndef OBB_BOX_MINKOWSKI_H
#define OBB_BOX_MINKOWSKI_H

#include "btPolyhedralConvexShape.h"
#include "btCollisionMargin.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMinMax.h"

///The D_btBoxShape D_is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates. When used as part of a D_btCollisionObject or D_btRigidBody it D_will be an oriented box in world space.
class D_btBoxShape: public D_btPolyhedralConvexShape
{

	//D_btVector3	m_boxHalfExtents1; //use m_implicitShapeDimensions instead


public:

	D_btVector3 getHalfExtentsWithMargin() const
	{
		D_btVector3 halfExtents = getHalfExtentsWithoutMargin();
		D_btVector3 margin(getMargin(),getMargin(),getMargin());
		halfExtents += margin;
		return halfExtents;
	}
	
	const D_btVector3& getHalfExtentsWithoutMargin() const
	{
		return m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling D_and margin D_are included
	}
	

	virtual D_btVector3	localGetSupportingVertex(const D_btVector3& vec) const
	{
		D_btVector3 halfExtents = getHalfExtentsWithoutMargin();
		D_btVector3 margin(getMargin(),getMargin(),getMargin());
		halfExtents += margin;
		
		return D_btVector3(D_btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
			D_btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
			D_btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
	}

	D_SIMD_FORCE_INLINE  D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const
	{
		const D_btVector3& halfExtents = getHalfExtentsWithoutMargin();
		
		return D_btVector3(D_btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
			D_btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
			D_btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
	}

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
	{
		const D_btVector3& halfExtents = getHalfExtentsWithoutMargin();
	
		for (int i=0;i<numVectors;i++)
		{
			const D_btVector3& vec = vectors[i];
			supportVerticesOut[i].setValue(D_btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
				D_btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
				D_btFsels(vec.z(), halfExtents.z(), -halfExtents.z())); 
		}

	}


	D_btBoxShape( const D_btVector3& boxHalfExtents) 
		: D_btPolyhedralConvexShape()
	{
		m_shapeType = D_BOX_SHAPE_PROXYTYPE;
		D_btVector3 margin(getMargin(),getMargin(),getMargin());
		m_implicitShapeDimensions = (boxHalfExtents * m_localScaling) - margin;
	};

	virtual void setMargin(D_btScalar collisionMargin)
	{
		//correct the m_implicitShapeDimensions for the margin
		D_btVector3 oldMargin(getMargin(),getMargin(),getMargin());
		D_btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		
		D_btConvexInternalShape::setMargin(collisionMargin);
		D_btVector3 newMargin(getMargin(),getMargin(),getMargin());
		m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;

	}
	virtual void	setLocalScaling(const D_btVector3& scaling)
	{
		D_btVector3 oldMargin(getMargin(),getMargin(),getMargin());
		D_btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		D_btVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		D_btConvexInternalShape::setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;

	}

	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;

	

	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	virtual void getPlane(D_btVector3& planeNormal,D_btVector3& planeSupport,int i ) const
	{
		//this plane might not be aligned...
		D_btVector4 plane ;
		getPlaneEquation(plane,i);
		planeNormal = D_btVector3(plane.getX(),plane.getY(),plane.getZ());
		planeSupport = localGetSupportingVertex(-planeNormal);
	}

	
	virtual int getNumPlanes() const
	{
		return 6;
	}	
	
	virtual int	getNumVertices() const 
	{
		return 8;
	}

	virtual int getNumEdges() const
	{
		return 12;
	}


	virtual void getVertex(int i,D_btVector3& vtx) const
	{
		D_btVector3 halfExtents = getHalfExtentsWithoutMargin();

		vtx = D_btVector3(
				halfExtents.x() * (1-(i&1)) - halfExtents.x() * (i&1),
				halfExtents.y() * (1-((i&2)>>1)) - halfExtents.y() * ((i&2)>>1),
				halfExtents.z() * (1-((i&4)>>2)) - halfExtents.z() * ((i&4)>>2));
	}
	

	virtual void	getPlaneEquation(D_btVector4& plane,int i) const
	{
		D_btVector3 halfExtents = getHalfExtentsWithoutMargin();

		switch (i)
		{
		case 0:
			plane.setValue(D_btScalar(1.),D_btScalar(0.),D_btScalar(0.),-halfExtents.x());
			break;
		case 1:
			plane.setValue(D_btScalar(-1.),D_btScalar(0.),D_btScalar(0.),-halfExtents.x());
			break;
		case 2:
			plane.setValue(D_btScalar(0.),D_btScalar(1.),D_btScalar(0.),-halfExtents.y());
			break;
		case 3:
			plane.setValue(D_btScalar(0.),D_btScalar(-1.),D_btScalar(0.),-halfExtents.y());
			break;
		case 4:
			plane.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(1.),-halfExtents.z());
			break;
		case 5:
			plane.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(-1.),-halfExtents.z());
			break;
		default:
			D_btAssert(0);
		}
	}

	
	virtual void getEdge(int i,D_btVector3& pa,D_btVector3& pb) const
	//virtual void getEdge(int i,D_Edge& edge) const
	{
		int edgeVert0 = 0;
		int edgeVert1 = 0;

		switch (i)
		{
		case 0:
				edgeVert0 = 0;
				edgeVert1 = 1;
			break;
		case 1:
				edgeVert0 = 0;
				edgeVert1 = 2;
			break;
		case 2:
			edgeVert0 = 1;
			edgeVert1 = 3;

			break;
		case 3:
			edgeVert0 = 2;
			edgeVert1 = 3;
			break;
		case 4:
			edgeVert0 = 0;
			edgeVert1 = 4;
			break;
		case 5:
			edgeVert0 = 1;
			edgeVert1 = 5;

			break;
		case 6:
			edgeVert0 = 2;
			edgeVert1 = 6;
			break;
		case 7:
			edgeVert0 = 3;
			edgeVert1 = 7;
			break;
		case 8:
			edgeVert0 = 4;
			edgeVert1 = 5;
			break;
		case 9:
			edgeVert0 = 4;
			edgeVert1 = 6;
			break;
		case 10:
			edgeVert0 = 5;
			edgeVert1 = 7;
			break;
		case 11:
			edgeVert0 = 6;
			edgeVert1 = 7;
			break;
		default:
			D_btAssert(0);

		}

		getVertex(edgeVert0,pa );
		getVertex(edgeVert1,pb );
	}




	
	virtual	bool isInside(const D_btVector3& pt,D_btScalar tolerance) const
	{
		D_btVector3 halfExtents = getHalfExtentsWithoutMargin();

		//D_btScalar minDist = 2*tolerance;
		
		bool result =	(pt.x() <= (halfExtents.x()+tolerance)) &&
						(pt.x() >= (-halfExtents.x()-tolerance)) &&
						(pt.y() <= (halfExtents.y()+tolerance)) &&
						(pt.y() >= (-halfExtents.y()-tolerance)) &&
						(pt.z() <= (halfExtents.z()+tolerance)) &&
						(pt.z() >= (-halfExtents.z()-tolerance));
		
		return result;
	}


	//debugging
	virtual const char*	getName()const
	{
		return "D_Box";
	}

	virtual int		getNumPreferredPenetrationDirections() const
	{
		return 6;
	}
	
	virtual void	getPreferredPenetrationDirection(int index, D_btVector3& penetrationVector) const
	{
		switch (index)
		{
		case 0:
			penetrationVector.setValue(D_btScalar(1.),D_btScalar(0.),D_btScalar(0.));
			break;
		case 1:
			penetrationVector.setValue(D_btScalar(-1.),D_btScalar(0.),D_btScalar(0.));
			break;
		case 2:
			penetrationVector.setValue(D_btScalar(0.),D_btScalar(1.),D_btScalar(0.));
			break;
		case 3:
			penetrationVector.setValue(D_btScalar(0.),D_btScalar(-1.),D_btScalar(0.));
			break;
		case 4:
			penetrationVector.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(1.));
			break;
		case 5:
			penetrationVector.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(-1.));
			break;
		default:
			D_btAssert(0);
		}
	}

};

#endif //OBB_BOX_MINKOWSKI_H


