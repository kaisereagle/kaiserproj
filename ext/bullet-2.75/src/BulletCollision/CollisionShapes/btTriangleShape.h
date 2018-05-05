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

#ifndef OBB_TRIANGLE_MINKOWSKI_H
#define OBB_TRIANGLE_MINKOWSKI_H

#include "btConvexShape.h"
#include "btBoxShape.h"

D_ATTRIBUTE_ALIGNED16(class) D_btTriangleShape : public D_btPolyhedralConvexShape
{


public:

	D_btVector3	m_vertices1[3];

	virtual int getNumVertices() const
	{
		return 3;
	}

	D_btVector3& getVertexPtr(int index)
	{
		return m_vertices1[index];
	}

	const D_btVector3& getVertexPtr(int index) const
	{
		return m_vertices1[index];
	}
	virtual void getVertex(int index,D_btVector3& vert) const
	{
		vert = m_vertices1[index];
	}

	virtual int getNumEdges() const
	{
		return 3;
	}
	
	virtual void getEdge(int i,D_btVector3& pa,D_btVector3& pb) const
	{
		getVertex(i,pa);
		getVertex((i+1)%3,pb);
	}


	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax)const 
	{
//		D_btAssert(0);
		getAabbSlow(t,aabbMin,aabbMax);
	}

	D_btVector3 localGetSupportingVertexWithoutMargin(const D_btVector3& dir)const 
	{
		D_btVector3 dots(dir.dot(m_vertices1[0]), dir.dot(m_vertices1[1]), dir.dot(m_vertices1[2]));
	  	return m_vertices1[dots.maxAxis()];

	}

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
	{
		for (int i=0;i<numVectors;i++)
		{
			const D_btVector3& dir = vectors[i];
			D_btVector3 dots(dir.dot(m_vertices1[0]), dir.dot(m_vertices1[1]), dir.dot(m_vertices1[2]));
  			supportVerticesOut[i] = m_vertices1[dots.maxAxis()];
		}

	}

	D_btTriangleShape() : D_btPolyhedralConvexShape ()
    {
		m_shapeType = D_TRIANGLE_SHAPE_PROXYTYPE;
	}

	D_btTriangleShape(const D_btVector3& p0,const D_btVector3& p1,const D_btVector3& p2) : D_btPolyhedralConvexShape ()
    {
		m_shapeType = D_TRIANGLE_SHAPE_PROXYTYPE;
        m_vertices1[0] = p0;
        m_vertices1[1] = p1;
        m_vertices1[2] = p2;
    }


	virtual void getPlane(D_btVector3& planeNormal,D_btVector3& planeSupport,int i) const
	{
		getPlaneEquation(i,planeNormal,planeSupport);
	}

	virtual int	getNumPlanes() const
	{
		return 1;
	}

	void calcNormal(D_btVector3& normal) const
	{
		normal = (m_vertices1[1]-m_vertices1[0]).cross(m_vertices1[2]-m_vertices1[0]);
		normal.normalize();
	}

	virtual void getPlaneEquation(int i, D_btVector3& planeNormal,D_btVector3& planeSupport) const
	{
		(void)i;
		calcNormal(planeNormal);
		planeSupport = m_vertices1[0];
	}

	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
	{
		(void)mass;
		D_btAssert(0);
		inertia.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	}

		virtual	bool isInside(const D_btVector3& pt,D_btScalar tolerance) const
	{
		D_btVector3 normal;
		calcNormal(normal);
		//distance D_to plane
		D_btScalar dist = pt.dot(normal);
		D_btScalar planeconst = m_vertices1[0].dot(normal);
		dist -= planeconst;
		if (dist >= -tolerance && dist <= tolerance)
		{
			//inside check on edge-planes
			int i;
			for (i=0;i<3;i++)
			{
				D_btVector3 pa,pb;
				getEdge(i,pa,pb);
				D_btVector3 edge = pb-pa;
				D_btVector3 edgeNormal = edge.cross(normal);
				edgeNormal.normalize();
				D_btScalar dist = pt.dot( edgeNormal);
				D_btScalar edgeConst = pa.dot(edgeNormal);
				dist -= edgeConst;
				if (dist < -tolerance)
					return false;
			}
			
			return true;
		}

		return false;
	}
		//debugging
		virtual const char*	getName()const
		{
			return "Triangle";
		}

		virtual int		getNumPreferredPenetrationDirections() const
		{
			return 2;
		}
		
		virtual void	getPreferredPenetrationDirection(int index, D_btVector3& penetrationVector) const
		{
			calcNormal(penetrationVector);
			if (index)
				penetrationVector *= D_btScalar(-1.);
		}


};

#endif //OBB_TRIANGLE_MINKOWSKI_H

