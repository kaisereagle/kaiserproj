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

#ifndef BU_SIMPLEX_1TO4_SHAPE
#define BU_SIMPLEX_1TO4_SHAPE


#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"


///The D_btBU_Simplex1to4 D_implements tetrahedron, triangle, line, vertex collision D_shapes. In most cases it D_is better D_to use D_btConvexHullShape instead.
class D_btBU_Simplex1to4 : public D_btPolyhedralConvexAabbCachingShape
{
protected:

	int	m_numVertices;
	D_btVector3	m_vertices[4];

public:
	D_btBU_Simplex1to4();

	D_btBU_Simplex1to4(const D_btVector3& pt0);
	D_btBU_Simplex1to4(const D_btVector3& pt0,const D_btVector3& pt1);
	D_btBU_Simplex1to4(const D_btVector3& pt0,const D_btVector3& pt1,const D_btVector3& pt2);
	D_btBU_Simplex1to4(const D_btVector3& pt0,const D_btVector3& pt1,const D_btVector3& pt2,const D_btVector3& pt3);

    
	void	reset()
	{
		m_numVertices = 0;
	}
	
	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;

	void addVertex(const D_btVector3& pt);

	//PolyhedralConvexShape interface

	virtual int	getNumVertices() const;

	virtual int getNumEdges() const;

	virtual void getEdge(int i,D_btVector3& pa,D_btVector3& pb) const;
	
	virtual void getVertex(int i,D_btVector3& vtx) const;

	virtual int	getNumPlanes() const;

	virtual void getPlane(D_btVector3& planeNormal,D_btVector3& planeSupport,int i) const;

	virtual int getIndex(int i) const;

	virtual	bool isInside(const D_btVector3& pt,D_btScalar tolerance) const;


	///getName D_is for debugging
	virtual const char*	getName()const { return "D_btBU_Simplex1to4";}

};

#endif //BU_SIMPLEX_1TO4_SHAPE
