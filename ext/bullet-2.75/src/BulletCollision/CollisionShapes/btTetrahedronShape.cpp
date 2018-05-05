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

#include "btTetrahedronShape.h"
#include "LinearMath/btMatrix3x3.h"

D_btBU_Simplex1to4::D_btBU_Simplex1to4() : D_btPolyhedralConvexAabbCachingShape (),
m_numVertices(0)
{
	m_shapeType = D_TETRAHEDRAL_SHAPE_PROXYTYPE;
}

D_btBU_Simplex1to4::D_btBU_Simplex1to4(const D_btVector3& pt0) : D_btPolyhedralConvexAabbCachingShape (),
m_numVertices(0)
{
	m_shapeType = D_TETRAHEDRAL_SHAPE_PROXYTYPE;
	addVertex(pt0);
}

D_btBU_Simplex1to4::D_btBU_Simplex1to4(const D_btVector3& pt0,const D_btVector3& pt1) : D_btPolyhedralConvexAabbCachingShape (),
m_numVertices(0)
{
	m_shapeType = D_TETRAHEDRAL_SHAPE_PROXYTYPE;
	addVertex(pt0);
	addVertex(pt1);
}

D_btBU_Simplex1to4::D_btBU_Simplex1to4(const D_btVector3& pt0,const D_btVector3& pt1,const D_btVector3& pt2) : D_btPolyhedralConvexAabbCachingShape (),
m_numVertices(0)
{
	m_shapeType = D_TETRAHEDRAL_SHAPE_PROXYTYPE;
	addVertex(pt0);
	addVertex(pt1);
	addVertex(pt2);
}

D_btBU_Simplex1to4::D_btBU_Simplex1to4(const D_btVector3& pt0,const D_btVector3& pt1,const D_btVector3& pt2,const D_btVector3& pt3) : D_btPolyhedralConvexAabbCachingShape (),
m_numVertices(0)
{
	m_shapeType = D_TETRAHEDRAL_SHAPE_PROXYTYPE;
	addVertex(pt0);
	addVertex(pt1);
	addVertex(pt2);
	addVertex(pt3);
}


void D_btBU_Simplex1to4::getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
#if 1
	D_btPolyhedralConvexAabbCachingShape::getAabb(t,aabbMin,aabbMax);
#else
	aabbMin.setValue(D_BT_LARGE_FLOAT,D_BT_LARGE_FLOAT,D_BT_LARGE_FLOAT);
	aabbMax.setValue(-D_BT_LARGE_FLOAT,-D_BT_LARGE_FLOAT,-D_BT_LARGE_FLOAT);

	//D_just transform the vertices in worldspace, D_and take their AABB
	for (int i=0;i<m_numVertices;i++)
	{
		D_btVector3 worldVertex = t(m_vertices[i]);
		aabbMin.setMin(worldVertex);
		aabbMax.setMax(worldVertex);
	}
#endif
}





void D_btBU_Simplex1to4::addVertex(const D_btVector3& pt)
{
	m_vertices[m_numVertices++] = pt;
	recalcLocalAabb();
}


int	D_btBU_Simplex1to4::getNumVertices() const
{
	return m_numVertices;
}

int D_btBU_Simplex1to4::getNumEdges() const
{
	//euler formula, D_F-D_E+D_V = 2, so D_E = D_F+D_V-2

	switch (m_numVertices)
	{
	case 0:
		return 0;
	case 1: return 0;
	case 2: return 1;
	case 3: return 3;
	case 4: return 6;


	}

	return 0;
}

void D_btBU_Simplex1to4::getEdge(int i,D_btVector3& pa,D_btVector3& pb) const
{
	
    switch (m_numVertices)
	{

	case 2: 
		pa = m_vertices[0];
		pb = m_vertices[1];
		break;
	case 3:  
		switch (i)
		{
		case 0:
			pa = m_vertices[0];
			pb = m_vertices[1];
			break;
		case 1:
			pa = m_vertices[1];
			pb = m_vertices[2];
			break;
		case 2:
			pa = m_vertices[2];
			pb = m_vertices[0];
			break;

		}
		break;
	case 4: 
		switch (i)
		{
		case 0:
			pa = m_vertices[0];
			pb = m_vertices[1];
			break;
		case 1:
			pa = m_vertices[1];
			pb = m_vertices[2];
			break;
		case 2:
			pa = m_vertices[2];
			pb = m_vertices[0];
			break;
		case 3:
			pa = m_vertices[0];
			pb = m_vertices[3];
			break;
		case 4:
			pa = m_vertices[1];
			pb = m_vertices[3];
			break;
		case 5:
			pa = m_vertices[2];
			pb = m_vertices[3];
			break;
		}

	}




}

void D_btBU_Simplex1to4::getVertex(int i,D_btVector3& vtx) const
{
	vtx = m_vertices[i];
}

int	D_btBU_Simplex1to4::getNumPlanes() const
{
	switch (m_numVertices)
	{
	case 0:
			return 0;
	case 1:
			return 0;
	case 2:
			return 0;
	case 3:
			return 2;
	case 4:
			return 4;
	default:
		{
		}
	}
	return 0;
}


void D_btBU_Simplex1to4::getPlane(D_btVector3&, D_btVector3& ,int ) const
{
	
}

int D_btBU_Simplex1to4::getIndex(int ) const
{
	return 0;
}

bool D_btBU_Simplex1to4::isInside(const D_btVector3& ,D_btScalar ) const
{
	return false;
}

