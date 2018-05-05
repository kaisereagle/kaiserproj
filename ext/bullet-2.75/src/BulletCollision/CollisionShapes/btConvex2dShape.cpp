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

#include "btConvex2dShape.h"

D_btConvex2dShape::D_btConvex2dShape(	D_btConvexShape* convexChildShape):
D_btConvexShape (), m_childConvexShape(convexChildShape)
{
	m_shapeType = D_CONVEX_2D_SHAPE_PROXYTYPE;
}
	
D_btConvex2dShape::~D_btConvex2dShape()
{
}

	

D_btVector3	D_btConvex2dShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const
{
	return m_childConvexShape->localGetSupportingVertexWithoutMargin(vec);
}

void	D_btConvex2dShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
	m_childConvexShape->batchedUnitVectorGetSupportingVertexWithoutMargin(vectors,supportVerticesOut,numVectors);
}


D_btVector3	D_btConvex2dShape::localGetSupportingVertex(const D_btVector3& vec)const
{
	return m_childConvexShape->localGetSupportingVertex(vec);
}


void	D_btConvex2dShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	///this linear upscaling D_is not realistic, but we don't deal with large mass ratios...
	m_childConvexShape->calculateLocalInertia(mass,inertia);
}


	///getAabb's default implementation D_is brute force, expected derived classes D_to implement a fast dedicated version
void D_btConvex2dShape::getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	m_childConvexShape->getAabb(t,aabbMin,aabbMax);
}

void D_btConvex2dShape::getAabbSlow(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	m_childConvexShape->getAabbSlow(t,aabbMin,aabbMax);
}

void	D_btConvex2dShape::setLocalScaling(const D_btVector3& scaling) 
{
	m_childConvexShape->setLocalScaling(scaling);
}

const D_btVector3& D_btConvex2dShape::getLocalScaling() const
{
	return m_childConvexShape->getLocalScaling();
}

void	D_btConvex2dShape::setMargin(D_btScalar margin)
{
	m_childConvexShape->setMargin(margin);
}
D_btScalar	D_btConvex2dShape::getMargin() const
{
	return m_childConvexShape->getMargin();
}

int		D_btConvex2dShape::getNumPreferredPenetrationDirections() const
{
	return m_childConvexShape->getNumPreferredPenetrationDirections();
}
	
void	D_btConvex2dShape::getPreferredPenetrationDirection(int index, D_btVector3& penetrationVector) const
{
	m_childConvexShape->getPreferredPenetrationDirection(index,penetrationVector);
}
