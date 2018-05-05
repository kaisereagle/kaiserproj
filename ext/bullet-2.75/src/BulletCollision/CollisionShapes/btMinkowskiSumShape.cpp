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


#include "btMinkowskiSumShape.h"


D_btMinkowskiSumShape::D_btMinkowskiSumShape(const D_btConvexShape* shapeA,const D_btConvexShape* shapeB)
: D_btConvexInternalShape (),
m_shapeA(shapeA),
m_shapeB(shapeB)
{
	m_shapeType = D_MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE;
	m_transA.setIdentity();
	m_transB.setIdentity();
}

D_btVector3 D_btMinkowskiSumShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const
{
	D_btVector3 supVertexA = m_transA(m_shapeA->localGetSupportingVertexWithoutMargin(vec*m_transA.getBasis()));
	D_btVector3 supVertexB = m_transB(m_shapeB->localGetSupportingVertexWithoutMargin(-vec*m_transB.getBasis()));
	return  supVertexA - supVertexB;
}

void	D_btMinkowskiSumShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
	///@todo: could make recursive use of batching. D_probably this shape D_is not used frequently.
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = localGetSupportingVertexWithoutMargin(vectors[i]);
	}

}



D_btScalar	D_btMinkowskiSumShape::getMargin() const
{
	return m_shapeA->getMargin() + m_shapeB->getMargin();
}


void	D_btMinkowskiSumShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	(void)mass;
	D_btAssert(0);
	inertia.setValue(0,0,0);
}
