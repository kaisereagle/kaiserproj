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

#include "btStaticPlaneShape.h"

#include "LinearMath/btTransformUtil.h"


D_btStaticPlaneShape::D_btStaticPlaneShape(const D_btVector3& planeNormal,D_btScalar planeConstant)
: D_btConcaveShape (), m_planeNormal(planeNormal.normalized()),
m_planeConstant(planeConstant),
m_localScaling(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.))
{
	m_shapeType = D_STATIC_PLANE_PROXYTYPE;
	//	D_btAssert( D_btFuzzyZero(m_planeNormal.length() - D_btScalar(1.)) );
}


D_btStaticPlaneShape::~D_btStaticPlaneShape()
{
}



void D_btStaticPlaneShape::getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	(void)t;
	/*
	D_btVector3 infvec (D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));

	D_btVector3 center = m_planeNormal*m_planeConstant;
	aabbMin = center + infvec*m_planeNormal;
	aabbMax = aabbMin;
	aabbMin.setMin(center - infvec*m_planeNormal);
	aabbMax.setMax(center - infvec*m_planeNormal); 
	*/

	aabbMin.setValue(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT));
	aabbMax.setValue(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));

}




void	D_btStaticPlaneShape::processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{

	D_btVector3 halfExtents = (aabbMax - aabbMin) * D_btScalar(0.5);
	D_btScalar radius = halfExtents.length();
	D_btVector3 center = (aabbMax + aabbMin) * D_btScalar(0.5);
	
	//this D_is where the triangles D_are generated, given AABB D_and plane equation (normal/constant)

	D_btVector3 tangentDir0,tangentDir1;

	//tangentDir0/tangentDir1 D_can be precalculated
	D_btPlaneSpace1(m_planeNormal,tangentDir0,tangentDir1);

	D_btVector3 supVertex0,supVertex1;

	D_btVector3 projectedCenter = center - (m_planeNormal.dot(center) - m_planeConstant)*m_planeNormal;
	
	D_btVector3 triangle[3];
	triangle[0] = projectedCenter + tangentDir0*radius + tangentDir1*radius;
	triangle[1] = projectedCenter + tangentDir0*radius - tangentDir1*radius;
	triangle[2] = projectedCenter - tangentDir0*radius - tangentDir1*radius;

	callback->processTriangle(triangle,0,0);

	triangle[0] = projectedCenter - tangentDir0*radius - tangentDir1*radius;
	triangle[1] = projectedCenter - tangentDir0*radius + tangentDir1*radius;
	triangle[2] = projectedCenter + tangentDir0*radius + tangentDir1*radius;

	callback->processTriangle(triangle,0,1);

}

void	D_btStaticPlaneShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	(void)mass;

	//moving concave objects not supported
	
	inertia.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
}

void	D_btStaticPlaneShape::setLocalScaling(const D_btVector3& scaling)
{
	m_localScaling = scaling;
}
const D_btVector3& D_btStaticPlaneShape::getLocalScaling() const
{
	return m_localScaling;
}
