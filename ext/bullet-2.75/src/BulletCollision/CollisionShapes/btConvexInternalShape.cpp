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


#include "btConvexInternalShape.h"



D_btConvexInternalShape::D_btConvexInternalShape()
: m_localScaling(D_btScalar(1.),D_btScalar(1.),D_btScalar(1.)),
m_collisionMargin(D_CONVEX_DISTANCE_MARGIN)
{
}


void	D_btConvexInternalShape::setLocalScaling(const D_btVector3& scaling)
{
	m_localScaling = scaling.absolute();
}



void	D_btConvexInternalShape::getAabbSlow(const D_btTransform& trans,D_btVector3&minAabb,D_btVector3&maxAabb) const
{
#ifndef __SPU__
	//use localGetSupportingVertexWithoutMargin?
	D_btScalar margin = getMargin();
	for (int i=0;i<3;i++)
	{
		D_btVector3 vec(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
		vec[i] = D_btScalar(1.);

		D_btVector3 sv = localGetSupportingVertex(vec*trans.getBasis());

		D_btVector3 tmp = trans(sv);
		maxAabb[i] = tmp[i]+margin;
		vec[i] = D_btScalar(-1.);
		tmp = trans(localGetSupportingVertex(vec*trans.getBasis()));
		minAabb[i] = tmp[i]-margin;
	}
#endif
}



D_btVector3	D_btConvexInternalShape::localGetSupportingVertex(const D_btVector3& vec)const
{
#ifndef __SPU__

	 D_btVector3	supVertex = localGetSupportingVertexWithoutMargin(vec);

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

#else
	D_btAssert(0);
	return D_btVector3(0,0,0);
#endif //__SPU__

 }


D_btConvexInternalAabbCachingShape::D_btConvexInternalAabbCachingShape()
	:	D_btConvexInternalShape(),
m_localAabbMin(1,1,1),
m_localAabbMax(-1,-1,-1),
m_isLocalAabbValid(false)
{
}


void D_btConvexInternalAabbCachingShape::getAabb(const D_btTransform& trans,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	getNonvirtualAabb(trans,aabbMin,aabbMax,getMargin());
}

void	D_btConvexInternalAabbCachingShape::setLocalScaling(const D_btVector3& scaling)
{
	D_btConvexInternalShape::setLocalScaling(scaling);
	recalcLocalAabb();
}


void	D_btConvexInternalAabbCachingShape::recalcLocalAabb()
{
	m_isLocalAabbValid = true;
	
	#if 1
	static const D_btVector3 _directions[] =
	{
		D_btVector3( 1.,  0.,  0.),
		D_btVector3( 0.,  1.,  0.),
		D_btVector3( 0.,  0.,  1.),
		D_btVector3( -1., 0.,  0.),
		D_btVector3( 0., -1.,  0.),
		D_btVector3( 0.,  0., -1.)
	};
	
	D_btVector3 _supporting[] =
	{
		D_btVector3( 0., 0., 0.),
		D_btVector3( 0., 0., 0.),
		D_btVector3( 0., 0., 0.),
		D_btVector3( 0., 0., 0.),
		D_btVector3( 0., 0., 0.),
		D_btVector3( 0., 0., 0.)
	};
	
	batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);
	
	for ( int i = 0; i < 3; ++i )
	{
		m_localAabbMax[i] = _supporting[i][i] + m_collisionMargin;
		m_localAabbMin[i] = _supporting[i + 3][i] - m_collisionMargin;
	}
	
	#else

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
	#endif
}
