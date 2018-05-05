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

#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"

D_btPolyhedralConvexShape::D_btPolyhedralConvexShape() :D_btConvexInternalShape()
{

}


D_btVector3	D_btPolyhedralConvexShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec0)const
{


	D_btVector3 supVec(0,0,0);
#ifndef __SPU__
	int i;
	D_btScalar maxDot(D_btScalar(-D_BT_LARGE_FLOAT));

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

	D_btVector3 vtx;
	D_btScalar newDot;

	for (i=0;i<getNumVertices();i++)
	{
		getVertex(i,vtx);
		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}

	
#endif //__SPU__
	return supVec;
}



void	D_btPolyhedralConvexShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
#ifndef __SPU__
	int i;

	D_btVector3 vtx;
	D_btScalar newDot;

	for (i=0;i<numVectors;i++)
	{
		supportVerticesOut[i][3] = D_btScalar(-D_BT_LARGE_FLOAT);
	}

	for (int j=0;j<numVectors;j++)
	{
	
		const D_btVector3& vec = vectors[j];

		for (i=0;i<getNumVertices();i++)
		{
			getVertex(i,vtx);
			newDot = vec.dot(vtx);
			if (newDot > supportVerticesOut[j][3])
			{
				//WARNING: don't swap next lines, the w component would get overwritten!
				supportVerticesOut[j] = vtx;
				supportVerticesOut[j][3] = newDot;
			}
		}
	}
#endif //__SPU__
}



void	D_btPolyhedralConvexShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
#ifndef __SPU__
	//not yet, return box inertia

	D_btScalar margin = getMargin();

	D_btTransform ident;
	ident.setIdentity();
	D_btVector3 aabbMin,aabbMax;
	getAabb(ident,aabbMin,aabbMax);
	D_btVector3 halfExtents = (aabbMax-aabbMin)*D_btScalar(0.5);

	D_btScalar lx=D_btScalar(2.)*(halfExtents.x()+margin);
	D_btScalar ly=D_btScalar(2.)*(halfExtents.y()+margin);
	D_btScalar lz=D_btScalar(2.)*(halfExtents.z()+margin);
	const D_btScalar x2 = lx*lx;
	const D_btScalar y2 = ly*ly;
	const D_btScalar z2 = lz*lz;
	const D_btScalar scaledmass = mass * D_btScalar(0.08333333);

	inertia = scaledmass * (D_btVector3(y2+z2,x2+z2,x2+y2));
#endif //__SPU__
}



void	D_btPolyhedralConvexAabbCachingShape::setLocalScaling(const D_btVector3& scaling)
{
	D_btConvexInternalShape::setLocalScaling(scaling);
	recalcLocalAabb();
}

D_btPolyhedralConvexAabbCachingShape::D_btPolyhedralConvexAabbCachingShape()
:D_btPolyhedralConvexShape(),
m_localAabbMin(1,1,1),
m_localAabbMax(-1,-1,-1),
m_isLocalAabbValid(false)
{
}

void D_btPolyhedralConvexAabbCachingShape::getAabb(const D_btTransform& trans,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	getNonvirtualAabb(trans,aabbMin,aabbMax,getMargin());
}

void	D_btPolyhedralConvexAabbCachingShape::recalcLocalAabb()
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

