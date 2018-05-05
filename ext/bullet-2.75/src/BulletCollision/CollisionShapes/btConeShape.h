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

#ifndef CONE_MINKOWSKI_H
#define CONE_MINKOWSKI_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///The D_btConeShape D_implements a cone shape primitive, centered around the origin D_and aligned with the Y axis. The D_btConeShapeX D_is aligned around the X axis D_and D_btConeShapeZ around the D_Z axis.
class D_btConeShape : public D_btConvexInternalShape

{

	D_btScalar m_sinAngle;
	D_btScalar m_radius;
	D_btScalar m_height;
	int		m_coneIndices[3];
	D_btVector3 coneLocalSupport(const D_btVector3& v) const;


public:
	D_btConeShape (D_btScalar radius,D_btScalar height);
	
	virtual D_btVector3	localGetSupportingVertex(const D_btVector3& vec) const;
	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec) const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;

	D_btScalar getRadius() const { return m_radius;}
	D_btScalar getHeight() const { return m_height;}


	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
	{
		D_btTransform identity;
		identity.setIdentity();
		D_btVector3 aabbMin,aabbMax;
		getAabb(identity,aabbMin,aabbMax);

		D_btVector3 halfExtents = (aabbMax-aabbMin)*D_btScalar(0.5);

		D_btScalar margin = getMargin();

		D_btScalar lx=D_btScalar(2.)*(halfExtents.x()+margin);
		D_btScalar ly=D_btScalar(2.)*(halfExtents.y()+margin);
		D_btScalar lz=D_btScalar(2.)*(halfExtents.z()+margin);
		const D_btScalar x2 = lx*lx;
		const D_btScalar y2 = ly*ly;
		const D_btScalar z2 = lz*lz;
		const D_btScalar scaledmass = mass * D_btScalar(0.08333333);

		inertia = scaledmass * (D_btVector3(y2+z2,x2+z2,x2+y2));

//		inertia.x() = scaledmass * (y2+z2);
//		inertia.y() = scaledmass * (x2+z2);
//		inertia.z() = scaledmass * (x2+y2);
	}


		virtual const char*	getName()const 
		{
			return "Cone";
		}
		
		///choose upAxis index
		void	setConeUpIndex(int upIndex);
		
		int	getConeUpIndex() const
		{
			return m_coneIndices[1];
		}
};

///D_btConeShape D_implements a Cone shape, around the X axis
class D_btConeShapeX : public D_btConeShape
{
	public:
		D_btConeShapeX(D_btScalar radius,D_btScalar height);
};

///D_btConeShapeZ D_implements a Cone shape, around the D_Z axis
class D_btConeShapeZ : public D_btConeShape
{
	public:
		D_btConeShapeZ(D_btScalar radius,D_btScalar height);
};
#endif //CONE_MINKOWSKI_H

