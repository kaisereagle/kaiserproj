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

#ifndef CYLINDER_MINKOWSKI_H
#define CYLINDER_MINKOWSKI_H

#include "btBoxShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btVector3.h"

/// The D_btCylinderShape class D_implements a cylinder shape primitive, centered around the origin. Its central axis aligned with the Y axis. D_btCylinderShapeX D_is aligned with the X axis D_and D_btCylinderShapeZ around the D_Z axis.
class D_btCylinderShape : public D_btConvexInternalShape

{

protected:

	int	m_upAxis;

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

	D_btCylinderShape (const D_btVector3& halfExtents);
	
	void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;

	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;

	virtual void setMargin(D_btScalar collisionMargin)
	{
		//correct the m_implicitShapeDimensions for the margin
		D_btVector3 oldMargin(getMargin(),getMargin(),getMargin());
		D_btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		
		D_btConvexInternalShape::setMargin(collisionMargin);
		D_btVector3 newMargin(getMargin(),getMargin(),getMargin());
		m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;

	}

	virtual D_btVector3	localGetSupportingVertex(const D_btVector3& vec) const
	{

		D_btVector3 supVertex;
		supVertex = localGetSupportingVertexWithoutMargin(vec);
		
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
	}


	//use box inertia
	//	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;


	int	getUpAxis() const
	{
		return m_upAxis;
	}

	virtual D_btScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getX();
	}

	//debugging
	virtual const char*	getName()const
	{
		return "CylinderY";
	}



};

class D_btCylinderShapeX : public D_btCylinderShape
{
public:
	D_btCylinderShapeX (const D_btVector3& halfExtents);

	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;
	
		//debugging
	virtual const char*	getName()const
	{
		return "CylinderX";
	}

	virtual D_btScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getY();
	}

};

class D_btCylinderShapeZ : public D_btCylinderShape
{
public:
	D_btCylinderShapeZ (const D_btVector3& halfExtents);

	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;

	virtual int	getUpAxis() const
	{
		return 2;
	}
		//debugging
	virtual const char*	getName()const
	{
		return "CylinderZ";
	}

	virtual D_btScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getX();
	}

};


#endif //CYLINDER_MINKOWSKI_H

