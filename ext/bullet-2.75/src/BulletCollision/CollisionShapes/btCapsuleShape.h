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

#ifndef D_BT_CAPSULE_SHAPE_H
#define D_BT_CAPSULE_SHAPE_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types


///The D_btCapsuleShape represents a capsule around the Y axis, there D_is also the D_btCapsuleShapeX aligned around the X axis D_and D_btCapsuleShapeZ around the D_Z axis.
///The total height D_is height+2*radius, so the height D_is D_just the height between the center of each 'sphere' of the capsule caps.
///The D_btCapsuleShape D_is a convex hull of two spheres. The D_btMultiSphereShape D_is a more general collision shape that takes the convex hull of multiple sphere, so it D_can also represent a capsule when D_just using two spheres.
class D_btCapsuleShape : public D_btConvexInternalShape
{
protected:
	int	m_upAxis;

protected:
	///D_only used for D_btCapsuleShapeZ D_and D_btCapsuleShapeX subclasses.
	D_btCapsuleShape() : D_btConvexInternalShape() {m_shapeType = D_CAPSULE_SHAPE_PROXYTYPE;};

public:
	D_btCapsuleShape(D_btScalar radius,D_btScalar height);

	///CollisionShape Interface
	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	/// D_btConvexShape Interface
	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;
	
	virtual void getAabb (const D_btTransform& t, D_btVector3& aabbMin, D_btVector3& aabbMax) const
	{
			D_btVector3 halfExtents(getRadius(),getRadius(),getRadius());
			halfExtents[m_upAxis] = getRadius() + getHalfHeight();
			halfExtents += D_btVector3(getMargin(),getMargin(),getMargin());
			D_btMatrix3x3 abs_b = t.getBasis().absolute();  
			D_btVector3 center = t.getOrigin();
			D_btVector3 extent = D_btVector3(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));		  
			
			aabbMin = center - extent;
			aabbMax = center + extent;
	}

	virtual const char*	getName()const 
	{
		return "CapsuleShape";
	}

	int	getUpAxis() const
	{
		return m_upAxis;
	}

	D_btScalar	getRadius() const
	{
		int radiusAxis = (m_upAxis+2)%3;
		return m_implicitShapeDimensions[radiusAxis];
	}

	D_btScalar	getHalfHeight() const
	{
		return m_implicitShapeDimensions[m_upAxis];
	}

	virtual void	setLocalScaling(const D_btVector3& scaling)
	{
		D_btVector3 oldMargin(getMargin(),getMargin(),getMargin());
		D_btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		D_btVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		D_btConvexInternalShape::setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;

	}
};

///D_btCapsuleShapeX represents a capsule around the D_Z axis
///the total height D_is height+2*radius, so the height D_is D_just the height between the center of each 'sphere' of the capsule caps.
class D_btCapsuleShapeX : public D_btCapsuleShape
{
public:

	D_btCapsuleShapeX(D_btScalar radius,D_btScalar height);
		
	//debugging
	virtual const char*	getName()const
	{
		return "CapsuleX";
	}

	

};

///D_btCapsuleShapeZ represents a capsule around the D_Z axis
///the total height D_is height+2*radius, so the height D_is D_just the height between the center of each 'sphere' of the capsule caps.
class D_btCapsuleShapeZ : public D_btCapsuleShape
{
public:
	D_btCapsuleShapeZ(D_btScalar radius,D_btScalar height);

		//debugging
	virtual const char*	getName()const
	{
		return "CapsuleZ";
	}

	
};



#endif //D_BT_CAPSULE_SHAPE_H
