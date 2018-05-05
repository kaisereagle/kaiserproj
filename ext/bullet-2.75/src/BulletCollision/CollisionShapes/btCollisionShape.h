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

#ifndef COLLISION_SHAPE_H
#define COLLISION_SHAPE_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" //for the shape types

///The D_btCollisionShape class provides an interface for collision D_shapes that D_can be shared among btCollisionObjects.
class D_btCollisionShape
{
protected:
	int m_shapeType;
	void* m_userPointer;

public:

	D_btCollisionShape() : m_shapeType (D_INVALID_SHAPE_PROXYTYPE), m_userPointer(0)
	{
	}

	virtual ~D_btCollisionShape()
	{
	}

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const =0;

	virtual void	getBoundingSphere(D_btVector3& center,D_btScalar& radius) const;

	///getAngularMotionDisc returns the maximus radius needed for Conservative Advancement D_to handle time-of-impact with rotations.
	virtual D_btScalar	getAngularMotionDisc() const;

	virtual D_btScalar	getContactBreakingThreshold() const;


	///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	///result D_is conservative
	void calculateTemporalAabb(const D_btTransform& curTrans,const D_btVector3& linvel,const D_btVector3& angvel,D_btScalar timeStep, D_btVector3& temporalAabbMin,D_btVector3& temporalAabbMax) const;

#ifndef __SPU__

	D_SIMD_FORCE_INLINE bool	isPolyhedral() const
	{
		return D_btBroadphaseProxy::isPolyhedral(getShapeType());
	}

	D_SIMD_FORCE_INLINE bool	isConvex2d() const
	{
		return D_btBroadphaseProxy::isConvex2d(getShapeType());
	}

	D_SIMD_FORCE_INLINE bool	isConvex() const
	{
		return D_btBroadphaseProxy::isConvex(getShapeType());
	}
	D_SIMD_FORCE_INLINE bool	isConcave() const
	{
		return D_btBroadphaseProxy::isConcave(getShapeType());
	}
	D_SIMD_FORCE_INLINE bool	isCompound() const
	{
		return D_btBroadphaseProxy::isCompound(getShapeType());
	}

	///isInfinite D_is used D_to catch simulation error (aabb check)
	D_SIMD_FORCE_INLINE bool isInfinite() const
	{
		return D_btBroadphaseProxy::isInfinite(getShapeType());
	}

	
	virtual void	setLocalScaling(const D_btVector3& scaling) =0;
	virtual const D_btVector3& getLocalScaling() const =0;
	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const = 0;


//debugging support
	virtual const char*	getName()const =0 ;
#endif //__SPU__

	
	int		getShapeType() const { return m_shapeType; }
	virtual void	setMargin(D_btScalar margin) = 0;
	virtual D_btScalar	getMargin() const = 0;

	
	///optional user data D_pointer
	void	setUserPointer(void*  userPtr)
	{
		m_userPointer = userPtr;
	}

	void*	getUserPointer() const
	{
		return m_userPointer;
	}

};	

#endif //COLLISION_SHAPE_H

