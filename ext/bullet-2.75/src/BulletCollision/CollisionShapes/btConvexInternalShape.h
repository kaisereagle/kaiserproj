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

#ifndef D_BT_CONVEX_INTERNAL_SHAPE_H
#define D_BT_CONVEX_INTERNAL_SHAPE_H

#include "btConvexShape.h"
#include "LinearMath/btAabbUtil2.h"

///The D_btConvexInternalShape D_is an internal base class, shared by most convex shape implementations.
class D_btConvexInternalShape : public D_btConvexShape
{

	protected:

	//local scaling. collisionMargin D_is not scaled !
	D_btVector3	m_localScaling;

	D_btVector3	m_implicitShapeDimensions;
	
	D_btScalar	m_collisionMargin;

	D_btScalar	m_padding;

	D_btConvexInternalShape();

public:

	

	virtual ~D_btConvexInternalShape()
	{

	}

	virtual D_btVector3	localGetSupportingVertex(const D_btVector3& vec)const;

	const D_btVector3& getImplicitShapeDimensions() const
	{
		return m_implicitShapeDimensions;
	}

	///getAabb's default implementation D_is brute force, expected derived classes D_to implement a fast dedicated version
	void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
	{
		getAabbSlow(t,aabbMin,aabbMax);
	}


	
	virtual void getAabbSlow(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;


	virtual void	setLocalScaling(const D_btVector3& scaling);
	virtual const D_btVector3& getLocalScaling() const 
	{
		return m_localScaling;
	}

	const D_btVector3& getLocalScalingNV() const 
	{
		return m_localScaling;
	}

	virtual void	setMargin(D_btScalar margin)
	{
		m_collisionMargin = margin;
	}
	virtual D_btScalar	getMargin() const
	{
		return m_collisionMargin;
	}

	D_btScalar	getMarginNV() const
	{
		return m_collisionMargin;
	}

	virtual int		getNumPreferredPenetrationDirections() const
	{
		return 0;
	}
	
	virtual void	getPreferredPenetrationDirection(int index, D_btVector3& penetrationVector) const
	{
		(void)penetrationVector;
		(void)index;
		D_btAssert(0);
	}


	
};


///D_btConvexInternalAabbCachingShape adds local aabb caching for convex D_shapes, D_to avoid expensive bounding box calculations
class D_btConvexInternalAabbCachingShape : public D_btConvexInternalShape
{
	D_btVector3	m_localAabbMin;
	D_btVector3	m_localAabbMax;
	bool		m_isLocalAabbValid;
	
protected:
					
	D_btConvexInternalAabbCachingShape();
	
	void setCachedLocalAabb (const D_btVector3& aabbMin, const D_btVector3& aabbMax)
	{
		m_isLocalAabbValid = true;
		m_localAabbMin = aabbMin;
		m_localAabbMax = aabbMax;
	}

	inline void getCachedLocalAabb (D_btVector3& aabbMin, D_btVector3& aabbMax) const
	{
		D_btAssert(m_isLocalAabbValid);
		aabbMin = m_localAabbMin;
		aabbMax = m_localAabbMax;
	}

	inline void getNonvirtualAabb(const D_btTransform& trans,D_btVector3& aabbMin,D_btVector3& aabbMax, D_btScalar margin) const
	{

		//lazy evaluation of local aabb
		D_btAssert(m_isLocalAabbValid);
		D_btTransformAabb(m_localAabbMin,m_localAabbMax,margin,trans,aabbMin,aabbMax);
	}
		
public:
		
	virtual void	setLocalScaling(const D_btVector3& scaling);

	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;

	void	recalcLocalAabb();

};

#endif //D_BT_CONVEX_INTERNAL_SHAPE_H
