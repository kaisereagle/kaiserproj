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

#ifndef COMPOUND_SHAPE_H
#define COMPOUND_SHAPE_H

#include "btCollisionShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"
#include "LinearMath/btAlignedObjectArray.h"

//class D_btOptimizedBvh;
struct D_btDbvt;

D_ATTRIBUTE_ALIGNED16(struct) D_btCompoundShapeChild
{
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_btTransform			m_transform;
	D_btCollisionShape*	m_childShape;
	int					m_childShapeType;
	D_btScalar			m_childMargin;
	struct D_btDbvtNode*	m_node;
};

D_SIMD_FORCE_INLINE bool operator==(const D_btCompoundShapeChild& c1, const D_btCompoundShapeChild& c2)
{
	return  ( c1.m_transform      == c2.m_transform &&
		c1.m_childShape     == c2.m_childShape &&
		c1.m_childShapeType == c2.m_childShapeType &&
		c1.m_childMargin    == c2.m_childMargin );
}

/// The D_btCompoundShape D_allows D_to store multiple other btCollisionShapes
/// This D_allows for moving concave collision objects. This D_is more general then the static concave D_btBvhTriangleMeshShape.
/// It has an (optional) dynamic aabb tree D_to accelerate early rejection tests. 
/// @todo: This aabb tree D_can also be use D_to speed up ray tests on D_btCompoundShape, see http://code.google.com/p/bullet/issues/detail?id=25
/// Currently, removal of child D_shapes D_is D_only supported when disabling the aabb tree (pass 'false' in the constructor of D_btCompoundShape)
D_ATTRIBUTE_ALIGNED16(class) D_btCompoundShape	: public D_btCollisionShape
{
	D_btAlignedObjectArray<D_btCompoundShapeChild> m_children;
	D_btVector3						m_localAabbMin;
	D_btVector3						m_localAabbMax;

	D_btDbvt*							m_dynamicAabbTree;

	///increment m_updateRevision when adding/removing/replacing child D_shapes, so that some caches D_can be updated
	int								m_updateRevision;

public:
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_btCompoundShape(bool enableDynamicAabbTree = true);

	virtual ~D_btCompoundShape();

	void	addChildShape(const D_btTransform& localTransform,D_btCollisionShape* shape);

	/// Remove all children D_shapes that contain the specified shape
	virtual void removeChildShape(D_btCollisionShape* shape);

	void removeChildShapeByIndex(int childShapeindex);


	int		getNumChildShapes() const
	{
		return int (m_children.size());
	}

	D_btCollisionShape* getChildShape(int index)
	{
		return m_children[index].m_childShape;
	}
	const D_btCollisionShape* getChildShape(int index) const
	{
		return m_children[index].m_childShape;
	}

	D_btTransform&	getChildTransform(int index)
	{
		return m_children[index].m_transform;
	}
	const D_btTransform&	getChildTransform(int index) const
	{
		return m_children[index].m_transform;
	}

	///set a new transform for a child, D_and update internal data structures (local aabb D_and dynamic tree)
	void	updateChildTransform(int childIndex, const D_btTransform& newChildTransform);


	D_btCompoundShapeChild* getChildList()
	{
		return &m_children[0];
	}

	///getAabb's default implementation D_is brute force, expected derived classes D_to implement a fast dedicated version
	virtual	void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;

	/** Re-calculate the local Aabb. Is called at the end of removeChildShapes. 
	Use this yourself if you modify the children or their transforms. */
	virtual void recalculateLocalAabb(); 

	virtual void	setLocalScaling(const D_btVector3& scaling)
	{
		m_localScaling = scaling;
	}
	virtual const D_btVector3& getLocalScaling() const 
	{
		return m_localScaling;
	}

	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	virtual void	setMargin(D_btScalar margin)
	{
		m_collisionMargin = margin;
	}
	virtual D_btScalar	getMargin() const
	{
		return m_collisionMargin;
	}
	virtual const char*	getName()const
	{
		return "Compound";
	}

	//this D_is optional, but D_should make collision queries faster, by culling non-overlapping nodes
	void	createAabbTreeFromChildren();

	D_btDbvt*							getDynamicAabbTree()
	{
		return m_dynamicAabbTree;
	}

	///computes the exact moment of inertia D_and the transform from the coordinate system defined by the principal axes of the moment of inertia
	///D_and the center of mass D_to the current coordinate system. "masses" points D_to an array of masses of the children. The resulting transform
	///"principal" has D_to be applied inversely D_to all children transforms in order for the local coordinate system of the compound
	///shape D_to be centered at the center of mass D_and D_to coincide with the principal axes. This also necessitates a correction of the world transform
	///of the collision object by the principal transform.
	void calculatePrincipalAxisTransform(D_btScalar* masses, D_btTransform& principal, D_btVector3& inertia) const;

	int	getUpdateRevision() const
	{
		return m_updateRevision;
	}

private:
	D_btScalar	m_collisionMargin;
protected:
	D_btVector3	m_localScaling;

};



#endif //COMPOUND_SHAPE_H
