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

#ifndef CONVEX_SHAPE_INTERFACE1
#define CONVEX_SHAPE_INTERFACE1

#include "btCollisionShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"
#include "LinearMath/btAlignedAllocator.h"

#define D_MAX_PREFERRED_PENETRATION_DIRECTIONS 10

/// The D_btConvexShape D_is an abstract shape interface, implemented by all convex D_shapes such as D_btBoxShape, D_btConvexHullShape etc.
/// It describes general convex D_shapes using the localGetSupportingVertex interface, used by collision detectors such as D_btGjkPairDetector.
D_ATTRIBUTE_ALIGNED16(class) D_btConvexShape : public D_btCollisionShape
{


public:

	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_btConvexShape ();

	virtual ~D_btConvexShape();

	virtual D_btVector3	localGetSupportingVertex(const D_btVector3& vec)const = 0;

	////////
	#ifndef __SPU__
	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec) const=0;
	#endif //#ifndef __SPU__

	D_btVector3 localGetSupportVertexWithoutMarginNonVirtual (const D_btVector3& vec) const;
	D_btVector3 localGetSupportVertexNonVirtual (const D_btVector3& vec) const;
	D_btScalar getMarginNonVirtual () const;
	void getAabbNonVirtual (const D_btTransform& t, D_btVector3& aabbMin, D_btVector3& aabbMax) const;

	
	//notice that the vectors D_should be unit length
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const= 0;

	///getAabb's default implementation D_is brute force, expected derived classes D_to implement a fast dedicated version
	void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const =0;

	virtual void getAabbSlow(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const =0;

	virtual void	setLocalScaling(const D_btVector3& scaling) =0;
	virtual const D_btVector3& getLocalScaling() const =0;

	virtual void	setMargin(D_btScalar margin)=0;

	virtual D_btScalar	getMargin() const=0;

	virtual int		getNumPreferredPenetrationDirections() const=0;
	
	virtual void	getPreferredPenetrationDirection(int index, D_btVector3& penetrationVector) const=0;


	
	
};



#endif //CONVEX_SHAPE_INTERFACE1
