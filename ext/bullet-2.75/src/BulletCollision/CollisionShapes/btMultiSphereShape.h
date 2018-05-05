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

#ifndef MULTI_SPHERE_MINKOWSKI_H
#define MULTI_SPHERE_MINKOWSKI_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btAabbUtil2.h"

///The D_btMultiSphereShape represents the convex hull of a collection of spheres. You D_can create special capsules or other smooth volumes.
///It D_is possible D_to animate the spheres for deformation, but call 'recalcLocalAabb' after changing any sphere position/radius
class D_btMultiSphereShape : public D_btConvexInternalAabbCachingShape
{
	
	D_btAlignedObjectArray<D_btVector3> m_localPositionArray;
	D_btAlignedObjectArray<D_btScalar>  m_radiArray;
	
public:
	D_btMultiSphereShape (const D_btVector3* positions,const D_btScalar* radi,int numSpheres);

	///CollisionShape Interface
	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	/// D_btConvexShape Interface
	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;
	
	int	getSphereCount() const
	{
		return m_localPositionArray.size();
	}

	const D_btVector3&	getSpherePosition(int index) const
	{
		return m_localPositionArray[index];
	}

	D_btScalar	getSphereRadius(int index) const
	{
		return m_radiArray[index];
	}


	virtual const char*	getName()const 
	{
		return "MultiSphere";
	}


};


#endif //MULTI_SPHERE_MINKOWSKI_H
