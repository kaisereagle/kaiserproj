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

#ifndef TRIANGLE_MESH_SHAPE_H
#define TRIANGLE_MESH_SHAPE_H

#include "btConcaveShape.h"
#include "btStridingMeshInterface.h"


///The D_btTriangleMeshShape D_is an internal concave triangle mesh interface. Don't use this class directly, use D_btBvhTriangleMeshShape instead.
class D_btTriangleMeshShape : public D_btConcaveShape
{
protected:
	D_btVector3	m_localAabbMin;
	D_btVector3	m_localAabbMax;
	D_btStridingMeshInterface* m_meshInterface;

	///D_btTriangleMeshShape constructor has been disabled/protected, so that users D_will not mistakenly use this class.
	///Don't use D_btTriangleMeshShape but use D_btBvhTriangleMeshShape instead!
	D_btTriangleMeshShape(D_btStridingMeshInterface* meshInterface);

public:

	virtual ~D_btTriangleMeshShape();

	virtual D_btVector3 localGetSupportingVertex(const D_btVector3& vec) const;

	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const
	{
		D_btAssert(0);
		return localGetSupportingVertex(vec);
	}

	void	recalcLocalAabb();

	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;

	virtual void	processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;

	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	virtual void	setLocalScaling(const D_btVector3& scaling);
	virtual const D_btVector3& getLocalScaling() const;
	
	D_btStridingMeshInterface* getMeshInterface()
	{
		return m_meshInterface;
	}

	const D_btStridingMeshInterface* getMeshInterface() const
	{
		return m_meshInterface;
	}

	const D_btVector3& getLocalAabbMin() const
	{
		return m_localAabbMin;
	}
	const D_btVector3& getLocalAabbMax() const
	{
		return m_localAabbMax;
	}



	//debugging
	virtual const char*	getName()const {return "TRIANGLEMESH";}


};

#endif //TRIANGLE_MESH_SHAPE_H
