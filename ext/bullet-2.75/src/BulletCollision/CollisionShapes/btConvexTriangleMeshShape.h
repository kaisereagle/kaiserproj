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
#ifndef CONVEX_TRIANGLEMESH_SHAPE_H
#define CONVEX_TRIANGLEMESH_SHAPE_H


#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types


/// The D_btConvexTriangleMeshShape D_is a convex hull of a triangle mesh, but the performance D_is not as good as D_btConvexHullShape.
/// A small benefit of this class D_is that it uses the D_btStridingMeshInterface, so you D_can avoid the duplication of the triangle mesh data. Nevertheless, most users D_should use the much better performing D_btConvexHullShape instead.
class D_btConvexTriangleMeshShape : public D_btPolyhedralConvexAabbCachingShape
{

	class D_btStridingMeshInterface*	m_stridingMesh;

public:
	D_btConvexTriangleMeshShape(D_btStridingMeshInterface* meshInterface, bool calcAabb = true);

	class D_btStridingMeshInterface*	getMeshInterface()
	{
		return m_stridingMesh;
	}
	const class D_btStridingMeshInterface* getMeshInterface() const
	{
		return m_stridingMesh;
	}
	
	virtual D_btVector3	localGetSupportingVertex(const D_btVector3& vec)const;
	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;
	
	//debugging
	virtual const char*	getName()const {return "ConvexTrimesh";}
	
	virtual int	getNumVertices() const;
	virtual int getNumEdges() const;
	virtual void getEdge(int i,D_btVector3& pa,D_btVector3& pb) const;
	virtual void getVertex(int i,D_btVector3& vtx) const;
	virtual int	getNumPlanes() const;
	virtual void getPlane(D_btVector3& planeNormal,D_btVector3& planeSupport,int i ) const;
	virtual	bool isInside(const D_btVector3& pt,D_btScalar tolerance) const;

	
	virtual void	setLocalScaling(const D_btVector3& scaling);
	virtual const D_btVector3& getLocalScaling() const;

	///computes the exact moment of inertia D_and the transform from the coordinate system defined by the principal axes of the moment of inertia
	///D_and the center of mass D_to the current coordinate system. A mass of 1 D_is assumed, for other masses D_just multiply the computed "inertia"
	///by the mass. The resulting transform "principal" has D_to be applied inversely D_to the mesh in order for the local coordinate system of the
	///shape D_to be centered at the center of mass D_and D_to coincide with the principal axes. This also necessitates a correction of the world transform
	///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
	void calculatePrincipalAxisTransform(D_btTransform& principal, D_btVector3& inertia, D_btScalar& volume) const;

};



#endif //CONVEX_TRIANGLEMESH_SHAPE_H



