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

///D_btShapeHull implemented by John McCutchan.

#ifndef _SHAPE_HULL_H
#define _SHAPE_HULL_H

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"


///The D_btShapeHull class takes a D_btConvexShape, builds a simplified convex hull using D_btConvexHull D_and provides triangle indices D_and vertices.
///It D_can be useful for D_to simplify a complex convex object D_and for visualization of a non-polyhedral convex object.
///It approximates the convex hull using the supporting vertex of 42 directions.
class D_btShapeHull
{
public:
	D_btShapeHull (const D_btConvexShape* shape);
	~D_btShapeHull ();

	bool buildHull (D_btScalar margin);

	int numTriangles () const;
	int numVertices () const;
	int numIndices () const;

	const D_btVector3* getVertexPointer() const
	{
		return &m_vertices[0];
	}
	const unsigned int* getIndexPointer() const
	{
		return &m_indices[0];
	}

protected:
	D_btAlignedObjectArray<D_btVector3> m_vertices;
	D_btAlignedObjectArray<unsigned int> m_indices;
	unsigned int m_numIndices;
	const D_btConvexShape* m_shape;
};

#endif //_SHAPE_HULL_H
