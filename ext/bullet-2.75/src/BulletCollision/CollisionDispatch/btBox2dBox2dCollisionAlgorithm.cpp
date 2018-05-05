/*
Bullet Continuous Collision Detection D_and Physics Library
* The b2CollidePolygons routines D_are Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///D_btBox2dBox2dCollisionAlgorithm, with modified b2CollidePolygons routines from the Box2D library.
///The modifications include: switching from b2Vec D_to D_btVector3, redefinition of D_b2Dot, D_b2Cross

#include "btBox2dBox2dCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btBoxBoxDetector.h"
#include "BulletCollision/CollisionShapes/btBox2dShape.h"

#define D_USE_PERSISTENT_CONTACTS 1

D_btBox2dBox2dCollisionAlgorithm::D_btBox2dBox2dCollisionAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* obj0,D_btCollisionObject* obj1)
: D_btActivatingCollisionAlgorithm(ci,obj0,obj1),
m_ownManifold(false),
m_manifoldPtr(mf)
{
	if (!m_manifoldPtr && m_dispatcher->needsCollision(obj0,obj1))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(obj0,obj1);
		m_ownManifold = true;
	}
}

D_btBox2dBox2dCollisionAlgorithm::~D_btBox2dBox2dCollisionAlgorithm()
{
	
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
	
}


void b2CollidePolygons(D_btManifoldResult* manifold,  const D_btBox2dShape* polyA, const D_btTransform& xfA, const D_btBox2dShape* polyB, const D_btTransform& xfB);

//#include <stdio.h>
void D_btBox2dBox2dCollisionAlgorithm::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
		return;

	D_btCollisionObject*	col0 = body0;
	D_btCollisionObject*	col1 = body1;
	D_btBox2dShape* box0 = (D_btBox2dShape*)col0->getCollisionShape();
	D_btBox2dShape* box1 = (D_btBox2dShape*)col1->getCollisionShape();

	resultOut->setPersistentManifold(m_manifoldPtr);

	b2CollidePolygons(resultOut,box0,col0->getWorldTransform(),box1,col1->getWorldTransform());

	//  refreshContactPoints D_is D_only necessary when using persistent contact points. otherwise all points D_are newly added
	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}

}

D_btScalar D_btBox2dBox2dCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* /*body0*/,D_btCollisionObject* /*body1*/,const D_btDispatcherInfo& /*dispatchInfo*/,D_btManifoldResult* /*resultOut*/)
{
	//not yet
	return 1.f;
}


struct D_ClipVertex
{
	D_btVector3 v;
	int id;
	//b2ContactID id;
	//b2ContactID id;
};

#define D_b2Dot(a,b) (a).dot(b)
#define D_b2Mul(a,b) (a)*(b)
#define D_b2MulT(a,b) (a).transpose()*(b)
#define D_b2Cross(a,b) (a).cross(b)
#define D_btCrossS(a,s) D_btVector3(s * a.getY(), -s * a.getX(),0.f)

int b2_maxManifoldPoints =2;

static int ClipSegmentToLine(D_ClipVertex vOut[2], D_ClipVertex vIn[2],
					  const D_btVector3& normal, D_btScalar offset)
{
	// Start with D_no output points
	int numOut = 0;

	// Calculate the distance of end points D_to the line
	D_btScalar distance0 = D_b2Dot(normal, vIn[0].v) - offset;
	D_btScalar distance1 = D_b2Dot(normal, vIn[1].v) - offset;

	// If the points D_are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points D_are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge D_and plane
		D_btScalar interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		if (distance0 > 0.0f)
		{
			vOut[numOut].id = vIn[0].id;
		}
		else
		{
			vOut[numOut].id = vIn[1].id;
		}
		++numOut;
	}

	return numOut;
}

// Find the separation between poly1 D_and poly2 for a give edge normal on poly1.
static D_btScalar EdgeSeparation(const D_btBox2dShape* poly1, const D_btTransform& xf1, int edge1,
							  const D_btBox2dShape* poly2, const D_btTransform& xf2)
{
	int count1 = poly1->getVertexCount();
	const D_btVector3* vertices1 = poly1->getVertices();
	const D_btVector3* normals1 = poly1->getNormals();

	int count2 = poly2->getVertexCount();
	const D_btVector3* vertices2 = poly2->getVertices();

	D_btAssert(0 <= edge1 && edge1 < count1);

	// Convert normal from poly1's frame into poly2's frame.
	D_btVector3 normal1World = D_b2Mul(xf1.getBasis(), normals1[edge1]);
	D_btVector3 normal1 = D_b2MulT(xf2.getBasis(), normal1World);

	// Find support vertex on poly2 for -normal.
	int index = 0;
	D_btScalar minDot = D_BT_LARGE_FLOAT;

	for (int i = 0; i < count2; ++i)
	{
		D_btScalar dot = D_b2Dot(vertices2[i], normal1);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	D_btVector3 v1 = D_b2Mul(xf1, vertices1[edge1]);
	D_btVector3 v2 = D_b2Mul(xf2, vertices2[index]);
	D_btScalar separation = D_b2Dot(v2 - v1, normal1World);
	return separation;
}

// Find the max separation between poly1 D_and poly2 using edge normals from poly1.
static D_btScalar FindMaxSeparation(int* edgeIndex,
								 const D_btBox2dShape* poly1, const D_btTransform& xf1,
								 const D_btBox2dShape* poly2, const D_btTransform& xf2)
{
	int count1 = poly1->getVertexCount();
	const D_btVector3* normals1 = poly1->getNormals();

	// Vector pointing from the centroid of poly1 D_to the centroid of poly2.
	D_btVector3 d = D_b2Mul(xf2, poly2->getCentroid()) - D_b2Mul(xf1, poly1->getCentroid());
	D_btVector3 D_dLocal1 = D_b2MulT(xf1.getBasis(), d);

	// Find edge normal on poly1 that has the largest projection onto d.
	int edge = 0;
	D_btScalar maxDot = -D_BT_LARGE_FLOAT;
	for (int i = 0; i < count1; ++i)
	{
		D_btScalar dot = D_b2Dot(normals1[i], D_dLocal1);
		if (dot > maxDot)
		{
			maxDot = dot;
			edge = i;
		}
	}

	// Get the separation for the edge normal.
	D_btScalar s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
	if (s > 0.0f)
	{
		return s;
	}

	// Check the separation for the previous edge normal.
	int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
	D_btScalar D_sPrev = EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
	if (D_sPrev > 0.0f)
	{
		return D_sPrev;
	}

	// Check the separation for the next edge normal.
	int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
	D_btScalar D_sNext = EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
	if (D_sNext > 0.0f)
	{
		return D_sNext;
	}

	// Find the best edge D_and the search direction.
	int bestEdge;
	D_btScalar bestSeparation;
	int increment;
	if (D_sPrev > s && D_sPrev > D_sNext)
	{
		increment = -1;
		bestEdge = prevEdge;
		bestSeparation = D_sPrev;
	}
	else if (D_sNext > s)
	{
		increment = 1;
		bestEdge = nextEdge;
		bestSeparation = D_sNext;
	}
	else
	{
		*edgeIndex = edge;
		return s;
	}

	// Perform a local search for the best edge normal.
	for ( ; ; )
	{
		if (increment == -1)
			edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
		else
			edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

		s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
		if (s > 0.0f)
		{
			return s;
		}

		if (s > bestSeparation)
		{
			bestEdge = edge;
			bestSeparation = s;
		}
		else
		{
			break;
		}
	}

	*edgeIndex = bestEdge;
	return bestSeparation;
}

static void FindIncidentEdge(D_ClipVertex c[2],
							 const D_btBox2dShape* poly1, const D_btTransform& xf1, int edge1,
							 const D_btBox2dShape* poly2, const D_btTransform& xf2)
{
	int count1 = poly1->getVertexCount();
	const D_btVector3* normals1 = poly1->getNormals();

	int count2 = poly2->getVertexCount();
	const D_btVector3* vertices2 = poly2->getVertices();
	const D_btVector3* normals2 = poly2->getNormals();

	D_btAssert(0 <= edge1 && edge1 < count1);

	// Get the normal of the D_reference edge in poly2's frame.
	D_btVector3 normal1 = D_b2MulT(xf2.getBasis(), D_b2Mul(xf1.getBasis(), normals1[edge1]));

	// Find the incident edge on poly2.
	int index = 0;
	D_btScalar minDot = D_BT_LARGE_FLOAT;
	for (int i = 0; i < count2; ++i)
	{
		D_btScalar dot = D_b2Dot(normal1, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	int i1 = index;
	int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0].v = D_b2Mul(xf2, vertices2[i1]);
//	c[0].id.features.referenceEdge = (unsigned char)edge1;
//	c[0].id.features.incidentEdge = (unsigned char)i1;
//	c[0].id.features.incidentVertex = 0;

	c[1].v = D_b2Mul(xf2, vertices2[i2]);
//	c[1].id.features.referenceEdge = (unsigned char)edge1;
//	c[1].id.features.incidentEdge = (unsigned char)i2;
//	c[1].id.features.incidentVertex = 1;
}

// Find edge normal of max separation on A - return if separating axis D_is found
// Find edge normal of max separation on B - return if separation axis D_is found
// Choose D_reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 D_to 2
void b2CollidePolygons(D_btManifoldResult* manifold,
					  const D_btBox2dShape* polyA, const D_btTransform& xfA,
					  const D_btBox2dShape* polyB, const D_btTransform& xfB)
{

	int edgeA = 0;
	D_btScalar separationA = FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);
	if (separationA > 0.0f)
		return;

	int edgeB = 0;
	D_btScalar separationB = FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);
	if (separationB > 0.0f)
		return;

	const D_btBox2dShape* poly1;	// D_reference poly
	const D_btBox2dShape* poly2;	// incident poly
	D_btTransform xf1, xf2;
	int edge1;		// D_reference edge
	unsigned char flip;
	const D_btScalar k_relativeTol = 0.98f;
	const D_btScalar k_absoluteTol = 0.001f;

	// TODO_ERIN use "radius" of poly for absolute tolerance.
	if (separationB > k_relativeTol * separationA + k_absoluteTol)
	{
		poly1 = polyB;
		poly2 = polyA;
		xf1 = xfB;
		xf2 = xfA;
		edge1 = edgeB;
		flip = 1;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		xf1 = xfA;
		xf2 = xfB;
		edge1 = edgeA;
		flip = 0;
	}

	D_ClipVertex incidentEdge[2];
	FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

	int count1 = poly1->getVertexCount();
	const D_btVector3* vertices1 = poly1->getVertices();

	D_btVector3 v11 = vertices1[edge1];
	D_btVector3 v12 = edge1 + 1 < count1 ? vertices1[edge1+1] : vertices1[0];

	D_btVector3 dv = v12 - v11;
	D_btVector3 sideNormal = D_b2Mul(xf1.getBasis(), v12 - v11);
	sideNormal.normalize();
	D_btVector3 frontNormal = D_btCrossS(sideNormal, 1.0f);
	
	
	v11 = D_b2Mul(xf1, v11);
	v12 = D_b2Mul(xf1, v12);

	D_btScalar frontOffset = D_b2Dot(frontNormal, v11);
	D_btScalar sideOffset1 = -D_b2Dot(sideNormal, v11);
	D_btScalar sideOffset2 = D_b2Dot(sideNormal, v12);

	// Clip incident edge against extruded edge1 side edges.
	D_ClipVertex clipPoints1[2];
	clipPoints1[0].v.setValue(0,0,0);
	clipPoints1[1].v.setValue(0,0,0);

	D_ClipVertex clipPoints2[2];
	clipPoints2[0].v.setValue(0,0,0);
	clipPoints2[1].v.setValue(0,0,0);


	int np;

	// Clip D_to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, sideOffset1);

	if (np < 2)
		return;

	// Clip D_to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1,  sideNormal, sideOffset2);

	if (np < 2)
	{
		return;
	}

	// Now clipPoints2 contains the clipped points.
	D_btVector3 manifoldNormal = flip ? -frontNormal : frontNormal;

	int pointCount = 0;
	for (int i = 0; i < b2_maxManifoldPoints; ++i)
	{
		D_btScalar separation = D_b2Dot(frontNormal, clipPoints2[i].v) - frontOffset;

		if (separation <= 0.0f)
		{
			
			//b2ManifoldPoint* cp = manifold->points + pointCount;
			//D_btScalar separation = separation;
			//cp->localPoint1 = D_b2MulT(xfA, clipPoints2[i].v);
			//cp->localPoint2 = D_b2MulT(xfB, clipPoints2[i].v);

			manifold->addContactPoint(-manifoldNormal,clipPoints2[i].v,separation);

//			cp->id = clipPoints2[i].id;
//			cp->id.features.flip = flip;
			++pointCount;
		}
	}

//	manifold->pointCount = pointCount;}
}
