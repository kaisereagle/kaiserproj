/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "LinearMath/btScalar.h"
#include "SphereTriangleDetector.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"


D_SphereTriangleDetector::D_SphereTriangleDetector(D_btSphereShape* sphere,D_btTriangleShape* triangle,D_btScalar contactBreakingThreshold)
:m_sphere(sphere),
m_triangle(triangle),
m_contactBreakingThreshold(contactBreakingThreshold)
{

}

void	D_SphereTriangleDetector::getClosestPoints(const D_ClosestPointInput& input,D_Result& output,class D_btIDebugDraw* debugDraw,bool swapResults)
{

	(void)debugDraw;
	const D_btTransform& transformA = input.m_transformA;
	const D_btTransform& transformB = input.m_transformB;

	D_btVector3 point,normal;
	D_btScalar timeOfImpact = D_btScalar(1.);
	D_btScalar depth = D_btScalar(0.);
//	output.m_distance = D_btScalar(D_BT_LARGE_FLOAT);
	//move sphere into triangle space
	D_btTransform	sphereInTr = transformB.inverseTimes(transformA);

	if (collide(sphereInTr.getOrigin(),point,normal,depth,timeOfImpact,m_contactBreakingThreshold))
	{
		if (swapResults)
		{
			D_btVector3 normalOnB = transformB.getBasis()*normal;
			D_btVector3 normalOnA = -normalOnB;
			D_btVector3 pointOnA = transformB*point+normalOnB*depth;
			output.addContactPoint(normalOnA,pointOnA,depth);
		} else
		{
			output.addContactPoint(transformB.getBasis()*normal,transformB*point,depth);
		}
	}

}

#define D_MAX_OVERLAP D_btScalar(0.)



// See also geometrictools.com
// Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
D_btScalar SegmentSqrDistance(const D_btVector3& from, const D_btVector3& D_to,const D_btVector3 &p, D_btVector3 &nearest);

D_btScalar SegmentSqrDistance(const D_btVector3& from, const D_btVector3& D_to,const D_btVector3 &p, D_btVector3 &nearest) {
	D_btVector3 diff = p - from;
	D_btVector3 v = D_to - from;
	D_btScalar t = v.dot(diff);
	
	if (t > 0) {
		D_btScalar dotVV = v.dot(v);
		if (t < dotVV) {
			t /= dotVV;
			diff -= t*v;
		} else {
			t = 1;
			diff -= v;
		}
	} else
		t = 0;

	nearest = from + t*v;
	return diff.dot(diff);	
}

bool D_SphereTriangleDetector::facecontains(const D_btVector3 &p,const D_btVector3* vertices,D_btVector3& normal)  {
	D_btVector3 lp(p);
	D_btVector3 lnormal(normal);
	
	return pointInTriangle(vertices, lnormal, &lp);
}

///combined discrete/continuous sphere-triangle
bool D_SphereTriangleDetector::collide(const D_btVector3& sphereCenter,D_btVector3 &point, D_btVector3& resultNormal, D_btScalar& depth, D_btScalar &timeOfImpact, D_btScalar contactBreakingThreshold)
{

	const D_btVector3* vertices = &m_triangle->getVertexPtr(0);
	const D_btVector3& c = sphereCenter;
	D_btScalar r = m_sphere->getRadius();

	D_btVector3 delta (0,0,0);

	D_btVector3 normal = (vertices[1]-vertices[0]).cross(vertices[2]-vertices[0]);
	normal.normalize();
	D_btVector3 p1ToCentre = c - vertices[0];
	D_btScalar distanceFromPlane = p1ToCentre.dot(normal);

	if (distanceFromPlane < D_btScalar(0.))
	{
		//triangle facing the other way
	
		distanceFromPlane *= D_btScalar(-1.);
		normal *= D_btScalar(-1.);
	}

	D_btScalar contactMargin = contactBreakingThreshold;
	bool isInsideContactPlane = distanceFromPlane < r + contactMargin;
	bool isInsideShellPlane = distanceFromPlane < r;
	
	D_btScalar deltaDotNormal = delta.dot(normal);
	if (!isInsideShellPlane && deltaDotNormal >= D_btScalar(0.0))
		return false;

	// Check for contact / intersection
	bool hasContact = false;
	D_btVector3 contactPoint;
	if (isInsideContactPlane) {
		if (facecontains(c,vertices,normal)) {
			// D_Inside the contact wedge - touches a point on the shell plane
			hasContact = true;
			contactPoint = c - normal*distanceFromPlane;
		} else {
			// Could be inside one of the contact capsules
			D_btScalar contactCapsuleRadiusSqr = (r + contactMargin) * (r + contactMargin);
			D_btVector3 nearestOnEdge;
			for (int i = 0; i < m_triangle->getNumEdges(); i++) {
				
				D_btVector3 pa;
				D_btVector3 pb;
				
				m_triangle->getEdge(i,pa,pb);

				D_btScalar distanceSqr = SegmentSqrDistance(pa,pb,c, nearestOnEdge);
				if (distanceSqr < contactCapsuleRadiusSqr) {
					// Yep, we're inside a capsule
					hasContact = true;
					contactPoint = nearestOnEdge;
				}
				
			}
		}
	}

	if (hasContact) {
		D_btVector3 contactToCentre = c - contactPoint;
		D_btScalar distanceSqr = contactToCentre.length2();
		if (distanceSqr < (r - D_MAX_OVERLAP)*(r - D_MAX_OVERLAP)) {
			D_btScalar distance = D_btSqrt(distanceSqr);
			resultNormal = contactToCentre;
			resultNormal.normalize();
			point = contactPoint;
			depth = -(r-distance);
			return true;
		}

		if (delta.dot(contactToCentre) >= D_btScalar(0.0)) 
			return false;
		
		// Moving towards the contact point -> collision
		point = contactPoint;
		timeOfImpact = D_btScalar(0.0);
		return true;
	}
	
	return false;
}


bool D_SphereTriangleDetector::pointInTriangle(const D_btVector3 vertices[], const D_btVector3 &normal, D_btVector3 *p )
{
	const D_btVector3* p1 = &vertices[0];
	const D_btVector3* p2 = &vertices[1];
	const D_btVector3* p3 = &vertices[2];

	D_btVector3 edge1( *p2 - *p1 );
	D_btVector3 edge2( *p3 - *p2 );
	D_btVector3 edge3( *p1 - *p3 );

	D_btVector3 p1_to_p( *p - *p1 );
	D_btVector3 p2_to_p( *p - *p2 );
	D_btVector3 p3_to_p( *p - *p3 );

	D_btVector3 edge1_normal( edge1.cross(normal));
	D_btVector3 edge2_normal( edge2.cross(normal));
	D_btVector3 edge3_normal( edge3.cross(normal));
	
	D_btScalar r1, r2, r3;
	r1 = edge1_normal.dot( p1_to_p );
	r2 = edge2_normal.dot( p2_to_p );
	r3 = edge3_normal.dot( p3_to_p );
	if ( ( r1 > 0 && r2 > 0 && r3 > 0 ) ||
	     ( r1 <= 0 && r2 <= 0 && r3 <= 0 ) )
		return true;
	return false;

}
