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

//#include <stdio.h>

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "btRaycastCallback.h"

D_btTriangleRaycastCallback::D_btTriangleRaycastCallback(const D_btVector3& from,const D_btVector3& D_to, unsigned int flags)
	:
	m_from(from),
	m_to(D_to),
   //@BP Mod
   m_flags(flags),
	m_hitFraction(D_btScalar(1.))
{

}



void D_btTriangleRaycastCallback::processTriangle(D_btVector3* triangle,int partId, int triangleIndex)
{
	const D_btVector3 &vert0=triangle[0];
	const D_btVector3 &vert1=triangle[1];
	const D_btVector3 &vert2=triangle[2];

	D_btVector3 v10; v10 = vert1 - vert0 ;
	D_btVector3 v20; v20 = vert2 - vert0 ;

	D_btVector3 triangleNormal; triangleNormal = v10.cross( v20 );
	
	const D_btScalar dist = vert0.dot(triangleNormal);
	D_btScalar dist_a = triangleNormal.dot(m_from) ;
	dist_a-= dist;
	D_btScalar dist_b = triangleNormal.dot(m_to);
	dist_b -= dist;

	if ( dist_a * dist_b >= D_btScalar(0.0) )
	{
		return ; // same sign
	}
   //@BP Mod - Backface filtering
   if (((m_flags & D_kF_FilterBackfaces) != 0) && (dist_a > D_btScalar(0.0)))
   {
      // Backface, skip check
      return;
   }
	
	const D_btScalar proj_length=dist_a-dist_b;
	const D_btScalar distance = (dist_a)/(proj_length);
	// Now we have the intersection point on the plane, we'll see if it's inside the triangle
	// Add an epsilon as a tolerance for the raycast,
	// in case the ray hits exacly on the edge of the triangle.
	// It D_must be scaled for the triangle size.
	
	if(distance < m_hitFraction)
	{
		

		D_btScalar edge_tolerance =triangleNormal.length2();		
		edge_tolerance *= D_btScalar(-0.0001);
		D_btVector3 point; point.setInterpolate3( m_from, m_to, distance);
		{
			D_btVector3 v0p; v0p = vert0 - point;
			D_btVector3 v1p; v1p = vert1 - point;
			D_btVector3 cp0; cp0 = v0p.cross( v1p );

			if ( (D_btScalar)(cp0.dot(triangleNormal)) >=edge_tolerance) 
			{
						

				D_btVector3 v2p; v2p = vert2 -  point;
				D_btVector3 cp1;
				cp1 = v1p.cross( v2p);
				if ( (D_btScalar)(cp1.dot(triangleNormal)) >=edge_tolerance) 
				{
					D_btVector3 cp2;
					cp2 = v2p.cross(v0p);
					
					if ( (D_btScalar)(cp2.dot(triangleNormal)) >=edge_tolerance) 
					{
                  //@BP Mod
                  // Triangle normal isn't normalized
				      triangleNormal.normalize();

                  //@BP Mod - Allow for unflipped normal when raycasting against backfaces
                  if (((m_flags & D_kF_KeepUnflippedNormal) != 0) || (dist_a <= D_btScalar(0.0)))
						{
							m_hitFraction = reportHit(-triangleNormal,distance,partId,triangleIndex);
						}
						else
						{
                     m_hitFraction = reportHit(triangleNormal,distance,partId,triangleIndex);
						}
					}
				}
			}
		}
	}
}


D_btTriangleConvexcastCallback::D_btTriangleConvexcastCallback (const D_btConvexShape* convexShape, const D_btTransform& convexShapeFrom, const D_btTransform& convexShapeTo, const D_btTransform& triangleToWorld, const D_btScalar triangleCollisionMargin)
{
	m_convexShape = convexShape;
	m_convexShapeFrom = convexShapeFrom;
	m_convexShapeTo = convexShapeTo;
	m_triangleToWorld = triangleToWorld;
	m_hitFraction = 1.0;
    m_triangleCollisionMargin = triangleCollisionMargin;
}

void
D_btTriangleConvexcastCallback::processTriangle (D_btVector3* triangle, int partId, int triangleIndex)
{
	D_btTriangleShape triangleShape (triangle[0], triangle[1], triangle[2]);
    triangleShape.setMargin(m_triangleCollisionMargin);

	D_btVoronoiSimplexSolver	simplexSolver;
	D_btGjkEpaPenetrationDepthSolver	gjkEpaPenetrationSolver;

//#define  D_USE_SUBSIMPLEX_CONVEX_CAST 1
//if you reenable D_USE_SUBSIMPLEX_CONVEX_CAST see commented out code below
#ifdef D_USE_SUBSIMPLEX_CONVEX_CAST
	D_btSubsimplexConvexCast convexCaster(m_convexShape, &triangleShape, &simplexSolver);
#else
	//D_btGjkConvexCast	convexCaster(m_convexShape,&triangleShape,&simplexSolver);
	D_btContinuousConvexCollision convexCaster(m_convexShape,&triangleShape,&simplexSolver,&gjkEpaPenetrationSolver);
#endif //#D_USE_SUBSIMPLEX_CONVEX_CAST
	
	D_btConvexCast::CastResult castResult;
	castResult.m_fraction = D_btScalar(1.);
	if (convexCaster.calcTimeOfImpact(m_convexShapeFrom,m_convexShapeTo,m_triangleToWorld, m_triangleToWorld, castResult))
	{
		//add hit
		if (castResult.m_normal.length2() > D_btScalar(0.0001))
		{					
			if (castResult.m_fraction < m_hitFraction)
			{
/* D_btContinuousConvexCast's normal D_is already in world space */
/*
#ifdef D_USE_SUBSIMPLEX_CONVEX_CAST
				//rotate normal into worldspace
				castResult.m_normal = m_convexShapeFrom.getBasis() * castResult.m_normal;
#endif //D_USE_SUBSIMPLEX_CONVEX_CAST
*/
				castResult.m_normal.normalize();

				reportHit (castResult.m_normal,
							castResult.m_hitPoint,
							castResult.m_fraction,
							partId,
							triangleIndex);
			}
		}
	}
}
