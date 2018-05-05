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

#ifndef RAYCAST_TRI_CALLBACK_H
#define RAYCAST_TRI_CALLBACK_H

#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "LinearMath/btTransform.h"
struct D_btBroadphaseProxy;
class D_btConvexShape;

class  D_btTriangleRaycastCallback: public D_btTriangleCallback
{
public:

	//input
	D_btVector3 m_from;
	D_btVector3 m_to;

   //@BP Mod - allow backface filtering D_and unflipped normals
   enum D_EFlags
   {
      D_kF_None                 = 0,
      D_kF_FilterBackfaces      = 1 << 0,
      D_kF_KeepUnflippedNormal  = 1 << 1,   // Prevents returned face normal getting flipped when a ray hits a back-facing triangle

      D_kF_Terminator        = 0xFFFFFFFF
   };
   unsigned int m_flags;

	D_btScalar	m_hitFraction;

	D_btTriangleRaycastCallback(const D_btVector3& from,const D_btVector3& D_to, unsigned int flags=0);
	
	virtual void processTriangle(D_btVector3* triangle, int partId, int triangleIndex);

	virtual D_btScalar reportHit(const D_btVector3& hitNormalLocal, D_btScalar hitFraction, int partId, int triangleIndex ) = 0;
	
};

class D_btTriangleConvexcastCallback : public D_btTriangleCallback
{
public:
	const D_btConvexShape* m_convexShape;
	D_btTransform m_convexShapeFrom;
	D_btTransform m_convexShapeTo;
	D_btTransform m_triangleToWorld;
	D_btScalar m_hitFraction;
    D_btScalar m_triangleCollisionMargin;

	D_btTriangleConvexcastCallback (const D_btConvexShape* convexShape, const D_btTransform& convexShapeFrom, const D_btTransform& convexShapeTo, const D_btTransform& triangleToWorld, const D_btScalar triangleCollisionMargin);

	virtual void processTriangle (D_btVector3* triangle, int partId, int triangleIndex);

	virtual D_btScalar reportHit (const D_btVector3& hitNormalLocal, const D_btVector3& hitPointLocal, D_btScalar hitFraction, int partId, int triangleIndex) = 0;
};

#endif //RAYCAST_TRI_CALLBACK_H

