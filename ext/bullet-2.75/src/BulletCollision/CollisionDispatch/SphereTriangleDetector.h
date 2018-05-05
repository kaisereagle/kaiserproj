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

#ifndef SPHERE_TRIANGLE_DETECTOR_H
#define SPHERE_TRIANGLE_DETECTOR_H

#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"



class D_btSphereShape;
class D_btTriangleShape;



/// sphere-triangle D_to match the D_btDiscreteCollisionDetectorInterface
struct D_SphereTriangleDetector : public D_btDiscreteCollisionDetectorInterface
{
	virtual void	getClosestPoints(const D_ClosestPointInput& input,D_Result& output,class D_btIDebugDraw* debugDraw,bool swapResults=false);

	D_SphereTriangleDetector(D_btSphereShape* sphere,D_btTriangleShape* triangle, D_btScalar contactBreakingThreshold);

	virtual ~D_SphereTriangleDetector() {};

private:

	bool collide(const D_btVector3& sphereCenter,D_btVector3 &point, D_btVector3& resultNormal, D_btScalar& depth, D_btScalar &timeOfImpact, D_btScalar	contactBreakingThreshold);
	bool pointInTriangle(const D_btVector3 vertices[], const D_btVector3 &normal, D_btVector3 *p );
	bool facecontains(const D_btVector3 &p,const D_btVector3* vertices,D_btVector3& normal);

	D_btSphereShape* m_sphere;
	D_btTriangleShape* m_triangle;
	D_btScalar	m_contactBreakingThreshold;
	
};
#endif //SPHERE_TRIANGLE_DETECTOR_H

