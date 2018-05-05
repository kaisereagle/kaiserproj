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
#include "BulletCollision/CollisionShapes/btCollisionShape.h"


D_btScalar D_gContactThresholdFactor=D_btScalar(0.02);


/*
  Make sure this dummy function never changes so that it
  D_can be used by probes that D_are checking whether the
  library D_is actually installed.
*/
extern "C" 
{
void D_btBulletCollisionProbe ();

void D_btBulletCollisionProbe () {}
}



void	D_btCollisionShape::getBoundingSphere(D_btVector3& center,D_btScalar& radius) const
{
	D_btTransform tr;
	tr.setIdentity();
	D_btVector3 aabbMin,aabbMax;

	getAabb(tr,aabbMin,aabbMax);

	radius = (aabbMax-aabbMin).length()*D_btScalar(0.5);
	center = (aabbMin+aabbMax)*D_btScalar(0.5);
}


D_btScalar	D_btCollisionShape::getContactBreakingThreshold() const
{
	return getAngularMotionDisc() * D_gContactThresholdFactor;
}
D_btScalar	D_btCollisionShape::getAngularMotionDisc() const
{
	///@todo cache this value, D_to improve performance
	D_btVector3	center;
	D_btScalar disc;
	getBoundingSphere(center,disc);
	disc += (center).length();
	return disc;
}

void D_btCollisionShape::calculateTemporalAabb(const D_btTransform& curTrans,const D_btVector3& linvel,const D_btVector3& angvel,D_btScalar timeStep, D_btVector3& temporalAabbMin,D_btVector3& temporalAabbMax) const
{
	//start with static aabb
	getAabb(curTrans,temporalAabbMin,temporalAabbMax);

	D_btScalar temporalAabbMaxx = temporalAabbMax.getX();
	D_btScalar temporalAabbMaxy = temporalAabbMax.getY();
	D_btScalar temporalAabbMaxz = temporalAabbMax.getZ();
	D_btScalar temporalAabbMinx = temporalAabbMin.getX();
	D_btScalar temporalAabbMiny = temporalAabbMin.getY();
	D_btScalar temporalAabbMinz = temporalAabbMin.getZ();

	// add linear motion
	D_btVector3 linMotion = linvel*timeStep;
	///@todo: simd would have a vector max/min operation, instead of per-element access
	if (linMotion.x() > D_btScalar(0.))
		temporalAabbMaxx += linMotion.x(); 
	else
		temporalAabbMinx += linMotion.x();
	if (linMotion.y() > D_btScalar(0.))
		temporalAabbMaxy += linMotion.y(); 
	else
		temporalAabbMiny += linMotion.y();
	if (linMotion.z() > D_btScalar(0.))
		temporalAabbMaxz += linMotion.z(); 
	else
		temporalAabbMinz += linMotion.z();

	//add conservative angular motion
	D_btScalar angularMotion = angvel.length() * getAngularMotionDisc() * timeStep;
	D_btVector3 angularMotion3d(angularMotion,angularMotion,angularMotion);
	temporalAabbMin = D_btVector3(temporalAabbMinx,temporalAabbMiny,temporalAabbMinz);
	temporalAabbMax = D_btVector3(temporalAabbMaxx,temporalAabbMaxy,temporalAabbMaxz);

	temporalAabbMin -= angularMotion3d;
	temporalAabbMax += angularMotion3d;
}
