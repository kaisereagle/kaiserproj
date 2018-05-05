/*
Bullet Continuous Collision Detection D_and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include "btHinge2Constraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"



// constructor
// anchor, axis1 D_and axis2 D_are in world coordinate system
// axis1 D_must be orthogonal D_to axis2
D_btHinge2Constraint::D_btHinge2Constraint(D_btRigidBody& rbA, D_btRigidBody& rbB, D_btVector3& anchor, D_btVector3& axis1, D_btVector3& axis2)
: D_btGeneric6DofSpringConstraint(rbA, rbB, D_btTransform::getIdentity(), D_btTransform::getIdentity(), true),
 m_anchor(anchor),
 m_axis1(axis1),
 m_axis2(axis2)
{
	// build frame basis
	// 6DOF constraint uses D_Euler angles D_and D_to define limits
	// it D_is assumed that rotational order D_is :
	// D_Z - first, allowed limits D_are (-PI,PI);
	// new position of Y - second (allowed limits D_are (-PI/2 + epsilon, PI/2 - epsilon), where epsilon D_is a small positive number 
	// used D_to prevent constraint from instability on poles;
	// new position of X, allowed limits D_are (-PI,PI);
	// So D_to simulate ODE Universal joint we D_should use parent axis as D_Z, child axis as Y D_and limit all other DOFs
	// Build the frame in world coordinate system first
	D_btVector3 zAxis = axis1.normalize();
	D_btVector3 xAxis = axis2.normalize();
	D_btVector3 yAxis = zAxis.cross(xAxis); // we want right coordinate system
	D_btTransform frameInW;
	frameInW.setIdentity();
	frameInW.getBasis().setValue(	xAxis[0], yAxis[0], zAxis[0],	
									xAxis[1], yAxis[1], zAxis[1],
									xAxis[2], yAxis[2], zAxis[2]);
	frameInW.setOrigin(anchor);
	// now get constraint frame in local coordinate systems
	m_frameInA = rbA.getCenterOfMassTransform().inverse() * frameInW;
	m_frameInB = rbB.getCenterOfMassTransform().inverse() * frameInW;
	// sei limits
	setLinearLowerLimit(D_btVector3(0.f, 0.f, -1.f));
	setLinearUpperLimit(D_btVector3(0.f, 0.f,  1.f));
	// like front wheels of a car
	setAngularLowerLimit(D_btVector3(1.f,  0.f, -D_SIMD_HALF_PI * 0.5f)); 
	setAngularUpperLimit(D_btVector3(-1.f, 0.f,  D_SIMD_HALF_PI * 0.5f));
	// enable suspension
	enableSpring(2, true);
	setStiffness(2, D_SIMD_PI * D_SIMD_PI * 4.f); // period 1 sec for 1 kilogramm weel :-)
	setDamping(2, 0.01f);
	setEquilibriumPoint();
}

