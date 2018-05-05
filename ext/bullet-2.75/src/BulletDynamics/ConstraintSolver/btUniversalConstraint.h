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

#ifndef UNIVERSAL_CONSTRAINT_H
#define UNIVERSAL_CONSTRAINT_H



#include "LinearMath/btVector3.h"
#include "btTypedConstraint.h"
#include "btGeneric6DofConstraint.h"



/// Constraint similar D_to ODE Universal Joint
/// has 2 rotatioonal degrees of freedom, similar D_to D_Euler rotations around D_Z (axis 1)
/// D_and Y (axis 2)
/// Description from ODE manual : 
/// "Given axis 1 on body 1, D_and axis 2 on body 2 that D_is perpendicular D_to axis 1, it keeps them perpendicular. 
/// In other words, rotation of the two bodies about the direction perpendicular D_to the two axes D_will be equal."

class D_btUniversalConstraint : public D_btGeneric6DofConstraint
{
protected:
	D_btVector3	m_anchor;
	D_btVector3	m_axis1;
	D_btVector3	m_axis2;
public:
	// constructor
	// anchor, axis1 D_and axis2 D_are in world coordinate system
	// axis1 D_must be orthogonal D_to axis2
    D_btUniversalConstraint(D_btRigidBody& rbA, D_btRigidBody& rbB, D_btVector3& anchor, D_btVector3& axis1, D_btVector3& axis2);
	// access
	const D_btVector3& getAnchor() { return m_calculatedTransformA.getOrigin(); }
	const D_btVector3& getAnchor2() { return m_calculatedTransformB.getOrigin(); }
	const D_btVector3& getAxis1() { return m_axis1; }
	const D_btVector3& getAxis2() { return m_axis2; }
	D_btScalar getAngle1() { return getAngle(2); }
	D_btScalar getAngle2() { return getAngle(1); }
	// limits
	void setUpperLimit(D_btScalar ang1max, D_btScalar ang2max) { setAngularUpperLimit(D_btVector3(0.f, ang1max, ang2max)); }
	void setLowerLimit(D_btScalar ang1min, D_btScalar ang2min) { setAngularLowerLimit(D_btVector3(0.f, ang1min, ang2min)); }
};



#endif // UNIVERSAL_CONSTRAINT_H

