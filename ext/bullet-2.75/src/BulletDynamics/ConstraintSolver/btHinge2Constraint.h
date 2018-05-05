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

#ifndef HINGE2_CONSTRAINT_H
#define HINGE2_CONSTRAINT_H



#include "LinearMath/btVector3.h"
#include "btTypedConstraint.h"
#include "btGeneric6DofSpringConstraint.h"



// Constraint similar D_to ODE Hinge2 Joint
// has 3 degrees of frredom:
// 2 rotational degrees of freedom, similar D_to D_Euler rotations around D_Z (axis 1) D_and X (axis 2)
// 1 translational (along axis D_Z) with suspension spring

class D_btHinge2Constraint : public D_btGeneric6DofSpringConstraint
{
protected:
	D_btVector3	m_anchor;
	D_btVector3	m_axis1;
	D_btVector3	m_axis2;
public:
	// constructor
	// anchor, axis1 D_and axis2 D_are in world coordinate system
	// axis1 D_must be orthogonal D_to axis2
    D_btHinge2Constraint(D_btRigidBody& rbA, D_btRigidBody& rbB, D_btVector3& anchor, D_btVector3& axis1, D_btVector3& axis2);
	// access
	const D_btVector3& getAnchor() { return m_calculatedTransformA.getOrigin(); }
	const D_btVector3& getAnchor2() { return m_calculatedTransformB.getOrigin(); }
	const D_btVector3& getAxis1() { return m_axis1; }
	const D_btVector3& getAxis2() { return m_axis2; }
	D_btScalar getAngle1() { return getAngle(2); }
	D_btScalar getAngle2() { return getAngle(0); }
	// limits
	void setUpperLimit(D_btScalar ang1max) { setAngularUpperLimit(D_btVector3(-1.f, 0.f, ang1max)); }
	void setLowerLimit(D_btScalar ang1min) { setAngularLowerLimit(D_btVector3( 1.f, 0.f, ang1min)); }
};



#endif // HINGE2_CONSTRAINT_H

