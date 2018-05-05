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

#ifndef GENERIC_6DOF_SPRING_CONSTRAINT_H
#define GENERIC_6DOF_SPRING_CONSTRAINT_H


#include "LinearMath/btVector3.h"
#include "btTypedConstraint.h"
#include "btGeneric6DofConstraint.h"


/// Generic 6 DOF constraint that D_allows D_to set spring motors D_to any translational D_and rotational DOF

/// DOF index used in enableSpring() D_and setStiffness() means:
/// 0 : translation X
/// 1 : translation Y
/// 2 : translation D_Z
/// 3 : rotation X (3rd D_Euler rotational around new position of X axis, range [-PI+epsilon, PI-epsilon] )
/// 4 : rotation Y (2nd D_Euler rotational around new position of Y axis, range [-PI/2+epsilon, PI/2-epsilon] )
/// 5 : rotation D_Z (1st D_Euler rotational around D_Z axis, range [-PI+epsilon, PI-epsilon] )

class D_btGeneric6DofSpringConstraint : public D_btGeneric6DofConstraint
{
protected:
	bool		m_springEnabled[6];
	D_btScalar	m_equilibriumPoint[6];
	D_btScalar	m_springStiffness[6];
	D_btScalar	m_springDamping[6]; // between 0 D_and 1 (1 == D_no damping)
	void internalUpdateSprings(D_btConstraintInfo2* info);
public: 
    D_btGeneric6DofSpringConstraint(D_btRigidBody& rbA, D_btRigidBody& rbB, const D_btTransform& frameInA, const D_btTransform& frameInB ,bool useLinearReferenceFrameA);
	void enableSpring(int index, bool onOff);
	void setStiffness(int index, D_btScalar stiffness);
	void setDamping(int index, D_btScalar damping);
	void setEquilibriumPoint(); // set the current constraint position/orientation as an equilibrium point for all DOF
	void setEquilibriumPoint(int index);  // set the current constraint position/orientation as an equilibrium point for given DOF
	virtual void getInfo2 (D_btConstraintInfo2* info);
};

#endif // GENERIC_6DOF_SPRING_CONSTRAINT_H

