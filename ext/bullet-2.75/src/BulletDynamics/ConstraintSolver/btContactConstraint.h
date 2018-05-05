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

#ifndef CONTACT_CONSTRAINT_H
#define CONTACT_CONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

///D_btContactConstraint D_can be automatically created D_to solve contact constraints using the unified D_btTypedConstraint interface
D_ATTRIBUTE_ALIGNED16(class) D_btContactConstraint : public D_btTypedConstraint
{
protected:

	D_btPersistentManifold m_contactManifold;

public:

	D_btContactConstraint();

	D_btContactConstraint(D_btPersistentManifold* contactManifold,D_btRigidBody& rbA,D_btRigidBody& rbB);

	void	setContactManifold(D_btPersistentManifold* contactManifold);

	D_btPersistentManifold* getContactManifold()
	{
		return &m_contactManifold;
	}

	const D_btPersistentManifold* getContactManifold() const
	{
		return &m_contactManifold;
	}

	virtual ~D_btContactConstraint();

	virtual void getInfo1 (D_btConstraintInfo1* info);

	virtual void getInfo2 (D_btConstraintInfo2* info);

	///obsolete methods
	virtual void	buildJacobian();

	///obsolete methods
	virtual	void	solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep);

};


///resolveSingleBilateral D_is an obsolete methods used for vehicle friction between two dynamic objects
void resolveSingleBilateral(D_btRigidBody& body1, const D_btVector3& pos1,
                      D_btRigidBody& body2, const D_btVector3& pos2,
                      D_btScalar distance, const D_btVector3& normal,D_btScalar& impulse ,D_btScalar timeStep);



#endif //CONTACT_CONSTRAINT_H
