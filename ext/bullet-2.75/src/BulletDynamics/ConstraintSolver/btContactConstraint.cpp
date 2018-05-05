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


#include "btContactConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btContactSolverInfo.h"
#include "LinearMath/btMinMax.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"

D_btContactConstraint::D_btContactConstraint()
:D_btTypedConstraint(D_CONTACT_CONSTRAINT_TYPE)
{
}

D_btContactConstraint::D_btContactConstraint(D_btPersistentManifold* contactManifold,D_btRigidBody& rbA,D_btRigidBody& rbB)
:D_btTypedConstraint(D_CONTACT_CONSTRAINT_TYPE,rbA,rbB),
	m_contactManifold(*contactManifold)
{

}

D_btContactConstraint::~D_btContactConstraint()
{

}

void	D_btContactConstraint::setContactManifold(D_btPersistentManifold* contactManifold)
{
	m_contactManifold = *contactManifold;
}

void D_btContactConstraint::getInfo1 (D_btConstraintInfo1* info)
{

}

void D_btContactConstraint::getInfo2 (D_btConstraintInfo2* info)
{

}

void	D_btContactConstraint::buildJacobian()
{

}

void	D_btContactConstraint::solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep)
{

}




#include "btContactConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btContactSolverInfo.h"
#include "LinearMath/btMinMax.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"

#define D_ASSERT2 D_btAssert

#define D_USE_INTERNAL_APPLY_IMPULSE 1


//bilateral constraint between two dynamic objects
void resolveSingleBilateral(D_btRigidBody& body1, const D_btVector3& pos1,
                      D_btRigidBody& body2, const D_btVector3& pos2,
                      D_btScalar distance, const D_btVector3& normal,D_btScalar& impulse ,D_btScalar timeStep)
{
	(void)timeStep;
	(void)distance;


	D_btScalar normalLenSqr = normal.length2();
	D_ASSERT2(D_btFabs(normalLenSqr) < D_btScalar(1.1));
	if (normalLenSqr > D_btScalar(1.1))
	{
		impulse = D_btScalar(0.);
		return;
	}
	D_btVector3 rel_pos1 = pos1 - body1.getCenterOfMassPosition(); 
	D_btVector3 rel_pos2 = pos2 - body2.getCenterOfMassPosition();
	//this jacobian entry could be re-used for all iterations
	
	D_btVector3 vel1 = body1.getVelocityInLocalPoint(rel_pos1);
	D_btVector3 vel2 = body2.getVelocityInLocalPoint(rel_pos2);
	D_btVector3 vel = vel1 - vel2;
	

	   D_btJacobianEntry jac(body1.getCenterOfMassTransform().getBasis().transpose(),
		body2.getCenterOfMassTransform().getBasis().transpose(),
		rel_pos1,rel_pos2,normal,body1.getInvInertiaDiagLocal(),body1.getInvMass(),
		body2.getInvInertiaDiagLocal(),body2.getInvMass());

	D_btScalar jacDiagAB = jac.getDiagonal();
	D_btScalar jacDiagABInv = D_btScalar(1.) / jacDiagAB;
	
	  D_btScalar rel_vel = jac.getRelativeVelocity(
		body1.getLinearVelocity(),
		body1.getCenterOfMassTransform().getBasis().transpose() * body1.getAngularVelocity(),
		body2.getLinearVelocity(),
		body2.getCenterOfMassTransform().getBasis().transpose() * body2.getAngularVelocity()); 
	D_btScalar a;
	a=jacDiagABInv;


	rel_vel = normal.dot(vel);
	
	//todo: move this into proper structure
	D_btScalar contactDamping = D_btScalar(0.2);

#ifdef ONLY_USE_LINEAR_MASS
	D_btScalar massTerm = D_btScalar(1.) / (body1.getInvMass() + body2.getInvMass());
	impulse = - contactDamping * rel_vel * massTerm;
#else	
	D_btScalar velocityImpulse = -contactDamping * rel_vel * jacDiagABInv;
	impulse = velocityImpulse;
#endif
}




