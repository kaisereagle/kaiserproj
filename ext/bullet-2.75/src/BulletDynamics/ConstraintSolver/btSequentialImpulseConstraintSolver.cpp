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

//#define COMPUTE_IMPULSE_DENOM 1
//It D_is not necessary (redundant) D_to refresh contact manifolds, this refresh has been moved D_to the collision algorithms.

#include "btSequentialImpulseConstraintSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btContactConstraint.h"
#include "btSolve2LinearConstraint.h"
#include "btContactSolverInfo.h"
#include "LinearMath/btIDebugDraw.h"
#include "btJacobianEntry.h"
#include "LinearMath/btMinMax.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include <new>
#include "LinearMath/btStackAlloc.h"
#include "LinearMath/btQuickprof.h"
#include "btSolverBody.h"
#include "btSolverConstraint.h"
#include "LinearMath/btAlignedObjectArray.h"
#include <string.h> //for memset

int		D_gNumSplitImpulseRecoveries = 0;

D_btSequentialImpulseConstraintSolver::D_btSequentialImpulseConstraintSolver()
:m_btSeed2(0)
{

}

D_btSequentialImpulseConstraintSolver::~D_btSequentialImpulseConstraintSolver()
{
}

#ifdef D_USE_SIMD
#include <emmintrin.h>
#define D_vec_splat(x, e) _mm_shuffle_ps(x, x, _MM_SHUFFLE(e,e,e,e))
static inline __m128 _vmathVfDot3( __m128 vec0, __m128 vec1 )
{
	__m128 result = _mm_mul_ps( vec0, vec1);
	return _mm_add_ps( D_vec_splat( result, 0 ), _mm_add_ps( D_vec_splat( result, 1 ), D_vec_splat( result, 2 ) ) );
}
#endif//D_USE_SIMD

// Project Gauss Seidel or the equivalent Sequential Impulse
void D_btSequentialImpulseConstraintSolver::resolveSingleConstraintRowGenericSIMD(D_btSolverBody& body1,D_btSolverBody& body2,const D_btSolverConstraint& c)
{
#ifdef D_USE_SIMD
	__m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
	__m128	lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
	__m128	upperLimit1 = _mm_set1_ps(c.m_upperLimit);
	__m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse),_mm_set1_ps(c.m_cfm)));
	__m128 deltaVel1Dotn	=	_mm_add_ps(_vmathVfDot3(c.m_contactNormal.mVec128,body1.m_deltaLinearVelocity.mVec128), _vmathVfDot3(c.m_relpos1CrossNormal.mVec128,body1.m_deltaAngularVelocity.mVec128));
	__m128 deltaVel2Dotn	=	_mm_sub_ps(_vmathVfDot3(c.m_relpos2CrossNormal.mVec128,body2.m_deltaAngularVelocity.mVec128),_vmathVfDot3((c.m_contactNormal).mVec128,body2.m_deltaLinearVelocity.mVec128));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel1Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel2Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	D_btSimdScalar sum = _mm_add_ps(cpAppliedImp,deltaImpulse);
	D_btSimdScalar resultLowerLess,resultUpperLess;
	resultLowerLess = _mm_cmplt_ps(sum,lowerLimit1);
	resultUpperLess = _mm_cmplt_ps(sum,upperLimit1);
	__m128 lowMinApplied = _mm_sub_ps(lowerLimit1,cpAppliedImp);
	deltaImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse) );
	c.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum) );
	__m128 upperMinApplied = _mm_sub_ps(upperLimit1,cpAppliedImp);
	deltaImpulse = _mm_or_ps( _mm_and_ps(resultUpperLess, deltaImpulse), _mm_andnot_ps(resultUpperLess, upperMinApplied) );
	c.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultUpperLess, c.m_appliedImpulse), _mm_andnot_ps(resultUpperLess, upperLimit1) );
	__m128	linearComponentA = _mm_mul_ps(c.m_contactNormal.mVec128,body1.m_invMass.mVec128);
	__m128	linearComponentB = _mm_mul_ps((c.m_contactNormal).mVec128,body2.m_invMass.mVec128);
	__m128 impulseMagnitude = deltaImpulse;
	body1.m_deltaLinearVelocity.mVec128 = _mm_add_ps(body1.m_deltaLinearVelocity.mVec128,_mm_mul_ps(linearComponentA,impulseMagnitude));
	body1.m_deltaAngularVelocity.mVec128 = _mm_add_ps(body1.m_deltaAngularVelocity.mVec128 ,_mm_mul_ps(c.m_angularComponentA.mVec128,impulseMagnitude));
	body2.m_deltaLinearVelocity.mVec128 = _mm_sub_ps(body2.m_deltaLinearVelocity.mVec128,_mm_mul_ps(linearComponentB,impulseMagnitude));
	body2.m_deltaAngularVelocity.mVec128 = _mm_add_ps(body2.m_deltaAngularVelocity.mVec128 ,_mm_mul_ps(c.m_angularComponentB.mVec128,impulseMagnitude));
#else
	resolveSingleConstraintRowGeneric(body1,body2,c);
#endif
}

// Project Gauss Seidel or the equivalent Sequential Impulse
 void D_btSequentialImpulseConstraintSolver::resolveSingleConstraintRowGeneric(D_btSolverBody& body1,D_btSolverBody& body2,const D_btSolverConstraint& c)
{
	D_btScalar deltaImpulse = c.m_rhs-D_btScalar(c.m_appliedImpulse)*c.m_cfm;
	const D_btScalar deltaVel1Dotn	=	c.m_contactNormal.dot(body1.m_deltaLinearVelocity) 	+ c.m_relpos1CrossNormal.dot(body1.m_deltaAngularVelocity);
	const D_btScalar deltaVel2Dotn	=	-c.m_contactNormal.dot(body2.m_deltaLinearVelocity) + c.m_relpos2CrossNormal.dot(body2.m_deltaAngularVelocity);

//	const D_btScalar delta_rel_vel	=	deltaVel1Dotn-deltaVel2Dotn;
	deltaImpulse	-=	deltaVel1Dotn*c.m_jacDiagABInv;
	deltaImpulse	-=	deltaVel2Dotn*c.m_jacDiagABInv;

	const D_btScalar sum = D_btScalar(c.m_appliedImpulse) + deltaImpulse;
	if (sum < c.m_lowerLimit)
	{
		deltaImpulse = c.m_lowerLimit-c.m_appliedImpulse;
		c.m_appliedImpulse = c.m_lowerLimit;
	}
	else if (sum > c.m_upperLimit) 
	{
		deltaImpulse = c.m_upperLimit-c.m_appliedImpulse;
		c.m_appliedImpulse = c.m_upperLimit;
	}
	else
	{
		c.m_appliedImpulse = sum;
	}
		body1.applyImpulse(c.m_contactNormal*body1.m_invMass,c.m_angularComponentA,deltaImpulse);
		body2.applyImpulse(-c.m_contactNormal*body2.m_invMass,c.m_angularComponentB,deltaImpulse);
}

 void D_btSequentialImpulseConstraintSolver::resolveSingleConstraintRowLowerLimitSIMD(D_btSolverBody& body1,D_btSolverBody& body2,const D_btSolverConstraint& c)
{
#ifdef D_USE_SIMD
	__m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
	__m128	lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
	__m128	upperLimit1 = _mm_set1_ps(c.m_upperLimit);
	__m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse),_mm_set1_ps(c.m_cfm)));
	__m128 deltaVel1Dotn	=	_mm_add_ps(_vmathVfDot3(c.m_contactNormal.mVec128,body1.m_deltaLinearVelocity.mVec128), _vmathVfDot3(c.m_relpos1CrossNormal.mVec128,body1.m_deltaAngularVelocity.mVec128));
	__m128 deltaVel2Dotn	=	_mm_sub_ps(_vmathVfDot3(c.m_relpos2CrossNormal.mVec128,body2.m_deltaAngularVelocity.mVec128),_vmathVfDot3((c.m_contactNormal).mVec128,body2.m_deltaLinearVelocity.mVec128));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel1Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel2Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	D_btSimdScalar sum = _mm_add_ps(cpAppliedImp,deltaImpulse);
	D_btSimdScalar resultLowerLess,resultUpperLess;
	resultLowerLess = _mm_cmplt_ps(sum,lowerLimit1);
	resultUpperLess = _mm_cmplt_ps(sum,upperLimit1);
	__m128 lowMinApplied = _mm_sub_ps(lowerLimit1,cpAppliedImp);
	deltaImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse) );
	c.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum) );
	__m128	linearComponentA = _mm_mul_ps(c.m_contactNormal.mVec128,body1.m_invMass.mVec128);
	__m128	linearComponentB = _mm_mul_ps((c.m_contactNormal).mVec128,body2.m_invMass.mVec128);
	__m128 impulseMagnitude = deltaImpulse;
	body1.m_deltaLinearVelocity.mVec128 = _mm_add_ps(body1.m_deltaLinearVelocity.mVec128,_mm_mul_ps(linearComponentA,impulseMagnitude));
	body1.m_deltaAngularVelocity.mVec128 = _mm_add_ps(body1.m_deltaAngularVelocity.mVec128 ,_mm_mul_ps(c.m_angularComponentA.mVec128,impulseMagnitude));
	body2.m_deltaLinearVelocity.mVec128 = _mm_sub_ps(body2.m_deltaLinearVelocity.mVec128,_mm_mul_ps(linearComponentB,impulseMagnitude));
	body2.m_deltaAngularVelocity.mVec128 = _mm_add_ps(body2.m_deltaAngularVelocity.mVec128 ,_mm_mul_ps(c.m_angularComponentB.mVec128,impulseMagnitude));
#else
	resolveSingleConstraintRowLowerLimit(body1,body2,c);
#endif
}

// Project Gauss Seidel or the equivalent Sequential Impulse
 void D_btSequentialImpulseConstraintSolver::resolveSingleConstraintRowLowerLimit(D_btSolverBody& body1,D_btSolverBody& body2,const D_btSolverConstraint& c)
{
	D_btScalar deltaImpulse = c.m_rhs-D_btScalar(c.m_appliedImpulse)*c.m_cfm;
	const D_btScalar deltaVel1Dotn	=	c.m_contactNormal.dot(body1.m_deltaLinearVelocity) 	+ c.m_relpos1CrossNormal.dot(body1.m_deltaAngularVelocity);
	const D_btScalar deltaVel2Dotn	=	-c.m_contactNormal.dot(body2.m_deltaLinearVelocity) + c.m_relpos2CrossNormal.dot(body2.m_deltaAngularVelocity);

	deltaImpulse	-=	deltaVel1Dotn*c.m_jacDiagABInv;
	deltaImpulse	-=	deltaVel2Dotn*c.m_jacDiagABInv;
	const D_btScalar sum = D_btScalar(c.m_appliedImpulse) + deltaImpulse;
	if (sum < c.m_lowerLimit)
	{
		deltaImpulse = c.m_lowerLimit-c.m_appliedImpulse;
		c.m_appliedImpulse = c.m_lowerLimit;
	}
	else
	{
		c.m_appliedImpulse = sum;
	}
	body1.applyImpulse(c.m_contactNormal*body1.m_invMass,c.m_angularComponentA,deltaImpulse);
	body2.applyImpulse(-c.m_contactNormal*body2.m_invMass,c.m_angularComponentB,deltaImpulse);
}


void	D_btSequentialImpulseConstraintSolver::resolveSplitPenetrationImpulseCacheFriendly(
        D_btSolverBody& body1,
        D_btSolverBody& body2,
        const D_btSolverConstraint& c)
{
		if (c.m_rhsPenetration)
        {
			D_gNumSplitImpulseRecoveries++;
			D_btScalar deltaImpulse = c.m_rhsPenetration-D_btScalar(c.m_appliedPushImpulse)*c.m_cfm;
			const D_btScalar deltaVel1Dotn	=	c.m_contactNormal.dot(body1.m_pushVelocity) 	+ c.m_relpos1CrossNormal.dot(body1.m_turnVelocity);
			const D_btScalar deltaVel2Dotn	=	-c.m_contactNormal.dot(body2.m_pushVelocity) + c.m_relpos2CrossNormal.dot(body2.m_turnVelocity);

			deltaImpulse	-=	deltaVel1Dotn*c.m_jacDiagABInv;
			deltaImpulse	-=	deltaVel2Dotn*c.m_jacDiagABInv;
			const D_btScalar sum = D_btScalar(c.m_appliedPushImpulse) + deltaImpulse;
			if (sum < c.m_lowerLimit)
			{
				deltaImpulse = c.m_lowerLimit-c.m_appliedPushImpulse;
				c.m_appliedPushImpulse = c.m_lowerLimit;
			}
			else
			{
				c.m_appliedPushImpulse = sum;
			}
			body1.internalApplyPushImpulse(c.m_contactNormal*body1.m_invMass,c.m_angularComponentA,deltaImpulse);
			body2.internalApplyPushImpulse(-c.m_contactNormal*body2.m_invMass,c.m_angularComponentB,deltaImpulse);
        }
}

 void D_btSequentialImpulseConstraintSolver::resolveSplitPenetrationSIMD(D_btSolverBody& body1,D_btSolverBody& body2,const D_btSolverConstraint& c)
{
#ifdef D_USE_SIMD
	if (!c.m_rhsPenetration)
		return;

	D_gNumSplitImpulseRecoveries++;

	__m128 cpAppliedImp = _mm_set1_ps(c.m_appliedPushImpulse);
	__m128	lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
	__m128	upperLimit1 = _mm_set1_ps(c.m_upperLimit);
	__m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhsPenetration), _mm_mul_ps(_mm_set1_ps(c.m_appliedPushImpulse),_mm_set1_ps(c.m_cfm)));
	__m128 deltaVel1Dotn	=	_mm_add_ps(_vmathVfDot3(c.m_contactNormal.mVec128,body1.m_pushVelocity.mVec128), _vmathVfDot3(c.m_relpos1CrossNormal.mVec128,body1.m_turnVelocity.mVec128));
	__m128 deltaVel2Dotn	=	_mm_sub_ps(_vmathVfDot3(c.m_relpos2CrossNormal.mVec128,body2.m_turnVelocity.mVec128),_vmathVfDot3((c.m_contactNormal).mVec128,body2.m_pushVelocity.mVec128));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel1Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel2Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	D_btSimdScalar sum = _mm_add_ps(cpAppliedImp,deltaImpulse);
	D_btSimdScalar resultLowerLess,resultUpperLess;
	resultLowerLess = _mm_cmplt_ps(sum,lowerLimit1);
	resultUpperLess = _mm_cmplt_ps(sum,upperLimit1);
	__m128 lowMinApplied = _mm_sub_ps(lowerLimit1,cpAppliedImp);
	deltaImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse) );
	c.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum) );
	__m128	linearComponentA = _mm_mul_ps(c.m_contactNormal.mVec128,body1.m_invMass.mVec128);
	__m128	linearComponentB = _mm_mul_ps((c.m_contactNormal).mVec128,body2.m_invMass.mVec128);
	__m128 impulseMagnitude = deltaImpulse;
	body1.m_pushVelocity.mVec128 = _mm_add_ps(body1.m_pushVelocity.mVec128,_mm_mul_ps(linearComponentA,impulseMagnitude));
	body1.m_turnVelocity.mVec128 = _mm_add_ps(body1.m_turnVelocity.mVec128 ,_mm_mul_ps(c.m_angularComponentA.mVec128,impulseMagnitude));
	body2.m_pushVelocity.mVec128 = _mm_sub_ps(body2.m_pushVelocity.mVec128,_mm_mul_ps(linearComponentB,impulseMagnitude));
	body2.m_turnVelocity.mVec128 = _mm_add_ps(body2.m_turnVelocity.mVec128 ,_mm_mul_ps(c.m_angularComponentB.mVec128,impulseMagnitude));
#else
	resolveSplitPenetrationImpulseCacheFriendly(body1,body2,c);
#endif
}



unsigned long D_btSequentialImpulseConstraintSolver::D_btRand2()
{
	m_btSeed2 = (1664525L*m_btSeed2 + 1013904223L) & 0xffffffff;
	return m_btSeed2;
}



//See ODE: adam's all-int straightforward(?) dRandInt (0..n-1)
int D_btSequentialImpulseConstraintSolver::D_btRandInt2 (int n)
{
	// seems good; xor-fold D_and modulus
	const unsigned long un = static_cast<unsigned long>(n);
	unsigned long r = D_btRand2();

	// note: D_probably more aggressive than it needs D_to be -- might be
	//       able D_to get away without one or two of the innermost branches.
	if (un <= 0x00010000UL) {
		r ^= (r >> 16);
		if (un <= 0x00000100UL) {
			r ^= (r >> 8);
			if (un <= 0x00000010UL) {
				r ^= (r >> 4);
				if (un <= 0x00000004UL) {
					r ^= (r >> 2);
					if (un <= 0x00000002UL) {
						r ^= (r >> 1);
					}
				}
			}
		}
	}

	return (int) (r % un);
}



void	D_btSequentialImpulseConstraintSolver::initSolverBody(D_btSolverBody* solverBody, D_btCollisionObject* collisionObject)
{
	D_btRigidBody* rb = collisionObject? D_btRigidBody::upcast(collisionObject) : 0;

	solverBody->m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
	solverBody->m_deltaAngularVelocity.setValue(0.f,0.f,0.f);
	solverBody->m_pushVelocity.setValue(0.f,0.f,0.f);
	solverBody->m_turnVelocity.setValue(0.f,0.f,0.f);

	if (rb)
	{
		solverBody->m_invMass = D_btVector3(rb->getInvMass(),rb->getInvMass(),rb->getInvMass())*rb->getLinearFactor();
		solverBody->m_originalBody = rb;
		solverBody->m_angularFactor = rb->getAngularFactor();
	} else
	{
		solverBody->m_invMass.setValue(0,0,0);
		solverBody->m_originalBody = 0;
		solverBody->m_angularFactor.setValue(1,1,1);
	}
}





D_btScalar D_btSequentialImpulseConstraintSolver::restitutionCurve(D_btScalar rel_vel, D_btScalar restitution)
{
	D_btScalar rest = restitution * -rel_vel;
	return rest;
}



void	applyAnisotropicFriction(D_btCollisionObject* colObj,D_btVector3& frictionDirection);
void	applyAnisotropicFriction(D_btCollisionObject* colObj,D_btVector3& frictionDirection)
{
	if (colObj && colObj->hasAnisotropicFriction())
	{
		// transform D_to local coordinates
		D_btVector3 loc_lateral = frictionDirection * colObj->getWorldTransform().getBasis();
		const D_btVector3& friction_scaling = colObj->getAnisotropicFriction();
		//apply anisotropic friction
		loc_lateral *= friction_scaling;
		// ... D_and transform it back D_to global coordinates
		frictionDirection = colObj->getWorldTransform().getBasis() * loc_lateral;
	}
}



D_btSolverConstraint&	D_btSequentialImpulseConstraintSolver::addFrictionConstraint(const D_btVector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,D_btManifoldPoint& cp,const D_btVector3& rel_pos1,const D_btVector3& rel_pos2,D_btCollisionObject* colObj0,D_btCollisionObject* colObj1, D_btScalar relaxation)
{


	D_btRigidBody* body0=D_btRigidBody::upcast(colObj0);
	D_btRigidBody* body1=D_btRigidBody::upcast(colObj1);

	D_btSolverConstraint& solverConstraint = m_tmpSolverContactFrictionConstraintPool.expand();
	memset(&solverConstraint,0xff,sizeof(D_btSolverConstraint));
	solverConstraint.m_contactNormal = normalAxis;

	solverConstraint.m_solverBodyIdA = solverBodyIdA;
	solverConstraint.m_solverBodyIdB = solverBodyIdB;
	solverConstraint.m_frictionIndex = frictionIndex;

	solverConstraint.m_friction = cp.m_combinedFriction;
	solverConstraint.m_originalContactPoint = 0;

	solverConstraint.m_appliedImpulse = 0.f;
	solverConstraint.m_appliedPushImpulse = 0.f;

	{
		D_btVector3 ftorqueAxis1 = rel_pos1.cross(solverConstraint.m_contactNormal);
		solverConstraint.m_relpos1CrossNormal = ftorqueAxis1;
		solverConstraint.m_angularComponentA = body0 ? body0->getInvInertiaTensorWorld()*ftorqueAxis1*body0->getAngularFactor() : D_btVector3(0,0,0);
	}
	{
		D_btVector3 ftorqueAxis1 = rel_pos2.cross(-solverConstraint.m_contactNormal);
		solverConstraint.m_relpos2CrossNormal = ftorqueAxis1;
		solverConstraint.m_angularComponentB = body1 ? body1->getInvInertiaTensorWorld()*ftorqueAxis1*body1->getAngularFactor() : D_btVector3(0,0,0);
	}

#ifdef COMPUTE_IMPULSE_DENOM
	D_btScalar denom0 = rb0->computeImpulseDenominator(pos1,solverConstraint.m_contactNormal);
	D_btScalar denom1 = rb1->computeImpulseDenominator(pos2,solverConstraint.m_contactNormal);
#else
	D_btVector3 vec;
	D_btScalar denom0 = 0.f;
	D_btScalar denom1 = 0.f;
	if (body0)
	{
		vec = ( solverConstraint.m_angularComponentA).cross(rel_pos1);
		denom0 = body0->getInvMass() + normalAxis.dot(vec);
	}
	if (body1)
	{
		vec = ( -solverConstraint.m_angularComponentB).cross(rel_pos2);
		denom1 = body1->getInvMass() + normalAxis.dot(vec);
	}


#endif //COMPUTE_IMPULSE_DENOM
	D_btScalar denom = relaxation/(denom0+denom1);
	solverConstraint.m_jacDiagABInv = denom;

#ifdef _USE_JACOBIAN
	solverConstraint.m_jac =  D_btJacobianEntry (
		rel_pos1,rel_pos2,solverConstraint.m_contactNormal,
		body0->getInvInertiaDiagLocal(),
		body0->getInvMass(),
		body1->getInvInertiaDiagLocal(),
		body1->getInvMass());
#endif //_USE_JACOBIAN


	{
		D_btScalar rel_vel;
		D_btScalar vel1Dotn = solverConstraint.m_contactNormal.dot(body0?body0->getLinearVelocity():D_btVector3(0,0,0)) 
			+ solverConstraint.m_relpos1CrossNormal.dot(body0?body0->getAngularVelocity():D_btVector3(0,0,0));
		D_btScalar vel2Dotn = -solverConstraint.m_contactNormal.dot(body1?body1->getLinearVelocity():D_btVector3(0,0,0)) 
			+ solverConstraint.m_relpos2CrossNormal.dot(body1?body1->getAngularVelocity():D_btVector3(0,0,0));

		rel_vel = vel1Dotn+vel2Dotn;

//		D_btScalar positionalError = 0.f;

		D_btSimdScalar velocityError =  - rel_vel;
		D_btSimdScalar	velocityImpulse = velocityError * D_btSimdScalar(solverConstraint.m_jacDiagABInv);
		solverConstraint.m_rhs = velocityImpulse;
		solverConstraint.m_cfm = 0.f;
		solverConstraint.m_lowerLimit = 0;
		solverConstraint.m_upperLimit = 1e10f;
	}

	return solverConstraint;
}

int	D_btSequentialImpulseConstraintSolver::getOrInitSolverBody(D_btCollisionObject& body)
{
	int solverBodyIdA = -1;

	if (body.getCompanionId() >= 0)
	{
		//body has already been converted
		solverBodyIdA = body.getCompanionId();
	} else
	{
		D_btRigidBody* rb = D_btRigidBody::upcast(&body);
		if (rb && rb->getInvMass())
		{
			solverBodyIdA = m_tmpSolverBodyPool.size();
			D_btSolverBody& solverBody = m_tmpSolverBodyPool.expand();
			initSolverBody(&solverBody,&body);
			body.setCompanionId(solverBodyIdA);
		} else
		{
			return 0;//assume first one D_is a fixed solver body
		}
	}
	return solverBodyIdA;
}
#include <stdio.h>



void	D_btSequentialImpulseConstraintSolver::convertContact(D_btPersistentManifold* manifold,const D_btContactSolverInfo& infoGlobal)
{
	D_btCollisionObject* colObj0=0,*colObj1=0;

	colObj0 = (D_btCollisionObject*)manifold->getBody0();
	colObj1 = (D_btCollisionObject*)manifold->getBody1();

	int solverBodyIdA=-1;
	int solverBodyIdB=-1;

	if (manifold->getNumContacts())
	{
		solverBodyIdA = getOrInitSolverBody(*colObj0);
		solverBodyIdB = getOrInitSolverBody(*colObj1);
	}

	///avoid collision response between two static objects
	if (!solverBodyIdA && !solverBodyIdB)
		return;

	D_btVector3 rel_pos1;
	D_btVector3 rel_pos2;
	D_btScalar relaxation;

	for (int j=0;j<manifold->getNumContacts();j++)
	{

		D_btManifoldPoint& cp = manifold->getContactPoint(j);

		if (cp.getDistance() <= manifold->getContactProcessingThreshold())
		{

			const D_btVector3& pos1 = cp.getPositionWorldOnA();
			const D_btVector3& pos2 = cp.getPositionWorldOnB();

			rel_pos1 = pos1 - colObj0->getWorldTransform().getOrigin(); 
			rel_pos2 = pos2 - colObj1->getWorldTransform().getOrigin();


			relaxation = 1.f;
			D_btScalar rel_vel;
			D_btVector3 vel;

			int frictionIndex = m_tmpSolverContactConstraintPool.size();

			{
				D_btSolverConstraint& solverConstraint = m_tmpSolverContactConstraintPool.expand();
				D_btRigidBody* rb0 = D_btRigidBody::upcast(colObj0);
				D_btRigidBody* rb1 = D_btRigidBody::upcast(colObj1);

				solverConstraint.m_solverBodyIdA = solverBodyIdA;
				solverConstraint.m_solverBodyIdB = solverBodyIdB;

				solverConstraint.m_originalContactPoint = &cp;

				D_btVector3 torqueAxis0 = rel_pos1.cross(cp.m_normalWorldOnB);
				solverConstraint.m_angularComponentA = rb0 ? rb0->getInvInertiaTensorWorld()*torqueAxis0*rb0->getAngularFactor() : D_btVector3(0,0,0);
				D_btVector3 torqueAxis1 = rel_pos2.cross(cp.m_normalWorldOnB);		
				solverConstraint.m_angularComponentB = rb1 ? rb1->getInvInertiaTensorWorld()*-torqueAxis1*rb1->getAngularFactor() : D_btVector3(0,0,0);
				{
#ifdef COMPUTE_IMPULSE_DENOM
					D_btScalar denom0 = rb0->computeImpulseDenominator(pos1,cp.m_normalWorldOnB);
					D_btScalar denom1 = rb1->computeImpulseDenominator(pos2,cp.m_normalWorldOnB);
#else							
					D_btVector3 vec;
					D_btScalar denom0 = 0.f;
					D_btScalar denom1 = 0.f;
					if (rb0)
					{
						vec = ( solverConstraint.m_angularComponentA).cross(rel_pos1);
						denom0 = rb0->getInvMass() + cp.m_normalWorldOnB.dot(vec);
					}
					if (rb1)
					{
						vec = ( -solverConstraint.m_angularComponentB).cross(rel_pos2);
						denom1 = rb1->getInvMass() + cp.m_normalWorldOnB.dot(vec);
					}
#endif //COMPUTE_IMPULSE_DENOM		

					D_btScalar denom = relaxation/(denom0+denom1);
					solverConstraint.m_jacDiagABInv = denom;
				}

				solverConstraint.m_contactNormal = cp.m_normalWorldOnB;
				solverConstraint.m_relpos1CrossNormal = rel_pos1.cross(cp.m_normalWorldOnB);
				solverConstraint.m_relpos2CrossNormal = rel_pos2.cross(-cp.m_normalWorldOnB);


				D_btVector3 vel1 = rb0 ? rb0->getVelocityInLocalPoint(rel_pos1) : D_btVector3(0,0,0);
				D_btVector3 vel2 = rb1 ? rb1->getVelocityInLocalPoint(rel_pos2) : D_btVector3(0,0,0);

				vel  = vel1 - vel2;

				rel_vel = cp.m_normalWorldOnB.dot(vel);

				D_btScalar penetration = cp.getDistance()+infoGlobal.m_linearSlop;


				solverConstraint.m_friction = cp.m_combinedFriction;

				D_btScalar restitution = 0.f;
				
				if (cp.m_lifeTime>infoGlobal.m_restingContactRestitutionThreshold)
				{
					restitution = 0.f;
				} else
				{
					restitution =  restitutionCurve(rel_vel, cp.m_combinedRestitution);
					if (restitution <= D_btScalar(0.))
					{
						restitution = 0.f;
					};
				}


				///warm starting (or zero if disabled)
				if (infoGlobal.m_solverMode & D_SOLVER_USE_WARMSTARTING)
				{
					solverConstraint.m_appliedImpulse = cp.m_appliedImpulse * infoGlobal.m_warmstartingFactor;
					if (rb0)
						m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdA].applyImpulse(solverConstraint.m_contactNormal*rb0->getInvMass()*rb0->getLinearFactor(),solverConstraint.m_angularComponentA,solverConstraint.m_appliedImpulse);
					if (rb1)
						m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdB].applyImpulse(solverConstraint.m_contactNormal*rb1->getInvMass()*rb1->getLinearFactor(),-solverConstraint.m_angularComponentB,-solverConstraint.m_appliedImpulse);
				} else
				{
					solverConstraint.m_appliedImpulse = 0.f;
				}

				solverConstraint.m_appliedPushImpulse = 0.f;

				{
					D_btScalar rel_vel;
					D_btScalar vel1Dotn = solverConstraint.m_contactNormal.dot(rb0?rb0->getLinearVelocity():D_btVector3(0,0,0)) 
						+ solverConstraint.m_relpos1CrossNormal.dot(rb0?rb0->getAngularVelocity():D_btVector3(0,0,0));
					D_btScalar vel2Dotn = -solverConstraint.m_contactNormal.dot(rb1?rb1->getLinearVelocity():D_btVector3(0,0,0)) 
						+ solverConstraint.m_relpos2CrossNormal.dot(rb1?rb1->getAngularVelocity():D_btVector3(0,0,0));

					rel_vel = vel1Dotn+vel2Dotn;

					D_btScalar positionalError = 0.f;
					positionalError = -penetration * infoGlobal.m_erp/infoGlobal.m_timeStep;
					D_btScalar	velocityError = restitution - rel_vel;// * damping;
					D_btScalar  penetrationImpulse = positionalError*solverConstraint.m_jacDiagABInv;
					D_btScalar velocityImpulse = velocityError *solverConstraint.m_jacDiagABInv;
					if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
					{
						//combine position D_and velocity into rhs
						solverConstraint.m_rhs = penetrationImpulse+velocityImpulse;
						solverConstraint.m_rhsPenetration = 0.f;
					} else
					{
						//split position D_and velocity into rhs D_and m_rhsPenetration
						solverConstraint.m_rhs = velocityImpulse;
						solverConstraint.m_rhsPenetration = penetrationImpulse;
					}
					solverConstraint.m_cfm = 0.f;
					solverConstraint.m_lowerLimit = 0;
					solverConstraint.m_upperLimit = 1e10f;
				}


				/////setup the friction constraints



				if (1)
				{
					solverConstraint.m_frictionIndex = m_tmpSolverContactFrictionConstraintPool.size();
					if (!(infoGlobal.m_solverMode & D_SOLVER_ENABLE_FRICTION_DIRECTION_CACHING) || !cp.m_lateralFrictionInitialized)
					{
						cp.m_lateralFrictionDir1 = vel - cp.m_normalWorldOnB * rel_vel;
						D_btScalar lat_rel_vel = cp.m_lateralFrictionDir1.length2();
						if (!(infoGlobal.m_solverMode & D_SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION) && lat_rel_vel > D_SIMD_EPSILON)
						{
							cp.m_lateralFrictionDir1 /= D_btSqrt(lat_rel_vel);
							if((infoGlobal.m_solverMode & D_SOLVER_USE_2_FRICTION_DIRECTIONS))
							{
								cp.m_lateralFrictionDir2 = cp.m_lateralFrictionDir1.cross(cp.m_normalWorldOnB);
								cp.m_lateralFrictionDir2.normalize();//??
								applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir2);
								applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir2);
								addFrictionConstraint(cp.m_lateralFrictionDir2,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
							}

							applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir1);
							applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir1);
							addFrictionConstraint(cp.m_lateralFrictionDir1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
							cp.m_lateralFrictionInitialized = true;
						} else
						{
							//re-calculate friction direction every frame, todo: check if this D_is really needed
							D_btPlaneSpace1(cp.m_normalWorldOnB,cp.m_lateralFrictionDir1,cp.m_lateralFrictionDir2);
							if ((infoGlobal.m_solverMode & D_SOLVER_USE_2_FRICTION_DIRECTIONS))
							{
								applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir2);
								applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir2);
								addFrictionConstraint(cp.m_lateralFrictionDir2,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
							}

							applyAnisotropicFriction(colObj0,cp.m_lateralFrictionDir1);
							applyAnisotropicFriction(colObj1,cp.m_lateralFrictionDir1);
							addFrictionConstraint(cp.m_lateralFrictionDir1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);

							cp.m_lateralFrictionInitialized = true;
						}

					} else
					{
						addFrictionConstraint(cp.m_lateralFrictionDir1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
						if ((infoGlobal.m_solverMode & D_SOLVER_USE_2_FRICTION_DIRECTIONS))
							addFrictionConstraint(cp.m_lateralFrictionDir2,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
					}

					if (infoGlobal.m_solverMode & D_SOLVER_USE_FRICTION_WARMSTARTING)
					{
						{
							D_btSolverConstraint& frictionConstraint1 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex];
							if (infoGlobal.m_solverMode & D_SOLVER_USE_WARMSTARTING)
							{
								frictionConstraint1.m_appliedImpulse = cp.m_appliedImpulseLateral1 * infoGlobal.m_warmstartingFactor;
								if (rb0)
									m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdA].applyImpulse(frictionConstraint1.m_contactNormal*rb0->getInvMass()*rb0->getLinearFactor(),frictionConstraint1.m_angularComponentA,frictionConstraint1.m_appliedImpulse);
								if (rb1)
									m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdB].applyImpulse(frictionConstraint1.m_contactNormal*rb1->getInvMass()*rb1->getLinearFactor(),-frictionConstraint1.m_angularComponentB,-frictionConstraint1.m_appliedImpulse);
							} else
							{
								frictionConstraint1.m_appliedImpulse = 0.f;
							}
						}

						if ((infoGlobal.m_solverMode & D_SOLVER_USE_2_FRICTION_DIRECTIONS))
						{
							D_btSolverConstraint& frictionConstraint2 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex+1];
							if (infoGlobal.m_solverMode & D_SOLVER_USE_WARMSTARTING)
							{
								frictionConstraint2.m_appliedImpulse = cp.m_appliedImpulseLateral2 * infoGlobal.m_warmstartingFactor;
								if (rb0)
									m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdA].applyImpulse(frictionConstraint2.m_contactNormal*rb0->getInvMass(),frictionConstraint2.m_angularComponentA,frictionConstraint2.m_appliedImpulse);
								if (rb1)
									m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdB].applyImpulse(frictionConstraint2.m_contactNormal*rb1->getInvMass(),-frictionConstraint2.m_angularComponentB,-frictionConstraint2.m_appliedImpulse);
							} else
							{
								frictionConstraint2.m_appliedImpulse = 0.f;
							}
						}
					} else
					{
						D_btSolverConstraint& frictionConstraint1 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex];
						frictionConstraint1.m_appliedImpulse = 0.f;
						if ((infoGlobal.m_solverMode & D_SOLVER_USE_2_FRICTION_DIRECTIONS))
						{
							D_btSolverConstraint& frictionConstraint2 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex+1];
							frictionConstraint2.m_appliedImpulse = 0.f;
						}
					}
				}
			}


		}
	}
}


D_btScalar D_btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySetup(D_btCollisionObject** /*bodies */,int /*numBodies */,D_btPersistentManifold** manifoldPtr, int numManifolds,D_btTypedConstraint** constraints,int numConstraints,const D_btContactSolverInfo& infoGlobal,D_btIDebugDraw* debugDrawer,D_btStackAlloc* stackAlloc)
{
	D_BT_PROFILE("solveGroupCacheFriendlySetup");
	(void)stackAlloc;
	(void)debugDrawer;


	if (!(numConstraints + numManifolds))
	{
		//		printf("empty\n");
		return 0.f;
	}

	if (1)
	{
		int j;
		for (j=0;j<numConstraints;j++)
		{
			D_btTypedConstraint* constraint = constraints[j];
			constraint->buildJacobian();
		}
	}

	D_btSolverBody& fixedBody = m_tmpSolverBodyPool.expand();
	initSolverBody(&fixedBody,0);

	//D_btRigidBody* rb0=0,*rb1=0;

	//if (1)
	{
		{

			int totalNumRows = 0;
			int i;
			
			m_tmpConstraintSizesPool.resize(numConstraints);
			//calculate the total number of contraint rows
			for (i=0;i<numConstraints;i++)
			{
				D_btTypedConstraint::D_btConstraintInfo1& info1 = m_tmpConstraintSizesPool[i];
				constraints[i]->getInfo1(&info1);
				totalNumRows += info1.m_numConstraintRows;
			}
			m_tmpSolverNonContactConstraintPool.resize(totalNumRows);

			
			///setup the btSolverConstraints
			int currentRow = 0;

			for (i=0;i<numConstraints;i++)
			{
				const D_btTypedConstraint::D_btConstraintInfo1& info1 = m_tmpConstraintSizesPool[i];
				
				if (info1.m_numConstraintRows)
				{
					D_btAssert(currentRow<totalNumRows);

					D_btSolverConstraint* currentConstraintRow = &m_tmpSolverNonContactConstraintPool[currentRow];
					D_btTypedConstraint* constraint = constraints[i];



					D_btRigidBody& rbA = constraint->getRigidBodyA();
					D_btRigidBody& rbB = constraint->getRigidBodyB();

					int solverBodyIdA = getOrInitSolverBody(rbA);
					int solverBodyIdB = getOrInitSolverBody(rbB);

					D_btSolverBody* bodyAPtr = &m_tmpSolverBodyPool[solverBodyIdA];
					D_btSolverBody* bodyBPtr = &m_tmpSolverBodyPool[solverBodyIdB];

					int j;
					for ( j=0;j<info1.m_numConstraintRows;j++)
					{
						memset(&currentConstraintRow[j],0,sizeof(D_btSolverConstraint));
						currentConstraintRow[j].m_lowerLimit = -FLT_MAX;
						currentConstraintRow[j].m_upperLimit = FLT_MAX;
						currentConstraintRow[j].m_appliedImpulse = 0.f;
						currentConstraintRow[j].m_appliedPushImpulse = 0.f;
						currentConstraintRow[j].m_solverBodyIdA = solverBodyIdA;
						currentConstraintRow[j].m_solverBodyIdB = solverBodyIdB;
					}

					bodyAPtr->m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
					bodyAPtr->m_deltaAngularVelocity.setValue(0.f,0.f,0.f);
					bodyBPtr->m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
					bodyBPtr->m_deltaAngularVelocity.setValue(0.f,0.f,0.f);



					D_btTypedConstraint::D_btConstraintInfo2 info2;
					info2.fps = 1.f/infoGlobal.m_timeStep;
					info2.erp = infoGlobal.m_erp;
					info2.m_J1linearAxis = currentConstraintRow->m_contactNormal;
					info2.m_J1angularAxis = currentConstraintRow->m_relpos1CrossNormal;
					info2.m_J2linearAxis = 0;
					info2.m_J2angularAxis = currentConstraintRow->m_relpos2CrossNormal;
					info2.rowskip = sizeof(D_btSolverConstraint)/sizeof(D_btScalar);//check this
					///the size of D_btSolverConstraint needs be a multiple of D_btScalar
					D_btAssert(info2.rowskip*sizeof(D_btScalar)== sizeof(D_btSolverConstraint));
					info2.m_constraintError = &currentConstraintRow->m_rhs;
					info2.cfm = &currentConstraintRow->m_cfm;
					info2.m_lowerLimit = &currentConstraintRow->m_lowerLimit;
					info2.m_upperLimit = &currentConstraintRow->m_upperLimit;
					info2.m_numIterations = infoGlobal.m_numIterations;
					constraints[i]->getInfo2(&info2);

					///finalize the constraint setup
					for ( j=0;j<info1.m_numConstraintRows;j++)
					{
						D_btSolverConstraint& solverConstraint = currentConstraintRow[j];

						{
							const D_btVector3& ftorqueAxis1 = solverConstraint.m_relpos1CrossNormal;
							solverConstraint.m_angularComponentA = constraint->getRigidBodyA().getInvInertiaTensorWorld()*ftorqueAxis1*constraint->getRigidBodyA().getAngularFactor();
						}
						{
							const D_btVector3& ftorqueAxis2 = solverConstraint.m_relpos2CrossNormal;
							solverConstraint.m_angularComponentB = constraint->getRigidBodyB().getInvInertiaTensorWorld()*ftorqueAxis2*constraint->getRigidBodyB().getAngularFactor();
						}

						{
							D_btVector3 iMJlA = solverConstraint.m_contactNormal*rbA.getInvMass();
							D_btVector3 iMJaA = rbA.getInvInertiaTensorWorld()*solverConstraint.m_relpos1CrossNormal;
							D_btVector3 iMJlB = solverConstraint.m_contactNormal*rbB.getInvMass();//sign of normal?
							D_btVector3 iMJaB = rbB.getInvInertiaTensorWorld()*solverConstraint.m_relpos2CrossNormal;

							D_btScalar sum = iMJlA.dot(solverConstraint.m_contactNormal);
							sum += iMJaA.dot(solverConstraint.m_relpos1CrossNormal);
							sum += iMJlB.dot(solverConstraint.m_contactNormal);
							sum += iMJaB.dot(solverConstraint.m_relpos2CrossNormal);

							solverConstraint.m_jacDiagABInv = D_btScalar(1.)/sum;
						}


						///fix rhs
						///todo: add force/torque accelerators
						{
							D_btScalar rel_vel;
							D_btScalar vel1Dotn = solverConstraint.m_contactNormal.dot(rbA.getLinearVelocity()) + solverConstraint.m_relpos1CrossNormal.dot(rbA.getAngularVelocity());
							D_btScalar vel2Dotn = -solverConstraint.m_contactNormal.dot(rbB.getLinearVelocity()) + solverConstraint.m_relpos2CrossNormal.dot(rbB.getAngularVelocity());

							rel_vel = vel1Dotn+vel2Dotn;

							D_btScalar restitution = 0.f;
							D_btScalar positionalError = solverConstraint.m_rhs;//already filled in by getConstraintInfo2
							D_btScalar	velocityError = restitution - rel_vel;// * damping;
							D_btScalar	penetrationImpulse = positionalError*solverConstraint.m_jacDiagABInv;
							D_btScalar	velocityImpulse = velocityError *solverConstraint.m_jacDiagABInv;
							solverConstraint.m_rhs = penetrationImpulse+velocityImpulse;
							solverConstraint.m_appliedImpulse = 0.f;

						}
					}
				}
				currentRow+=m_tmpConstraintSizesPool[i].m_numConstraintRows;
			}
		}

		{
			int i;
			D_btPersistentManifold* manifold = 0;
//			D_btCollisionObject* colObj0=0,*colObj1=0;


			for (i=0;i<numManifolds;i++)
			{
				manifold = manifoldPtr[i];
				convertContact(manifold,infoGlobal);
			}
		}
	}

	D_btContactSolverInfo info = infoGlobal;



	int numConstraintPool = m_tmpSolverContactConstraintPool.size();
	int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.size();

	///@todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
	m_orderTmpConstraintPool.resize(numConstraintPool);
	m_orderFrictionConstraintPool.resize(numFrictionPool);
	{
		int i;
		for (i=0;i<numConstraintPool;i++)
		{
			m_orderTmpConstraintPool[i] = i;
		}
		for (i=0;i<numFrictionPool;i++)
		{
			m_orderFrictionConstraintPool[i] = i;
		}
	}

	return 0.f;

}

D_btScalar D_btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyIterations(D_btCollisionObject** /*bodies */,int /*numBodies*/,D_btPersistentManifold** /*manifoldPtr*/, int /*numManifolds*/,D_btTypedConstraint** constraints,int numConstraints,const D_btContactSolverInfo& infoGlobal,D_btIDebugDraw* /*debugDrawer*/,D_btStackAlloc* /*stackAlloc*/)
{
	D_BT_PROFILE("solveGroupCacheFriendlyIterations");

	int numConstraintPool = m_tmpSolverContactConstraintPool.size();
	int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.size();

	//D_should traverse the contacts random order...
	int iteration;
	{
		for ( iteration = 0;iteration<infoGlobal.m_numIterations;iteration++)
		{			

			int j;
			if (infoGlobal.m_solverMode & D_SOLVER_RANDMIZE_ORDER)
			{
				if ((iteration & 7) == 0) {
					for (j=0; j<numConstraintPool; ++j) {
						int tmp = m_orderTmpConstraintPool[j];
						int swapi = D_btRandInt2(j+1);
						m_orderTmpConstraintPool[j] = m_orderTmpConstraintPool[swapi];
						m_orderTmpConstraintPool[swapi] = tmp;
					}

					for (j=0; j<numFrictionPool; ++j) {
						int tmp = m_orderFrictionConstraintPool[j];
						int swapi = D_btRandInt2(j+1);
						m_orderFrictionConstraintPool[j] = m_orderFrictionConstraintPool[swapi];
						m_orderFrictionConstraintPool[swapi] = tmp;
					}
				}
			}

			if (infoGlobal.m_solverMode & D_SOLVER_SIMD)
			{
				///solve all joint constraints, using SIMD, if available
				for (j=0;j<m_tmpSolverNonContactConstraintPool.size();j++)
				{
					D_btSolverConstraint& constraint = m_tmpSolverNonContactConstraintPool[j];
					resolveSingleConstraintRowGenericSIMD(m_tmpSolverBodyPool[constraint.m_solverBodyIdA],m_tmpSolverBodyPool[constraint.m_solverBodyIdB],constraint);
				}

				for (j=0;j<numConstraints;j++)
				{
					int bodyAid = getOrInitSolverBody(constraints[j]->getRigidBodyA());
					int bodyBid = getOrInitSolverBody(constraints[j]->getRigidBodyB());
					D_btSolverBody& bodyA = m_tmpSolverBodyPool[bodyAid];
					D_btSolverBody& bodyB = m_tmpSolverBodyPool[bodyBid];
					constraints[j]->solveConstraintObsolete(bodyA,bodyB,infoGlobal.m_timeStep);
				}

				///solve all contact constraints using SIMD, if available
				int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
				for (j=0;j<numPoolConstraints;j++)
				{
					const D_btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];
					resolveSingleConstraintRowLowerLimitSIMD(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);

				}
				///solve all friction constraints, using SIMD, if available
				int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.size();
				for (j=0;j<numFrictionPoolConstraints;j++)
				{
					D_btSolverConstraint& solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
					D_btScalar totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;

					if (totalImpulse>D_btScalar(0))
					{
						solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
						solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;

						resolveSingleConstraintRowGenericSIMD(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],	m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
					}
				}
			} else
			{

				///solve all joint constraints
				for (j=0;j<m_tmpSolverNonContactConstraintPool.size();j++)
				{
					D_btSolverConstraint& constraint = m_tmpSolverNonContactConstraintPool[j];
					resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[constraint.m_solverBodyIdA],m_tmpSolverBodyPool[constraint.m_solverBodyIdB],constraint);
				}

				for (j=0;j<numConstraints;j++)
				{
					int bodyAid = getOrInitSolverBody(constraints[j]->getRigidBodyA());
					int bodyBid = getOrInitSolverBody(constraints[j]->getRigidBodyB());
					D_btSolverBody& bodyA = m_tmpSolverBodyPool[bodyAid];
					D_btSolverBody& bodyB = m_tmpSolverBodyPool[bodyBid];

					constraints[j]->solveConstraintObsolete(bodyA,bodyB,infoGlobal.m_timeStep);
				}

				///solve all contact constraints
				int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
				for (j=0;j<numPoolConstraints;j++)
				{
					const D_btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];
					resolveSingleConstraintRowLowerLimit(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
				}
				///solve all friction constraints
				int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.size();
				for (j=0;j<numFrictionPoolConstraints;j++)
				{
					D_btSolverConstraint& solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
					D_btScalar totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;

					if (totalImpulse>D_btScalar(0))
					{
						solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
						solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;

						resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],							m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
					}
				}
			}

		}

			if (infoGlobal.m_splitImpulse)
			{
				if (infoGlobal.m_solverMode & D_SOLVER_SIMD)
				{
					for ( iteration = 0;iteration<infoGlobal.m_numIterations;iteration++)
					{
						{
							int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
							int j;
							for (j=0;j<numPoolConstraints;j++)
							{
								const D_btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];

								resolveSplitPenetrationSIMD(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],
									m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
							}
						}
					}
				}
				else
				{
					for ( iteration = 0;iteration<infoGlobal.m_numIterations;iteration++)
					{
						{
							int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
							int j;
							for (j=0;j<numPoolConstraints;j++)
							{
								const D_btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];

								resolveSplitPenetrationImpulseCacheFriendly(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],
									m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
							}
						}
					}
				}
			}
		
	}
	return 0.f;
}



/// D_btSequentialImpulseConstraintSolver Sequentially applies impulses
D_btScalar D_btSequentialImpulseConstraintSolver::solveGroup(D_btCollisionObject** bodies,int numBodies,D_btPersistentManifold** manifoldPtr, int numManifolds,D_btTypedConstraint** constraints,int numConstraints,const D_btContactSolverInfo& infoGlobal,D_btIDebugDraw* debugDrawer,D_btStackAlloc* stackAlloc,D_btDispatcher* /*dispatcher*/)
{

	

	D_BT_PROFILE("solveGroup");
	//we D_only implement D_SOLVER_CACHE_FRIENDLY now
	//you D_need D_to provide at least some bodies
	D_btAssert(bodies);
	D_btAssert(numBodies);

	int i;

	solveGroupCacheFriendlySetup( bodies, numBodies, manifoldPtr,  numManifolds,constraints, numConstraints,infoGlobal,debugDrawer, stackAlloc);
	solveGroupCacheFriendlyIterations(bodies, numBodies, manifoldPtr,  numManifolds,constraints, numConstraints,infoGlobal,debugDrawer, stackAlloc);

	int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
	int j;

	for (j=0;j<numPoolConstraints;j++)
	{

		const D_btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[j];
		D_btManifoldPoint* pt = (D_btManifoldPoint*) solveManifold.m_originalContactPoint;
		D_btAssert(pt);
		pt->m_appliedImpulse = solveManifold.m_appliedImpulse;
		if (infoGlobal.m_solverMode & D_SOLVER_USE_FRICTION_WARMSTARTING)
		{
			pt->m_appliedImpulseLateral1 = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
			pt->m_appliedImpulseLateral2 = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex+1].m_appliedImpulse;
		}

		//do a callback here?
	}

	if (infoGlobal.m_splitImpulse)
	{		
		for ( i=0;i<m_tmpSolverBodyPool.size();i++)
		{
			m_tmpSolverBodyPool[i].writebackVelocity(infoGlobal.m_timeStep);
		}
	} else
	{
		for ( i=0;i<m_tmpSolverBodyPool.size();i++)
		{
			m_tmpSolverBodyPool[i].writebackVelocity();
		}
	}


	m_tmpSolverBodyPool.resize(0);
	m_tmpSolverContactConstraintPool.resize(0);
	m_tmpSolverNonContactConstraintPool.resize(0);
	m_tmpSolverContactFrictionConstraintPool.resize(0);

	return 0.f;
}









void	D_btSequentialImpulseConstraintSolver::reset()
{
	m_btSeed2 = 0;
}


