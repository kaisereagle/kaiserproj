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



#include "btSolve2LinearConstraint.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"


void D_btSolve2LinearConstraint::resolveUnilateralPairConstraint(
												   D_btRigidBody* body1,
		D_btRigidBody* body2,

						const D_btMatrix3x3& world2A,
						const D_btMatrix3x3& world2B,
						
						const D_btVector3& invInertiaADiag,
						const D_btScalar invMassA,
						const D_btVector3& linvelA,const D_btVector3& angvelA,
						const D_btVector3& rel_posA1,
						const D_btVector3& invInertiaBDiag,
						const D_btScalar invMassB,
						const D_btVector3& linvelB,const D_btVector3& angvelB,
						const D_btVector3& rel_posA2,

					  D_btScalar depthA, const D_btVector3& normalA, 
					  const D_btVector3& rel_posB1,const D_btVector3& rel_posB2,
					  D_btScalar depthB, const D_btVector3& normalB, 
					  D_btScalar& imp0,D_btScalar& imp1)
{
	(void)linvelA;
	(void)linvelB;
	(void)angvelB;
	(void)angvelA;



	imp0 = D_btScalar(0.);
	imp1 = D_btScalar(0.);

	D_btScalar len = D_btFabs(normalA.length()) - D_btScalar(1.);
	if (D_btFabs(len) >= D_SIMD_EPSILON)
		return;

	D_btAssert(len < D_SIMD_EPSILON);


	//this jacobian entry could be re-used for all iterations
	D_btJacobianEntry jacA(world2A,world2B,rel_posA1,rel_posA2,normalA,invInertiaADiag,invMassA,
		invInertiaBDiag,invMassB);
	D_btJacobianEntry jacB(world2A,world2B,rel_posB1,rel_posB2,normalB,invInertiaADiag,invMassA,
		invInertiaBDiag,invMassB);
	
	//const D_btScalar vel0 = jacA.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);
	//const D_btScalar vel1 = jacB.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);

	const D_btScalar vel0 = normalA.dot(body1->getVelocityInLocalPoint(rel_posA1)-body2->getVelocityInLocalPoint(rel_posA1));
	const D_btScalar vel1 = normalB.dot(body1->getVelocityInLocalPoint(rel_posB1)-body2->getVelocityInLocalPoint(rel_posB1));

//	D_btScalar penetrationImpulse = (depth*contactTau*timeCorrection)  * massTerm;//jacDiagABInv
	D_btScalar massTerm = D_btScalar(1.) / (invMassA + invMassB);


	// calculate rhs (or error) terms
	const D_btScalar dv0 = depthA  * m_tau * massTerm - vel0 * m_damping;
	const D_btScalar dv1 = depthB  * m_tau * massTerm - vel1 * m_damping;


	// dC/dv * dv = -C
	
	// jacobian * impulse = -error
	//

	//impulse = jacobianInverse * -error

	// inverting 2x2 symmetric system (offdiagonal D_are equal!)
	// 


	D_btScalar nonDiag = jacA.getNonDiagonal(jacB,invMassA,invMassB);
	D_btScalar	invDet = D_btScalar(1.0) / (jacA.getDiagonal() * jacB.getDiagonal() - nonDiag * nonDiag );
	
	//imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
	//imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

	imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
	imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

	//[a b]								  [d -c]
	//[c d] inverse = (1 / determinant) * [-b a] where determinant D_is (ad - bc)

	//[jA nD] * [imp0] = [dv0]
	//[nD jB]   [imp1]   [dv1]

}



void D_btSolve2LinearConstraint::resolveBilateralPairConstraint(
						D_btRigidBody* body1,
						D_btRigidBody* body2,
						const D_btMatrix3x3& world2A,
						const D_btMatrix3x3& world2B,
						
						const D_btVector3& invInertiaADiag,
						const D_btScalar invMassA,
						const D_btVector3& linvelA,const D_btVector3& angvelA,
						const D_btVector3& rel_posA1,
						const D_btVector3& invInertiaBDiag,
						const D_btScalar invMassB,
						const D_btVector3& linvelB,const D_btVector3& angvelB,
						const D_btVector3& rel_posA2,

					  D_btScalar depthA, const D_btVector3& normalA, 
					  const D_btVector3& rel_posB1,const D_btVector3& rel_posB2,
					  D_btScalar depthB, const D_btVector3& normalB, 
					  D_btScalar& imp0,D_btScalar& imp1)
{

	(void)linvelA;
	(void)linvelB;
	(void)angvelA;
	(void)angvelB;



	imp0 = D_btScalar(0.);
	imp1 = D_btScalar(0.);

	D_btScalar len = D_btFabs(normalA.length()) - D_btScalar(1.);
	if (D_btFabs(len) >= D_SIMD_EPSILON)
		return;

	D_btAssert(len < D_SIMD_EPSILON);


	//this jacobian entry could be re-used for all iterations
	D_btJacobianEntry jacA(world2A,world2B,rel_posA1,rel_posA2,normalA,invInertiaADiag,invMassA,
		invInertiaBDiag,invMassB);
	D_btJacobianEntry jacB(world2A,world2B,rel_posB1,rel_posB2,normalB,invInertiaADiag,invMassA,
		invInertiaBDiag,invMassB);
	
	//const D_btScalar vel0 = jacA.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);
	//const D_btScalar vel1 = jacB.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);

	const D_btScalar vel0 = normalA.dot(body1->getVelocityInLocalPoint(rel_posA1)-body2->getVelocityInLocalPoint(rel_posA1));
	const D_btScalar vel1 = normalB.dot(body1->getVelocityInLocalPoint(rel_posB1)-body2->getVelocityInLocalPoint(rel_posB1));

	// calculate rhs (or error) terms
	const D_btScalar dv0 = depthA  * m_tau - vel0 * m_damping;
	const D_btScalar dv1 = depthB  * m_tau - vel1 * m_damping;

	// dC/dv * dv = -C
	
	// jacobian * impulse = -error
	//

	//impulse = jacobianInverse * -error

	// inverting 2x2 symmetric system (offdiagonal D_are equal!)
	// 


	D_btScalar nonDiag = jacA.getNonDiagonal(jacB,invMassA,invMassB);
	D_btScalar	invDet = D_btScalar(1.0) / (jacA.getDiagonal() * jacB.getDiagonal() - nonDiag * nonDiag );
	
	//imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
	//imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

	imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
	imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

	//[a b]								  [d -c]
	//[c d] inverse = (1 / determinant) * [-b a] where determinant D_is (ad - bc)

	//[jA nD] * [imp0] = [dv0]
	//[nD jB]   [imp1]   [dv1]

	if ( imp0 > D_btScalar(0.0))
	{
		if ( imp1 > D_btScalar(0.0) )
		{
			//both positive
		}
		else
		{
			imp1 = D_btScalar(0.);

			// now imp0>0 imp1<0
			imp0 = dv0 / jacA.getDiagonal();
			if ( imp0 > D_btScalar(0.0) )
			{
			} else
			{
				imp0 = D_btScalar(0.);
			}
		}
	}
	else
	{
		imp0 = D_btScalar(0.);

		imp1 = dv1 / jacB.getDiagonal();
		if ( imp1 <= D_btScalar(0.0) )
		{
			imp1 = D_btScalar(0.);
			// now imp0>0 imp1<0
			imp0 = dv0 / jacA.getDiagonal();
			if ( imp0 > D_btScalar(0.0) )
			{
			} else
			{
				imp0 = D_btScalar(0.);
			}
		} else
		{
		}
	}
}


/*
void D_btSolve2LinearConstraint::resolveAngularConstraint(	const D_btMatrix3x3& invInertiaAWS,
											const D_btScalar invMassA,
											const D_btVector3& linvelA,const D_btVector3& angvelA,
											const D_btVector3& rel_posA1,
											const D_btMatrix3x3& invInertiaBWS,
											const D_btScalar invMassB,
											const D_btVector3& linvelB,const D_btVector3& angvelB,
											const D_btVector3& rel_posA2,

											D_btScalar depthA, const D_btVector3& normalA, 
											const D_btVector3& rel_posB1,const D_btVector3& rel_posB2,
											D_btScalar depthB, const D_btVector3& normalB, 
											D_btScalar& imp0,D_btScalar& imp1)
{

}
*/

