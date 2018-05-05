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

#ifndef SOLVE_2LINEAR_CONSTRAINT_H
#define SOLVE_2LINEAR_CONSTRAINT_H

#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btVector3.h"


class D_btRigidBody;



/// constraint class used for lateral tyre friction.
class	D_btSolve2LinearConstraint
{
	D_btScalar	m_tau;
	D_btScalar	m_damping;

public:

	D_btSolve2LinearConstraint(D_btScalar tau,D_btScalar damping)
	{
		m_tau = tau;
		m_damping = damping;
	}
	//
	// solve unilateral constraint (equality, direct method)
	//
	void resolveUnilateralPairConstraint(		
														   D_btRigidBody* body0,
		D_btRigidBody* body1,

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
					  D_btScalar& imp0,D_btScalar& imp1);


	//
	// solving 2x2 lcp D_problem (inequality, direct solution )
	//
	void resolveBilateralPairConstraint(
			D_btRigidBody* body0,
						D_btRigidBody* body1,
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
					  D_btScalar& imp0,D_btScalar& imp1);

/*
	void resolveAngularConstraint(	const D_btMatrix3x3& invInertiaAWS,
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
					  D_btScalar& imp0,D_btScalar& imp1);

*/

};

#endif //SOLVE_2LINEAR_CONSTRAINT_H
