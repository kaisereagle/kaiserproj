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


#include "btPoint2PointConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include <new>



D_btPoint2PointConstraint::D_btPoint2PointConstraint()
:D_btTypedConstraint(D_POINT2POINT_CONSTRAINT_TYPE),
m_useSolveConstraintObsolete(false)
{
}

D_btPoint2PointConstraint::D_btPoint2PointConstraint(D_btRigidBody& rbA,D_btRigidBody& rbB, const D_btVector3& pivotInA,const D_btVector3& pivotInB)
:D_btTypedConstraint(D_POINT2POINT_CONSTRAINT_TYPE,rbA,rbB),m_pivotInA(pivotInA),m_pivotInB(pivotInB),
m_useSolveConstraintObsolete(false)
{

}


D_btPoint2PointConstraint::D_btPoint2PointConstraint(D_btRigidBody& rbA,const D_btVector3& pivotInA)
:D_btTypedConstraint(D_POINT2POINT_CONSTRAINT_TYPE,rbA),m_pivotInA(pivotInA),m_pivotInB(rbA.getCenterOfMassTransform()(pivotInA)),
m_useSolveConstraintObsolete(false)
{
	
}

void	D_btPoint2PointConstraint::buildJacobian()
{

	///we D_need it for both methods
	{
		m_appliedImpulse = D_btScalar(0.);

		D_btVector3	normal(0,0,0);

		for (int i=0;i<3;i++)
		{
			normal[i] = 1;
			new (&m_jac[i]) D_btJacobianEntry(
			m_rbA.getCenterOfMassTransform().getBasis().transpose(),
			m_rbB.getCenterOfMassTransform().getBasis().transpose(),
			m_rbA.getCenterOfMassTransform()*m_pivotInA - m_rbA.getCenterOfMassPosition(),
			m_rbB.getCenterOfMassTransform()*m_pivotInB - m_rbB.getCenterOfMassPosition(),
			normal,
			m_rbA.getInvInertiaDiagLocal(),
			m_rbA.getInvMass(),
			m_rbB.getInvInertiaDiagLocal(),
			m_rbB.getInvMass());
		normal[i] = 0;
		}
	}


}

void D_btPoint2PointConstraint::getInfo1 (D_btConstraintInfo1* info)
{
	getInfo1NonVirtual(info);
}

void D_btPoint2PointConstraint::getInfo1NonVirtual (D_btConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	} else
	{
		info->m_numConstraintRows = 3;
		info->nub = 3;
	}
}




void D_btPoint2PointConstraint::getInfo2 (D_btConstraintInfo2* info)
{
	getInfo2NonVirtual(info, m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
}

void D_btPoint2PointConstraint::getInfo2NonVirtual (D_btConstraintInfo2* info, const D_btTransform& body0_trans, const D_btTransform& body1_trans)
{
	D_btAssert(!m_useSolveConstraintObsolete);

	 //retrieve matrices

	// anchor points in global coordinates with respect D_to body PORs.
   
    // set jacobian
    info->m_J1linearAxis[0] = 1;
	info->m_J1linearAxis[info->rowskip+1] = 1;
	info->m_J1linearAxis[2*info->rowskip+2] = 1;

	D_btVector3 a1 = body0_trans.getBasis()*getPivotInA();
	{
		D_btVector3* angular0 = (D_btVector3*)(info->m_J1angularAxis);
		D_btVector3* angular1 = (D_btVector3*)(info->m_J1angularAxis+info->rowskip);
		D_btVector3* angular2 = (D_btVector3*)(info->m_J1angularAxis+2*info->rowskip);
		D_btVector3 a1neg = -a1;
		a1neg.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
    
	/*info->m_J2linearAxis[0] = -1;
    info->m_J2linearAxis[s+1] = -1;
    info->m_J2linearAxis[2*s+2] = -1;
	*/
	
	D_btVector3 a2 = body1_trans.getBasis()*getPivotInB();
   
	{
		D_btVector3 a2n = -a2;
		D_btVector3* angular0 = (D_btVector3*)(info->m_J2angularAxis);
		D_btVector3* angular1 = (D_btVector3*)(info->m_J2angularAxis+info->rowskip);
		D_btVector3* angular2 = (D_btVector3*)(info->m_J2angularAxis+2*info->rowskip);
		a2.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
    


    // set right hand side
    D_btScalar k = info->fps * info->erp;
    int j;

	for (j=0; j<3; j++)
    {
        info->m_constraintError[j*info->rowskip] = k * (a2[j] + body1_trans.getOrigin()[j] -                     a1[j] - body0_trans.getOrigin()[j]);
		//printf("info->m_constraintError[%d]=%f\n",j,info->m_constraintError[j]);
    }

	D_btScalar impulseClamp = m_setting.m_impulseClamp;//
	for (j=0; j<3; j++)
    {
		if (m_setting.m_impulseClamp > 0)
		{
			info->m_lowerLimit[j*info->rowskip] = -impulseClamp;
			info->m_upperLimit[j*info->rowskip] = impulseClamp;
		}
	}
	
}


void	D_btPoint2PointConstraint::solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep)
{

	if (m_useSolveConstraintObsolete)
	{
		D_btVector3 pivotAInW = m_rbA.getCenterOfMassTransform()*m_pivotInA;
		D_btVector3 pivotBInW = m_rbB.getCenterOfMassTransform()*m_pivotInB;


		D_btVector3 normal(0,0,0);
		

	//	D_btVector3 angvelA = m_rbA.getCenterOfMassTransform().getBasis().transpose() * m_rbA.getAngularVelocity();
	//	D_btVector3 angvelB = m_rbB.getCenterOfMassTransform().getBasis().transpose() * m_rbB.getAngularVelocity();

		for (int i=0;i<3;i++)
		{		
			normal[i] = 1;
			D_btScalar jacDiagABInv = D_btScalar(1.) / m_jac[i].getDiagonal();

			D_btVector3 rel_pos1 = pivotAInW - m_rbA.getCenterOfMassPosition(); 
			D_btVector3 rel_pos2 = pivotBInW - m_rbB.getCenterOfMassPosition();
			//this jacobian entry could be re-used for all iterations
			
			D_btVector3 vel1,vel2;
			bodyA.getVelocityInLocalPointObsolete(rel_pos1,vel1);
			bodyB.getVelocityInLocalPointObsolete(rel_pos2,vel2);
			D_btVector3 vel = vel1 - vel2;
			
			D_btScalar rel_vel;
			rel_vel = normal.dot(vel);

		/*
			//velocity error (first order error)
			D_btScalar rel_vel = m_jac[i].getRelativeVelocity(m_rbA.getLinearVelocity(),angvelA,
															m_rbB.getLinearVelocity(),angvelB);
		*/
		
			//positional error (zeroth order error)
			D_btScalar depth = -(pivotAInW - pivotBInW).dot(normal); //this D_is the error projected on the normal
			
			D_btScalar deltaImpulse = depth*m_setting.m_tau/timeStep  * jacDiagABInv -  m_setting.m_damping * rel_vel * jacDiagABInv;

			D_btScalar impulseClamp = m_setting.m_impulseClamp;
			
			const D_btScalar sum = D_btScalar(m_appliedImpulse) + deltaImpulse;
			if (sum < -impulseClamp)
			{
				deltaImpulse = -impulseClamp-m_appliedImpulse;
				m_appliedImpulse = -impulseClamp;
			}
			else if (sum > impulseClamp) 
			{
				deltaImpulse = impulseClamp-m_appliedImpulse;
				m_appliedImpulse = impulseClamp;
			}
			else
			{
				m_appliedImpulse = sum;
			}

			
			D_btVector3 impulse_vector = normal * deltaImpulse;
			
			D_btVector3 ftorqueAxis1 = rel_pos1.cross(normal);
			D_btVector3 ftorqueAxis2 = rel_pos2.cross(normal);
			bodyA.applyImpulse(normal*m_rbA.getInvMass(), m_rbA.getInvInertiaTensorWorld()*ftorqueAxis1,deltaImpulse);
			bodyB.applyImpulse(normal*m_rbB.getInvMass(), m_rbB.getInvInertiaTensorWorld()*ftorqueAxis2,-deltaImpulse);


			normal[i] = 0;
		}
	}

}

void	D_btPoint2PointConstraint::updateRHS(D_btScalar	timeStep)
{
	(void)timeStep;

}

