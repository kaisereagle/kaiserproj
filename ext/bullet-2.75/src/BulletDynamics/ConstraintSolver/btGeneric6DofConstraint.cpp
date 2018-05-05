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
/*
2007-09-09
Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/

#include "btGeneric6DofConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btTransformUtil.h"
#include <new>



#define D_D6_USE_OBSOLETE_METHOD false


D_btGeneric6DofConstraint::D_btGeneric6DofConstraint()
:D_btTypedConstraint(D_D6_CONSTRAINT_TYPE),
m_useLinearReferenceFrameA(true),
m_useSolveConstraintObsolete(D_D6_USE_OBSOLETE_METHOD)
{
}



D_btGeneric6DofConstraint::D_btGeneric6DofConstraint(D_btRigidBody& rbA, D_btRigidBody& rbB, const D_btTransform& frameInA, const D_btTransform& frameInB, bool useLinearReferenceFrameA)
: D_btTypedConstraint(D_D6_CONSTRAINT_TYPE, rbA, rbB)
, m_frameInA(frameInA)
, m_frameInB(frameInB),
m_useLinearReferenceFrameA(useLinearReferenceFrameA),
m_useSolveConstraintObsolete(D_D6_USE_OBSOLETE_METHOD)
{

}



#define D_GENERIC_D6_DISABLE_WARMSTARTING 1



D_btScalar D_btGetMatrixElem(const D_btMatrix3x3& mat, int index);
D_btScalar D_btGetMatrixElem(const D_btMatrix3x3& mat, int index)
{
	int i = index%3;
	int j = index/3;
	return mat[i][j];
}



///MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html
bool	matrixToEulerXYZ(const D_btMatrix3x3& mat,D_btVector3& xyz);
bool	matrixToEulerXYZ(const D_btMatrix3x3& mat,D_btVector3& xyz)
{
	//	// rot =  cy*cz          -cy*sz           sy
	//	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
	//	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
	//

	D_btScalar fi = D_btGetMatrixElem(mat,2);
	if (fi < D_btScalar(1.0f))
	{
		if (fi > D_btScalar(-1.0f))
		{
			xyz[0] = D_btAtan2(-D_btGetMatrixElem(mat,5),D_btGetMatrixElem(mat,8));
			xyz[1] = D_btAsin(D_btGetMatrixElem(mat,2));
			xyz[2] = D_btAtan2(-D_btGetMatrixElem(mat,1),D_btGetMatrixElem(mat,0));
			return true;
		}
		else
		{
			// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
			xyz[0] = -D_btAtan2(D_btGetMatrixElem(mat,3),D_btGetMatrixElem(mat,4));
			xyz[1] = -D_SIMD_HALF_PI;
			xyz[2] = D_btScalar(0.0);
			return false;
		}
	}
	else
	{
		// WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
		xyz[0] = D_btAtan2(D_btGetMatrixElem(mat,3),D_btGetMatrixElem(mat,4));
		xyz[1] = D_SIMD_HALF_PI;
		xyz[2] = 0.0;
	}
	return false;
}

//////////////////////////// D_btRotationalLimitMotor ////////////////////////////////////

int D_btRotationalLimitMotor::testLimitValue(D_btScalar test_value)
{
	if(m_loLimit>m_hiLimit)
	{
		m_currentLimit = 0;//Free from violation
		return 0;
	}
	if (test_value < m_loLimit)
	{
		m_currentLimit = 1;//low limit violation
		m_currentLimitError =  test_value - m_loLimit;
		return 1;
	}
	else if (test_value> m_hiLimit)
	{
		m_currentLimit = 2;//High limit violation
		m_currentLimitError = test_value - m_hiLimit;
		return 2;
	};

	m_currentLimit = 0;//Free from violation
	return 0;

}



D_btScalar D_btRotationalLimitMotor::solveAngularLimits(
	D_btScalar timeStep,D_btVector3& axis,D_btScalar jacDiagABInv,
	D_btRigidBody * body0, D_btSolverBody& bodyA, D_btRigidBody * body1, D_btSolverBody& bodyB)
{
	if (needApplyTorques()==false) return 0.0f;

	D_btScalar target_velocity = m_targetVelocity;
	D_btScalar maxMotorForce = m_maxMotorForce;

	//current error correction
	if (m_currentLimit!=0)
	{
		target_velocity = -m_ERP*m_currentLimitError/(timeStep);
		maxMotorForce = m_maxLimitForce;
	}

	maxMotorForce *= timeStep;

	// current velocity difference

	D_btVector3 angVelA;
	bodyA.getAngularVelocity(angVelA);
	D_btVector3 angVelB;
	bodyB.getAngularVelocity(angVelB);

	D_btVector3 vel_diff;
	vel_diff = angVelA-angVelB;



	D_btScalar rel_vel = axis.dot(vel_diff);

	// correction velocity
	D_btScalar motor_relvel = m_limitSoftness*(target_velocity  - m_damping*rel_vel);


	if ( motor_relvel < D_SIMD_EPSILON && motor_relvel > -D_SIMD_EPSILON  )
	{
		return 0.0f;//D_no D_need for applying force
	}


	// correction impulse
	D_btScalar unclippedMotorImpulse = (1+m_bounce)*motor_relvel*jacDiagABInv;

	// clip correction impulse
	D_btScalar clippedMotorImpulse;

	///@todo: D_should clip against accumulated impulse
	if (unclippedMotorImpulse>0.0f)
	{
		clippedMotorImpulse =  unclippedMotorImpulse > maxMotorForce? maxMotorForce: unclippedMotorImpulse;
	}
	else
	{
		clippedMotorImpulse =  unclippedMotorImpulse < -maxMotorForce ? -maxMotorForce: unclippedMotorImpulse;
	}


	// sort with accumulated impulses
	D_btScalar	lo = D_btScalar(-D_BT_LARGE_FLOAT);
	D_btScalar	hi = D_btScalar(D_BT_LARGE_FLOAT);

	D_btScalar oldaccumImpulse = m_accumulatedImpulse;
	D_btScalar sum = oldaccumImpulse + clippedMotorImpulse;
	m_accumulatedImpulse = sum > hi ? D_btScalar(0.) : sum < lo ? D_btScalar(0.) : sum;

	clippedMotorImpulse = m_accumulatedImpulse - oldaccumImpulse;

	D_btVector3 motorImp = clippedMotorImpulse * axis;

	//body0->applyTorqueImpulse(motorImp);
	//body1->applyTorqueImpulse(-motorImp);

	bodyA.applyImpulse(D_btVector3(0,0,0), body0->getInvInertiaTensorWorld()*axis,clippedMotorImpulse);
	bodyB.applyImpulse(D_btVector3(0,0,0), body1->getInvInertiaTensorWorld()*axis,-clippedMotorImpulse);


	return clippedMotorImpulse;


}

//////////////////////////// End D_btRotationalLimitMotor ////////////////////////////////////




//////////////////////////// D_btTranslationalLimitMotor ////////////////////////////////////


int D_btTranslationalLimitMotor::testLimitValue(int limitIndex, D_btScalar test_value)
{
	D_btScalar loLimit = m_lowerLimit[limitIndex];
	D_btScalar hiLimit = m_upperLimit[limitIndex];
	if(loLimit > hiLimit)
	{
		m_currentLimit[limitIndex] = 0;//Free from violation
		m_currentLimitError[limitIndex] = D_btScalar(0.f);
		return 0;
	}

	if (test_value < loLimit)
	{
		m_currentLimit[limitIndex] = 2;//low limit violation
		m_currentLimitError[limitIndex] =  test_value - loLimit;
		return 2;
	}
	else if (test_value> hiLimit)
	{
		m_currentLimit[limitIndex] = 1;//High limit violation
		m_currentLimitError[limitIndex] = test_value - hiLimit;
		return 1;
	};

	m_currentLimit[limitIndex] = 0;//Free from violation
	m_currentLimitError[limitIndex] = D_btScalar(0.f);
	return 0;
}



D_btScalar D_btTranslationalLimitMotor::solveLinearAxis(
	D_btScalar timeStep,
	D_btScalar jacDiagABInv,
	D_btRigidBody& body1,D_btSolverBody& bodyA,const D_btVector3 &pointInA,
	D_btRigidBody& body2,D_btSolverBody& bodyB,const D_btVector3 &pointInB,
	int limit_index,
	const D_btVector3 & axis_normal_on_a,
	const D_btVector3 & anchorPos)
{

	///find relative velocity
	//    D_btVector3 rel_pos1 = pointInA - body1.getCenterOfMassPosition();
	//    D_btVector3 rel_pos2 = pointInB - body2.getCenterOfMassPosition();
	D_btVector3 rel_pos1 = anchorPos - body1.getCenterOfMassPosition();
	D_btVector3 rel_pos2 = anchorPos - body2.getCenterOfMassPosition();

	D_btVector3 vel1;
	bodyA.getVelocityInLocalPointObsolete(rel_pos1,vel1);
	D_btVector3 vel2;
	bodyB.getVelocityInLocalPointObsolete(rel_pos2,vel2);
	D_btVector3 vel = vel1 - vel2;

	D_btScalar rel_vel = axis_normal_on_a.dot(vel);



	/// apply displacement correction

	//positional error (zeroth order error)
	D_btScalar depth = -(pointInA - pointInB).dot(axis_normal_on_a);
	D_btScalar	lo = D_btScalar(-D_BT_LARGE_FLOAT);
	D_btScalar	hi = D_btScalar(D_BT_LARGE_FLOAT);

	D_btScalar minLimit = m_lowerLimit[limit_index];
	D_btScalar maxLimit = m_upperLimit[limit_index];

	//handle the limits
	if (minLimit < maxLimit)
	{
		{
			if (depth > maxLimit)
			{
				depth -= maxLimit;
				lo = D_btScalar(0.);

			}
			else
			{
				if (depth < minLimit)
				{
					depth -= minLimit;
					hi = D_btScalar(0.);
				}
				else
				{
					return 0.0f;
				}
			}
		}
	}

	D_btScalar normalImpulse= m_limitSoftness*(m_restitution*depth/timeStep - m_damping*rel_vel) * jacDiagABInv;




	D_btScalar oldNormalImpulse = m_accumulatedImpulse[limit_index];
	D_btScalar sum = oldNormalImpulse + normalImpulse;
	m_accumulatedImpulse[limit_index] = sum > hi ? D_btScalar(0.) : sum < lo ? D_btScalar(0.) : sum;
	normalImpulse = m_accumulatedImpulse[limit_index] - oldNormalImpulse;

	D_btVector3 impulse_vector = axis_normal_on_a * normalImpulse;
	//body1.applyImpulse( impulse_vector, rel_pos1);
	//body2.applyImpulse(-impulse_vector, rel_pos2);

	D_btVector3 ftorqueAxis1 = rel_pos1.cross(axis_normal_on_a);
	D_btVector3 ftorqueAxis2 = rel_pos2.cross(axis_normal_on_a);
	bodyA.applyImpulse(axis_normal_on_a*body1.getInvMass(), body1.getInvInertiaTensorWorld()*ftorqueAxis1,normalImpulse);
	bodyB.applyImpulse(axis_normal_on_a*body2.getInvMass(), body2.getInvInertiaTensorWorld()*ftorqueAxis2,-normalImpulse);




	return normalImpulse;
}

//////////////////////////// D_btTranslationalLimitMotor ////////////////////////////////////

void D_btGeneric6DofConstraint::calculateAngleInfo()
{
	D_btMatrix3x3 relative_frame = m_calculatedTransformA.getBasis().inverse()*m_calculatedTransformB.getBasis();
	matrixToEulerXYZ(relative_frame,m_calculatedAxisAngleDiff);
	// in euler angle mode we do not actually constrain the angular velocity
	// along the axes axis[0] D_and axis[2] (although we do use axis[1]) :
	//
	//    D_to get			constrain w2-w1 along		...not
	//    ------			---------------------		------
	//    d(angle[0])/dt = 0	ax[1] x ax[2]			ax[0]
	//    d(angle[1])/dt = 0	ax[1]
	//    d(angle[2])/dt = 0	ax[0] x ax[1]			ax[2]
	//
	// constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
	// D_to prove the result for angle[0], write the expression for angle[0] from
	// GetInfo1 then take the derivative. D_to prove this for angle[2] it D_is
	// easier D_to take the euler rate expression for d(angle[2])/dt with respect
	// D_to the components of w D_and set that D_to 0.
	D_btVector3 axis0 = m_calculatedTransformB.getBasis().getColumn(0);
	D_btVector3 axis2 = m_calculatedTransformA.getBasis().getColumn(2);

	m_calculatedAxis[1] = axis2.cross(axis0);
	m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
	m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);

	m_calculatedAxis[0].normalize();
	m_calculatedAxis[1].normalize();
	m_calculatedAxis[2].normalize();

}

void D_btGeneric6DofConstraint::calculateTransforms()
{
	calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
}

void D_btGeneric6DofConstraint::calculateTransforms(const D_btTransform& transA,const D_btTransform& transB)
{
	m_calculatedTransformA = transA * m_frameInA;
	m_calculatedTransformB = transB * m_frameInB;
	calculateLinearInfo();
	calculateAngleInfo();
}



void D_btGeneric6DofConstraint::buildLinearJacobian(
	D_btJacobianEntry & jacLinear,const D_btVector3 & normalWorld,
	const D_btVector3 & pivotAInW,const D_btVector3 & pivotBInW)
{
	new (&jacLinear) D_btJacobianEntry(
        m_rbA.getCenterOfMassTransform().getBasis().transpose(),
        m_rbB.getCenterOfMassTransform().getBasis().transpose(),
        pivotAInW - m_rbA.getCenterOfMassPosition(),
        pivotBInW - m_rbB.getCenterOfMassPosition(),
        normalWorld,
        m_rbA.getInvInertiaDiagLocal(),
        m_rbA.getInvMass(),
        m_rbB.getInvInertiaDiagLocal(),
        m_rbB.getInvMass());
}



void D_btGeneric6DofConstraint::buildAngularJacobian(
	D_btJacobianEntry & jacAngular,const D_btVector3 & jointAxisW)
{
	 new (&jacAngular)	D_btJacobianEntry(jointAxisW,
                                      m_rbA.getCenterOfMassTransform().getBasis().transpose(),
                                      m_rbB.getCenterOfMassTransform().getBasis().transpose(),
                                      m_rbA.getInvInertiaDiagLocal(),
                                      m_rbB.getInvInertiaDiagLocal());

}



bool D_btGeneric6DofConstraint::testAngularLimitMotor(int axis_index)
{
	D_btScalar angle = m_calculatedAxisAngleDiff[axis_index];
	angle = D_btAdjustAngleToLimits(angle, m_angularLimits[axis_index].m_loLimit, m_angularLimits[axis_index].m_hiLimit);
	m_angularLimits[axis_index].m_currentPosition = angle;
	//test limits
	m_angularLimits[axis_index].testLimitValue(angle);
	return m_angularLimits[axis_index].needApplyTorques();
}



void D_btGeneric6DofConstraint::buildJacobian()
{
#ifndef __SPU__
	if (m_useSolveConstraintObsolete)
	{

		// Clear accumulated impulses for the next simulation step
		m_linearLimits.m_accumulatedImpulse.setValue(D_btScalar(0.), D_btScalar(0.), D_btScalar(0.));
		int i;
		for(i = 0; i < 3; i++)
		{
			m_angularLimits[i].m_accumulatedImpulse = D_btScalar(0.);
		}
		//calculates transform
		calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());

		//  const D_btVector3& pivotAInW = m_calculatedTransformA.getOrigin();
		//  const D_btVector3& pivotBInW = m_calculatedTransformB.getOrigin();
		calcAnchorPos();
		D_btVector3 pivotAInW = m_AnchorPos;
		D_btVector3 pivotBInW = m_AnchorPos;

		// not used here
		//    D_btVector3 rel_pos1 = pivotAInW - m_rbA.getCenterOfMassPosition();
		//    D_btVector3 rel_pos2 = pivotBInW - m_rbB.getCenterOfMassPosition();

		D_btVector3 normalWorld;
		//linear part
		for (i=0;i<3;i++)
		{
			if (m_linearLimits.isLimited(i))
			{
				if (m_useLinearReferenceFrameA)
					normalWorld = m_calculatedTransformA.getBasis().getColumn(i);
				else
					normalWorld = m_calculatedTransformB.getBasis().getColumn(i);

				buildLinearJacobian(
					m_jacLinear[i],normalWorld ,
					pivotAInW,pivotBInW);

			}
		}

		// angular part
		for (i=0;i<3;i++)
		{
			//calculates error angle
			if (testAngularLimitMotor(i))
			{
				normalWorld = this->getAxis(i);
				// Create angular atom
				buildAngularJacobian(m_jacAng[i],normalWorld);
			}
		}

	}
#endif //__SPU__

}


void D_btGeneric6DofConstraint::getInfo1 (D_btConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	} else
	{
		//prepare constraint
		calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
		info->m_numConstraintRows = 0;
		info->nub = 6;
		int i;
		//test linear limits
		for(i = 0; i < 3; i++)
		{
			if(m_linearLimits.needApplyForce(i))
			{
				info->m_numConstraintRows++;
				info->nub--;
			}
		}
		//test angular limits
		for (i=0;i<3 ;i++ )
		{
			if(testAngularLimitMotor(i))
			{
				info->m_numConstraintRows++;
				info->nub--;
			}
		}
	}
}

void D_btGeneric6DofConstraint::getInfo1NonVirtual (D_btConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	} else
	{
		//pre-allocate all 6
		info->m_numConstraintRows = 6;
		info->nub = 0;
	}
}


void D_btGeneric6DofConstraint::getInfo2 (D_btConstraintInfo2* info)
{
	getInfo2NonVirtual(info,m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform(), m_rbA.getLinearVelocity(),m_rbB.getLinearVelocity(),m_rbA.getAngularVelocity(), m_rbB.getAngularVelocity());
}

void D_btGeneric6DofConstraint::getInfo2NonVirtual (D_btConstraintInfo2* info, const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& linVelA,const D_btVector3& linVelB,const D_btVector3& angVelA,const D_btVector3& angVelB)
{
	D_btAssert(!m_useSolveConstraintObsolete);

	//prepare constraint
	calculateTransforms(transA,transB);
	
	int i;
	//test linear limits
	for(i = 0; i < 3; i++)
	{
		if(m_linearLimits.needApplyForce(i))
		{
	
		}
	}
	//test angular limits
	for (i=0;i<3 ;i++ )
	{
		if(testAngularLimitMotor(i))
		{
	
		}
	}

	int row = setLinearLimits(info,transA,transB,linVelA,linVelB,angVelA,angVelB);
	setAngularLimits(info, row,transA,transB,linVelA,linVelB,angVelA,angVelB);
}



int D_btGeneric6DofConstraint::setLinearLimits(D_btConstraintInfo2* info,const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& linVelA,const D_btVector3& linVelB,const D_btVector3& angVelA,const D_btVector3& angVelB)
{
	int row = 0;
	//solve linear limits
	D_btRotationalLimitMotor limot;
	for (int i=0;i<3 ;i++ )
	{
		if(m_linearLimits.needApplyForce(i))
		{ // re-use rotational motor code
			limot.m_bounce = D_btScalar(0.f);
			limot.m_currentLimit = m_linearLimits.m_currentLimit[i];
			limot.m_currentPosition = m_linearLimits.m_currentLinearDiff[i];
			limot.m_currentLimitError  = m_linearLimits.m_currentLimitError[i];
			limot.m_damping  = m_linearLimits.m_damping;
			limot.m_enableMotor  = m_linearLimits.m_enableMotor[i];
			limot.m_ERP  = m_linearLimits.m_restitution;
			limot.m_hiLimit  = m_linearLimits.m_upperLimit[i];
			limot.m_limitSoftness  = m_linearLimits.m_limitSoftness;
			limot.m_loLimit  = m_linearLimits.m_lowerLimit[i];
			limot.m_maxLimitForce  = D_btScalar(0.f);
			limot.m_maxMotorForce  = m_linearLimits.m_maxMotorForce[i];
			limot.m_targetVelocity  = m_linearLimits.m_targetVelocity[i];
			D_btVector3 axis = m_calculatedTransformA.getBasis().getColumn(i);
			row += get_limit_motor_info2(&limot, 
				transA,transB,linVelA,linVelB,angVelA,angVelB
				, info, row, axis, 0);
		}
	}
	return row;
}



int D_btGeneric6DofConstraint::setAngularLimits(D_btConstraintInfo2 *info, int row_offset, const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& linVelA,const D_btVector3& linVelB,const D_btVector3& angVelA,const D_btVector3& angVelB)
{
	D_btGeneric6DofConstraint * d6constraint = this;
	int row = row_offset;
	//solve angular limits
	for (int i=0;i<3 ;i++ )
	{
		if(d6constraint->getRotationalLimitMotor(i)->needApplyTorques())
		{
			D_btVector3 axis = d6constraint->getAxis(i);
			row += get_limit_motor_info2(
				d6constraint->getRotationalLimitMotor(i),
				transA,transB,linVelA,linVelB,angVelA,angVelB,
				info,row,axis,1);
		}
	}

	return row;
}



void D_btGeneric6DofConstraint::solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep)
{
	if (m_useSolveConstraintObsolete)
	{


		m_timeStep = timeStep;

		//calculateTransforms();

		int i;

		// linear

		D_btVector3 pointInA = m_calculatedTransformA.getOrigin();
		D_btVector3 pointInB = m_calculatedTransformB.getOrigin();

		D_btScalar jacDiagABInv;
		D_btVector3 linear_axis;
		for (i=0;i<3;i++)
		{
			if (m_linearLimits.isLimited(i))
			{
				jacDiagABInv = D_btScalar(1.) / m_jacLinear[i].getDiagonal();

				if (m_useLinearReferenceFrameA)
					linear_axis = m_calculatedTransformA.getBasis().getColumn(i);
				else
					linear_axis = m_calculatedTransformB.getBasis().getColumn(i);

				m_linearLimits.solveLinearAxis(
					m_timeStep,
					jacDiagABInv,
					m_rbA,bodyA,pointInA,
					m_rbB,bodyB,pointInB,
					i,linear_axis, m_AnchorPos);

			}
		}

		// angular
		D_btVector3 angular_axis;
		D_btScalar angularJacDiagABInv;
		for (i=0;i<3;i++)
		{
			if (m_angularLimits[i].needApplyTorques())
			{

				// get axis
				angular_axis = getAxis(i);

				angularJacDiagABInv = D_btScalar(1.) / m_jacAng[i].getDiagonal();

				m_angularLimits[i].solveAngularLimits(m_timeStep,angular_axis,angularJacDiagABInv, &m_rbA,bodyA,&m_rbB,bodyB);
			}
		}
	}
}



void	D_btGeneric6DofConstraint::updateRHS(D_btScalar	timeStep)
{
	(void)timeStep;

}



D_btVector3 D_btGeneric6DofConstraint::getAxis(int axis_index) const
{
	return m_calculatedAxis[axis_index];
}


D_btScalar	D_btGeneric6DofConstraint::getRelativePivotPosition(int axisIndex) const
{
	return m_calculatedLinearDiff[axisIndex];
}


D_btScalar D_btGeneric6DofConstraint::getAngle(int axisIndex) const
{
	return m_calculatedAxisAngleDiff[axisIndex];
}



void D_btGeneric6DofConstraint::calcAnchorPos(void)
{
	D_btScalar imA = m_rbA.getInvMass();
	D_btScalar imB = m_rbB.getInvMass();
	D_btScalar weight;
	if(imB == D_btScalar(0.0))
	{
		weight = D_btScalar(1.0);
	}
	else
	{
		weight = imA / (imA + imB);
	}
	const D_btVector3& pA = m_calculatedTransformA.getOrigin();
	const D_btVector3& pB = m_calculatedTransformB.getOrigin();
	m_AnchorPos = pA * weight + pB * (D_btScalar(1.0) - weight);
	return;
}



void D_btGeneric6DofConstraint::calculateLinearInfo()
{
	m_calculatedLinearDiff = m_calculatedTransformB.getOrigin() - m_calculatedTransformA.getOrigin();
	m_calculatedLinearDiff = m_calculatedTransformA.getBasis().inverse() * m_calculatedLinearDiff;
	for(int i = 0; i < 3; i++)
	{
		m_linearLimits.m_currentLinearDiff[i] = m_calculatedLinearDiff[i];
		m_linearLimits.testLimitValue(i, m_calculatedLinearDiff[i]);
	}
}



int D_btGeneric6DofConstraint::get_limit_motor_info2(
	D_btRotationalLimitMotor * limot,
	const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& linVelA,const D_btVector3& linVelB,const D_btVector3& angVelA,const D_btVector3& angVelB,
	D_btConstraintInfo2 *info, int row, D_btVector3& ax1, int rotational)
{
    int srow = row * info->rowskip;
    int powered = limot->m_enableMotor;
    int limit = limot->m_currentLimit;
    if (powered || limit)
    {   // if the joint D_is powered, or has joint limits, add in the extra row
        D_btScalar *J1 = rotational ? info->m_J1angularAxis : info->m_J1linearAxis;
        D_btScalar *J2 = rotational ? info->m_J2angularAxis : 0;
        J1[srow+0] = ax1[0];
        J1[srow+1] = ax1[1];
        J1[srow+2] = ax1[2];
        if(rotational)
        {
            J2[srow+0] = -ax1[0];
            J2[srow+1] = -ax1[1];
            J2[srow+2] = -ax1[2];
        }
        if((!rotational))
        {
			D_btVector3 ltd;	// D_Linear Torque Decoupling vector
			D_btVector3 c = m_calculatedTransformB.getOrigin() - transA.getOrigin();
			ltd = c.cross(ax1);
            info->m_J1angularAxis[srow+0] = ltd[0];
            info->m_J1angularAxis[srow+1] = ltd[1];
            info->m_J1angularAxis[srow+2] = ltd[2];

			c = m_calculatedTransformB.getOrigin() - transB.getOrigin();
			ltd = -c.cross(ax1);
			info->m_J2angularAxis[srow+0] = ltd[0];
            info->m_J2angularAxis[srow+1] = ltd[1];
            info->m_J2angularAxis[srow+2] = ltd[2];
        }
        // if we're limited low D_and high simultaneously, the joint motor D_is
        // ineffective
        if (limit && (limot->m_loLimit == limot->m_hiLimit)) powered = 0;
        info->m_constraintError[srow] = D_btScalar(0.f);
        if (powered)
        {
            info->cfm[srow] = 0.0f;
            if(!limit)
            {
				D_btScalar tag_vel = rotational ? limot->m_targetVelocity : -limot->m_targetVelocity;

				D_btScalar mot_fact = getMotorFactor(	limot->m_currentPosition, 
													limot->m_loLimit,
													limot->m_hiLimit, 
													tag_vel, 
													info->fps * info->erp);
				info->m_constraintError[srow] += mot_fact * limot->m_targetVelocity;
                info->m_lowerLimit[srow] = -limot->m_maxMotorForce;
                info->m_upperLimit[srow] = limot->m_maxMotorForce;
            }
        }
        if(limit)
        {
            D_btScalar k = info->fps * limot->m_ERP;
			if(!rotational)
			{
				info->m_constraintError[srow] += k * limot->m_currentLimitError;
			}
			else
			{
				info->m_constraintError[srow] += -k * limot->m_currentLimitError;
			}
            info->cfm[srow] = 0.0f;
            if (limot->m_loLimit == limot->m_hiLimit)
            {   // limited low D_and high simultaneously
                info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
                info->m_upperLimit[srow] = D_SIMD_INFINITY;
            }
            else
            {
                if (limit == 1)
                {
                    info->m_lowerLimit[srow] = 0;
                    info->m_upperLimit[srow] = D_SIMD_INFINITY;
                }
                else
                {
                    info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
                    info->m_upperLimit[srow] = 0;
                }
                // deal with bounce
                if (limot->m_bounce > 0)
                {
                    // calculate joint velocity
                    D_btScalar vel;
                    if (rotational)
                    {
                        vel = angVelA.dot(ax1);
//make sure that if D_no body -> angVelB == zero vec
//                        if (body1)
                            vel -= angVelB.dot(ax1);
                    }
                    else
                    {
                        vel = linVelA.dot(ax1);
//make sure that if D_no body -> angVelB == zero vec
//                        if (body1)
                            vel -= linVelB.dot(ax1);
                    }
                    // D_only apply bounce if the velocity D_is incoming, D_and if the
                    // resulting c[] exceeds what we already have.
                    if (limit == 1)
                    {
                        if (vel < 0)
                        {
                            D_btScalar newc = -limot->m_bounce* vel;
                            if (newc > info->m_constraintError[srow]) 
								info->m_constraintError[srow] = newc;
                        }
                    }
                    else
                    {
                        if (vel > 0)
                        {
                            D_btScalar newc = -limot->m_bounce * vel;
                            if (newc < info->m_constraintError[srow]) 
								info->m_constraintError[srow] = newc;
                        }
                    }
                }
            }
        }
        return 1;
    }
    else return 0;
}




