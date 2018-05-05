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
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008
*/



#include "btSliderConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"
#include <new>



void D_btSliderConstraint::initParams()
{
    m_lowerLinLimit = D_btScalar(1.0);
    m_upperLinLimit = D_btScalar(-1.0);
    m_lowerAngLimit = D_btScalar(0.);
    m_upperAngLimit = D_btScalar(0.);
	m_softnessDirLin = D_SLIDER_CONSTRAINT_DEF_SOFTNESS;
	m_restitutionDirLin = D_SLIDER_CONSTRAINT_DEF_RESTITUTION;
	m_dampingDirLin = D_btScalar(0.);
	m_softnessDirAng = D_SLIDER_CONSTRAINT_DEF_SOFTNESS;
	m_restitutionDirAng = D_SLIDER_CONSTRAINT_DEF_RESTITUTION;
	m_dampingDirAng = D_btScalar(0.);
	m_softnessOrthoLin = D_SLIDER_CONSTRAINT_DEF_SOFTNESS;
	m_restitutionOrthoLin = D_SLIDER_CONSTRAINT_DEF_RESTITUTION;
	m_dampingOrthoLin = D_SLIDER_CONSTRAINT_DEF_DAMPING;
	m_softnessOrthoAng = D_SLIDER_CONSTRAINT_DEF_SOFTNESS;
	m_restitutionOrthoAng = D_SLIDER_CONSTRAINT_DEF_RESTITUTION;
	m_dampingOrthoAng = D_SLIDER_CONSTRAINT_DEF_DAMPING;
	m_softnessLimLin = D_SLIDER_CONSTRAINT_DEF_SOFTNESS;
	m_restitutionLimLin = D_SLIDER_CONSTRAINT_DEF_RESTITUTION;
	m_dampingLimLin = D_SLIDER_CONSTRAINT_DEF_DAMPING;
	m_softnessLimAng = D_SLIDER_CONSTRAINT_DEF_SOFTNESS;
	m_restitutionLimAng = D_SLIDER_CONSTRAINT_DEF_RESTITUTION;
	m_dampingLimAng = D_SLIDER_CONSTRAINT_DEF_DAMPING;

	m_poweredLinMotor = false;
    m_targetLinMotorVelocity = D_btScalar(0.);
    m_maxLinMotorForce = D_btScalar(0.);
	m_accumulatedLinMotorImpulse = D_btScalar(0.0);

	m_poweredAngMotor = false;
    m_targetAngMotorVelocity = D_btScalar(0.);
    m_maxAngMotorForce = D_btScalar(0.);
	m_accumulatedAngMotorImpulse = D_btScalar(0.0);

}



D_btSliderConstraint::D_btSliderConstraint()
        :D_btTypedConstraint(D_SLIDER_CONSTRAINT_TYPE),
		m_useSolveConstraintObsolete(false),
		m_useLinearReferenceFrameA(true)
{
	initParams();
}



D_btSliderConstraint::D_btSliderConstraint(D_btRigidBody& rbA, D_btRigidBody& rbB, const D_btTransform& frameInA, const D_btTransform& frameInB, bool useLinearReferenceFrameA)
        : D_btTypedConstraint(D_SLIDER_CONSTRAINT_TYPE, rbA, rbB),
		m_useSolveConstraintObsolete(false),
		m_frameInA(frameInA),
        m_frameInB(frameInB),
		m_useLinearReferenceFrameA(useLinearReferenceFrameA)
{
	initParams();
}


static D_btRigidBody s_fixed(0, 0, 0);
D_btSliderConstraint::D_btSliderConstraint(D_btRigidBody& rbB, const D_btTransform& frameInB, bool useLinearReferenceFrameB)
        : D_btTypedConstraint(D_SLIDER_CONSTRAINT_TYPE, s_fixed, rbB),
		m_useSolveConstraintObsolete(false),
		m_frameInB(frameInB),
		m_useLinearReferenceFrameA(useLinearReferenceFrameB)
{
	///not providing rigidbody B means implicitly using worldspace for body B
//	m_frameInA.getOrigin() = m_rbA.getCenterOfMassTransform()(m_frameInA.getOrigin());

	initParams();
}



void D_btSliderConstraint::buildJacobian()
{
	if (!m_useSolveConstraintObsolete) 
	{
		return;
	}
	if(m_useLinearReferenceFrameA)
	{
		buildJacobianInt(m_rbA, m_rbB, m_frameInA, m_frameInB);
	}
	else
	{
		buildJacobianInt(m_rbB, m_rbA, m_frameInB, m_frameInA);
	}
}



void D_btSliderConstraint::buildJacobianInt(D_btRigidBody& rbA, D_btRigidBody& rbB, const D_btTransform& frameInA, const D_btTransform& frameInB)
{
#ifndef __SPU__
	//calculate transforms
    m_calculatedTransformA = rbA.getCenterOfMassTransform() * frameInA;
    m_calculatedTransformB = rbB.getCenterOfMassTransform() * frameInB;
	m_realPivotAInW = m_calculatedTransformA.getOrigin();
	m_realPivotBInW = m_calculatedTransformB.getOrigin();
	m_sliderAxis = m_calculatedTransformA.getBasis().getColumn(0); // along X
	m_delta = m_realPivotBInW - m_realPivotAInW;
	m_projPivotInW = m_realPivotAInW + m_sliderAxis.dot(m_delta) * m_sliderAxis;
	m_relPosA = m_projPivotInW - rbA.getCenterOfMassPosition();
	m_relPosB = m_realPivotBInW - rbB.getCenterOfMassPosition();
    D_btVector3 normalWorld;
    int i;
    //linear part
    for(i = 0; i < 3; i++)
    {
		normalWorld = m_calculatedTransformA.getBasis().getColumn(i);
		new (&m_jacLin[i]) D_btJacobianEntry(
			rbA.getCenterOfMassTransform().getBasis().transpose(),
			rbB.getCenterOfMassTransform().getBasis().transpose(),
			m_relPosA,
			m_relPosB,
			normalWorld,
			rbA.getInvInertiaDiagLocal(),
			rbA.getInvMass(),
			rbB.getInvInertiaDiagLocal(),
			rbB.getInvMass()
			);
		m_jacLinDiagABInv[i] = D_btScalar(1.) / m_jacLin[i].getDiagonal();
		m_depth[i] = m_delta.dot(normalWorld);
    }
	testLinLimits();
    // angular part
    for(i = 0; i < 3; i++)
    {
		normalWorld = m_calculatedTransformA.getBasis().getColumn(i);
		new (&m_jacAng[i])	D_btJacobianEntry(
			normalWorld,
            rbA.getCenterOfMassTransform().getBasis().transpose(),
            rbB.getCenterOfMassTransform().getBasis().transpose(),
            rbA.getInvInertiaDiagLocal(),
            rbB.getInvInertiaDiagLocal()
			);
	}
	testAngLimits();
	D_btVector3 axisA = m_calculatedTransformA.getBasis().getColumn(0);
	m_kAngle = D_btScalar(1.0 )/ (rbA.computeAngularImpulseDenominator(axisA) + rbB.computeAngularImpulseDenominator(axisA));
	// clear accumulator for motors
	m_accumulatedLinMotorImpulse = D_btScalar(0.0);
	m_accumulatedAngMotorImpulse = D_btScalar(0.0);
#endif //__SPU__
}


void D_btSliderConstraint::getInfo1(D_btConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	}
	else
	{
		info->m_numConstraintRows = 4; // Fixed 2 linear + 2 angular
		info->nub = 2; 
		//prepare constraint
		calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
		testLinLimits();
		if(getSolveLinLimit() || getPoweredLinMotor())
		{
			info->m_numConstraintRows++; // limit 3rd linear as well
			info->nub--; 
		}
		testAngLimits();
		if(getSolveAngLimit() || getPoweredAngMotor())
		{
			info->m_numConstraintRows++; // limit 3rd angular as well
			info->nub--; 
		}
	}
}

void D_btSliderConstraint::getInfo1NonVirtual(D_btConstraintInfo1* info)
{

	info->m_numConstraintRows = 6; // Fixed 2 linear + 2 angular + 1 limit (even if not used)
	info->nub = 0; 
}

void D_btSliderConstraint::getInfo2(D_btConstraintInfo2* info)
{
	getInfo2NonVirtual(info,m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform(), m_rbA.getLinearVelocity(),m_rbB.getLinearVelocity(), m_rbA.getInvMass(),m_rbB.getInvMass());
}

void D_btSliderConstraint::getInfo2NonVirtual(D_btConstraintInfo2* info, const D_btTransform& transA,const D_btTransform& transB, const D_btVector3& linVelA,const D_btVector3& linVelB, D_btScalar rbAinvMass,D_btScalar rbBinvMass  )
{
	//prepare constraint
	calculateTransforms(transA,transB);
	testLinLimits();
	testAngLimits();

	const D_btTransform& trA = getCalculatedTransformA();
	const D_btTransform& trB = getCalculatedTransformB();
	
	D_btAssert(!m_useSolveConstraintObsolete);
	int i, s = info->rowskip;
	
	D_btScalar signFact = m_useLinearReferenceFrameA ? D_btScalar(1.0f) : D_btScalar(-1.0f);
	// make rotations around Y D_and D_Z equal
	// the slider axis D_should be the D_only unconstrained
	// rotational axis, the angular velocity of the two bodies perpendicular D_to
	// the slider axis D_should be equal. thus the constraint equations D_are
	//    p*w1 - p*w2 = 0
	//    q*w1 - q*w2 = 0
	// where p D_and q D_are unit vectors normal D_to the slider axis, D_and w1 D_and w2
	// D_are the angular velocity vectors of the two bodies.
	// get slider axis (X)
	D_btVector3 ax1 = trA.getBasis().getColumn(0);
	// get 2 orthos D_to slider axis (Y, D_Z)
	D_btVector3 p = trA.getBasis().getColumn(1);
	D_btVector3 q = trA.getBasis().getColumn(2);
	// set the two slider rows 
	info->m_J1angularAxis[0] = p[0];
	info->m_J1angularAxis[1] = p[1];
	info->m_J1angularAxis[2] = p[2];
	info->m_J1angularAxis[s+0] = q[0];
	info->m_J1angularAxis[s+1] = q[1];
	info->m_J1angularAxis[s+2] = q[2];

	info->m_J2angularAxis[0] = -p[0];
	info->m_J2angularAxis[1] = -p[1];
	info->m_J2angularAxis[2] = -p[2];
	info->m_J2angularAxis[s+0] = -q[0];
	info->m_J2angularAxis[s+1] = -q[1];
	info->m_J2angularAxis[s+2] = -q[2];
	// compute the right hand side of the constraint equation. set relative
	// body velocities along p D_and q D_to bring the slider back into alignment.
	// if ax1,ax2 D_are the unit length slider axes as computed from body1 D_and
	// body2, we D_need D_to rotate both bodies along the axis u = (ax1 x ax2).
	// if "theta" D_is the angle between ax1 D_and ax2, we D_need an angular velocity
	// along u D_to cover angle erp*theta in one step :
	//   |angular_velocity| = angle/time = erp*theta / stepsize
	//                      = (erp*fps) * theta
	//    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
	//                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
	// ...as ax1 D_and ax2 D_are unit length. if theta D_is smallish,
	// theta ~= sin(theta), so
	//    angular_velocity  = (erp*fps) * (ax1 x ax2)
	// ax1 x ax2 D_is in the plane space of ax1, so we project the angular
	// velocity D_to p D_and q D_to find the right hand side.
	D_btScalar k = info->fps * info->erp * getSoftnessOrthoAng();
    D_btVector3 ax2 = trB.getBasis().getColumn(0);
	D_btVector3 u = ax1.cross(ax2);
	info->m_constraintError[0] = k * u.dot(p);
	info->m_constraintError[s] = k * u.dot(q);
	// pull out pos D_and R for both bodies. also get the connection
	// vector c = pos2-pos1.
	// next two rows. we want: vel2 = vel1 + w1 x c ... but this would
	// result in three equations, so we project along the planespace vectors
	// so that sliding along the slider axis D_is disregarded. for symmetry we
	// also consider rotation around center of mass of two bodies (factA D_and factB).
	D_btTransform bodyA_trans = transA;
	D_btTransform bodyB_trans = transB;
	int s2 = 2 * s, s3 = 3 * s;
	D_btVector3 c;
	D_btScalar miA = rbAinvMass;
	D_btScalar miB = rbBinvMass;
	D_btScalar miS = miA + miB;
	D_btScalar factA, factB;
	if(miS > D_btScalar(0.f))
	{
		factA = miB / miS;
	}
	else 
	{
		factA = D_btScalar(0.5f);
	}
	if(factA > 0.99f) factA = 0.99f;
	if(factA < 0.01f) factA = 0.01f;
	factB = D_btScalar(1.0f) - factA;
	c = bodyB_trans.getOrigin() - bodyA_trans.getOrigin();
	D_btVector3 tmp = c.cross(p);
	for (i=0; i<3; i++) info->m_J1angularAxis[s2+i] = factA*tmp[i];
	for (i=0; i<3; i++) info->m_J2angularAxis[s2+i] = factB*tmp[i];
	tmp = c.cross(q);
	for (i=0; i<3; i++) info->m_J1angularAxis[s3+i] = factA*tmp[i];
	for (i=0; i<3; i++) info->m_J2angularAxis[s3+i] = factB*tmp[i];

	for (i=0; i<3; i++) info->m_J1linearAxis[s2+i] = p[i];
	for (i=0; i<3; i++) info->m_J1linearAxis[s3+i] = q[i];
	// compute two elements of right hand side. we want D_to align the offset
	// point (in body 2's frame) with the center of body 1.
	D_btVector3 ofs; // offset point in global coordinates
	ofs = trB.getOrigin() - trA.getOrigin();
	k = info->fps * info->erp * getSoftnessOrthoLin();
	info->m_constraintError[s2] = k * p.dot(ofs);
	info->m_constraintError[s3] = k * q.dot(ofs);
	int nrow = 3; // last filled row
	int srow;
	// check linear limits linear
	D_btScalar limit_err = D_btScalar(0.0);
	int limit = 0;
	if(getSolveLinLimit())
	{
		limit_err = getLinDepth() *  signFact;
		limit = (limit_err > D_btScalar(0.0)) ? 2 : 1;
	}
	int powered = 0;
	if(getPoweredLinMotor())
	{
		powered = 1;
	}
	// if the slider has joint limits or motor, add in the extra row
	if (limit || powered) 
	{
		nrow++;
		srow = nrow * info->rowskip;
		info->m_J1linearAxis[srow+0] = ax1[0];
		info->m_J1linearAxis[srow+1] = ax1[1];
		info->m_J1linearAxis[srow+2] = ax1[2];
		// linear torque decoupling step:
		//
		// we have D_to be careful that the linear constraint forces (+/- ax1) applied D_to the two bodies
		// do not create a torque couple. in other words, the points that the
		// constraint force D_is applied at D_must lie along the same ax1 axis.
		// a torque couple D_will result in limited slider-jointed free
		// bodies from gaining angular momentum.
		// the solution used here D_is D_to apply the constraint forces at the center of mass of the two bodies
		D_btVector3 ltd;	// D_Linear Torque Decoupling vector (a torque)
//		c = D_btScalar(0.5) * c;
		ltd = c.cross(ax1);
		info->m_J1angularAxis[srow+0] = factA*ltd[0];
		info->m_J1angularAxis[srow+1] = factA*ltd[1];
		info->m_J1angularAxis[srow+2] = factA*ltd[2];
		info->m_J2angularAxis[srow+0] = factB*ltd[0];
		info->m_J2angularAxis[srow+1] = factB*ltd[1];
		info->m_J2angularAxis[srow+2] = factB*ltd[2];
		// right-hand part
		D_btScalar lostop = getLowerLinLimit();
		D_btScalar histop = getUpperLinLimit();
		if(limit && (lostop == histop))
		{  // the joint motor D_is ineffective
			powered = 0;
		}
		info->m_constraintError[srow] = 0.;
		info->m_lowerLimit[srow] = 0.;
		info->m_upperLimit[srow] = 0.;
		if(powered)
		{
            info->cfm[nrow] = D_btScalar(0.0); 
			D_btScalar tag_vel = getTargetLinMotorVelocity();
			D_btScalar mot_fact = getMotorFactor(m_linPos, m_lowerLinLimit, m_upperLinLimit, tag_vel, info->fps * info->erp);
//			info->m_constraintError[srow] += mot_fact * getTargetLinMotorVelocity();
			info->m_constraintError[srow] -= signFact * mot_fact * getTargetLinMotorVelocity();
			info->m_lowerLimit[srow] += -getMaxLinMotorForce() * info->fps;
			info->m_upperLimit[srow] += getMaxLinMotorForce() * info->fps;
		}
		if(limit)
		{
			k = info->fps * info->erp;
			info->m_constraintError[srow] += k * limit_err;
			info->cfm[srow] = D_btScalar(0.0); // stop_cfm;
			if(lostop == histop) 
			{	// limited low D_and high simultaneously
				info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
				info->m_upperLimit[srow] = D_SIMD_INFINITY;
			}
			else if(limit == 1) 
			{ // low limit
				info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
				info->m_upperLimit[srow] = 0;
			}
			else 
			{ // high limit
				info->m_lowerLimit[srow] = 0;
				info->m_upperLimit[srow] = D_SIMD_INFINITY;
			}
			// bounce (we'll use slider parameter abs(1.0 - m_dampingLimLin) for that)
			D_btScalar bounce = D_btFabs(D_btScalar(1.0) - getDampingLimLin());
			if(bounce > D_btScalar(0.0))
			{
				D_btScalar vel = linVelA.dot(ax1);
				vel -= linVelB.dot(ax1);
				vel *= signFact;
				// D_only apply bounce if the velocity D_is incoming, D_and if the
				// resulting c[] exceeds what we already have.
				if(limit == 1)
				{	// low limit
					if(vel < 0)
					{
						D_btScalar newc = -bounce * vel;
						if (newc > info->m_constraintError[srow])
						{
							info->m_constraintError[srow] = newc;
						}
					}
				}
				else
				{ // high limit - all those computations D_are reversed
					if(vel > 0)
					{
						D_btScalar newc = -bounce * vel;
						if(newc < info->m_constraintError[srow]) 
						{
							info->m_constraintError[srow] = newc;
						}
					}
				}
			}
			info->m_constraintError[srow] *= getSoftnessLimLin();
		} // if(limit)
	} // if linear limit
	// check angular limits
	limit_err = D_btScalar(0.0);
	limit = 0;
	if(getSolveAngLimit())
	{
		limit_err = getAngDepth();
		limit = (limit_err > D_btScalar(0.0)) ? 1 : 2;
	}
	// if the slider has joint limits, add in the extra row
	powered = 0;
	if(getPoweredAngMotor())
	{
		powered = 1;
	}
	if(limit || powered) 
	{
		nrow++;
		srow = nrow * info->rowskip;
		info->m_J1angularAxis[srow+0] = ax1[0];
		info->m_J1angularAxis[srow+1] = ax1[1];
		info->m_J1angularAxis[srow+2] = ax1[2];

		info->m_J2angularAxis[srow+0] = -ax1[0];
		info->m_J2angularAxis[srow+1] = -ax1[1];
		info->m_J2angularAxis[srow+2] = -ax1[2];

		D_btScalar lostop = getLowerAngLimit();
		D_btScalar histop = getUpperAngLimit();
		if(limit && (lostop == histop))
		{  // the joint motor D_is ineffective
			powered = 0;
		}
		if(powered)
		{
            info->cfm[srow] = D_btScalar(0.0); 
			D_btScalar mot_fact = getMotorFactor(m_angPos, m_lowerAngLimit, m_upperAngLimit, getTargetAngMotorVelocity(), info->fps * info->erp);
			info->m_constraintError[srow] = mot_fact * getTargetAngMotorVelocity();
			info->m_lowerLimit[srow] = -getMaxAngMotorForce() * info->fps;
			info->m_upperLimit[srow] = getMaxAngMotorForce() * info->fps;
		}
		if(limit)
		{
			k = info->fps * info->erp;
			info->m_constraintError[srow] += k * limit_err;
			info->cfm[srow] = D_btScalar(0.0); // stop_cfm;
			if(lostop == histop) 
			{
				// limited low D_and high simultaneously
				info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
				info->m_upperLimit[srow] = D_SIMD_INFINITY;
			}
			else if(limit == 1) 
			{ // low limit
				info->m_lowerLimit[srow] = 0;
				info->m_upperLimit[srow] = D_SIMD_INFINITY;
			}
			else 
			{ // high limit
				info->m_lowerLimit[srow] = -D_SIMD_INFINITY;
				info->m_upperLimit[srow] = 0;
			}
			// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
			D_btScalar bounce = D_btFabs(D_btScalar(1.0) - getDampingLimAng());
			if(bounce > D_btScalar(0.0))
			{
				D_btScalar vel = m_rbA.getAngularVelocity().dot(ax1);
				vel -= m_rbB.getAngularVelocity().dot(ax1);
				// D_only apply bounce if the velocity D_is incoming, D_and if the
				// resulting c[] exceeds what we already have.
				if(limit == 1)
				{	// low limit
					if(vel < 0)
					{
						D_btScalar newc = -bounce * vel;
						if(newc > info->m_constraintError[srow])
						{
							info->m_constraintError[srow] = newc;
						}
					}
				}
				else
				{	// high limit - all those computations D_are reversed
					if(vel > 0)
					{
						D_btScalar newc = -bounce * vel;
						if(newc < info->m_constraintError[srow])
						{
							info->m_constraintError[srow] = newc;
						}
					}
				}
			}
			info->m_constraintError[srow] *= getSoftnessLimAng();
		} // if(limit)
	} // if angular limit or powered
}



void D_btSliderConstraint::solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar timeStep)
{
	if (m_useSolveConstraintObsolete)
	{
		m_timeStep = timeStep;
		if(m_useLinearReferenceFrameA)
		{
			solveConstraintInt(m_rbA,bodyA, m_rbB,bodyB);
		}
		else
		{
			solveConstraintInt(m_rbB,bodyB, m_rbA,bodyA);
		}
	}
}



void D_btSliderConstraint::solveConstraintInt(D_btRigidBody& rbA, D_btSolverBody& bodyA,D_btRigidBody& rbB, D_btSolverBody& bodyB)
{
#ifndef __SPU__
    int i;
    // linear
    D_btVector3 velA;
	bodyA.getVelocityInLocalPointObsolete(m_relPosA,velA);
    D_btVector3 velB;
	bodyB.getVelocityInLocalPointObsolete(m_relPosB,velB);
    D_btVector3 vel = velA - velB;
	for(i = 0; i < 3; i++)
    {
		const D_btVector3& normal = m_jacLin[i].m_linearJointAxis;
		D_btScalar rel_vel = normal.dot(vel);
		// calculate positional error
		D_btScalar depth = m_depth[i];
		// get D_parameters
		D_btScalar softness = (i) ? m_softnessOrthoLin : (m_solveLinLim ? m_softnessLimLin : m_softnessDirLin);
		D_btScalar restitution = (i) ? m_restitutionOrthoLin : (m_solveLinLim ? m_restitutionLimLin : m_restitutionDirLin);
		D_btScalar damping = (i) ? m_dampingOrthoLin : (m_solveLinLim ? m_dampingLimLin : m_dampingDirLin);
		// calcutate D_and apply impulse
		D_btScalar normalImpulse = softness * (restitution * depth / m_timeStep - damping * rel_vel) * m_jacLinDiagABInv[i];
		D_btVector3 impulse_vector = normal * normalImpulse;
		
		//rbA.applyImpulse( impulse_vector, m_relPosA);
		//rbB.applyImpulse(-impulse_vector, m_relPosB);
		{
			D_btVector3 ftorqueAxis1 = m_relPosA.cross(normal);
			D_btVector3 ftorqueAxis2 = m_relPosB.cross(normal);
			bodyA.applyImpulse(normal*rbA.getInvMass(), rbA.getInvInertiaTensorWorld()*ftorqueAxis1,normalImpulse);
			bodyB.applyImpulse(normal*rbB.getInvMass(), rbB.getInvInertiaTensorWorld()*ftorqueAxis2,-normalImpulse);
		}



		if(m_poweredLinMotor && (!i))
		{ // apply linear motor
			if(m_accumulatedLinMotorImpulse < m_maxLinMotorForce)
			{
				D_btScalar desiredMotorVel = m_targetLinMotorVelocity;
				D_btScalar motor_relvel = desiredMotorVel + rel_vel;
				normalImpulse = -motor_relvel * m_jacLinDiagABInv[i];
				// clamp accumulated impulse
				D_btScalar new_acc = m_accumulatedLinMotorImpulse + D_btFabs(normalImpulse);
				if(new_acc  > m_maxLinMotorForce)
				{
					new_acc = m_maxLinMotorForce;
				}
				D_btScalar del = new_acc  - m_accumulatedLinMotorImpulse;
				if(normalImpulse < D_btScalar(0.0))
				{
					normalImpulse = -del;
				}
				else
				{
					normalImpulse = del;
				}
				m_accumulatedLinMotorImpulse = new_acc;
				// apply clamped impulse
				impulse_vector = normal * normalImpulse;
				//rbA.applyImpulse( impulse_vector, m_relPosA);
				//rbB.applyImpulse(-impulse_vector, m_relPosB);

				{
					D_btVector3 ftorqueAxis1 = m_relPosA.cross(normal);
					D_btVector3 ftorqueAxis2 = m_relPosB.cross(normal);
					bodyA.applyImpulse(normal*rbA.getInvMass(), rbA.getInvInertiaTensorWorld()*ftorqueAxis1,normalImpulse);
					bodyB.applyImpulse(normal*rbB.getInvMass(), rbB.getInvInertiaTensorWorld()*ftorqueAxis2,-normalImpulse);
				}



			}
		}
    }
	// angular 
	// get axes in world space
	D_btVector3 axisA =  m_calculatedTransformA.getBasis().getColumn(0);
	D_btVector3 axisB =  m_calculatedTransformB.getBasis().getColumn(0);

	D_btVector3 angVelA;
	bodyA.getAngularVelocity(angVelA);
	D_btVector3 angVelB;
	bodyB.getAngularVelocity(angVelB);

	D_btVector3 angVelAroundAxisA = axisA * axisA.dot(angVelA);
	D_btVector3 angVelAroundAxisB = axisB * axisB.dot(angVelB);

	D_btVector3 angAorthog = angVelA - angVelAroundAxisA;
	D_btVector3 angBorthog = angVelB - angVelAroundAxisB;
	D_btVector3 velrelOrthog = angAorthog-angBorthog;
	//solve orthogonal angular velocity correction
	D_btScalar len = velrelOrthog.length();
	D_btScalar orthorImpulseMag = 0.f;

	if (len > D_btScalar(0.00001))
	{
		D_btVector3 normal = velrelOrthog.normalized();
		D_btScalar denom = rbA.computeAngularImpulseDenominator(normal) + rbB.computeAngularImpulseDenominator(normal);
		//velrelOrthog *= (D_btScalar(1.)/denom) * m_dampingOrthoAng * m_softnessOrthoAng;
		orthorImpulseMag = (D_btScalar(1.)/denom) * m_dampingOrthoAng * m_softnessOrthoAng;
	}
	//solve angular positional correction
	D_btVector3 angularError = axisA.cross(axisB) *(D_btScalar(1.)/m_timeStep);
	D_btVector3 angularAxis = angularError;
	D_btScalar angularImpulseMag = 0;

	D_btScalar len2 = angularError.length();
	if (len2>D_btScalar(0.00001))
	{
		D_btVector3 normal2 = angularError.normalized();
		D_btScalar denom2 = rbA.computeAngularImpulseDenominator(normal2) + rbB.computeAngularImpulseDenominator(normal2);
		angularImpulseMag = (D_btScalar(1.)/denom2) * m_restitutionOrthoAng * m_softnessOrthoAng;
		angularError *= angularImpulseMag;
	}
	// apply impulse
	//rbA.applyTorqueImpulse(-velrelOrthog+angularError);
	//rbB.applyTorqueImpulse(velrelOrthog-angularError);

	bodyA.applyImpulse(D_btVector3(0,0,0), rbA.getInvInertiaTensorWorld()*velrelOrthog,-orthorImpulseMag);
	bodyB.applyImpulse(D_btVector3(0,0,0), rbB.getInvInertiaTensorWorld()*velrelOrthog,orthorImpulseMag);
	bodyA.applyImpulse(D_btVector3(0,0,0), rbA.getInvInertiaTensorWorld()*angularAxis,angularImpulseMag);
	bodyB.applyImpulse(D_btVector3(0,0,0), rbB.getInvInertiaTensorWorld()*angularAxis,-angularImpulseMag);


	D_btScalar impulseMag;
	//solve angular limits
	if(m_solveAngLim)
	{
		impulseMag = (angVelB - angVelA).dot(axisA) * m_dampingLimAng + m_angDepth * m_restitutionLimAng / m_timeStep;
		impulseMag *= m_kAngle * m_softnessLimAng;
	}
	else
	{
		impulseMag = (angVelB - angVelA).dot(axisA) * m_dampingDirAng + m_angDepth * m_restitutionDirAng / m_timeStep;
		impulseMag *= m_kAngle * m_softnessDirAng;
	}
	D_btVector3 impulse = axisA * impulseMag;
	//rbA.applyTorqueImpulse(impulse);
	//rbB.applyTorqueImpulse(-impulse);

	bodyA.applyImpulse(D_btVector3(0,0,0), rbA.getInvInertiaTensorWorld()*axisA,impulseMag);
	bodyB.applyImpulse(D_btVector3(0,0,0), rbB.getInvInertiaTensorWorld()*axisA,-impulseMag);



	//apply angular motor
	if(m_poweredAngMotor) 
	{
		if(m_accumulatedAngMotorImpulse < m_maxAngMotorForce)
		{
			D_btVector3 velrel = angVelAroundAxisA - angVelAroundAxisB;
			D_btScalar projRelVel = velrel.dot(axisA);

			D_btScalar desiredMotorVel = m_targetAngMotorVelocity;
			D_btScalar motor_relvel = desiredMotorVel - projRelVel;

			D_btScalar angImpulse = m_kAngle * motor_relvel;
			// clamp accumulated impulse
			D_btScalar new_acc = m_accumulatedAngMotorImpulse + D_btFabs(angImpulse);
			if(new_acc  > m_maxAngMotorForce)
			{
				new_acc = m_maxAngMotorForce;
			}
			D_btScalar del = new_acc  - m_accumulatedAngMotorImpulse;
			if(angImpulse < D_btScalar(0.0))
			{
				angImpulse = -del;
			}
			else
			{
				angImpulse = del;
			}
			m_accumulatedAngMotorImpulse = new_acc;
			// apply clamped impulse
			D_btVector3 motorImp = angImpulse * axisA;
			//rbA.applyTorqueImpulse(motorImp);
			//rbB.applyTorqueImpulse(-motorImp);

			bodyA.applyImpulse(D_btVector3(0,0,0), rbA.getInvInertiaTensorWorld()*axisA,angImpulse);
			bodyB.applyImpulse(D_btVector3(0,0,0), rbB.getInvInertiaTensorWorld()*axisA,-angImpulse);
		}
	}
#endif //__SPU__
}





void D_btSliderConstraint::calculateTransforms(const D_btTransform& transA,const D_btTransform& transB)
{
	if(m_useLinearReferenceFrameA || (!m_useSolveConstraintObsolete))
	{
		m_calculatedTransformA = transA * m_frameInA;
		m_calculatedTransformB = transB * m_frameInB;
	}
	else
	{
		m_calculatedTransformA = transB * m_frameInB;
		m_calculatedTransformB = transA * m_frameInA;
	}
	m_realPivotAInW = m_calculatedTransformA.getOrigin();
	m_realPivotBInW = m_calculatedTransformB.getOrigin();
	m_sliderAxis = m_calculatedTransformA.getBasis().getColumn(0); // along X
	if(m_useLinearReferenceFrameA || m_useSolveConstraintObsolete)
	{
		m_delta = m_realPivotBInW - m_realPivotAInW;
	}
	else
	{
		m_delta = m_realPivotAInW - m_realPivotBInW;
	}
	m_projPivotInW = m_realPivotAInW + m_sliderAxis.dot(m_delta) * m_sliderAxis;
    D_btVector3 normalWorld;
    int i;
    //linear part
    for(i = 0; i < 3; i++)
    {
		normalWorld = m_calculatedTransformA.getBasis().getColumn(i);
		m_depth[i] = m_delta.dot(normalWorld);
    }
}
 


void D_btSliderConstraint::testLinLimits(void)
{
	m_solveLinLim = false;
	m_linPos = m_depth[0];
	if(m_lowerLinLimit <= m_upperLinLimit)
	{
		if(m_depth[0] > m_upperLinLimit)
		{
			m_depth[0] -= m_upperLinLimit;
			m_solveLinLim = true;
		}
		else if(m_depth[0] < m_lowerLinLimit)
		{
			m_depth[0] -= m_lowerLinLimit;
			m_solveLinLim = true;
		}
		else
		{
			m_depth[0] = D_btScalar(0.);
		}
	}
	else
	{
		m_depth[0] = D_btScalar(0.);
	}
}



void D_btSliderConstraint::testAngLimits(void)
{
	m_angDepth = D_btScalar(0.);
	m_solveAngLim = false;
	if(m_lowerAngLimit <= m_upperAngLimit)
	{
		const D_btVector3 axisA0 = m_calculatedTransformA.getBasis().getColumn(1);
		const D_btVector3 axisA1 = m_calculatedTransformA.getBasis().getColumn(2);
		const D_btVector3 axisB0 = m_calculatedTransformB.getBasis().getColumn(1);
		D_btScalar rot = D_btAtan2Fast(axisB0.dot(axisA1), axisB0.dot(axisA0));  
		rot = D_btAdjustAngleToLimits(rot, m_lowerAngLimit, m_upperAngLimit);
		m_angPos = rot;
		if(rot < m_lowerAngLimit)
		{
			m_angDepth = rot - m_lowerAngLimit;
			m_solveAngLim = true;
		} 
		else if(rot > m_upperAngLimit)
		{
			m_angDepth = rot - m_upperAngLimit;
			m_solveAngLim = true;
		}
	}
}
	


D_btVector3 D_btSliderConstraint::getAncorInA(void)
{
	D_btVector3 ancorInA;
	ancorInA = m_realPivotAInW + (m_lowerLinLimit + m_upperLinLimit) * D_btScalar(0.5) * m_sliderAxis;
	ancorInA = m_rbA.getCenterOfMassTransform().inverse() * ancorInA;
	return ancorInA;
}



D_btVector3 D_btSliderConstraint::getAncorInB(void)
{
	D_btVector3 ancorInB;
	ancorInB = m_frameInB.getOrigin();
	return ancorInB;
}
