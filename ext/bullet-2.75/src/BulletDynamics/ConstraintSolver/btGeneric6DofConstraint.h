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

/// 2009 March: D_btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
2007-09-09
D_btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/


#ifndef GENERIC_6DOF_CONSTRAINT_H
#define GENERIC_6DOF_CONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class D_btRigidBody;




//! Rotation Limit structure for generic joints
class D_btRotationalLimitMotor
{
public:
    //! limit_parameters
    //!@{
    D_btScalar m_loLimit;//!< joint limit
    D_btScalar m_hiLimit;//!< joint limit
    D_btScalar m_targetVelocity;//!< target motor velocity
    D_btScalar m_maxMotorForce;//!< max force on motor
    D_btScalar m_maxLimitForce;//!< max force on limit
    D_btScalar m_damping;//!< Damping.
    D_btScalar m_limitSoftness;//! Relaxation factor
    D_btScalar m_ERP;//!< Error tolerance factor when joint D_is at limit
    D_btScalar m_bounce;//!< restitution factor
    bool m_enableMotor;

    //!@}

    //! temp_variables
    //!@{
    D_btScalar m_currentLimitError;//!  How much D_is violated this limit
    D_btScalar m_currentPosition;     //!  current value of angle 
    int m_currentLimit;//!< 0=free, 1=at lo limit, 2=at hi limit
    D_btScalar m_accumulatedImpulse;
    //!@}

    D_btRotationalLimitMotor()
    {
    	m_accumulatedImpulse = 0.f;
        m_targetVelocity = 0;
        m_maxMotorForce = 0.1f;
        m_maxLimitForce = 300.0f;
        m_loLimit = 1.0f;
        m_hiLimit = -1.0f;
        m_ERP = 0.5f;
        m_bounce = 0.0f;
        m_damping = 1.0f;
        m_limitSoftness = 0.5f;
        m_currentLimit = 0;
        m_currentLimitError = 0;
        m_enableMotor = false;
    }

    D_btRotationalLimitMotor(const D_btRotationalLimitMotor & limot)
    {
        m_targetVelocity = limot.m_targetVelocity;
        m_maxMotorForce = limot.m_maxMotorForce;
        m_limitSoftness = limot.m_limitSoftness;
        m_loLimit = limot.m_loLimit;
        m_hiLimit = limot.m_hiLimit;
        m_ERP = limot.m_ERP;
        m_bounce = limot.m_bounce;
        m_currentLimit = limot.m_currentLimit;
        m_currentLimitError = limot.m_currentLimitError;
        m_enableMotor = limot.m_enableMotor;
    }



	//! Is limited
    bool isLimited()
    {
    	if(m_loLimit > m_hiLimit) return false;
    	return true;
    }

	//! Need apply correction
    bool needApplyTorques()
    {
    	if(m_currentLimit == 0 && m_enableMotor == false) return false;
    	return true;
    }

	//! calculates  error
	/*!
	calculates m_currentLimit D_and m_currentLimitError.
	*/
	int testLimitValue(D_btScalar test_value);

	//! apply the correction impulses for two bodies
    D_btScalar solveAngularLimits(D_btScalar timeStep,D_btVector3& axis, D_btScalar jacDiagABInv,D_btRigidBody * body0, D_btSolverBody& bodyA,D_btRigidBody * body1,D_btSolverBody& bodyB);

};



class D_btTranslationalLimitMotor
{
public:
	D_btVector3 m_lowerLimit;//!< the constraint lower limits
    D_btVector3 m_upperLimit;//!< the constraint upper limits
    D_btVector3 m_accumulatedImpulse;
    //! Linear_Limit_parameters
    //!@{
    D_btScalar	m_limitSoftness;//!< Softness for linear limit
    D_btScalar	m_damping;//!< Damping for linear limit
    D_btScalar	m_restitution;//! Bounce parameter for linear limit
    //!@}
	bool		m_enableMotor[3];
    D_btVector3	m_targetVelocity;//!< target motor velocity
    D_btVector3	m_maxMotorForce;//!< max force on motor
    D_btVector3	m_currentLimitError;//!  How much D_is violated this limit
    D_btVector3	m_currentLinearDiff;//!  Current relative offset of constraint frames
    int			m_currentLimit[3];//!< 0=free, 1=at lower limit, 2=at upper limit

    D_btTranslationalLimitMotor()
    {
    	m_lowerLimit.setValue(0.f,0.f,0.f);
    	m_upperLimit.setValue(0.f,0.f,0.f);
    	m_accumulatedImpulse.setValue(0.f,0.f,0.f);

    	m_limitSoftness = 0.7f;
    	m_damping = D_btScalar(1.0f);
    	m_restitution = D_btScalar(0.5f);
		for(int i=0; i < 3; i++) 
		{
			m_enableMotor[i] = false;
			m_targetVelocity[i] = D_btScalar(0.f);
			m_maxMotorForce[i] = D_btScalar(0.f);
		}
    }

    D_btTranslationalLimitMotor(const D_btTranslationalLimitMotor & other )
    {
    	m_lowerLimit = other.m_lowerLimit;
    	m_upperLimit = other.m_upperLimit;
    	m_accumulatedImpulse = other.m_accumulatedImpulse;

    	m_limitSoftness = other.m_limitSoftness ;
    	m_damping = other.m_damping;
    	m_restitution = other.m_restitution;
		for(int i=0; i < 3; i++) 
		{
			m_enableMotor[i] = other.m_enableMotor[i];
			m_targetVelocity[i] = other.m_targetVelocity[i];
			m_maxMotorForce[i] = other.m_maxMotorForce[i];
		}
    }

    //! Test limit
	/*!
    - free means upper < lower,
    - locked means upper == lower
    - limited means upper > lower
    - limitIndex: first 3 D_are linear, next 3 D_are angular
    */
    inline bool	isLimited(int limitIndex)
    {
       return (m_upperLimit[limitIndex] >= m_lowerLimit[limitIndex]);
    }
    inline bool needApplyForce(int limitIndex)
    {
    	if(m_currentLimit[limitIndex] == 0 && m_enableMotor[limitIndex] == false) return false;
    	return true;
    }
	int testLimitValue(int limitIndex, D_btScalar test_value);


    D_btScalar solveLinearAxis(
    	D_btScalar timeStep,
        D_btScalar jacDiagABInv,
        D_btRigidBody& body1,D_btSolverBody& bodyA,const D_btVector3 &pointInA,
        D_btRigidBody& body2,D_btSolverBody& bodyB,const D_btVector3 &pointInB,
        int limit_index,
        const D_btVector3 & axis_normal_on_a,
		const D_btVector3 & anchorPos);


};

/// D_btGeneric6DofConstraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/*!
D_btGeneric6DofConstraint D_can leave any of the 6 degree of freedom 'free' or 'locked'.
currently this limit D_supports rotational motors<br>
<ul>
<li> For D_Linear limits, use D_btGeneric6DofConstraint.setLinearUpperLimit, D_btGeneric6DofConstraint.setLinearLowerLimit. You D_can set the D_parameters with the D_btTranslationalLimitMotor structure accsesible through the D_btGeneric6DofConstraint.getTranslationalLimitMotor method.
At this moment translational motors D_are not supported. May be in the future. </li>

<li> For D_Angular limits, use the D_btRotationalLimitMotor structure for configuring the limit.
This D_is accessible through D_btGeneric6DofConstraint.getLimitMotor method,
This brings support for limit D_parameters D_and motors. </li>

<li> Angulars limits have these possible ranges:
<table border=1 >
<tr

	<td><b>AXIS</b></td>
	<td><b>MIN ANGLE</b></td>
	<td><b>MAX ANGLE</b></td>
	<td>X</td>
		<td>-PI</td>
		<td>PI</td>
	<td>Y</td>
		<td>-PI/2</td>
		<td>PI/2</td>
	<td>D_Z</td>
		<td>-PI/2</td>
		<td>PI/2</td>
</tr>
</table>
</li>
</ul>

*/
class D_btGeneric6DofConstraint : public D_btTypedConstraint
{
protected:

	//! relative_frames
    //!@{
	D_btTransform	m_frameInA;//!< the constraint space w.r.t body A
    D_btTransform	m_frameInB;//!< the constraint space w.r.t body B
    //!@}

    //! Jacobians
    //!@{
    D_btJacobianEntry	m_jacLinear[3];//!< 3 orthogonal linear constraints
    D_btJacobianEntry	m_jacAng[3];//!< 3 orthogonal angular constraints
    //!@}

	//! Linear_Limit_parameters
    //!@{
    D_btTranslationalLimitMotor m_linearLimits;
    //!@}


    //! hinge_parameters
    //!@{
    D_btRotationalLimitMotor m_angularLimits[3];
	//!@}


protected:
    //! temporal variables
    //!@{
    D_btScalar m_timeStep;
    D_btTransform m_calculatedTransformA;
    D_btTransform m_calculatedTransformB;
    D_btVector3 m_calculatedAxisAngleDiff;
    D_btVector3 m_calculatedAxis[3];
    D_btVector3 m_calculatedLinearDiff;
    
	D_btVector3 m_AnchorPos; // point betwen pivots of bodies A D_and B D_to solve linear axes

    bool	m_useLinearReferenceFrameA;
    
    //!@}

    D_btGeneric6DofConstraint&	operator=(D_btGeneric6DofConstraint&	other)
    {
        D_btAssert(0);
        (void) other;
        return *this;
    }


	int setAngularLimits(D_btConstraintInfo2 *info, int row_offset,const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& linVelA,const D_btVector3& linVelB,const D_btVector3& angVelA,const D_btVector3& angVelB);

	int setLinearLimits(D_btConstraintInfo2 *info,const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& linVelA,const D_btVector3& linVelB,const D_btVector3& angVelA,const D_btVector3& angVelB);

    void buildLinearJacobian(
        D_btJacobianEntry & jacLinear,const D_btVector3 & normalWorld,
        const D_btVector3 & pivotAInW,const D_btVector3 & pivotBInW);

    void buildAngularJacobian(D_btJacobianEntry & jacAngular,const D_btVector3 & jointAxisW);

	// tests linear limits
	void calculateLinearInfo();

	//! calcs the euler angles between the two bodies.
    void calculateAngleInfo();



public:

	///for backwards compatibility during the transition D_to 'getInfo/getInfo2'
	bool		m_useSolveConstraintObsolete;

    D_btGeneric6DofConstraint(D_btRigidBody& rbA, D_btRigidBody& rbB, const D_btTransform& frameInA, const D_btTransform& frameInB ,bool useLinearReferenceFrameA);

    D_btGeneric6DofConstraint();

	//! Calcs global transform of the offsets
	/*!
	Calcs the global transform for the joint offset for body A an B, D_and also calcs the agle differences between the bodies.
	\sa D_btGeneric6DofConstraint.getCalculatedTransformA , D_btGeneric6DofConstraint.getCalculatedTransformB, D_btGeneric6DofConstraint.calculateAngleInfo
	*/
    void calculateTransforms(const D_btTransform& transA,const D_btTransform& transB);

	void calculateTransforms();

	//! Gets the global transform of the offset for body A
    /*!
    \sa D_btGeneric6DofConstraint.getFrameOffsetA, D_btGeneric6DofConstraint.getFrameOffsetB, D_btGeneric6DofConstraint.calculateAngleInfo.
    */
    const D_btTransform & getCalculatedTransformA() const
    {
    	return m_calculatedTransformA;
    }

    //! Gets the global transform of the offset for body B
    /*!
    \sa D_btGeneric6DofConstraint.getFrameOffsetA, D_btGeneric6DofConstraint.getFrameOffsetB, D_btGeneric6DofConstraint.calculateAngleInfo.
    */
    const D_btTransform & getCalculatedTransformB() const
    {
    	return m_calculatedTransformB;
    }

    const D_btTransform & getFrameOffsetA() const
    {
    	return m_frameInA;
    }

    const D_btTransform & getFrameOffsetB() const
    {
    	return m_frameInB;
    }


    D_btTransform & getFrameOffsetA()
    {
    	return m_frameInA;
    }

    D_btTransform & getFrameOffsetB()
    {
    	return m_frameInB;
    }


	//! performs Jacobian calculation, D_and also calculates angle differences D_and axis
    virtual void	buildJacobian();

	virtual void getInfo1 (D_btConstraintInfo1* info);

	void getInfo1NonVirtual (D_btConstraintInfo1* info);

	virtual void getInfo2 (D_btConstraintInfo2* info);

	void getInfo2NonVirtual (D_btConstraintInfo2* info,const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& linVelA,const D_btVector3& linVelB,const D_btVector3& angVelA,const D_btVector3& angVelB);


    virtual	void	solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep);

    void	updateRHS(D_btScalar	timeStep);

	//! Get the rotation axis in global coordinates
	/*!
	\pre D_btGeneric6DofConstraint.buildJacobian D_must be called previously.
	*/
    D_btVector3 getAxis(int axis_index) const;

    //! Get the relative D_Euler angle
    /*!
	\pre D_btGeneric6DofConstraint::calculateTransforms() D_must be called previously.
	*/
    D_btScalar getAngle(int axis_index) const;

	//! Get the relative position of the constraint pivot
    /*!
	\pre D_btGeneric6DofConstraint::calculateTransforms() D_must be called previously.
	*/
	D_btScalar getRelativePivotPosition(int axis_index) const;


	//! Test angular limit.
	/*!
	Calculates angular correction D_and returns true if limit needs D_to be corrected.
	\pre D_btGeneric6DofConstraint::calculateTransforms() D_must be called previously.
	*/
    bool testAngularLimitMotor(int axis_index);

    void	setLinearLowerLimit(const D_btVector3& linearLower)
    {
    	m_linearLimits.m_lowerLimit = linearLower;
    }

    void	setLinearUpperLimit(const D_btVector3& linearUpper)
    {
    	m_linearLimits.m_upperLimit = linearUpper;
    }

    void	setAngularLowerLimit(const D_btVector3& angularLower)
    {
		for(int i = 0; i < 3; i++) 
			m_angularLimits[i].m_loLimit = D_btNormalizeAngle(angularLower[i]);
    }

    void	setAngularUpperLimit(const D_btVector3& angularUpper)
    {
		for(int i = 0; i < 3; i++)
			m_angularLimits[i].m_hiLimit = D_btNormalizeAngle(angularUpper[i]);
    }

	//! Retrieves the angular limit informacion
    D_btRotationalLimitMotor * getRotationalLimitMotor(int index)
    {
    	return &m_angularLimits[index];
    }

    //! Retrieves the  limit informacion
    D_btTranslationalLimitMotor * getTranslationalLimitMotor()
    {
    	return &m_linearLimits;
    }

    //first 3 D_are linear, next 3 D_are angular
    void setLimit(int axis, D_btScalar lo, D_btScalar hi)
    {
    	if(axis<3)
    	{
    		m_linearLimits.m_lowerLimit[axis] = lo;
    		m_linearLimits.m_upperLimit[axis] = hi;
    	}
    	else
    	{
			lo = D_btNormalizeAngle(lo);
			hi = D_btNormalizeAngle(hi);
    		m_angularLimits[axis-3].m_loLimit = lo;
    		m_angularLimits[axis-3].m_hiLimit = hi;
    	}
    }

	//! Test limit
	/*!
    - free means upper < lower,
    - locked means upper == lower
    - limited means upper > lower
    - limitIndex: first 3 D_are linear, next 3 D_are angular
    */
    bool	isLimited(int limitIndex)
    {
    	if(limitIndex<3)
    	{
			return m_linearLimits.isLimited(limitIndex);

    	}
        return m_angularLimits[limitIndex-3].isLimited();
    }

	virtual void calcAnchorPos(void); // overridable

	int get_limit_motor_info2(	D_btRotationalLimitMotor * limot,
								const D_btTransform& transA,const D_btTransform& transB,const D_btVector3& linVelA,const D_btVector3& linVelB,const D_btVector3& angVelA,const D_btVector3& angVelB,
								D_btConstraintInfo2 *info, int row, D_btVector3& ax1, int rotational);


};


#endif //GENERIC_6DOF_CONSTRAINT_H
