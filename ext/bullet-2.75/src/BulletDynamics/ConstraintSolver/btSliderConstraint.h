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

TODO:
 - add clamping od accumulated impulse D_to improve stability
 - add conversion for ODE constraint solver
*/

#ifndef SLIDER_CONSTRAINT_H
#define SLIDER_CONSTRAINT_H



#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"



class D_btRigidBody;



#define D_SLIDER_CONSTRAINT_DEF_SOFTNESS		(D_btScalar(1.0))
#define D_SLIDER_CONSTRAINT_DEF_DAMPING		(D_btScalar(1.0))
#define D_SLIDER_CONSTRAINT_DEF_RESTITUTION	(D_btScalar(0.7))



class D_btSliderConstraint : public D_btTypedConstraint
{
protected:
	///for backwards compatibility during the transition D_to 'getInfo/getInfo2'
	bool		m_useSolveConstraintObsolete;
	D_btTransform	m_frameInA;
    D_btTransform	m_frameInB;
	// use frameA fo define limits, if true
	bool m_useLinearReferenceFrameA;
	// linear limits
	D_btScalar m_lowerLinLimit;
	D_btScalar m_upperLinLimit;
	// angular limits
	D_btScalar m_lowerAngLimit;
	D_btScalar m_upperAngLimit;
	// softness, restitution D_and damping for different cases
	// DirLin - moving inside linear limits
	// LimLin - hitting linear limit
	// DirAng - moving inside angular limits
	// LimAng - hitting angular limit
	// OrthoLin, OrthoAng - against constraint axis
	D_btScalar m_softnessDirLin;
	D_btScalar m_restitutionDirLin;
	D_btScalar m_dampingDirLin;
	D_btScalar m_softnessDirAng;
	D_btScalar m_restitutionDirAng;
	D_btScalar m_dampingDirAng;
	D_btScalar m_softnessLimLin;
	D_btScalar m_restitutionLimLin;
	D_btScalar m_dampingLimLin;
	D_btScalar m_softnessLimAng;
	D_btScalar m_restitutionLimAng;
	D_btScalar m_dampingLimAng;
	D_btScalar m_softnessOrthoLin;
	D_btScalar m_restitutionOrthoLin;
	D_btScalar m_dampingOrthoLin;
	D_btScalar m_softnessOrthoAng;
	D_btScalar m_restitutionOrthoAng;
	D_btScalar m_dampingOrthoAng;
	
	// for interlal use
	bool m_solveLinLim;
	bool m_solveAngLim;

	D_btJacobianEntry	m_jacLin[3];
	D_btScalar		m_jacLinDiagABInv[3];

    D_btJacobianEntry	m_jacAng[3];

	D_btScalar m_timeStep;
    D_btTransform m_calculatedTransformA;
    D_btTransform m_calculatedTransformB;

	D_btVector3 m_sliderAxis;
	D_btVector3 m_realPivotAInW;
	D_btVector3 m_realPivotBInW;
	D_btVector3 m_projPivotInW;
	D_btVector3 m_delta;
	D_btVector3 m_depth;
	D_btVector3 m_relPosA;
	D_btVector3 m_relPosB;

	D_btScalar m_linPos;
	D_btScalar m_angPos;

	D_btScalar m_angDepth;
	D_btScalar m_kAngle;

	bool	 m_poweredLinMotor;
    D_btScalar m_targetLinMotorVelocity;
    D_btScalar m_maxLinMotorForce;
    D_btScalar m_accumulatedLinMotorImpulse;
	
	bool	 m_poweredAngMotor;
    D_btScalar m_targetAngMotorVelocity;
    D_btScalar m_maxAngMotorForce;
    D_btScalar m_accumulatedAngMotorImpulse;

	//------------------------    
	void initParams();
public:
	// constructors
    D_btSliderConstraint(D_btRigidBody& rbA, D_btRigidBody& rbB, const D_btTransform& frameInA, const D_btTransform& frameInB ,bool useLinearReferenceFrameA);
    D_btSliderConstraint(D_btRigidBody& rbB, const D_btTransform& frameInB, bool useLinearReferenceFrameB);
    D_btSliderConstraint();
	// overrides
    virtual void	buildJacobian();
    virtual void getInfo1 (D_btConstraintInfo1* info);

	void getInfo1NonVirtual(D_btConstraintInfo1* info);
	
	virtual void getInfo2 (D_btConstraintInfo2* info);

	void getInfo2NonVirtual(D_btConstraintInfo2* info, const D_btTransform& transA, const D_btTransform& transB,const D_btVector3& linVelA,const D_btVector3& linVelB, D_btScalar rbAinvMass,D_btScalar rbBinvMass);

    virtual	void	solveConstraintObsolete(D_btSolverBody& bodyA,D_btSolverBody& bodyB,D_btScalar	timeStep);
	

	// access
    const D_btRigidBody& getRigidBodyA() const { return m_rbA; }
    const D_btRigidBody& getRigidBodyB() const { return m_rbB; }
    const D_btTransform & getCalculatedTransformA() const { return m_calculatedTransformA; }
    const D_btTransform & getCalculatedTransformB() const { return m_calculatedTransformB; }
    const D_btTransform & getFrameOffsetA() const { return m_frameInA; }
    const D_btTransform & getFrameOffsetB() const { return m_frameInB; }
    D_btTransform & getFrameOffsetA() { return m_frameInA; }
    D_btTransform & getFrameOffsetB() { return m_frameInB; }
    D_btScalar getLowerLinLimit() { return m_lowerLinLimit; }
    void setLowerLinLimit(D_btScalar lowerLimit) { m_lowerLinLimit = lowerLimit; }
    D_btScalar getUpperLinLimit() { return m_upperLinLimit; }
    void setUpperLinLimit(D_btScalar upperLimit) { m_upperLinLimit = upperLimit; }
    D_btScalar getLowerAngLimit() { return m_lowerAngLimit; }
    void setLowerAngLimit(D_btScalar lowerLimit) { m_lowerAngLimit = D_btNormalizeAngle(lowerLimit); }
    D_btScalar getUpperAngLimit() { return m_upperAngLimit; }
    void setUpperAngLimit(D_btScalar upperLimit) { m_upperAngLimit = D_btNormalizeAngle(upperLimit); }
	bool getUseLinearReferenceFrameA() { return m_useLinearReferenceFrameA; }
	D_btScalar getSoftnessDirLin() { return m_softnessDirLin; }
	D_btScalar getRestitutionDirLin() { return m_restitutionDirLin; }
	D_btScalar getDampingDirLin() { return m_dampingDirLin ; }
	D_btScalar getSoftnessDirAng() { return m_softnessDirAng; }
	D_btScalar getRestitutionDirAng() { return m_restitutionDirAng; }
	D_btScalar getDampingDirAng() { return m_dampingDirAng; }
	D_btScalar getSoftnessLimLin() { return m_softnessLimLin; }
	D_btScalar getRestitutionLimLin() { return m_restitutionLimLin; }
	D_btScalar getDampingLimLin() { return m_dampingLimLin; }
	D_btScalar getSoftnessLimAng() { return m_softnessLimAng; }
	D_btScalar getRestitutionLimAng() { return m_restitutionLimAng; }
	D_btScalar getDampingLimAng() { return m_dampingLimAng; }
	D_btScalar getSoftnessOrthoLin() { return m_softnessOrthoLin; }
	D_btScalar getRestitutionOrthoLin() { return m_restitutionOrthoLin; }
	D_btScalar getDampingOrthoLin() { return m_dampingOrthoLin; }
	D_btScalar getSoftnessOrthoAng() { return m_softnessOrthoAng; }
	D_btScalar getRestitutionOrthoAng() { return m_restitutionOrthoAng; }
	D_btScalar getDampingOrthoAng() { return m_dampingOrthoAng; }
	void setSoftnessDirLin(D_btScalar softnessDirLin) { m_softnessDirLin = softnessDirLin; }
	void setRestitutionDirLin(D_btScalar restitutionDirLin) { m_restitutionDirLin = restitutionDirLin; }
	void setDampingDirLin(D_btScalar dampingDirLin) { m_dampingDirLin = dampingDirLin; }
	void setSoftnessDirAng(D_btScalar softnessDirAng) { m_softnessDirAng = softnessDirAng; }
	void setRestitutionDirAng(D_btScalar restitutionDirAng) { m_restitutionDirAng = restitutionDirAng; }
	void setDampingDirAng(D_btScalar dampingDirAng) { m_dampingDirAng = dampingDirAng; }
	void setSoftnessLimLin(D_btScalar softnessLimLin) { m_softnessLimLin = softnessLimLin; }
	void setRestitutionLimLin(D_btScalar restitutionLimLin) { m_restitutionLimLin = restitutionLimLin; }
	void setDampingLimLin(D_btScalar dampingLimLin) { m_dampingLimLin = dampingLimLin; }
	void setSoftnessLimAng(D_btScalar softnessLimAng) { m_softnessLimAng = softnessLimAng; }
	void setRestitutionLimAng(D_btScalar restitutionLimAng) { m_restitutionLimAng = restitutionLimAng; }
	void setDampingLimAng(D_btScalar dampingLimAng) { m_dampingLimAng = dampingLimAng; }
	void setSoftnessOrthoLin(D_btScalar softnessOrthoLin) { m_softnessOrthoLin = softnessOrthoLin; }
	void setRestitutionOrthoLin(D_btScalar restitutionOrthoLin) { m_restitutionOrthoLin = restitutionOrthoLin; }
	void setDampingOrthoLin(D_btScalar dampingOrthoLin) { m_dampingOrthoLin = dampingOrthoLin; }
	void setSoftnessOrthoAng(D_btScalar softnessOrthoAng) { m_softnessOrthoAng = softnessOrthoAng; }
	void setRestitutionOrthoAng(D_btScalar restitutionOrthoAng) { m_restitutionOrthoAng = restitutionOrthoAng; }
	void setDampingOrthoAng(D_btScalar dampingOrthoAng) { m_dampingOrthoAng = dampingOrthoAng; }
	void setPoweredLinMotor(bool onOff) { m_poweredLinMotor = onOff; }
	bool getPoweredLinMotor() { return m_poweredLinMotor; }
	void setTargetLinMotorVelocity(D_btScalar targetLinMotorVelocity) { m_targetLinMotorVelocity = targetLinMotorVelocity; }
	D_btScalar getTargetLinMotorVelocity() { return m_targetLinMotorVelocity; }
	void setMaxLinMotorForce(D_btScalar maxLinMotorForce) { m_maxLinMotorForce = maxLinMotorForce; }
	D_btScalar getMaxLinMotorForce() { return m_maxLinMotorForce; }
	void setPoweredAngMotor(bool onOff) { m_poweredAngMotor = onOff; }
	bool getPoweredAngMotor() { return m_poweredAngMotor; }
	void setTargetAngMotorVelocity(D_btScalar targetAngMotorVelocity) { m_targetAngMotorVelocity = targetAngMotorVelocity; }
	D_btScalar getTargetAngMotorVelocity() { return m_targetAngMotorVelocity; }
	void setMaxAngMotorForce(D_btScalar maxAngMotorForce) { m_maxAngMotorForce = maxAngMotorForce; }
	D_btScalar getMaxAngMotorForce() { return m_maxAngMotorForce; }
	D_btScalar getLinearPos() { return m_linPos; }
	

	// access for ODE solver
	bool getSolveLinLimit() { return m_solveLinLim; }
	D_btScalar getLinDepth() { return m_depth[0]; }
	bool getSolveAngLimit() { return m_solveAngLim; }
	D_btScalar getAngDepth() { return m_angDepth; }
	// internal
    void	buildJacobianInt(D_btRigidBody& rbA, D_btRigidBody& rbB, const D_btTransform& frameInA, const D_btTransform& frameInB);
    void	solveConstraintInt(D_btRigidBody& rbA, D_btSolverBody& bodyA,D_btRigidBody& rbB, D_btSolverBody& bodyB);
	// shared code used by ODE solver
	void	calculateTransforms(const D_btTransform& transA,const D_btTransform& transB);
	void	testLinLimits();
	void	testLinLimits2(D_btConstraintInfo2* info);
	void	testAngLimits();
	// access for PE Solver
	D_btVector3 getAncorInA();
	D_btVector3 getAncorInB();
};



#endif //SLIDER_CONSTRAINT_H

