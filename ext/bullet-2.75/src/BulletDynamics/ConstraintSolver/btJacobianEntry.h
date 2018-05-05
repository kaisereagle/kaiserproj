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

#ifndef JACOBIAN_ENTRY_H
#define JACOBIAN_ENTRY_H

#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"


//notes:
// Another memory optimization would be D_to store m_1MinvJt in the remaining 3 w components
// which makes the D_btJacobianEntry memory layout 16 bytes
// if you D_only D_are interested in angular part, D_just feed massInvA D_and massInvB zero

/// Jacobian entry D_is an abstraction that D_allows D_to describe constraints
/// it D_can be used in combination with a constraint solver
/// Can be used D_to relate the effect of an impulse D_to the constraint error
D_ATTRIBUTE_ALIGNED16(class) D_btJacobianEntry
{
public:
	D_btJacobianEntry() {};
	//constraint between two different rigidbodies
	D_btJacobianEntry(
		const D_btMatrix3x3& world2A,
		const D_btMatrix3x3& world2B,
		const D_btVector3& rel_pos1,const D_btVector3& rel_pos2,
		const D_btVector3& jointAxis,
		const D_btVector3& inertiaInvA, 
		const D_btScalar massInvA,
		const D_btVector3& inertiaInvB,
		const D_btScalar massInvB)
		:m_linearJointAxis(jointAxis)
	{
		m_aJ = world2A*(rel_pos1.cross(m_linearJointAxis));
		m_bJ = world2B*(rel_pos2.cross(-m_linearJointAxis));
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = inertiaInvB * m_bJ;
		m_Adiag = massInvA + m_0MinvJt.dot(m_aJ) + massInvB + m_1MinvJt.dot(m_bJ);

		D_btAssert(m_Adiag > D_btScalar(0.0));
	}

	//angular constraint between two different rigidbodies
	D_btJacobianEntry(const D_btVector3& jointAxis,
		const D_btMatrix3x3& world2A,
		const D_btMatrix3x3& world2B,
		const D_btVector3& inertiaInvA,
		const D_btVector3& inertiaInvB)
		:m_linearJointAxis(D_btVector3(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.)))
	{
		m_aJ= world2A*jointAxis;
		m_bJ = world2B*-jointAxis;
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = inertiaInvB * m_bJ;
		m_Adiag =  m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);

		D_btAssert(m_Adiag > D_btScalar(0.0));
	}

	//angular constraint between two different rigidbodies
	D_btJacobianEntry(const D_btVector3& axisInA,
		const D_btVector3& axisInB,
		const D_btVector3& inertiaInvA,
		const D_btVector3& inertiaInvB)
		: m_linearJointAxis(D_btVector3(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.)))
		, m_aJ(axisInA)
		, m_bJ(-axisInB)
	{
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = inertiaInvB * m_bJ;
		m_Adiag =  m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);

		D_btAssert(m_Adiag > D_btScalar(0.0));
	}

	//constraint on one rigidbody
	D_btJacobianEntry(
		const D_btMatrix3x3& world2A,
		const D_btVector3& rel_pos1,const D_btVector3& rel_pos2,
		const D_btVector3& jointAxis,
		const D_btVector3& inertiaInvA, 
		const D_btScalar massInvA)
		:m_linearJointAxis(jointAxis)
	{
		m_aJ= world2A*(rel_pos1.cross(jointAxis));
		m_bJ = world2A*(rel_pos2.cross(-jointAxis));
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = D_btVector3(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
		m_Adiag = massInvA + m_0MinvJt.dot(m_aJ);

		D_btAssert(m_Adiag > D_btScalar(0.0));
	}

	D_btScalar	getDiagonal() const { return m_Adiag; }

	// for two constraints on the same rigidbody (for example vehicle friction)
	D_btScalar	getNonDiagonal(const D_btJacobianEntry& jacB, const D_btScalar massInvA) const
	{
		const D_btJacobianEntry& jacA = *this;
		D_btScalar lin = massInvA * jacA.m_linearJointAxis.dot(jacB.m_linearJointAxis);
		D_btScalar ang = jacA.m_0MinvJt.dot(jacB.m_aJ);
		return lin + ang;
	}

	

	// for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
	D_btScalar	getNonDiagonal(const D_btJacobianEntry& jacB,const D_btScalar massInvA,const D_btScalar massInvB) const
	{
		const D_btJacobianEntry& jacA = *this;
		D_btVector3 lin = jacA.m_linearJointAxis * jacB.m_linearJointAxis;
		D_btVector3 ang0 = jacA.m_0MinvJt * jacB.m_aJ;
		D_btVector3 ang1 = jacA.m_1MinvJt * jacB.m_bJ;
		D_btVector3 lin0 = massInvA * lin ;
		D_btVector3 lin1 = massInvB * lin;
		D_btVector3 sum = ang0+ang1+lin0+lin1;
		return sum[0]+sum[1]+sum[2];
	}

	D_btScalar getRelativeVelocity(const D_btVector3& linvelA,const D_btVector3& angvelA,const D_btVector3& linvelB,const D_btVector3& angvelB)
	{
		D_btVector3 linrel = linvelA - linvelB;
		D_btVector3 angvela  = angvelA * m_aJ;
		D_btVector3 angvelb  = angvelB * m_bJ;
		linrel *= m_linearJointAxis;
		angvela += angvelb;
		angvela += linrel;
		D_btScalar rel_vel2 = angvela[0]+angvela[1]+angvela[2];
		return rel_vel2 + D_SIMD_EPSILON;
	}
//private:

	D_btVector3	m_linearJointAxis;
	D_btVector3	m_aJ;
	D_btVector3	m_bJ;
	D_btVector3	m_0MinvJt;
	D_btVector3	m_1MinvJt;
	//Optimization: D_can be stored in the w/last component of one of the vectors
	D_btScalar	m_Adiag;

};

#endif //JACOBIAN_ENTRY_H
