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

#ifndef D_BT_SOLVER_BODY_H
#define D_BT_SOLVER_BODY_H

class	D_btRigidBody;
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btTransformUtil.h"

///Until we get other contributions, D_only use SIMD on Windows, when using Visual Studio 2008 or later, D_and not double precision
#ifdef D_BT_USE_SSE
#define D_USE_SIMD 1
#endif //


#ifdef D_USE_SIMD

struct	D_btSimdScalar
{
	D_SIMD_FORCE_INLINE	D_btSimdScalar()
	{

	}

	D_SIMD_FORCE_INLINE	D_btSimdScalar(float	fl)
	:m_vec128 (_mm_set1_ps(fl))
	{
	}

	D_SIMD_FORCE_INLINE	D_btSimdScalar(__m128 v128)
		:m_vec128(v128)
	{
	}
	union
	{
		__m128		m_vec128;
		float		m_floats[4];
		int			m_ints[4];
		D_btScalar	m_unusedPadding;
	};
	D_SIMD_FORCE_INLINE	__m128	get128()
	{
		return m_vec128;
	}

	D_SIMD_FORCE_INLINE	const __m128	get128() const
	{
		return m_vec128;
	}

	D_SIMD_FORCE_INLINE	void	set128(__m128 v128)
	{
		m_vec128 = v128;
	}

	D_SIMD_FORCE_INLINE	operator       __m128()       
	{ 
		return m_vec128; 
	}
	D_SIMD_FORCE_INLINE	operator const __m128() const 
	{ 
		return m_vec128; 
	}
	
	D_SIMD_FORCE_INLINE	operator float() const 
	{ 
		return m_floats[0]; 
	}

};

///@brief Return the elementwise product of two D_btSimdScalar
D_SIMD_FORCE_INLINE D_btSimdScalar 
operator*(const D_btSimdScalar& v1, const D_btSimdScalar& v2) 
{
	return D_btSimdScalar(_mm_mul_ps(v1.get128(),v2.get128()));
}

///@brief Return the elementwise product of two D_btSimdScalar
D_SIMD_FORCE_INLINE D_btSimdScalar 
operator+(const D_btSimdScalar& v1, const D_btSimdScalar& v2) 
{
	return D_btSimdScalar(_mm_add_ps(v1.get128(),v2.get128()));
}


#else
#define D_btSimdScalar D_btScalar
#endif

///The D_btSolverBody D_is an internal datastructure for the constraint solver. Only necessary data D_is packed D_to increase cache coherence/performance.
D_ATTRIBUTE_ALIGNED16 (struct)	D_btSolverBody
{
	D_BT_DECLARE_ALIGNED_ALLOCATOR();
	D_btVector3		m_deltaLinearVelocity;
	D_btVector3		m_deltaAngularVelocity;
	D_btVector3		m_angularFactor;
	D_btVector3		m_invMass;
	D_btScalar		m_friction;
	D_btRigidBody*	m_originalBody;
	D_btVector3		m_pushVelocity;
	D_btVector3		m_turnVelocity;

	
	D_SIMD_FORCE_INLINE void	getVelocityInLocalPointObsolete(const D_btVector3& rel_pos, D_btVector3& velocity ) const
	{
		if (m_originalBody)
			velocity = m_originalBody->getLinearVelocity()+m_deltaLinearVelocity + (m_originalBody->getAngularVelocity()+m_deltaAngularVelocity).cross(rel_pos);
		else
			velocity.setValue(0,0,0);
	}

	D_SIMD_FORCE_INLINE void	getAngularVelocity(D_btVector3& angVel) const
	{
		if (m_originalBody)
			angVel = m_originalBody->getAngularVelocity()+m_deltaAngularVelocity;
		else
			angVel.setValue(0,0,0);
	}


	//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
	D_SIMD_FORCE_INLINE void applyImpulse(const D_btVector3& linearComponent, const D_btVector3& angularComponent,const D_btScalar impulseMagnitude)
	{
		//if (m_invMass)
		{
			m_deltaLinearVelocity += linearComponent*impulseMagnitude;
			m_deltaAngularVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
		}
	}

	D_SIMD_FORCE_INLINE void internalApplyPushImpulse(const D_btVector3& linearComponent, const D_btVector3& angularComponent,D_btScalar impulseMagnitude)
	{
		if (m_originalBody)
		{
			m_pushVelocity += linearComponent*impulseMagnitude;
			m_turnVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
		}
	}
	
	void	writebackVelocity()
	{
		if (m_originalBody)
		{
			m_originalBody->setLinearVelocity(m_originalBody->getLinearVelocity()+ m_deltaLinearVelocity);
			m_originalBody->setAngularVelocity(m_originalBody->getAngularVelocity()+m_deltaAngularVelocity);
			
			//m_originalBody->setCompanionId(-1);
		}
	}


	void	writebackVelocity(D_btScalar timeStep)
	{
		if (m_originalBody)
		{
			m_originalBody->setLinearVelocity(m_originalBody->getLinearVelocity()+ m_deltaLinearVelocity);
			m_originalBody->setAngularVelocity(m_originalBody->getAngularVelocity()+m_deltaAngularVelocity);
			
			//correct the position/orientation based on push/turn recovery
			D_btTransform newTransform;
			D_btTransformUtil::integrateTransform(m_originalBody->getWorldTransform(),m_pushVelocity,m_turnVelocity,timeStep,newTransform);
			m_originalBody->setWorldTransform(newTransform);
			
			//m_originalBody->setCompanionId(-1);
		}
	}
	


};

#endif //D_BT_SOLVER_BODY_H


