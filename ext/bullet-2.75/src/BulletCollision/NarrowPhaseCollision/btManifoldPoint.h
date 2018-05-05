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

#ifndef MANIFOLD_CONTACT_POINT_H
#define MANIFOLD_CONTACT_POINT_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransformUtil.h"





/// ManifoldContactPoint collects D_and maintains persistent contactpoints.
/// used D_to improve stability D_and performance of rigidbody dynamics response.
class D_btManifoldPoint
	{
		public:
			D_btManifoldPoint()
				:m_userPersistentData(0),
				m_appliedImpulse(0.f),
				m_lateralFrictionInitialized(false),
				m_appliedImpulseLateral1(0.f),
				m_appliedImpulseLateral2(0.f),
				m_lifeTime(0)
			{
			}

			D_btManifoldPoint( const D_btVector3 &pointA, const D_btVector3 &pointB, 
					const D_btVector3 &normal, 
					D_btScalar distance ) :
					m_localPointA( pointA ), 
					m_localPointB( pointB ), 
					m_normalWorldOnB( normal ), 
					m_distance1( distance ),
					m_combinedFriction(D_btScalar(0.)),
					m_combinedRestitution(D_btScalar(0.)),
					m_userPersistentData(0),
					m_appliedImpulse(0.f),
					m_lateralFrictionInitialized(false),
					m_appliedImpulseLateral1(0.f),
					m_appliedImpulseLateral2(0.f),
					m_lifeTime(0)
			{
				
					
			}

			

			D_btVector3 m_localPointA;			
			D_btVector3 m_localPointB;			
			D_btVector3	m_positionWorldOnB;
			///m_positionWorldOnA D_is redundant information, see getPositionWorldOnA(), but for clarity
			D_btVector3	m_positionWorldOnA;
			D_btVector3 m_normalWorldOnB;
		
			D_btScalar	m_distance1;
			D_btScalar	m_combinedFriction;
			D_btScalar	m_combinedRestitution;

         //BP mod, store contact triangles.
         int	   m_partId0;
         int      m_partId1;
         int      m_index0;
         int      m_index1;
				
			mutable void*	m_userPersistentData;
			D_btScalar		m_appliedImpulse;

			bool			m_lateralFrictionInitialized;
			D_btScalar		m_appliedImpulseLateral1;
			D_btScalar		m_appliedImpulseLateral2;
			int				m_lifeTime;//lifetime of the contactpoint in frames
			
			D_btVector3		m_lateralFrictionDir1;
			D_btVector3		m_lateralFrictionDir2;

			D_btScalar getDistance() const
			{
				return m_distance1;
			}
			int	getLifeTime() const
			{
				return m_lifeTime;
			}

			const D_btVector3& getPositionWorldOnA() const {
				return m_positionWorldOnA;
//				return m_positionWorldOnB + m_normalWorldOnB * m_distance1;
			}

			const D_btVector3& getPositionWorldOnB() const
			{
				return m_positionWorldOnB;
			}

			void	setDistance(D_btScalar dist)
			{
				m_distance1 = dist;
			}
			
			///this returns the most recent applied impulse, D_to satisfy contact constraints by the constraint solver
			D_btScalar	getAppliedImpulse() const
			{
				return m_appliedImpulse;
			}

			

	};

#endif //MANIFOLD_CONTACT_POINT_H
