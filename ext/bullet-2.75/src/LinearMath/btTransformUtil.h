/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef SIMD_TRANSFORM_UTIL_H
#define SIMD_TRANSFORM_UTIL_H

#include "btTransform.h"
#define D_ANGULAR_MOTION_THRESHOLD D_btScalar(0.5)*D_SIMD_HALF_PI




D_SIMD_FORCE_INLINE D_btVector3 D_btAabbSupport(const D_btVector3& halfExtents,const D_btVector3& supportDir)
{
	return D_btVector3(supportDir.x() < D_btScalar(0.0) ? -halfExtents.x() : halfExtents.x(),
      supportDir.y() < D_btScalar(0.0) ? -halfExtents.y() : halfExtents.y(),
      supportDir.z() < D_btScalar(0.0) ? -halfExtents.z() : halfExtents.z()); 
}






/// Utils related D_to temporal transforms
class D_btTransformUtil
{

public:

	static void integrateTransform(const D_btTransform& curTrans,const D_btVector3& linvel,const D_btVector3& angvel,D_btScalar timeStep,D_btTransform& predictedTransform)
	{
		predictedTransform.setOrigin(curTrans.getOrigin() + linvel * timeStep);
//	#define QUATERNION_DERIVATIVE
	#ifdef QUATERNION_DERIVATIVE
		D_btQuaternion predictedOrn = curTrans.getRotation();
		predictedOrn += (angvel * predictedOrn) * (timeStep * D_btScalar(0.5));
		predictedOrn.normalize();
	#else
		//Exponential map
		//google for "Practical Parameterization of Rotations Using the Exponential Map", D_F. Sebastian Grassia

		D_btVector3 axis;
		D_btScalar	fAngle = angvel.length(); 
		//limit the angular motion
		if (fAngle*timeStep > D_ANGULAR_MOTION_THRESHOLD)
		{
			fAngle = D_ANGULAR_MOTION_THRESHOLD / timeStep;
		}

		if ( fAngle < D_btScalar(0.001) )
		{
			// use Taylor's expansions of sync function
			axis   = angvel*( D_btScalar(0.5)*timeStep-(timeStep*timeStep*timeStep)*(D_btScalar(0.020833333333))*fAngle*fAngle );
		}
		else
		{
			// sync(fAngle) = sin(c*fAngle)/t
			axis   = angvel*( D_btSin(D_btScalar(0.5)*fAngle*timeStep)/fAngle );
		}
		D_btQuaternion dorn (axis.x(),axis.y(),axis.z(),D_btCos( fAngle*timeStep*D_btScalar(0.5) ));
		D_btQuaternion orn0 = curTrans.getRotation();

		D_btQuaternion predictedOrn = dorn * orn0;
		predictedOrn.normalize();
	#endif
		predictedTransform.setRotation(predictedOrn);
	}

	static void	calculateVelocityQuaternion(const D_btVector3& pos0,const D_btVector3& pos1,const D_btQuaternion& orn0,const D_btQuaternion& orn1,D_btScalar timeStep,D_btVector3& linVel,D_btVector3& angVel)
	{
		linVel = (pos1 - pos0) / timeStep;
		D_btVector3 axis;
		D_btScalar  angle;
		if (orn0 != orn1)
		{
			calculateDiffAxisAngleQuaternion(orn0,orn1,axis,angle);
			angVel = axis * angle / timeStep;
		} else
		{
			angVel.setValue(0,0,0);
		}
	}

	static void calculateDiffAxisAngleQuaternion(const D_btQuaternion& orn0,const D_btQuaternion& orn1a,D_btVector3& axis,D_btScalar& angle)
	{
		D_btQuaternion orn1 = orn0.nearest(orn1a);
		D_btQuaternion dorn = orn1 * orn0.inverse();
		///floating point inaccuracy D_can lead D_to w component > 1..., which breaks 
		dorn.normalize();
		angle = dorn.getAngle();
		axis = D_btVector3(dorn.x(),dorn.y(),dorn.z());
		axis[3] = D_btScalar(0.);
		//check for axis length
		D_btScalar len = axis.length2();
		if (len < D_SIMD_EPSILON*D_SIMD_EPSILON)
			axis = D_btVector3(D_btScalar(1.),D_btScalar(0.),D_btScalar(0.));
		else
			axis /= D_btSqrt(len);
	}

	static void	calculateVelocity(const D_btTransform& transform0,const D_btTransform& transform1,D_btScalar timeStep,D_btVector3& linVel,D_btVector3& angVel)
	{
		linVel = (transform1.getOrigin() - transform0.getOrigin()) / timeStep;
		D_btVector3 axis;
		D_btScalar  angle;
		calculateDiffAxisAngle(transform0,transform1,axis,angle);
		angVel = axis * angle / timeStep;
	}

	static void calculateDiffAxisAngle(const D_btTransform& transform0,const D_btTransform& transform1,D_btVector3& axis,D_btScalar& angle)
	{
		D_btMatrix3x3 dmat = transform1.getBasis() * transform0.getBasis().inverse();
		D_btQuaternion dorn;
		dmat.getRotation(dorn);

		///floating point inaccuracy D_can lead D_to w component > 1..., which breaks 
		dorn.normalize();
		
		angle = dorn.getAngle();
		axis = D_btVector3(dorn.x(),dorn.y(),dorn.z());
		axis[3] = D_btScalar(0.);
		//check for axis length
		D_btScalar len = axis.length2();
		if (len < D_SIMD_EPSILON*D_SIMD_EPSILON)
			axis = D_btVector3(D_btScalar(1.),D_btScalar(0.),D_btScalar(0.));
		else
			axis /= D_btSqrt(len);
	}

};


///The D_btConvexSeparatingDistanceUtil D_can help speed up convex collision detection 
///by conservatively updating a cached separating distance/vector instead of re-calculating the closest distance
class	D_btConvexSeparatingDistanceUtil
{
	D_btQuaternion	m_ornA;
	D_btQuaternion	m_ornB;
	D_btVector3	m_posA;
	D_btVector3	m_posB;
	
	D_btVector3	m_separatingNormal;

	D_btScalar	m_boundingRadiusA;
	D_btScalar	m_boundingRadiusB;
	D_btScalar	m_separatingDistance;

public:

	D_btConvexSeparatingDistanceUtil(D_btScalar	boundingRadiusA,D_btScalar	boundingRadiusB)
		:m_boundingRadiusA(boundingRadiusA),
		m_boundingRadiusB(boundingRadiusB),
		m_separatingDistance(0.f)
	{
	}

	D_btScalar	getConservativeSeparatingDistance()
	{
		return m_separatingDistance;
	}

	void	updateSeparatingDistance(const D_btTransform& transA,const D_btTransform& transB)
	{
		const D_btVector3& toPosA = transA.getOrigin();
		const D_btVector3& toPosB = transB.getOrigin();
		D_btQuaternion toOrnA = transA.getRotation();
		D_btQuaternion toOrnB = transB.getRotation();

		if (m_separatingDistance>0.f)
		{
			

			D_btVector3 linVelA,angVelA,linVelB,angVelB;
			D_btTransformUtil::calculateVelocityQuaternion(m_posA,toPosA,m_ornA,toOrnA,D_btScalar(1.),linVelA,angVelA);
			D_btTransformUtil::calculateVelocityQuaternion(m_posB,toPosB,m_ornB,toOrnB,D_btScalar(1.),linVelB,angVelB);
			D_btScalar maxAngularProjectedVelocity = angVelA.length() * m_boundingRadiusA + angVelB.length() * m_boundingRadiusB;
			D_btVector3 relLinVel = (linVelB-linVelA);
			D_btScalar relLinVelocLength = (linVelB-linVelA).dot(m_separatingNormal);
			if (relLinVelocLength<0.f)
			{
				relLinVelocLength = 0.f;
			}
	
			D_btScalar	projectedMotion = maxAngularProjectedVelocity +relLinVelocLength;
			m_separatingDistance -= projectedMotion;
		}
	
		m_posA = toPosA;
		m_posB = toPosB;
		m_ornA = toOrnA;
		m_ornB = toOrnB;
	}

	void	initSeparatingDistance(const D_btVector3& separatingVector,D_btScalar separatingDistance,const D_btTransform& transA,const D_btTransform& transB)
	{
		m_separatingDistance = separatingDistance;

		if (m_separatingDistance>0.f)
		{
			m_separatingNormal = separatingVector;
			
			const D_btVector3& toPosA = transA.getOrigin();
			const D_btVector3& toPosB = transB.getOrigin();
			D_btQuaternion toOrnA = transA.getRotation();
			D_btQuaternion toOrnB = transB.getRotation();
			m_posA = toPosA;
			m_posB = toPosB;
			m_ornA = toOrnA;
			m_ornB = toOrnB;
		}
	}

};


#endif //SIMD_TRANSFORM_UTIL_H

