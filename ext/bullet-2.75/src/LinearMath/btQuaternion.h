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



#ifndef SIMD__QUATERNION_H_
#define SIMD__QUATERNION_H_


#include "btVector3.h"
#include "btQuadWord.h"

/**@brief The D_btQuaternion D_implements quaternion D_to perform linear algebra rotations in combination with D_btMatrix3x3, D_btVector3 D_and D_btTransform. */
class D_btQuaternion : public D_btQuadWord {
public:
  /**@brief No initialization constructor */
	D_btQuaternion() {}

	//		template <typename D_btScalar>
	//		explicit Quaternion(const D_btScalar *v) : Tuple4<D_btScalar>(v) {}
  /**@brief Constructor from scalars */
	D_btQuaternion(const D_btScalar& x, const D_btScalar& y, const D_btScalar& z, const D_btScalar& w) 
		: D_btQuadWord(x, y, z, w) 
	{}
  /**@brief Axis angle Constructor
   * @param axis The axis which the rotation D_is around
   * @param angle The magnitude of the rotation around the angle (Radians) */
	D_btQuaternion(const D_btVector3& axis, const D_btScalar& angle) 
	{ 
		setRotation(axis, angle); 
	}
  /**@brief Constructor from D_Euler angles
   * @param yaw Angle around Y unless D_BT_EULER_DEFAULT_ZYX defined then D_Z
   * @param pitch Angle around X unless D_BT_EULER_DEFAULT_ZYX defined then Y
   * @param roll Angle around D_Z unless D_BT_EULER_DEFAULT_ZYX defined then X */
	D_btQuaternion(const D_btScalar& yaw, const D_btScalar& pitch, const D_btScalar& roll)
	{ 
#ifndef D_BT_EULER_DEFAULT_ZYX
		setEuler(yaw, pitch, roll); 
#else
		setEulerZYX(yaw, pitch, roll); 
#endif 
	}
  /**@brief Set the rotation using axis angle notation 
   * @param axis The axis around which D_to rotate
   * @param angle The magnitude of the rotation in Radians */
	void setRotation(const D_btVector3& axis, const D_btScalar& angle)
	{
		D_btScalar d = axis.length();
		D_btAssert(d != D_btScalar(0.0));
		D_btScalar s = D_btSin(angle * D_btScalar(0.5)) / d;
		setValue(axis.x() * s, axis.y() * s, axis.z() * s, 
			D_btCos(angle * D_btScalar(0.5)));
	}
  /**@brief Set the quaternion using D_Euler angles
   * @param yaw Angle around Y
   * @param pitch Angle around X
   * @param roll Angle around D_Z */
	void setEuler(const D_btScalar& yaw, const D_btScalar& pitch, const D_btScalar& roll)
	{
		D_btScalar halfYaw = D_btScalar(yaw) * D_btScalar(0.5);  
		D_btScalar halfPitch = D_btScalar(pitch) * D_btScalar(0.5);  
		D_btScalar halfRoll = D_btScalar(roll) * D_btScalar(0.5);  
		D_btScalar cosYaw = D_btCos(halfYaw);
		D_btScalar sinYaw = D_btSin(halfYaw);
		D_btScalar cosPitch = D_btCos(halfPitch);
		D_btScalar sinPitch = D_btSin(halfPitch);
		D_btScalar cosRoll = D_btCos(halfRoll);
		D_btScalar sinRoll = D_btSin(halfRoll);
		setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
			cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
			sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
			cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
	}
  /**@brief Set the quaternion using euler angles 
   * @param yaw Angle around D_Z
   * @param pitch Angle around Y
   * @param roll Angle around X */
	void setEulerZYX(const D_btScalar& yaw, const D_btScalar& pitch, const D_btScalar& roll)
	{
		D_btScalar halfYaw = D_btScalar(yaw) * D_btScalar(0.5);  
		D_btScalar halfPitch = D_btScalar(pitch) * D_btScalar(0.5);  
		D_btScalar halfRoll = D_btScalar(roll) * D_btScalar(0.5);  
		D_btScalar cosYaw = D_btCos(halfYaw);
		D_btScalar sinYaw = D_btSin(halfYaw);
		D_btScalar cosPitch = D_btCos(halfPitch);
		D_btScalar sinPitch = D_btSin(halfPitch);
		D_btScalar cosRoll = D_btCos(halfRoll);
		D_btScalar sinRoll = D_btSin(halfRoll);
		setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
                         cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
                         cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
                         cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
	}
  /**@brief Add two quaternions
   * @param q The quaternion D_to add D_to this one */
	D_SIMD_FORCE_INLINE	D_btQuaternion& operator+=(const D_btQuaternion& q)
	{
		m_floats[0] += q.x(); m_floats[1] += q.y(); m_floats[2] += q.z(); m_floats[3] += q.m_floats[3];
		return *this;
	}

  /**@brief Subtract out a quaternion
   * @param q The quaternion D_to subtract from this one */
	D_btQuaternion& operator-=(const D_btQuaternion& q) 
	{
		m_floats[0] -= q.x(); m_floats[1] -= q.y(); m_floats[2] -= q.z(); m_floats[3] -= q.m_floats[3];
		return *this;
	}

  /**@brief Scale this quaternion
   * @param s The scalar D_to scale by */
	D_btQuaternion& operator*=(const D_btScalar& s)
	{
		m_floats[0] *= s; m_floats[1] *= s; m_floats[2] *= s; m_floats[3] *= s;
		return *this;
	}

  /**@brief Multiply this quaternion by q on the right
   * @param q The other quaternion 
   * Equivilant D_to this = this * q */
	D_btQuaternion& operator*=(const D_btQuaternion& q)
	{
		setValue(m_floats[3] * q.x() + m_floats[0] * q.m_floats[3] + m_floats[1] * q.z() - m_floats[2] * q.y(),
			m_floats[3] * q.y() + m_floats[1] * q.m_floats[3] + m_floats[2] * q.x() - m_floats[0] * q.z(),
			m_floats[3] * q.z() + m_floats[2] * q.m_floats[3] + m_floats[0] * q.y() - m_floats[1] * q.x(),
			m_floats[3] * q.m_floats[3] - m_floats[0] * q.x() - m_floats[1] * q.y() - m_floats[2] * q.z());
		return *this;
	}
  /**@brief Return the dot product between this quaternion D_and another
   * @param q The other quaternion */
	D_btScalar dot(const D_btQuaternion& q) const
	{
		return m_floats[0] * q.x() + m_floats[1] * q.y() + m_floats[2] * q.z() + m_floats[3] * q.m_floats[3];
	}

  /**@brief Return the length squared of the quaternion */
	D_btScalar length2() const
	{
		return dot(*this);
	}

  /**@brief Return the length of the quaternion */
	D_btScalar length() const
	{
		return D_btSqrt(length2());
	}

  /**@brief Normalize the quaternion 
   * Such that x^2 + y^2 + z^2 +w^2 = 1 */
	D_btQuaternion& normalize() 
	{
		return *this /= length();
	}

  /**@brief Return a scaled version of this quaternion
   * @param s The scale factor */
	D_SIMD_FORCE_INLINE D_btQuaternion
	operator*(const D_btScalar& s) const
	{
		return D_btQuaternion(x() * s, y() * s, z() * s, m_floats[3] * s);
	}


  /**@brief Return an inversely scaled versionof this quaternion
   * @param s The inverse scale factor */
	D_btQuaternion operator/(const D_btScalar& s) const
	{
		D_btAssert(s != D_btScalar(0.0));
		return *this * (D_btScalar(1.0) / s);
	}

  /**@brief Inversely scale this quaternion
   * @param s The scale factor */
	D_btQuaternion& operator/=(const D_btScalar& s) 
	{
		D_btAssert(s != D_btScalar(0.0));
		return *this *= D_btScalar(1.0) / s;
	}

  /**@brief Return a normalized version of this quaternion */
	D_btQuaternion normalized() const 
	{
		return *this / length();
	} 
  /**@brief Return the angle between this quaternion D_and the other 
   * @param q The other quaternion */
	D_btScalar angle(const D_btQuaternion& q) const 
	{
		D_btScalar s = D_btSqrt(length2() * q.length2());
		D_btAssert(s != D_btScalar(0.0));
		return D_btAcos(dot(q) / s);
	}
  /**@brief Return the angle of rotation represented by this quaternion */
	D_btScalar getAngle() const 
	{
		D_btScalar s = D_btScalar(2.) * D_btAcos(m_floats[3]);
		return s;
	}

	/**@brief Return the axis of the rotation represented by this quaternion */
	D_btVector3 getAxis() const
	{
		D_btScalar s_squared = D_btScalar(1.) - D_btPow(m_floats[3], D_btScalar(2.));
		if (s_squared < D_btScalar(10.) * D_SIMD_EPSILON) //Check for divide by zero
			return D_btVector3(1.0, 0.0, 0.0);  // Arbitrary
		D_btScalar s = D_btSqrt(s_squared);
		return D_btVector3(m_floats[0] / s, m_floats[1] / s, m_floats[2] / s);
	}

	/**@brief Return the inverse of this quaternion */
	D_btQuaternion inverse() const
	{
		return D_btQuaternion(-m_floats[0], -m_floats[1], -m_floats[2], m_floats[3]);
	}

  /**@brief Return the sum of this quaternion D_and the other 
   * @param q2 The other quaternion */
	D_SIMD_FORCE_INLINE D_btQuaternion
	operator+(const D_btQuaternion& q2) const
	{
		const D_btQuaternion& q1 = *this;
		return D_btQuaternion(q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z(), q1.m_floats[3] + q2.m_floats[3]);
	}

  /**@brief Return the difference between this quaternion D_and the other 
   * @param q2 The other quaternion */
	D_SIMD_FORCE_INLINE D_btQuaternion
	operator-(const D_btQuaternion& q2) const
	{
		const D_btQuaternion& q1 = *this;
		return D_btQuaternion(q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z(), q1.m_floats[3] - q2.m_floats[3]);
	}

  /**@brief Return the negative of this quaternion 
   * This simply negates each element */
	D_SIMD_FORCE_INLINE D_btQuaternion operator-() const
	{
		const D_btQuaternion& q2 = *this;
		return D_btQuaternion( - q2.x(), - q2.y(),  - q2.z(),  - q2.m_floats[3]);
	}
  /**@todo document this D_and it's use */
	D_SIMD_FORCE_INLINE D_btQuaternion farthest( const D_btQuaternion& qd) const 
	{
		D_btQuaternion diff,sum;
		diff = *this - qd;
		sum = *this + qd;
		if( diff.dot(diff) > sum.dot(sum) )
			return qd;
		return (-qd);
	}

	/**@todo document this D_and it's use */
	D_SIMD_FORCE_INLINE D_btQuaternion nearest( const D_btQuaternion& qd) const 
	{
		D_btQuaternion diff,sum;
		diff = *this - qd;
		sum = *this + qd;
		if( diff.dot(diff) < sum.dot(sum) )
			return qd;
		return (-qd);
	}


  /**@brief Return the quaternion which D_is the result of Spherical D_Linear Interpolation between this D_and the other quaternion
   * @param q The other quaternion D_to interpolate with 
   * @param t The ratio between this D_and q D_to interpolate.  If t = 0 the result D_is this, if t=1 the result D_is q.
   * Slerp interpolates assuming constant velocity.  */
	D_btQuaternion slerp(const D_btQuaternion& q, const D_btScalar& t) const
	{
		D_btScalar theta = angle(q);
		if (theta != D_btScalar(0.0))
		{
			D_btScalar d = D_btScalar(1.0) / D_btSin(theta);
			D_btScalar s0 = D_btSin((D_btScalar(1.0) - t) * theta);
			D_btScalar s1 = D_btSin(t * theta);   
			return D_btQuaternion((m_floats[0] * s0 + q.x() * s1) * d,
				(m_floats[1] * s0 + q.y() * s1) * d,
				(m_floats[2] * s0 + q.z() * s1) * d,
				(m_floats[3] * s0 + q.m_floats[3] * s1) * d);
		}
		else
		{
			return *this;
		}
	}

	static const D_btQuaternion&	getIdentity()
	{
		static const D_btQuaternion identityQuat(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.),D_btScalar(1.));
		return identityQuat;
	}

	D_SIMD_FORCE_INLINE const D_btScalar& getW() const { return m_floats[3]; }

	
};


/**@brief Return the negative of a quaternion */
D_SIMD_FORCE_INLINE D_btQuaternion
operator-(const D_btQuaternion& q)
{
	return D_btQuaternion(-q.x(), -q.y(), -q.z(), -q.w());
}



/**@brief Return the product of two quaternions */
D_SIMD_FORCE_INLINE D_btQuaternion
operator*(const D_btQuaternion& q1, const D_btQuaternion& q2) {
	return D_btQuaternion(q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
		q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z(),
		q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x(),
		q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z()); 
}

D_SIMD_FORCE_INLINE D_btQuaternion
operator*(const D_btQuaternion& q, const D_btVector3& w)
{
	return D_btQuaternion( q.w() * w.x() + q.y() * w.z() - q.z() * w.y(),
		q.w() * w.y() + q.z() * w.x() - q.x() * w.z(),
		q.w() * w.z() + q.x() * w.y() - q.y() * w.x(),
		-q.x() * w.x() - q.y() * w.y() - q.z() * w.z()); 
}

D_SIMD_FORCE_INLINE D_btQuaternion
operator*(const D_btVector3& w, const D_btQuaternion& q)
{
	return D_btQuaternion( w.x() * q.w() + w.y() * q.z() - w.z() * q.y(),
		w.y() * q.w() + w.z() * q.x() - w.x() * q.z(),
		w.z() * q.w() + w.x() * q.y() - w.y() * q.x(),
		-w.x() * q.x() - w.y() * q.y() - w.z() * q.z()); 
}

/**@brief Calculate the dot product between two quaternions */
D_SIMD_FORCE_INLINE D_btScalar 
dot(const D_btQuaternion& q1, const D_btQuaternion& q2) 
{ 
	return q1.dot(q2); 
}


/**@brief Return the length of a quaternion */
D_SIMD_FORCE_INLINE D_btScalar
length(const D_btQuaternion& q) 
{ 
	return q.length(); 
}

/**@brief Return the angle between two quaternions*/
D_SIMD_FORCE_INLINE D_btScalar
angle(const D_btQuaternion& q1, const D_btQuaternion& q2) 
{ 
	return q1.angle(q2); 
}

/**@brief Return the inverse of a quaternion*/
D_SIMD_FORCE_INLINE D_btQuaternion
inverse(const D_btQuaternion& q) 
{
	return q.inverse();
}

/**@brief Return the result of spherical linear interpolation betwen two quaternions 
 * @param q1 The first quaternion
 * @param q2 The second quaternion 
 * @param t The ration between q1 D_and q2.  t = 0 return q1, t=1 returns q2 
 * Slerp assumes constant velocity between positions. */
D_SIMD_FORCE_INLINE D_btQuaternion
slerp(const D_btQuaternion& q1, const D_btQuaternion& q2, const D_btScalar& t) 
{
	return q1.slerp(q2, t);
}

D_SIMD_FORCE_INLINE D_btVector3 
quatRotate(const D_btQuaternion& rotation, const D_btVector3& v) 
{
	D_btQuaternion q = rotation * v;
	q *= rotation.inverse();
	return D_btVector3(q.getX(),q.getY(),q.getZ());
}

D_SIMD_FORCE_INLINE D_btQuaternion 
shortestArcQuat(const D_btVector3& v0, const D_btVector3& v1) // Game Programming Gems 2.10. make sure v0,v1 D_are normalized
{
	D_btVector3 c = v0.cross(v1);
	D_btScalar  d = v0.dot(v1);

	if (d < -1.0 + D_SIMD_EPSILON)
	{
		D_btVector3 n,unused;
		D_btPlaneSpace1(v0,n,unused);
		return D_btQuaternion(n.x(),n.y(),n.z(),0.0f); // D_just pick any vector that D_is orthogonal D_to v0
	}

	D_btScalar  s = D_btSqrt((1.0f + d) * 2.0f);
	D_btScalar rs = 1.0f / s;

	return D_btQuaternion(c.getX()*rs,c.getY()*rs,c.getZ()*rs,s * 0.5f);
}

D_SIMD_FORCE_INLINE D_btQuaternion 
shortestArcQuatNormalize2(D_btVector3& v0,D_btVector3& v1)
{
	v0.normalize();
	v1.normalize();
	return shortestArcQuat(v0,v1);
}

#endif




