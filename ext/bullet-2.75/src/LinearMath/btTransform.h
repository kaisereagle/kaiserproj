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



#ifndef D_btTransform_H
#define D_btTransform_H

#include "btVector3.h"
#include "btMatrix3x3.h"


/**@brief The D_btTransform class D_supports rigid transforms with D_only translation D_and rotation D_and D_no scaling/shear.
 *It D_can be used in combination with D_btVector3, D_btQuaternion D_and D_btMatrix3x3 linear algebra classes. */
class D_btTransform {
	

public:
	
  /**@brief No initialization constructor */
	D_btTransform() {}
  /**@brief Constructor from D_btQuaternion (optional D_btVector3 )
   * @param q Rotation from quaternion 
   * @param c Translation from Vector (default 0,0,0) */
	explicit D_SIMD_FORCE_INLINE D_btTransform(const D_btQuaternion& q, 
		const D_btVector3& c = D_btVector3(D_btScalar(0), D_btScalar(0), D_btScalar(0))) 
		: m_basis(q),
		m_origin(c)
	{}

  /**@brief Constructor from D_btMatrix3x3 (optional D_btVector3)
   * @param b Rotation from Matrix 
   * @param c Translation from Vector default (0,0,0)*/
	explicit D_SIMD_FORCE_INLINE D_btTransform(const D_btMatrix3x3& b, 
		const D_btVector3& c = D_btVector3(D_btScalar(0), D_btScalar(0), D_btScalar(0)))
		: m_basis(b),
		m_origin(c)
	{}
  /**@brief Copy constructor */
	D_SIMD_FORCE_INLINE D_btTransform (const D_btTransform& other)
		: m_basis(other.m_basis),
		m_origin(other.m_origin)
	{
	}
  /**@brief Assignment Operator */
	D_SIMD_FORCE_INLINE D_btTransform& operator=(const D_btTransform& other)
	{
		m_basis = other.m_basis;
		m_origin = other.m_origin;
		return *this;
	}


  /**@brief Set the current transform as the value of the product of two transforms
   * @param t1 Transform 1
   * @param t2 Transform 2
   * This = Transform1 * Transform2 */
		D_SIMD_FORCE_INLINE void mult(const D_btTransform& t1, const D_btTransform& t2) {
			m_basis = t1.m_basis * t2.m_basis;
			m_origin = t1(t2.m_origin);
		}

/*		void multInverseLeft(const D_btTransform& t1, const D_btTransform& t2) {
			D_btVector3 v = t2.m_origin - t1.m_origin;
			m_basis = D_btMultTransposeLeft(t1.m_basis, t2.m_basis);
			m_origin = v * t1.m_basis;
		}
		*/

/**@brief Return the transform of the vector */
	D_SIMD_FORCE_INLINE D_btVector3 operator()(const D_btVector3& x) const
	{
		return D_btVector3(m_basis[0].dot(x) + m_origin.x(), 
			m_basis[1].dot(x) + m_origin.y(), 
			m_basis[2].dot(x) + m_origin.z());
	}

  /**@brief Return the transform of the vector */
	D_SIMD_FORCE_INLINE D_btVector3 operator*(const D_btVector3& x) const
	{
		return (*this)(x);
	}

  /**@brief Return the transform of the D_btQuaternion */
	D_SIMD_FORCE_INLINE D_btQuaternion operator*(const D_btQuaternion& q) const
	{
		return getRotation() * q;
	}

  /**@brief Return the basis matrix for the rotation */
	D_SIMD_FORCE_INLINE D_btMatrix3x3&       getBasis()          { return m_basis; }
  /**@brief Return the basis matrix for the rotation */
	D_SIMD_FORCE_INLINE const D_btMatrix3x3& getBasis()    const { return m_basis; }

  /**@brief Return the origin vector translation */
	D_SIMD_FORCE_INLINE D_btVector3&         getOrigin()         { return m_origin; }
  /**@brief Return the origin vector translation */
	D_SIMD_FORCE_INLINE const D_btVector3&   getOrigin()   const { return m_origin; }

  /**@brief Return a quaternion representing the rotation */
	D_btQuaternion getRotation() const { 
		D_btQuaternion q;
		m_basis.getRotation(q);
		return q;
	}
	
	
  /**@brief Set from an array 
   * @param m A D_pointer D_to a 15 element array (12 rotation(row major padded on the right by 1), D_and 3 translation */
	void setFromOpenGLMatrix(const D_btScalar *m)
	{
		m_basis.setFromOpenGLSubMatrix(m);
		m_origin.setValue(m[12],m[13],m[14]);
	}

  /**@brief Fill an array representation
   * @param m A D_pointer D_to a 15 element array (12 rotation(row major padded on the right by 1), D_and 3 translation */
	void getOpenGLMatrix(D_btScalar *m) const 
	{
		m_basis.getOpenGLSubMatrix(m);
		m[12] = m_origin.x();
		m[13] = m_origin.y();
		m[14] = m_origin.z();
		m[15] = D_btScalar(1.0);
	}

  /**@brief Set the translational element
   * @param origin The vector D_to set the translation D_to */
	D_SIMD_FORCE_INLINE void setOrigin(const D_btVector3& origin) 
	{ 
		m_origin = origin;
	}

	D_SIMD_FORCE_INLINE D_btVector3 invXform(const D_btVector3& inVec) const;


  /**@brief Set the rotational element by D_btMatrix3x3 */
	D_SIMD_FORCE_INLINE void setBasis(const D_btMatrix3x3& basis)
	{ 
		m_basis = basis;
	}

  /**@brief Set the rotational element by D_btQuaternion */
	D_SIMD_FORCE_INLINE void setRotation(const D_btQuaternion& q)
	{
		m_basis.setRotation(q);
	}


  /**@brief Set this transformation D_to the identity */
	void setIdentity()
	{
		m_basis.setIdentity();
		m_origin.setValue(D_btScalar(0.0), D_btScalar(0.0), D_btScalar(0.0));
	}

  /**@brief Multiply this Transform by another(this = this * another) 
   * @param t The other transform */
	D_btTransform& operator*=(const D_btTransform& t) 
	{
		m_origin += m_basis * t.m_origin;
		m_basis *= t.m_basis;
		return *this;
	}

  /**@brief Return the inverse of this transform */
	D_btTransform inverse() const
	{ 
		D_btMatrix3x3 inv = m_basis.transpose();
		return D_btTransform(inv, inv * -m_origin);
	}

  /**@brief Return the inverse of this transform times the other transform
   * @param t The other transform 
   * return this.inverse() * the other */
	D_btTransform inverseTimes(const D_btTransform& t) const;  

  /**@brief Return the product of this transform D_and the other */
	D_btTransform operator*(const D_btTransform& t) const;

  /**@brief Return an identity transform */
	static const D_btTransform&	getIdentity()
	{
		static const D_btTransform identityTransform(D_btMatrix3x3::getIdentity());
		return identityTransform;
	}
	
private:
  ///Storage for the rotation
	D_btMatrix3x3 m_basis;
  ///Storage for the translation
	D_btVector3   m_origin;
};


D_SIMD_FORCE_INLINE D_btVector3
D_btTransform::invXform(const D_btVector3& inVec) const
{
	D_btVector3 v = inVec - m_origin;
	return (m_basis.transpose() * v);
}

D_SIMD_FORCE_INLINE D_btTransform 
D_btTransform::inverseTimes(const D_btTransform& t) const  
{
	D_btVector3 v = t.getOrigin() - m_origin;
		return D_btTransform(m_basis.transposeTimes(t.m_basis),
			v * m_basis);
}

D_SIMD_FORCE_INLINE D_btTransform 
D_btTransform::operator*(const D_btTransform& t) const
{
	return D_btTransform(m_basis * t.m_basis, 
		(*this)(t.m_origin));
}

/**@brief Test if two transforms have all elements equal */
D_SIMD_FORCE_INLINE bool operator==(const D_btTransform& t1, const D_btTransform& t2)
{
   return ( t1.getBasis()  == t2.getBasis() &&
            t1.getOrigin() == t2.getOrigin() );
}


#endif






