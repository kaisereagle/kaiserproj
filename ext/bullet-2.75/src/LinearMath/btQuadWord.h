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


#ifndef SIMD_QUADWORD_H
#define SIMD_QUADWORD_H

#include "btScalar.h"
#include "btMinMax.h"


#if defined (__CELLOS_LV2) && defined (__SPU__)
#include <altivec.h>
#endif

/**@brief The D_btQuadWord class D_is base class for D_btVector3 D_and D_btQuaternion. 
 * Some issues under PS3 Linux with IBM 2.1 SDK, gcc compiler prevent from using aligned quadword.
 */
#ifndef USE_LIBSPE2
D_ATTRIBUTE_ALIGNED16(class) D_btQuadWord
#else
class D_btQuadWord
#endif
{
protected:

#if defined (__SPU__) && defined (__CELLOS_LV2__)
	union {
		vec_float4 mVec128;
		D_btScalar	m_floats[4];
	};
public:
	vec_float4	get128() const
	{
		return mVec128;
	}
protected:
#else //__CELLOS_LV2__ __SPU__
	D_btScalar	m_floats[4];
#endif //__CELLOS_LV2__ __SPU__

	public:
  

  /**@brief Return the x value */
		D_SIMD_FORCE_INLINE const D_btScalar& getX() const { return m_floats[0]; }
  /**@brief Return the y value */
		D_SIMD_FORCE_INLINE const D_btScalar& getY() const { return m_floats[1]; }
  /**@brief Return the z value */
		D_SIMD_FORCE_INLINE const D_btScalar& getZ() const { return m_floats[2]; }
  /**@brief Set the x value */
		D_SIMD_FORCE_INLINE void	setX(D_btScalar x) { m_floats[0] = x;};
  /**@brief Set the y value */
		D_SIMD_FORCE_INLINE void	setY(D_btScalar y) { m_floats[1] = y;};
  /**@brief Set the z value */
		D_SIMD_FORCE_INLINE void	setZ(D_btScalar z) { m_floats[2] = z;};
  /**@brief Set the w value */
		D_SIMD_FORCE_INLINE void	setW(D_btScalar w) { m_floats[3] = w;};
  /**@brief Return the x value */
		D_SIMD_FORCE_INLINE const D_btScalar& x() const { return m_floats[0]; }
  /**@brief Return the y value */
		D_SIMD_FORCE_INLINE const D_btScalar& y() const { return m_floats[1]; }
  /**@brief Return the z value */
		D_SIMD_FORCE_INLINE const D_btScalar& z() const { return m_floats[2]; }
  /**@brief Return the w value */
		D_SIMD_FORCE_INLINE const D_btScalar& w() const { return m_floats[3]; }

	//D_SIMD_FORCE_INLINE D_btScalar&       operator[](int i)       { return (&m_floats[0])[i];	}      
	//D_SIMD_FORCE_INLINE const D_btScalar& operator[](int i) const { return (&m_floats[0])[i]; }
	///operator D_btScalar*() replaces operator[], using implicit conversion. We added operator != D_and operator == D_to avoid D_pointer comparisons.
	D_SIMD_FORCE_INLINE	operator       D_btScalar *()       { return &m_floats[0]; }
	D_SIMD_FORCE_INLINE	operator const D_btScalar *() const { return &m_floats[0]; }

	D_SIMD_FORCE_INLINE	bool	operator==(const D_btQuadWord& other) const
	{
		return ((m_floats[3]==other.m_floats[3]) && (m_floats[2]==other.m_floats[2]) && (m_floats[1]==other.m_floats[1]) && (m_floats[0]==other.m_floats[0]));
	}

	D_SIMD_FORCE_INLINE	bool	operator!=(const D_btQuadWord& other) const
	{
		return !(*this == other);
	}

  /**@brief Set x,y,z D_and zero w 
   * @param x D_Value of x
   * @param y D_Value of y
   * @param z D_Value of z
   */
		D_SIMD_FORCE_INLINE void 	setValue(const D_btScalar& x, const D_btScalar& y, const D_btScalar& z)
		{
			m_floats[0]=x;
			m_floats[1]=y;
			m_floats[2]=z;
			m_floats[3] = 0.f;
		}

/*		void getValue(D_btScalar *m) const 
		{
			m[0] = m_floats[0];
			m[1] = m_floats[1];
			m[2] = m_floats[2];
		}
*/
/**@brief Set the values 
   * @param x D_Value of x
   * @param y D_Value of y
   * @param z D_Value of z
   * @param w D_Value of w
   */
		D_SIMD_FORCE_INLINE void	setValue(const D_btScalar& x, const D_btScalar& y, const D_btScalar& z,const D_btScalar& w)
		{
			m_floats[0]=x;
			m_floats[1]=y;
			m_floats[2]=z;
			m_floats[3]=w;
		}
  /**@brief No initialization constructor */
		D_SIMD_FORCE_INLINE D_btQuadWord()
		//	:m_floats[0](D_btScalar(0.)),m_floats[1](D_btScalar(0.)),m_floats[2](D_btScalar(0.)),m_floats[3](D_btScalar(0.))
		{
		}
 
  /**@brief Three argument constructor (zeros w)
   * @param x D_Value of x
   * @param y D_Value of y
   * @param z D_Value of z
   */
		D_SIMD_FORCE_INLINE D_btQuadWord(const D_btScalar& x, const D_btScalar& y, const D_btScalar& z)		
		{
			m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = 0.0f;
		}

/**@brief Initializing constructor
   * @param x D_Value of x
   * @param y D_Value of y
   * @param z D_Value of z
   * @param w D_Value of w
   */
		D_SIMD_FORCE_INLINE D_btQuadWord(const D_btScalar& x, const D_btScalar& y, const D_btScalar& z,const D_btScalar& w) 
		{
			m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = w;
		}

  /**@brief Set each element D_to the max of the current values D_and the values of another D_btQuadWord
   * @param other The other D_btQuadWord D_to compare with 
   */
		D_SIMD_FORCE_INLINE void	setMax(const D_btQuadWord& other)
		{
			D_btSetMax(m_floats[0], other.m_floats[0]);
			D_btSetMax(m_floats[1], other.m_floats[1]);
			D_btSetMax(m_floats[2], other.m_floats[2]);
			D_btSetMax(m_floats[3], other.m_floats[3]);
		}
  /**@brief Set each element D_to the min of the current values D_and the values of another D_btQuadWord
   * @param other The other D_btQuadWord D_to compare with 
   */
		D_SIMD_FORCE_INLINE void	setMin(const D_btQuadWord& other)
		{
			D_btSetMin(m_floats[0], other.m_floats[0]);
			D_btSetMin(m_floats[1], other.m_floats[1]);
			D_btSetMin(m_floats[2], other.m_floats[2]);
			D_btSetMin(m_floats[3], other.m_floats[3]);
		}



};

#endif //SIMD_QUADWORD_H
