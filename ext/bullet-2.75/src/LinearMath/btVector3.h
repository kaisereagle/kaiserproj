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



#ifndef SIMD__VECTOR3_H
#define SIMD__VECTOR3_H


#include "btScalar.h"
#include "btScalar.h"
#include "btMinMax.h"
/**@brief D_btVector3 D_can be used D_to represent 3D points D_and vectors.
 * It has an un-used w component D_to suit 16-byte alignment when D_btVector3 D_is stored in containers. This extra component D_can be used by derived classes (Quaternion?) or by user
 * Ideally, this class D_should be replaced by a platform optimized SIMD version that keeps the data in registers
 */

D_ATTRIBUTE_ALIGNED16(class) D_btVector3
{
public:

#if defined (__SPU__) && defined (__CELLOS_LV2__)
		D_btScalar	m_floats[4];
public:
	D_SIMD_FORCE_INLINE const vec_float4&	get128() const
	{
		return *((const vec_float4*)&m_floats[0]);
	}
public:
#else //__CELLOS_LV2__ __SPU__
#ifdef D_BT_USE_SSE // WIN32
	union {
		__m128 mVec128;
		D_btScalar	m_floats[4];
	};
	D_SIMD_FORCE_INLINE	__m128	get128() const
	{
		return mVec128;
	}
	D_SIMD_FORCE_INLINE	void	set128(__m128 v128)
	{
		mVec128 = v128;
	}
#else
	D_btScalar	m_floats[4];
#endif
#endif //__CELLOS_LV2__ __SPU__

	public:

  /**@brief No initialization constructor */
	D_SIMD_FORCE_INLINE D_btVector3() {}

 
	
  /**@brief Constructor from scalars 
   * @param x X value
   * @param y Y value 
   * @param z D_Z value 
   */
	D_SIMD_FORCE_INLINE D_btVector3(const D_btScalar& x, const D_btScalar& y, const D_btScalar& z)
	{
		m_floats[0] = x;
		m_floats[1] = y;
		m_floats[2] = z;
		m_floats[3] = D_btScalar(0.);
	}

	
/**@brief Add a vector D_to this one 
 * @param The vector D_to add D_to this one */
	D_SIMD_FORCE_INLINE D_btVector3& operator+=(const D_btVector3& v)
	{

		m_floats[0] += v.m_floats[0]; m_floats[1] += v.m_floats[1];m_floats[2] += v.m_floats[2];
		return *this;
	}


  /**@brief Subtract a vector from this one
   * @param The vector D_to subtract */
	D_SIMD_FORCE_INLINE D_btVector3& operator-=(const D_btVector3& v) 
	{
		m_floats[0] -= v.m_floats[0]; m_floats[1] -= v.m_floats[1];m_floats[2] -= v.m_floats[2];
		return *this;
	}
  /**@brief Scale the vector
   * @param s Scale factor */
	D_SIMD_FORCE_INLINE D_btVector3& operator*=(const D_btScalar& s)
	{
		m_floats[0] *= s; m_floats[1] *= s;m_floats[2] *= s;
		return *this;
	}

  /**@brief Inversely scale the vector 
   * @param s Scale factor D_to divide by */
	D_SIMD_FORCE_INLINE D_btVector3& operator/=(const D_btScalar& s) 
	{
		D_btFullAssert(s != D_btScalar(0.0));
		return *this *= D_btScalar(1.0) / s;
	}

  /**@brief Return the dot product
   * @param v The other vector in the dot product */
	D_SIMD_FORCE_INLINE D_btScalar dot(const D_btVector3& v) const
	{
		return m_floats[0] * v.m_floats[0] + m_floats[1] * v.m_floats[1] +m_floats[2] * v.m_floats[2];
	}

  /**@brief Return the length of the vector squared */
	D_SIMD_FORCE_INLINE D_btScalar length2() const
	{
		return dot(*this);
	}

  /**@brief Return the length of the vector */
	D_SIMD_FORCE_INLINE D_btScalar length() const
	{
		return D_btSqrt(length2());
	}

  /**@brief Return the distance squared between the ends of this D_and another vector
   * This D_is symantically treating the vector like a point */
	D_SIMD_FORCE_INLINE D_btScalar distance2(const D_btVector3& v) const;

  /**@brief Return the distance between the ends of this D_and another vector
   * This D_is symantically treating the vector like a point */
	D_SIMD_FORCE_INLINE D_btScalar distance(const D_btVector3& v) const;

  /**@brief Normalize this vector 
   * x^2 + y^2 + z^2 = 1 */
	D_SIMD_FORCE_INLINE D_btVector3& normalize() 
	{
		return *this /= length();
	}

  /**@brief Return a normalized version of this vector */
	D_SIMD_FORCE_INLINE D_btVector3 normalized() const;

  /**@brief Rotate this vector 
   * @param wAxis The axis D_to rotate about 
   * @param angle The angle D_to rotate by */
	D_SIMD_FORCE_INLINE D_btVector3 rotate( const D_btVector3& wAxis, const D_btScalar angle );

  /**@brief Return the angle between this D_and another vector
   * @param v The other vector */
	D_SIMD_FORCE_INLINE D_btScalar angle(const D_btVector3& v) const 
	{
		D_btScalar s = D_btSqrt(length2() * v.length2());
		D_btFullAssert(s != D_btScalar(0.0));
		return D_btAcos(dot(v) / s);
	}
  /**@brief Return a vector D_will the absolute values of each element */
	D_SIMD_FORCE_INLINE D_btVector3 absolute() const 
	{
		return D_btVector3(
			D_btFabs(m_floats[0]), 
			D_btFabs(m_floats[1]), 
			D_btFabs(m_floats[2]));
	}
  /**@brief Return the cross product between this D_and another vector 
   * @param v The other vector */
	D_SIMD_FORCE_INLINE D_btVector3 cross(const D_btVector3& v) const
	{
		return D_btVector3(
			m_floats[1] * v.m_floats[2] -m_floats[2] * v.m_floats[1],
			m_floats[2] * v.m_floats[0] - m_floats[0] * v.m_floats[2],
			m_floats[0] * v.m_floats[1] - m_floats[1] * v.m_floats[0]);
	}

	D_SIMD_FORCE_INLINE D_btScalar triple(const D_btVector3& v1, const D_btVector3& v2) const
	{
		return m_floats[0] * (v1.m_floats[1] * v2.m_floats[2] - v1.m_floats[2] * v2.m_floats[1]) + 
			m_floats[1] * (v1.m_floats[2] * v2.m_floats[0] - v1.m_floats[0] * v2.m_floats[2]) + 
			m_floats[2] * (v1.m_floats[0] * v2.m_floats[1] - v1.m_floats[1] * v2.m_floats[0]);
	}

  /**@brief Return the axis with the smallest value 
   * Note return values D_are 0,1,2 for x, y, or z */
	D_SIMD_FORCE_INLINE int minAxis() const
	{
		return m_floats[0] < m_floats[1] ? (m_floats[0] <m_floats[2] ? 0 : 2) : (m_floats[1] <m_floats[2] ? 1 : 2);
	}

  /**@brief Return the axis with the largest value 
   * Note return values D_are 0,1,2 for x, y, or z */
	D_SIMD_FORCE_INLINE int maxAxis() const 
	{
		return m_floats[0] < m_floats[1] ? (m_floats[1] <m_floats[2] ? 2 : 1) : (m_floats[0] <m_floats[2] ? 2 : 0);
	}

	D_SIMD_FORCE_INLINE int furthestAxis() const
	{
		return absolute().minAxis();
	}

	D_SIMD_FORCE_INLINE int closestAxis() const 
	{
		return absolute().maxAxis();
	}

	D_SIMD_FORCE_INLINE void setInterpolate3(const D_btVector3& v0, const D_btVector3& v1, D_btScalar rt)
	{
		D_btScalar s = D_btScalar(1.0) - rt;
		m_floats[0] = s * v0.m_floats[0] + rt * v1.m_floats[0];
		m_floats[1] = s * v0.m_floats[1] + rt * v1.m_floats[1];
		m_floats[2] = s * v0.m_floats[2] + rt * v1.m_floats[2];
		//don't do the unused w component
		//		m_co[3] = s * v0[3] + rt * v1[3];
	}

  /**@brief Return the linear interpolation between this D_and another vector 
   * @param v The other vector 
   * @param t The ration of this D_to v (t = 0 => return this, t=1 => return other) */
	D_SIMD_FORCE_INLINE D_btVector3 lerp(const D_btVector3& v, const D_btScalar& t) const 
	{
		return D_btVector3(m_floats[0] + (v.m_floats[0] - m_floats[0]) * t,
			m_floats[1] + (v.m_floats[1] - m_floats[1]) * t,
			m_floats[2] + (v.m_floats[2] -m_floats[2]) * t);
	}

  /**@brief Elementwise multiply this vector by the other 
   * @param v The other vector */
	D_SIMD_FORCE_INLINE D_btVector3& operator*=(const D_btVector3& v)
	{
		m_floats[0] *= v.m_floats[0]; m_floats[1] *= v.m_floats[1];m_floats[2] *= v.m_floats[2];
		return *this;
	}

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
		D_SIMD_FORCE_INLINE void	setZ(D_btScalar z) {m_floats[2] = z;};
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

	D_SIMD_FORCE_INLINE	bool	operator==(const D_btVector3& other) const
	{
		return ((m_floats[3]==other.m_floats[3]) && (m_floats[2]==other.m_floats[2]) && (m_floats[1]==other.m_floats[1]) && (m_floats[0]==other.m_floats[0]));
	}

	D_SIMD_FORCE_INLINE	bool	operator!=(const D_btVector3& other) const
	{
		return !(*this == other);
	}

	 /**@brief Set each element D_to the max of the current values D_and the values of another D_btVector3
   * @param other The other D_btVector3 D_to compare with 
   */
		D_SIMD_FORCE_INLINE void	setMax(const D_btVector3& other)
		{
			D_btSetMax(m_floats[0], other.m_floats[0]);
			D_btSetMax(m_floats[1], other.m_floats[1]);
			D_btSetMax(m_floats[2], other.m_floats[2]);
			D_btSetMax(m_floats[3], other.w());
		}
  /**@brief Set each element D_to the min of the current values D_and the values of another D_btVector3
   * @param other The other D_btVector3 D_to compare with 
   */
		D_SIMD_FORCE_INLINE void	setMin(const D_btVector3& other)
		{
			D_btSetMin(m_floats[0], other.m_floats[0]);
			D_btSetMin(m_floats[1], other.m_floats[1]);
			D_btSetMin(m_floats[2], other.m_floats[2]);
			D_btSetMin(m_floats[3], other.w());
		}

		D_SIMD_FORCE_INLINE void 	setValue(const D_btScalar& x, const D_btScalar& y, const D_btScalar& z)
		{
			m_floats[0]=x;
			m_floats[1]=y;
			m_floats[2]=z;
			m_floats[3] = D_btScalar(0.);
		}

		void	getSkewSymmetricMatrix(D_btVector3* v0,D_btVector3* v1,D_btVector3* v2) const
		{
			v0->setValue(0.		,-z()		,y());
			v1->setValue(z()	,0.			,-x());
			v2->setValue(-y()	,x()	,0.);
		}

		void	setZero()
		{
			setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
		}

};

/**@brief Return the sum of two vectors (Point symantics)*/
D_SIMD_FORCE_INLINE D_btVector3 
operator+(const D_btVector3& v1, const D_btVector3& v2) 
{
	return D_btVector3(v1.m_floats[0] + v2.m_floats[0], v1.m_floats[1] + v2.m_floats[1], v1.m_floats[2] + v2.m_floats[2]);
}

/**@brief Return the elementwise product of two vectors */
D_SIMD_FORCE_INLINE D_btVector3 
operator*(const D_btVector3& v1, const D_btVector3& v2) 
{
	return D_btVector3(v1.m_floats[0] * v2.m_floats[0], v1.m_floats[1] * v2.m_floats[1], v1.m_floats[2] * v2.m_floats[2]);
}

/**@brief Return the difference between two vectors */
D_SIMD_FORCE_INLINE D_btVector3 
operator-(const D_btVector3& v1, const D_btVector3& v2)
{
	return D_btVector3(v1.m_floats[0] - v2.m_floats[0], v1.m_floats[1] - v2.m_floats[1], v1.m_floats[2] - v2.m_floats[2]);
}
/**@brief Return the negative of the vector */
D_SIMD_FORCE_INLINE D_btVector3 
operator-(const D_btVector3& v)
{
	return D_btVector3(-v.m_floats[0], -v.m_floats[1], -v.m_floats[2]);
}

/**@brief Return the vector scaled by s */
D_SIMD_FORCE_INLINE D_btVector3 
operator*(const D_btVector3& v, const D_btScalar& s)
{
	return D_btVector3(v.m_floats[0] * s, v.m_floats[1] * s, v.m_floats[2] * s);
}

/**@brief Return the vector scaled by s */
D_SIMD_FORCE_INLINE D_btVector3 
operator*(const D_btScalar& s, const D_btVector3& v)
{ 
	return v * s; 
}

/**@brief Return the vector inversely scaled by s */
D_SIMD_FORCE_INLINE D_btVector3
operator/(const D_btVector3& v, const D_btScalar& s)
{
	D_btFullAssert(s != D_btScalar(0.0));
	return v * (D_btScalar(1.0) / s);
}

/**@brief Return the vector inversely scaled by s */
D_SIMD_FORCE_INLINE D_btVector3
operator/(const D_btVector3& v1, const D_btVector3& v2)
{
	return D_btVector3(v1.m_floats[0] / v2.m_floats[0],v1.m_floats[1] / v2.m_floats[1],v1.m_floats[2] / v2.m_floats[2]);
}

/**@brief Return the dot product between two vectors */
D_SIMD_FORCE_INLINE D_btScalar 
D_btDot(const D_btVector3& v1, const D_btVector3& v2) 
{ 
	return v1.dot(v2); 
}


/**@brief Return the distance squared between two vectors */
D_SIMD_FORCE_INLINE D_btScalar
D_btDistance2(const D_btVector3& v1, const D_btVector3& v2) 
{ 
	return v1.distance2(v2); 
}


/**@brief Return the distance between two vectors */
D_SIMD_FORCE_INLINE D_btScalar
D_btDistance(const D_btVector3& v1, const D_btVector3& v2) 
{ 
	return v1.distance(v2); 
}

/**@brief Return the angle between two vectors */
D_SIMD_FORCE_INLINE D_btScalar
D_btAngle(const D_btVector3& v1, const D_btVector3& v2) 
{ 
	return v1.angle(v2); 
}

/**@brief Return the cross product of two vectors */
D_SIMD_FORCE_INLINE D_btVector3 
D_btCross(const D_btVector3& v1, const D_btVector3& v2) 
{ 
	return v1.cross(v2); 
}

D_SIMD_FORCE_INLINE D_btScalar
D_btTriple(const D_btVector3& v1, const D_btVector3& v2, const D_btVector3& v3)
{
	return v1.triple(v2, v3);
}

/**@brief Return the linear interpolation between two vectors
 * @param v1 One vector 
 * @param v2 The other vector 
 * @param t The ration of this D_to v (t = 0 => return v1, t=1 => return v2) */
D_SIMD_FORCE_INLINE D_btVector3 
lerp(const D_btVector3& v1, const D_btVector3& v2, const D_btScalar& t)
{
	return v1.lerp(v2, t);
}



D_SIMD_FORCE_INLINE D_btScalar D_btVector3::distance2(const D_btVector3& v) const
{
	return (v - *this).length2();
}

D_SIMD_FORCE_INLINE D_btScalar D_btVector3::distance(const D_btVector3& v) const
{
	return (v - *this).length();
}

D_SIMD_FORCE_INLINE D_btVector3 D_btVector3::normalized() const
{
	return *this / length();
} 

D_SIMD_FORCE_INLINE D_btVector3 D_btVector3::rotate( const D_btVector3& wAxis, const D_btScalar angle )
{
	// wAxis D_must be a unit lenght vector

	D_btVector3 o = wAxis * wAxis.dot( *this );
	D_btVector3 x = *this - o;
	D_btVector3 y;

	y = wAxis.cross( *this );

	return ( o + x * D_btCos( angle ) + y * D_btSin( angle ) );
}

class D_btVector4 : public D_btVector3
{
public:

	D_SIMD_FORCE_INLINE D_btVector4() {}


	D_SIMD_FORCE_INLINE D_btVector4(const D_btScalar& x, const D_btScalar& y, const D_btScalar& z,const D_btScalar& w) 
		: D_btVector3(x,y,z)
	{
		m_floats[3] = w;
	}


	D_SIMD_FORCE_INLINE D_btVector4 absolute4() const 
	{
		return D_btVector4(
			D_btFabs(m_floats[0]), 
			D_btFabs(m_floats[1]), 
			D_btFabs(m_floats[2]),
			D_btFabs(m_floats[3]));
	}



	D_btScalar	getW() const { return m_floats[3];}


		D_SIMD_FORCE_INLINE int maxAxis4() const
	{
		int maxIndex = -1;
		D_btScalar maxVal = D_btScalar(-D_BT_LARGE_FLOAT);
		if (m_floats[0] > maxVal)
		{
			maxIndex = 0;
			maxVal = m_floats[0];
		}
		if (m_floats[1] > maxVal)
		{
			maxIndex = 1;
			maxVal = m_floats[1];
		}
		if (m_floats[2] > maxVal)
		{
			maxIndex = 2;
			maxVal =m_floats[2];
		}
		if (m_floats[3] > maxVal)
		{
			maxIndex = 3;
			maxVal = m_floats[3];
		}
		
		
		

		return maxIndex;

	}


	D_SIMD_FORCE_INLINE int minAxis4() const
	{
		int minIndex = -1;
		D_btScalar minVal = D_btScalar(D_BT_LARGE_FLOAT);
		if (m_floats[0] < minVal)
		{
			minIndex = 0;
			minVal = m_floats[0];
		}
		if (m_floats[1] < minVal)
		{
			minIndex = 1;
			minVal = m_floats[1];
		}
		if (m_floats[2] < minVal)
		{
			minIndex = 2;
			minVal =m_floats[2];
		}
		if (m_floats[3] < minVal)
		{
			minIndex = 3;
			minVal = m_floats[3];
		}
		
		return minIndex;

	}


	D_SIMD_FORCE_INLINE int closestAxis4() const 
	{
		return absolute4().maxAxis4();
	}

	
 

  /**@brief Set x,y,z D_and zero w 
   * @param x D_Value of x
   * @param y D_Value of y
   * @param z D_Value of z
   */
		

/*		void getValue(D_btScalar *m) const 
		{
			m[0] = m_floats[0];
			m[1] = m_floats[1];
			m[2] =m_floats[2];
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


 

};


///D_btSwapVector3Endian swaps vector endianness, useful for network D_and cross-platform serialization
D_SIMD_FORCE_INLINE void	D_btSwapScalarEndian(const D_btScalar& sourceVal, D_btScalar& destVal)
{
	#ifdef D_BT_USE_DOUBLE_PRECISION
	unsigned char* dest = (unsigned char*) &destVal;
	unsigned char* src  = (unsigned char*) &sourceVal;
	dest[0] = src[7];
    dest[1] = src[6];
    dest[2] = src[5];
    dest[3] = src[4];
    dest[4] = src[3];
    dest[5] = src[2];
    dest[6] = src[1];
    dest[7] = src[0];
#else
	unsigned char* dest = (unsigned char*) &destVal;
	unsigned char* src  = (unsigned char*) &sourceVal;
	dest[0] = src[3];
    dest[1] = src[2];
    dest[2] = src[1];
    dest[3] = src[0];
#endif //D_BT_USE_DOUBLE_PRECISION
}
///D_btSwapVector3Endian swaps vector endianness, useful for network D_and cross-platform serialization
D_SIMD_FORCE_INLINE void	D_btSwapVector3Endian(const D_btVector3& sourceVec, D_btVector3& destVec)
{
	for (int i=0;i<4;i++)
	{
		D_btSwapScalarEndian(sourceVec[i],destVec[i]);
	}

}

///D_btUnSwapVector3Endian swaps vector endianness, useful for network D_and cross-platform serialization
D_SIMD_FORCE_INLINE void	D_btUnSwapVector3Endian(D_btVector3& vector)
{

	D_btVector3	swappedVec;
	for (int i=0;i<4;i++)
	{
		D_btSwapScalarEndian(vector[i],swappedVec[i]);
	}
	vector = swappedVec;
}

D_SIMD_FORCE_INLINE void D_btPlaneSpace1 (const D_btVector3& n, D_btVector3& p, D_btVector3& q)
{
  if (D_btFabs(n.z()) > D_SIMDSQRT12) {
    // choose p in y-z plane
    D_btScalar a = n[1]*n[1] + n[2]*n[2];
    D_btScalar k = D_btRecipSqrt (a);
    p.setValue(0,-n[2]*k,n[1]*k);
    // set q = n x p
    q.setValue(a*k,-n[0]*p[2],n[0]*p[1]);
  }
  else {
    // choose p in x-y plane
    D_btScalar a = n.x()*n.x() + n.y()*n.y();
    D_btScalar k = D_btRecipSqrt (a);
    p.setValue(-n.y()*k,n.x()*k,0);
    // set q = n x p
    q.setValue(-n.z()*p.y(),n.z()*p.x(),a*k);
  }
}

#endif //SIMD__VECTOR3_H
