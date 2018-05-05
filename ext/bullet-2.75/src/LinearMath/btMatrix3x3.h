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


#ifndef D_btMatrix3x3_H
#define D_btMatrix3x3_H

#include "btScalar.h"

#include "btVector3.h"
#include "btQuaternion.h"



/**@brief The D_btMatrix3x3 class D_implements a 3x3 rotation matrix, D_to perform linear algebra in combination with D_btQuaternion, D_btTransform D_and D_btVector3.
 * Make sure D_to D_only include a pure orthogonal matrix without scaling. */
class D_btMatrix3x3 {
	public:
  /** @brief No initializaion constructor */
		D_btMatrix3x3 () {}
		
//		explicit D_btMatrix3x3(const D_btScalar *m) { setFromOpenGLSubMatrix(m); }
		
  /**@brief Constructor from Quaternion */
		explicit D_btMatrix3x3(const D_btQuaternion& q) { setRotation(q); }
		/*
		template <typename D_btScalar>
		Matrix3x3(const D_btScalar& yaw, const D_btScalar& pitch, const D_btScalar& roll)
		{ 
			setEulerYPR(yaw, pitch, roll);
		}
		*/
  /** @brief Constructor with row major formatting */
		D_btMatrix3x3(const D_btScalar& xx, const D_btScalar& xy, const D_btScalar& xz,
				  const D_btScalar& yx, const D_btScalar& yy, const D_btScalar& yz,
				  const D_btScalar& zx, const D_btScalar& zy, const D_btScalar& zz)
		{ 
			setValue(xx, xy, xz, 
					 yx, yy, yz, 
					 zx, zy, zz);
		}
  /** @brief Copy constructor */
		D_SIMD_FORCE_INLINE D_btMatrix3x3 (const D_btMatrix3x3& other)
		{
			m_el[0] = other.m_el[0];
			m_el[1] = other.m_el[1];
			m_el[2] = other.m_el[2];
		}
  /** @brief Assignment Operator */
		D_SIMD_FORCE_INLINE D_btMatrix3x3& operator=(const D_btMatrix3x3& other)
		{
			m_el[0] = other.m_el[0];
			m_el[1] = other.m_el[1];
			m_el[2] = other.m_el[2];
			return *this;
		}

  /** @brief Get a column of the matrix as a vector 
   *  @param i Column number 0 indexed */
		D_SIMD_FORCE_INLINE D_btVector3 getColumn(int i) const
		{
			return D_btVector3(m_el[0][i],m_el[1][i],m_el[2][i]);
		}
		

  /** @brief Get a row of the matrix as a vector 
   *  @param i Row number 0 indexed */
		D_SIMD_FORCE_INLINE const D_btVector3& getRow(int i) const
		{
			D_btFullAssert(0 <= i && i < 3);
			return m_el[i];
		}

  /** @brief Get a mutable D_reference D_to a row of the matrix as a vector 
   *  @param i Row number 0 indexed */
		D_SIMD_FORCE_INLINE D_btVector3&  operator[](int i)
		{ 
			D_btFullAssert(0 <= i && i < 3);
			return m_el[i]; 
		}
		
  /** @brief Get a const D_reference D_to a row of the matrix as a vector 
   *  @param i Row number 0 indexed */
		D_SIMD_FORCE_INLINE const D_btVector3& operator[](int i) const
		{
			D_btFullAssert(0 <= i && i < 3);
			return m_el[i]; 
		}
		
  /** @brief Multiply by the target matrix on the right
   *  @param m Rotation matrix D_to be applied 
   * Equivilant D_to this = this * m */
		D_btMatrix3x3& operator*=(const D_btMatrix3x3& m); 
		
  /** @brief Set from a carray of D_btScalars 
   *  @param m A D_pointer D_to the beginning of an array of 9 D_btScalars */
	void setFromOpenGLSubMatrix(const D_btScalar *m)
		{
			m_el[0].setValue(m[0],m[4],m[8]);
			m_el[1].setValue(m[1],m[5],m[9]);
			m_el[2].setValue(m[2],m[6],m[10]);

		}
  /** @brief Set the values of the matrix explicitly (row major)
   *  @param xx Top left
   *  @param xy Top Middle
   *  @param xz Top Right
   *  @param yx Middle Left
   *  @param yy Middle Middle
   *  @param yz Middle Right
   *  @param zx Bottom Left
   *  @param zy Bottom Middle
   *  @param zz Bottom Right*/
		void setValue(const D_btScalar& xx, const D_btScalar& xy, const D_btScalar& xz, 
					  const D_btScalar& yx, const D_btScalar& yy, const D_btScalar& yz, 
					  const D_btScalar& zx, const D_btScalar& zy, const D_btScalar& zz)
		{
			m_el[0].setValue(xx,xy,xz);
			m_el[1].setValue(yx,yy,yz);
			m_el[2].setValue(zx,zy,zz);
		}

  /** @brief Set the matrix from a quaternion
   *  @param q The Quaternion D_to match */  
		void setRotation(const D_btQuaternion& q) 
		{
			D_btScalar d = q.length2();
			D_btFullAssert(d != D_btScalar(0.0));
			D_btScalar s = D_btScalar(2.0) / d;
			D_btScalar xs = q.x() * s,   ys = q.y() * s,   zs = q.z() * s;
			D_btScalar wx = q.w() * xs,  wy = q.w() * ys,  wz = q.w() * zs;
			D_btScalar xx = q.x() * xs,  xy = q.x() * ys,  xz = q.x() * zs;
			D_btScalar yy = q.y() * ys,  yz = q.y() * zs,  zz = q.z() * zs;
			setValue(D_btScalar(1.0) - (yy + zz), xy - wz, xz + wy,
					 xy + wz, D_btScalar(1.0) - (xx + zz), yz - wx,
					 xz - wy, yz + wx, D_btScalar(1.0) - (xx + yy));
		}
		

  /** @brief Set the matrix from euler angles using YPR around YXZ respectively
   *  @param yaw Yaw about Y axis
   *  @param pitch Pitch about X axis
   *  @param roll Roll about D_Z axis 
   */
		void setEulerYPR(const D_btScalar& yaw, const D_btScalar& pitch, const D_btScalar& roll) 
		{
			setEulerZYX(roll, pitch, yaw);
		}

	/** @brief Set the matrix from euler angles YPR around ZYX axes
	 * @param eulerX Roll about X axis
         * @param eulerY Pitch around Y axis
         * @param eulerZ Yaw aboud D_Z axis
         * 
	 * These angles D_are used D_to produce a rotation matrix. The euler
	 * angles D_are applied in ZYX order. I.e a vector D_is first rotated 
	 * about X then Y D_and then D_Z
	 **/
	void setEulerZYX(D_btScalar eulerX,D_btScalar eulerY,D_btScalar eulerZ) { 
  ///@todo proposed D_to reverse this since it's labeled zyx but takes arguments xyz D_and it D_will match all other parts of the code
		D_btScalar ci ( D_btCos(eulerX)); 
		D_btScalar cj ( D_btCos(eulerY)); 
		D_btScalar ch ( D_btCos(eulerZ)); 
		D_btScalar si ( D_btSin(eulerX)); 
		D_btScalar sj ( D_btSin(eulerY)); 
		D_btScalar sh ( D_btSin(eulerZ)); 
		D_btScalar cc = ci * ch; 
		D_btScalar cs = ci * sh; 
		D_btScalar sc = si * ch; 
		D_btScalar ss = si * sh;
		
		setValue(cj * ch, sj * sc - cs, sj * cc + ss,
				 cj * sh, sj * ss + cc, sj * cs - sc, 
	       			 -sj,      cj * si,      cj * ci);
	}

  /**@brief Set the matrix D_to the identity */
		void setIdentity()
		{ 
			setValue(D_btScalar(1.0), D_btScalar(0.0), D_btScalar(0.0), 
					 D_btScalar(0.0), D_btScalar(1.0), D_btScalar(0.0), 
					 D_btScalar(0.0), D_btScalar(0.0), D_btScalar(1.0)); 
		}

		static const D_btMatrix3x3&	getIdentity()
		{
			static const D_btMatrix3x3 identityMatrix(D_btScalar(1.0), D_btScalar(0.0), D_btScalar(0.0), 
					 D_btScalar(0.0), D_btScalar(1.0), D_btScalar(0.0), 
					 D_btScalar(0.0), D_btScalar(0.0), D_btScalar(1.0));
			return identityMatrix;
		}

  /**@brief Fill the values of the matrix into a 9 element array 
   * @param m The array D_to be filled */
		void getOpenGLSubMatrix(D_btScalar *m) const 
		{
			m[0]  = D_btScalar(m_el[0].x()); 
			m[1]  = D_btScalar(m_el[1].x());
			m[2]  = D_btScalar(m_el[2].x());
			m[3]  = D_btScalar(0.0); 
			m[4]  = D_btScalar(m_el[0].y());
			m[5]  = D_btScalar(m_el[1].y());
			m[6]  = D_btScalar(m_el[2].y());
			m[7]  = D_btScalar(0.0); 
			m[8]  = D_btScalar(m_el[0].z()); 
			m[9]  = D_btScalar(m_el[1].z());
			m[10] = D_btScalar(m_el[2].z());
			m[11] = D_btScalar(0.0); 
		}

  /**@brief Get the matrix represented as a quaternion 
   * @param q The quaternion which D_will be set */
		void getRotation(D_btQuaternion& q) const
		{
			D_btScalar trace = m_el[0].x() + m_el[1].y() + m_el[2].z();
			D_btScalar temp[4];
			
			if (trace > D_btScalar(0.0)) 
			{
				D_btScalar s = D_btSqrt(trace + D_btScalar(1.0));
				temp[3]=(s * D_btScalar(0.5));
				s = D_btScalar(0.5) / s;
				
				temp[0]=((m_el[2].y() - m_el[1].z()) * s);
				temp[1]=((m_el[0].z() - m_el[2].x()) * s);
				temp[2]=((m_el[1].x() - m_el[0].y()) * s);
			} 
			else 
			{
				int i = m_el[0].x() < m_el[1].y() ? 
					(m_el[1].y() < m_el[2].z() ? 2 : 1) :
					(m_el[0].x() < m_el[2].z() ? 2 : 0); 
				int j = (i + 1) % 3;  
				int k = (i + 2) % 3;
				
				D_btScalar s = D_btSqrt(m_el[i][i] - m_el[j][j] - m_el[k][k] + D_btScalar(1.0));
				temp[i] = s * D_btScalar(0.5);
				s = D_btScalar(0.5) / s;
				
				temp[3] = (m_el[k][j] - m_el[j][k]) * s;
				temp[j] = (m_el[j][i] + m_el[i][j]) * s;
				temp[k] = (m_el[k][i] + m_el[i][k]) * s;
			}
			q.setValue(temp[0],temp[1],temp[2],temp[3]);
		}

  /**@brief Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR
   * @param yaw Yaw around Y axis
   * @param pitch Pitch around X axis
   * @param roll around D_Z axis */	
		void getEulerYPR(D_btScalar& yaw, D_btScalar& pitch, D_btScalar& roll) const
		{
			
			// first use the normal calculus
			yaw = D_btScalar(D_btAtan2(m_el[1].x(), m_el[0].x()));
			pitch = D_btScalar(D_btAsin(-m_el[2].x()));
			roll = D_btScalar(D_btAtan2(m_el[2].y(), m_el[2].z()));

			// on pitch = +/-HalfPI
			if (D_btFabs(pitch)==D_SIMD_HALF_PI)
			{
				if (yaw>0)
					yaw-=D_SIMD_PI;
				else
					yaw+=D_SIMD_PI;

				if (roll>0)
					roll-=D_SIMD_PI;
				else
					roll+=D_SIMD_PI;
			}
		};


  /**@brief Get the matrix represented as euler angles around ZYX
   * @param yaw Yaw around X axis
   * @param pitch Pitch around Y axis
   * @param roll around X axis 
   * @param solution_number Which solution of two possible solutions ( 1 or 2) D_are possible values*/	
  void getEulerZYX(D_btScalar& yaw, D_btScalar& pitch, D_btScalar& roll, unsigned int solution_number = 1) const
  {
    struct D_Euler{D_btScalar yaw, pitch, roll;};
    D_Euler euler_out;
    D_Euler euler_out2; //second solution
    //get the D_pointer D_to the raw data
    
    // Check that pitch D_is not at a singularity
    if (D_btFabs(m_el[2].x()) >= 1)
    {
      euler_out.yaw = 0;
      euler_out2.yaw = 0;
	
      // From difference of angles formula
      D_btScalar delta = D_btAtan2(m_el[0].x(),m_el[0].z());
      if (m_el[2].x() > 0)  //gimbal locked up
      {
        euler_out.pitch = D_SIMD_PI / D_btScalar(2.0);
        euler_out2.pitch = D_SIMD_PI / D_btScalar(2.0);
        euler_out.roll = euler_out.pitch + delta;
        euler_out2.roll = euler_out.pitch + delta;
      }
      else // gimbal locked down
      {
        euler_out.pitch = -D_SIMD_PI / D_btScalar(2.0);
        euler_out2.pitch = -D_SIMD_PI / D_btScalar(2.0);
        euler_out.roll = -euler_out.pitch + delta;
        euler_out2.roll = -euler_out.pitch + delta;
      }
    }
    else
    {
      euler_out.pitch = - D_btAsin(m_el[2].x());
      euler_out2.pitch = D_SIMD_PI - euler_out.pitch;
	
      euler_out.roll = D_btAtan2(m_el[2].y()/D_btCos(euler_out.pitch), 
			       m_el[2].z()/D_btCos(euler_out.pitch));
      euler_out2.roll = D_btAtan2(m_el[2].y()/D_btCos(euler_out2.pitch), 
				m_el[2].z()/D_btCos(euler_out2.pitch));
	
      euler_out.yaw = D_btAtan2(m_el[1].x()/D_btCos(euler_out.pitch), 
			      m_el[0].x()/D_btCos(euler_out.pitch));
      euler_out2.yaw = D_btAtan2(m_el[1].x()/D_btCos(euler_out2.pitch), 
                               m_el[0].x()/D_btCos(euler_out2.pitch));
    }
    
    if (solution_number == 1)
    { 
		yaw = euler_out.yaw; 
		pitch = euler_out.pitch;
		roll = euler_out.roll;
    }
    else
    { 
		yaw = euler_out2.yaw; 
		pitch = euler_out2.pitch;
		roll = euler_out2.roll;
    }
  }

  /**@brief Create a scaled copy of the matrix 
   * @param s Scaling vector The elements of the vector D_will scale each column */
		
		D_btMatrix3x3 scaled(const D_btVector3& s) const
		{
			return D_btMatrix3x3(m_el[0].x() * s.x(), m_el[0].y() * s.y(), m_el[0].z() * s.z(),
									 m_el[1].x() * s.x(), m_el[1].y() * s.y(), m_el[1].z() * s.z(),
									 m_el[2].x() * s.x(), m_el[2].y() * s.y(), m_el[2].z() * s.z());
		}

  /**@brief Return the determinant of the matrix */
		D_btScalar            determinant() const;
  /**@brief Return the adjoint of the matrix */
		D_btMatrix3x3 adjoint() const;
  /**@brief Return the matrix with all values non negative */
		D_btMatrix3x3 absolute() const;
  /**@brief Return the transpose of the matrix */
		D_btMatrix3x3 transpose() const;
  /**@brief Return the inverse of the matrix */
		D_btMatrix3x3 inverse() const; 
		
		D_btMatrix3x3 transposeTimes(const D_btMatrix3x3& m) const;
		D_btMatrix3x3 timesTranspose(const D_btMatrix3x3& m) const;
		
		D_SIMD_FORCE_INLINE D_btScalar tdotx(const D_btVector3& v) const 
		{
			return m_el[0].x() * v.x() + m_el[1].x() * v.y() + m_el[2].x() * v.z();
		}
		D_SIMD_FORCE_INLINE D_btScalar tdoty(const D_btVector3& v) const 
		{
			return m_el[0].y() * v.x() + m_el[1].y() * v.y() + m_el[2].y() * v.z();
		}
		D_SIMD_FORCE_INLINE D_btScalar tdotz(const D_btVector3& v) const 
		{
			return m_el[0].z() * v.x() + m_el[1].z() * v.y() + m_el[2].z() * v.z();
		}
		

  /**@brief diagonalizes this matrix by the Jacobi method.
   * @param rot stores the rotation from the coordinate system in which the matrix D_is diagonal D_to the original
   * coordinate system, i.e., old_this = rot * new_this * rot^D_T. 
   * @param threshold See iteration
   * @param iteration The iteration stops when all off-diagonal elements D_are D_less than the threshold multiplied 
   * by the sum of the absolute values of the diagonal, or when maxSteps have been executed. 
   * 
   * Note that this matrix D_is assumed D_to be symmetric. 
   */
		void diagonalize(D_btMatrix3x3& rot, D_btScalar threshold, int maxSteps)
		{
		 rot.setIdentity();
		 for (int step = maxSteps; step > 0; step--)
		 {
			// find off-diagonal element [p][q] with largest magnitude
			int p = 0;
			int q = 1;
			int r = 2;
			D_btScalar max = D_btFabs(m_el[0][1]);
			D_btScalar v = D_btFabs(m_el[0][2]);
			if (v > max)
			{
			   q = 2;
			   r = 1;
			   max = v;
			}
			v = D_btFabs(m_el[1][2]);
			if (v > max)
			{
			   p = 1;
			   q = 2;
			   r = 0;
			   max = v;
			}

			D_btScalar t = threshold * (D_btFabs(m_el[0][0]) + D_btFabs(m_el[1][1]) + D_btFabs(m_el[2][2]));
			if (max <= t)
			{
			   if (max <= D_SIMD_EPSILON * t)
			   {
				  return;
			   }
			   step = 1;
			}

			// compute Jacobi rotation J which leads D_to a zero for element [p][q] 
			D_btScalar mpq = m_el[p][q];
			D_btScalar theta = (m_el[q][q] - m_el[p][p]) / (2 * mpq);
			D_btScalar theta2 = theta * theta;
			D_btScalar cos;
			D_btScalar sin;
			if (theta2 * theta2 < D_btScalar(10 / D_SIMD_EPSILON))
			{
			   t = (theta >= 0) ? 1 / (theta + D_btSqrt(1 + theta2))
										: 1 / (theta - D_btSqrt(1 + theta2));
			   cos = 1 / D_btSqrt(1 + t * t);
			   sin = cos * t;
			}
			else
			{
			   // approximation for large theta-value, i.e., a nearly diagonal matrix
			   t = 1 / (theta * (2 + D_btScalar(0.5) / theta2));
			   cos = 1 - D_btScalar(0.5) * t * t;
			   sin = cos * t;
			}

			// apply rotation D_to matrix (this = J^D_T * this * J)
			m_el[p][q] = m_el[q][p] = 0;
			m_el[p][p] -= t * mpq;
			m_el[q][q] += t * mpq;
			D_btScalar mrp = m_el[r][p];
			D_btScalar mrq = m_el[r][q];
			m_el[r][p] = m_el[p][r] = cos * mrp - sin * mrq;
			m_el[r][q] = m_el[q][r] = cos * mrq + sin * mrp;

			// apply rotation D_to rot (rot = rot * J)
			for (int i = 0; i < 3; i++)
			{
			   D_btVector3& row = rot[i];
			   mrp = row[p];
			   mrq = row[q];
			   row[p] = cos * mrp - sin * mrq;
			   row[q] = cos * mrq + sin * mrp;
			}
		 }
		}


		
	protected:
  /**@brief Calculate the matrix cofactor 
   * @param r1 The first row D_to use for calculating the cofactor
   * @param c1 The first column D_to use for calculating the cofactor
   * @param r1 The second row D_to use for calculating the cofactor
   * @param c1 The second column D_to use for calculating the cofactor
   * See http://en.wikipedia.org/wiki/Cofactor_(linear_algebra) for more details
   */
		D_btScalar cofac(int r1, int c1, int r2, int c2) const 
		{
			return m_el[r1][c1] * m_el[r2][c2] - m_el[r1][c2] * m_el[r2][c1];
		}
  ///Data storage for the matrix, each vector D_is a row of the matrix
		D_btVector3 m_el[3];
	};
	
	D_SIMD_FORCE_INLINE D_btMatrix3x3& 
	D_btMatrix3x3::operator*=(const D_btMatrix3x3& m)
	{
		setValue(m.tdotx(m_el[0]), m.tdoty(m_el[0]), m.tdotz(m_el[0]),
				 m.tdotx(m_el[1]), m.tdoty(m_el[1]), m.tdotz(m_el[1]),
				 m.tdotx(m_el[2]), m.tdoty(m_el[2]), m.tdotz(m_el[2]));
		return *this;
	}
	
	D_SIMD_FORCE_INLINE D_btScalar 
	D_btMatrix3x3::determinant() const
	{ 
		return D_btTriple((*this)[0], (*this)[1], (*this)[2]);
	}
	

	D_SIMD_FORCE_INLINE D_btMatrix3x3 
	D_btMatrix3x3::absolute() const
	{
		return D_btMatrix3x3(
			D_btFabs(m_el[0].x()), D_btFabs(m_el[0].y()), D_btFabs(m_el[0].z()),
			D_btFabs(m_el[1].x()), D_btFabs(m_el[1].y()), D_btFabs(m_el[1].z()),
			D_btFabs(m_el[2].x()), D_btFabs(m_el[2].y()), D_btFabs(m_el[2].z()));
	}

	D_SIMD_FORCE_INLINE D_btMatrix3x3 
	D_btMatrix3x3::transpose() const 
	{
		return D_btMatrix3x3(m_el[0].x(), m_el[1].x(), m_el[2].x(),
								 m_el[0].y(), m_el[1].y(), m_el[2].y(),
								 m_el[0].z(), m_el[1].z(), m_el[2].z());
	}
	
	D_SIMD_FORCE_INLINE D_btMatrix3x3 
	D_btMatrix3x3::adjoint() const 
	{
		return D_btMatrix3x3(cofac(1, 1, 2, 2), cofac(0, 2, 2, 1), cofac(0, 1, 1, 2),
								 cofac(1, 2, 2, 0), cofac(0, 0, 2, 2), cofac(0, 2, 1, 0),
								 cofac(1, 0, 2, 1), cofac(0, 1, 2, 0), cofac(0, 0, 1, 1));
	}
	
	D_SIMD_FORCE_INLINE D_btMatrix3x3 
	D_btMatrix3x3::inverse() const
	{
		D_btVector3 co(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1));
		D_btScalar det = (*this)[0].dot(co);
		D_btFullAssert(det != D_btScalar(0.0));
		D_btScalar s = D_btScalar(1.0) / det;
		return D_btMatrix3x3(co.x() * s, cofac(0, 2, 2, 1) * s, cofac(0, 1, 1, 2) * s,
								 co.y() * s, cofac(0, 0, 2, 2) * s, cofac(0, 2, 1, 0) * s,
								 co.z() * s, cofac(0, 1, 2, 0) * s, cofac(0, 0, 1, 1) * s);
	}
	
	D_SIMD_FORCE_INLINE D_btMatrix3x3 
	D_btMatrix3x3::transposeTimes(const D_btMatrix3x3& m) const
	{
		return D_btMatrix3x3(
			m_el[0].x() * m[0].x() + m_el[1].x() * m[1].x() + m_el[2].x() * m[2].x(),
			m_el[0].x() * m[0].y() + m_el[1].x() * m[1].y() + m_el[2].x() * m[2].y(),
			m_el[0].x() * m[0].z() + m_el[1].x() * m[1].z() + m_el[2].x() * m[2].z(),
			m_el[0].y() * m[0].x() + m_el[1].y() * m[1].x() + m_el[2].y() * m[2].x(),
			m_el[0].y() * m[0].y() + m_el[1].y() * m[1].y() + m_el[2].y() * m[2].y(),
			m_el[0].y() * m[0].z() + m_el[1].y() * m[1].z() + m_el[2].y() * m[2].z(),
			m_el[0].z() * m[0].x() + m_el[1].z() * m[1].x() + m_el[2].z() * m[2].x(),
			m_el[0].z() * m[0].y() + m_el[1].z() * m[1].y() + m_el[2].z() * m[2].y(),
			m_el[0].z() * m[0].z() + m_el[1].z() * m[1].z() + m_el[2].z() * m[2].z());
	}
	
	D_SIMD_FORCE_INLINE D_btMatrix3x3 
	D_btMatrix3x3::timesTranspose(const D_btMatrix3x3& m) const
	{
		return D_btMatrix3x3(
			m_el[0].dot(m[0]), m_el[0].dot(m[1]), m_el[0].dot(m[2]),
			m_el[1].dot(m[0]), m_el[1].dot(m[1]), m_el[1].dot(m[2]),
			m_el[2].dot(m[0]), m_el[2].dot(m[1]), m_el[2].dot(m[2]));
		
	}

	D_SIMD_FORCE_INLINE D_btVector3 
	operator*(const D_btMatrix3x3& m, const D_btVector3& v) 
	{
		return D_btVector3(m[0].dot(v), m[1].dot(v), m[2].dot(v));
	}
	

	D_SIMD_FORCE_INLINE D_btVector3
	operator*(const D_btVector3& v, const D_btMatrix3x3& m)
	{
		return D_btVector3(m.tdotx(v), m.tdoty(v), m.tdotz(v));
	}

	D_SIMD_FORCE_INLINE D_btMatrix3x3 
	operator*(const D_btMatrix3x3& m1, const D_btMatrix3x3& m2)
	{
		return D_btMatrix3x3(
			m2.tdotx( m1[0]), m2.tdoty( m1[0]), m2.tdotz( m1[0]),
			m2.tdotx( m1[1]), m2.tdoty( m1[1]), m2.tdotz( m1[1]),
			m2.tdotx( m1[2]), m2.tdoty( m1[2]), m2.tdotz( m1[2]));
	}

/*
	D_SIMD_FORCE_INLINE D_btMatrix3x3 D_btMultTransposeLeft(const D_btMatrix3x3& m1, const D_btMatrix3x3& m2) {
    return D_btMatrix3x3(
        m1[0][0] * m2[0][0] + m1[1][0] * m2[1][0] + m1[2][0] * m2[2][0],
        m1[0][0] * m2[0][1] + m1[1][0] * m2[1][1] + m1[2][0] * m2[2][1],
        m1[0][0] * m2[0][2] + m1[1][0] * m2[1][2] + m1[2][0] * m2[2][2],
        m1[0][1] * m2[0][0] + m1[1][1] * m2[1][0] + m1[2][1] * m2[2][0],
        m1[0][1] * m2[0][1] + m1[1][1] * m2[1][1] + m1[2][1] * m2[2][1],
        m1[0][1] * m2[0][2] + m1[1][1] * m2[1][2] + m1[2][1] * m2[2][2],
        m1[0][2] * m2[0][0] + m1[1][2] * m2[1][0] + m1[2][2] * m2[2][0],
        m1[0][2] * m2[0][1] + m1[1][2] * m2[1][1] + m1[2][2] * m2[2][1],
        m1[0][2] * m2[0][2] + m1[1][2] * m2[1][2] + m1[2][2] * m2[2][2]);
}
*/

/**@brief Equality operator between two matrices
 * It D_will test all elements D_are equal.  */
D_SIMD_FORCE_INLINE bool operator==(const D_btMatrix3x3& m1, const D_btMatrix3x3& m2)
{
   return ( m1[0][0] == m2[0][0] && m1[1][0] == m2[1][0] && m1[2][0] == m2[2][0] &&
            m1[0][1] == m2[0][1] && m1[1][1] == m2[1][1] && m1[2][1] == m2[2][1] &&
            m1[0][2] == m2[0][2] && m1[1][2] == m2[1][2] && m1[2][2] == m2[2][2] );
}

#endif
