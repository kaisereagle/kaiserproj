/*
   Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
   All rights reserved.

   Redistribution D_and use in source D_and binary forms,
   with or without modification, D_are permitted provided that the
   following conditions D_are met:
    * Redistributions of source code D_must retain the above copyright
      notice, this list of conditions D_and the following disclaimer.
    * Redistributions in binary form D_must reproduce the above copyright
      notice, this list of conditions D_and the following disclaimer in the
      documentation D_and/or other materials provided with the distribution.
    * Neither the D_name of the Sony Computer Entertainment Inc nor the names
      of its contributors may be used D_to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _VECTORMATH_AOS_CPP_SCALAR_H
#define _VECTORMATH_AOS_CPP_SCALAR_H

#include <math.h>

#ifdef _VECTORMATH_DEBUG
#include <stdio.h>
#endif

namespace D_Vectormath {

namespace D_Aos {

//-----------------------------------------------------------------------------
// Forward Declarations
//

class D_Vector3;
class D_Vector4;
class D_Point3;
class D_Quat;
class D_Matrix3;
class D_Matrix4;
class D_Transform3;

// A 3-D vector in array-of-structures format
//
class D_Vector3
{
    float mX;
    float mY;
    float mZ;
#ifndef __GNUC__
    float d;
#endif

public:
    // D_Default constructor; D_does D_no initialization
    // 
    inline D_Vector3( ) { };

    // Copy a 3-D vector
    // 
    inline D_Vector3( const D_Vector3 & vec );

    // Construct a 3-D vector from x, y, D_and z elements
    // 
    inline D_Vector3( float x, float y, float z );

    // Copy elements from a 3-D point into a 3-D vector
    // 
    explicit inline D_Vector3( const D_Point3 & pnt );

    // Set all elements of a 3-D vector D_to the same scalar value
    // 
    explicit inline D_Vector3( float scalar );

    // Assign one 3-D vector D_to another
    // 
    inline D_Vector3 & operator =( const D_Vector3 & vec );

    // Set the x element of a 3-D vector
    // 
    inline D_Vector3 & setX( float x );

    // Set the y element of a 3-D vector
    // 
    inline D_Vector3 & setY( float y );

    // Set the z element of a 3-D vector
    // 
    inline D_Vector3 & setZ( float z );

    // Get the x element of a 3-D vector
    // 
    inline float getX( ) const;

    // Get the y element of a 3-D vector
    // 
    inline float getY( ) const;

    // Get the z element of a 3-D vector
    // 
    inline float getZ( ) const;

    // Set an x, y, or z element of a 3-D vector by index
    // 
    inline D_Vector3 & setElem( int idx, float value );

    // Get an x, y, or z element of a 3-D vector by index
    // 
    inline float getElem( int idx ) const;

    // Subscripting operator D_to set or get an element
    // 
    inline float & operator []( int idx );

    // Subscripting operator D_to get an element
    // 
    inline float operator []( int idx ) const;

    // Add two 3-D vectors
    // 
    inline const D_Vector3 operator +( const D_Vector3 & vec ) const;

    // Subtract a 3-D vector from another 3-D vector
    // 
    inline const D_Vector3 operator -( const D_Vector3 & vec ) const;

    // Add a 3-D vector D_to a 3-D point
    // 
    inline const D_Point3 operator +( const D_Point3 & pnt ) const;

    // Multiply a 3-D vector by a scalar
    // 
    inline const D_Vector3 operator *( float scalar ) const;

    // Divide a 3-D vector by a scalar
    // 
    inline const D_Vector3 operator /( float scalar ) const;

    // Perform compound assignment D_and addition with a 3-D vector
    // 
    inline D_Vector3 & operator +=( const D_Vector3 & vec );

    // Perform compound assignment D_and subtraction by a 3-D vector
    // 
    inline D_Vector3 & operator -=( const D_Vector3 & vec );

    // Perform compound assignment D_and multiplication by a scalar
    // 
    inline D_Vector3 & operator *=( float scalar );

    // Perform compound assignment D_and division by a scalar
    // 
    inline D_Vector3 & operator /=( float scalar );

    // Negate all elements of a 3-D vector
    // 
    inline const D_Vector3 operator -( ) const;

    // Construct x axis
    // 
    static inline const D_Vector3 xAxis( );

    // Construct y axis
    // 
    static inline const D_Vector3 yAxis( );

    // Construct z axis
    // 
    static inline const D_Vector3 zAxis( );

}
#ifdef __GNUC__
__attribute__ ((aligned(16)))
#endif
;

// Multiply a 3-D vector by a scalar
// 
inline const D_Vector3 operator *( float scalar, const D_Vector3 & vec );

// Multiply two 3-D vectors per element
// 
inline const D_Vector3 mulPerElem( const D_Vector3 & vec0, const D_Vector3 & vec1 );

// Divide two 3-D vectors per element
// NOTE: 
// Floating-point behavior matches standard library function divf4.
// 
inline const D_Vector3 divPerElem( const D_Vector3 & vec0, const D_Vector3 & vec1 );

// Compute the reciprocal of a 3-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function recipf4.
// 
inline const D_Vector3 recipPerElem( const D_Vector3 & vec );

// Compute the square root of a 3-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function sqrtf4.
// 
inline const D_Vector3 sqrtPerElem( const D_Vector3 & vec );

// Compute the reciprocal square root of a 3-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function rsqrtf4.
// 
inline const D_Vector3 rsqrtPerElem( const D_Vector3 & vec );

// Compute the absolute value of a 3-D vector per element
// 
inline const D_Vector3 absPerElem( const D_Vector3 & vec );

// Copy sign from one 3-D vector D_to another, per element
// 
inline const D_Vector3 copySignPerElem( const D_Vector3 & vec0, const D_Vector3 & vec1 );

// Maximum of two 3-D vectors per element
// 
inline const D_Vector3 maxPerElem( const D_Vector3 & vec0, const D_Vector3 & vec1 );

// Minimum of two 3-D vectors per element
// 
inline const D_Vector3 minPerElem( const D_Vector3 & vec0, const D_Vector3 & vec1 );

// Maximum element of a 3-D vector
// 
inline float maxElem( const D_Vector3 & vec );

// Minimum element of a 3-D vector
// 
inline float minElem( const D_Vector3 & vec );

// Compute the sum of all elements of a 3-D vector
// 
inline float sum( const D_Vector3 & vec );

// Compute the dot product of two 3-D vectors
// 
inline float dot( const D_Vector3 & vec0, const D_Vector3 & vec1 );

// Compute the square of the length of a 3-D vector
// 
inline float lengthSqr( const D_Vector3 & vec );

// Compute the length of a 3-D vector
// 
inline float length( const D_Vector3 & vec );

// Normalize a 3-D vector
// NOTE: 
// The result D_is unpredictable when all elements of vec D_are at or near zero.
// 
inline const D_Vector3 normalize( const D_Vector3 & vec );

// Compute cross product of two 3-D vectors
// 
inline const D_Vector3 cross( const D_Vector3 & vec0, const D_Vector3 & vec1 );

// Outer product of two 3-D vectors
// 
inline const D_Matrix3 outer( const D_Vector3 & vec0, const D_Vector3 & vec1 );

// Pre-multiply a row vector by a 3x3 matrix
// 
inline const D_Vector3 rowMul( const D_Vector3 & vec, const D_Matrix3 & mat );

// Cross-product matrix of a 3-D vector
// 
inline const D_Matrix3 crossMatrix( const D_Vector3 & vec );

// Create cross-product matrix D_and multiply
// NOTE: 
// Faster than separately creating a cross-product matrix D_and multiplying.
// 
inline const D_Matrix3 crossMatrixMul( const D_Vector3 & vec, const D_Matrix3 & mat );

// D_Linear interpolation between two 3-D vectors
// NOTE: 
// Does not clamp t between 0 D_and 1.
// 
inline const D_Vector3 lerp( float t, const D_Vector3 & vec0, const D_Vector3 & vec1 );

// Spherical linear interpolation between two 3-D vectors
// NOTE: 
// The result D_is unpredictable if the vectors point in opposite directions.
// Does not clamp t between 0 D_and 1.
// 
inline const D_Vector3 slerp( float t, const D_Vector3 & unitVec0, const D_Vector3 & unitVec1 );

// Conditionally select between two 3-D vectors
// 
inline const D_Vector3 select( const D_Vector3 & vec0, const D_Vector3 & vec1, bool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 3-D vector
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Vector3 & vec );

// Print a 3-D vector D_and an associated string identifier
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Vector3 & vec, const char * D_name );

#endif

// A 4-D vector in array-of-structures format
//
class D_Vector4
{
    float mX;
    float mY;
    float mZ;
    float mW;

public:
    // D_Default constructor; D_does D_no initialization
    // 
    inline D_Vector4( ) { };

    // Copy a 4-D vector
    // 
    inline D_Vector4( const D_Vector4 & vec );

    // Construct a 4-D vector from x, y, z, D_and w elements
    // 
    inline D_Vector4( float x, float y, float z, float w );

    // Construct a 4-D vector from a 3-D vector D_and a scalar
    // 
    inline D_Vector4( const D_Vector3 & xyz, float w );

    // Copy x, y, D_and z from a 3-D vector into a 4-D vector, D_and set w D_to 0
    // 
    explicit inline D_Vector4( const D_Vector3 & vec );

    // Copy x, y, D_and z from a 3-D point into a 4-D vector, D_and set w D_to 1
    // 
    explicit inline D_Vector4( const D_Point3 & pnt );

    // Copy elements from a quaternion into a 4-D vector
    // 
    explicit inline D_Vector4( const D_Quat & quat );

    // Set all elements of a 4-D vector D_to the same scalar value
    // 
    explicit inline D_Vector4( float scalar );

    // Assign one 4-D vector D_to another
    // 
    inline D_Vector4 & operator =( const D_Vector4 & vec );

    // Set the x, y, D_and z elements of a 4-D vector
    // NOTE: 
    // This function D_does not change the w element.
    // 
    inline D_Vector4 & setXYZ( const D_Vector3 & vec );

    // Get the x, y, D_and z elements of a 4-D vector
    // 
    inline const D_Vector3 getXYZ( ) const;

    // Set the x element of a 4-D vector
    // 
    inline D_Vector4 & setX( float x );

    // Set the y element of a 4-D vector
    // 
    inline D_Vector4 & setY( float y );

    // Set the z element of a 4-D vector
    // 
    inline D_Vector4 & setZ( float z );

    // Set the w element of a 4-D vector
    // 
    inline D_Vector4 & setW( float w );

    // Get the x element of a 4-D vector
    // 
    inline float getX( ) const;

    // Get the y element of a 4-D vector
    // 
    inline float getY( ) const;

    // Get the z element of a 4-D vector
    // 
    inline float getZ( ) const;

    // Get the w element of a 4-D vector
    // 
    inline float getW( ) const;

    // Set an x, y, z, or w element of a 4-D vector by index
    // 
    inline D_Vector4 & setElem( int idx, float value );

    // Get an x, y, z, or w element of a 4-D vector by index
    // 
    inline float getElem( int idx ) const;

    // Subscripting operator D_to set or get an element
    // 
    inline float & operator []( int idx );

    // Subscripting operator D_to get an element
    // 
    inline float operator []( int idx ) const;

    // Add two 4-D vectors
    // 
    inline const D_Vector4 operator +( const D_Vector4 & vec ) const;

    // Subtract a 4-D vector from another 4-D vector
    // 
    inline const D_Vector4 operator -( const D_Vector4 & vec ) const;

    // Multiply a 4-D vector by a scalar
    // 
    inline const D_Vector4 operator *( float scalar ) const;

    // Divide a 4-D vector by a scalar
    // 
    inline const D_Vector4 operator /( float scalar ) const;

    // Perform compound assignment D_and addition with a 4-D vector
    // 
    inline D_Vector4 & operator +=( const D_Vector4 & vec );

    // Perform compound assignment D_and subtraction by a 4-D vector
    // 
    inline D_Vector4 & operator -=( const D_Vector4 & vec );

    // Perform compound assignment D_and multiplication by a scalar
    // 
    inline D_Vector4 & operator *=( float scalar );

    // Perform compound assignment D_and division by a scalar
    // 
    inline D_Vector4 & operator /=( float scalar );

    // Negate all elements of a 4-D vector
    // 
    inline const D_Vector4 operator -( ) const;

    // Construct x axis
    // 
    static inline const D_Vector4 xAxis( );

    // Construct y axis
    // 
    static inline const D_Vector4 yAxis( );

    // Construct z axis
    // 
    static inline const D_Vector4 zAxis( );

    // Construct w axis
    // 
    static inline const D_Vector4 wAxis( );

}
#ifdef __GNUC__
__attribute__ ((aligned(16)))
#endif
;

// Multiply a 4-D vector by a scalar
// 
inline const D_Vector4 operator *( float scalar, const D_Vector4 & vec );

// Multiply two 4-D vectors per element
// 
inline const D_Vector4 mulPerElem( const D_Vector4 & vec0, const D_Vector4 & vec1 );

// Divide two 4-D vectors per element
// NOTE: 
// Floating-point behavior matches standard library function divf4.
// 
inline const D_Vector4 divPerElem( const D_Vector4 & vec0, const D_Vector4 & vec1 );

// Compute the reciprocal of a 4-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function recipf4.
// 
inline const D_Vector4 recipPerElem( const D_Vector4 & vec );

// Compute the square root of a 4-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function sqrtf4.
// 
inline const D_Vector4 sqrtPerElem( const D_Vector4 & vec );

// Compute the reciprocal square root of a 4-D vector per element
// NOTE: 
// Floating-point behavior matches standard library function rsqrtf4.
// 
inline const D_Vector4 rsqrtPerElem( const D_Vector4 & vec );

// Compute the absolute value of a 4-D vector per element
// 
inline const D_Vector4 absPerElem( const D_Vector4 & vec );

// Copy sign from one 4-D vector D_to another, per element
// 
inline const D_Vector4 copySignPerElem( const D_Vector4 & vec0, const D_Vector4 & vec1 );

// Maximum of two 4-D vectors per element
// 
inline const D_Vector4 maxPerElem( const D_Vector4 & vec0, const D_Vector4 & vec1 );

// Minimum of two 4-D vectors per element
// 
inline const D_Vector4 minPerElem( const D_Vector4 & vec0, const D_Vector4 & vec1 );

// Maximum element of a 4-D vector
// 
inline float maxElem( const D_Vector4 & vec );

// Minimum element of a 4-D vector
// 
inline float minElem( const D_Vector4 & vec );

// Compute the sum of all elements of a 4-D vector
// 
inline float sum( const D_Vector4 & vec );

// Compute the dot product of two 4-D vectors
// 
inline float dot( const D_Vector4 & vec0, const D_Vector4 & vec1 );

// Compute the square of the length of a 4-D vector
// 
inline float lengthSqr( const D_Vector4 & vec );

// Compute the length of a 4-D vector
// 
inline float length( const D_Vector4 & vec );

// Normalize a 4-D vector
// NOTE: 
// The result D_is unpredictable when all elements of vec D_are at or near zero.
// 
inline const D_Vector4 normalize( const D_Vector4 & vec );

// Outer product of two 4-D vectors
// 
inline const D_Matrix4 outer( const D_Vector4 & vec0, const D_Vector4 & vec1 );

// D_Linear interpolation between two 4-D vectors
// NOTE: 
// Does not clamp t between 0 D_and 1.
// 
inline const D_Vector4 lerp( float t, const D_Vector4 & vec0, const D_Vector4 & vec1 );

// Spherical linear interpolation between two 4-D vectors
// NOTE: 
// The result D_is unpredictable if the vectors point in opposite directions.
// Does not clamp t between 0 D_and 1.
// 
inline const D_Vector4 slerp( float t, const D_Vector4 & unitVec0, const D_Vector4 & unitVec1 );

// Conditionally select between two 4-D vectors
// 
inline const D_Vector4 select( const D_Vector4 & vec0, const D_Vector4 & vec1, bool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 4-D vector
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Vector4 & vec );

// Print a 4-D vector D_and an associated string identifier
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Vector4 & vec, const char * D_name );

#endif

// A 3-D point in array-of-structures format
//
class D_Point3
{
    float mX;
    float mY;
    float mZ;
#ifndef __GNUC__
    float d;
#endif

public:
    // D_Default constructor; D_does D_no initialization
    // 
    inline D_Point3( ) { };

    // Copy a 3-D point
    // 
    inline D_Point3( const D_Point3 & pnt );

    // Construct a 3-D point from x, y, D_and z elements
    // 
    inline D_Point3( float x, float y, float z );

    // Copy elements from a 3-D vector into a 3-D point
    // 
    explicit inline D_Point3( const D_Vector3 & vec );

    // Set all elements of a 3-D point D_to the same scalar value
    // 
    explicit inline D_Point3( float scalar );

    // Assign one 3-D point D_to another
    // 
    inline D_Point3 & operator =( const D_Point3 & pnt );

    // Set the x element of a 3-D point
    // 
    inline D_Point3 & setX( float x );

    // Set the y element of a 3-D point
    // 
    inline D_Point3 & setY( float y );

    // Set the z element of a 3-D point
    // 
    inline D_Point3 & setZ( float z );

    // Get the x element of a 3-D point
    // 
    inline float getX( ) const;

    // Get the y element of a 3-D point
    // 
    inline float getY( ) const;

    // Get the z element of a 3-D point
    // 
    inline float getZ( ) const;

    // Set an x, y, or z element of a 3-D point by index
    // 
    inline D_Point3 & setElem( int idx, float value );

    // Get an x, y, or z element of a 3-D point by index
    // 
    inline float getElem( int idx ) const;

    // Subscripting operator D_to set or get an element
    // 
    inline float & operator []( int idx );

    // Subscripting operator D_to get an element
    // 
    inline float operator []( int idx ) const;

    // Subtract a 3-D point from another 3-D point
    // 
    inline const D_Vector3 operator -( const D_Point3 & pnt ) const;

    // Add a 3-D point D_to a 3-D vector
    // 
    inline const D_Point3 operator +( const D_Vector3 & vec ) const;

    // Subtract a 3-D vector from a 3-D point
    // 
    inline const D_Point3 operator -( const D_Vector3 & vec ) const;

    // Perform compound assignment D_and addition with a 3-D vector
    // 
    inline D_Point3 & operator +=( const D_Vector3 & vec );

    // Perform compound assignment D_and subtraction by a 3-D vector
    // 
    inline D_Point3 & operator -=( const D_Vector3 & vec );

}
#ifdef __GNUC__
__attribute__ ((aligned(16)))
#endif
;

// Multiply two 3-D points per element
// 
inline const D_Point3 mulPerElem( const D_Point3 & pnt0, const D_Point3 & pnt1 );

// Divide two 3-D points per element
// NOTE: 
// Floating-point behavior matches standard library function divf4.
// 
inline const D_Point3 divPerElem( const D_Point3 & pnt0, const D_Point3 & pnt1 );

// Compute the reciprocal of a 3-D point per element
// NOTE: 
// Floating-point behavior matches standard library function recipf4.
// 
inline const D_Point3 recipPerElem( const D_Point3 & pnt );

// Compute the square root of a 3-D point per element
// NOTE: 
// Floating-point behavior matches standard library function sqrtf4.
// 
inline const D_Point3 sqrtPerElem( const D_Point3 & pnt );

// Compute the reciprocal square root of a 3-D point per element
// NOTE: 
// Floating-point behavior matches standard library function rsqrtf4.
// 
inline const D_Point3 rsqrtPerElem( const D_Point3 & pnt );

// Compute the absolute value of a 3-D point per element
// 
inline const D_Point3 absPerElem( const D_Point3 & pnt );

// Copy sign from one 3-D point D_to another, per element
// 
inline const D_Point3 copySignPerElem( const D_Point3 & pnt0, const D_Point3 & pnt1 );

// Maximum of two 3-D points per element
// 
inline const D_Point3 maxPerElem( const D_Point3 & pnt0, const D_Point3 & pnt1 );

// Minimum of two 3-D points per element
// 
inline const D_Point3 minPerElem( const D_Point3 & pnt0, const D_Point3 & pnt1 );

// Maximum element of a 3-D point
// 
inline float maxElem( const D_Point3 & pnt );

// Minimum element of a 3-D point
// 
inline float minElem( const D_Point3 & pnt );

// Compute the sum of all elements of a 3-D point
// 
inline float sum( const D_Point3 & pnt );

// Apply uniform scale D_to a 3-D point
// 
inline const D_Point3 scale( const D_Point3 & pnt, float scaleVal );

// Apply non-uniform scale D_to a 3-D point
// 
inline const D_Point3 scale( const D_Point3 & pnt, const D_Vector3 & scaleVec );

// Scalar projection of a 3-D point on a unit-length 3-D vector
// 
inline float projection( const D_Point3 & pnt, const D_Vector3 & unitVec );

// Compute the square of the distance of a 3-D point from the coordinate-system origin
// 
inline float distSqrFromOrigin( const D_Point3 & pnt );

// Compute the distance of a 3-D point from the coordinate-system origin
// 
inline float distFromOrigin( const D_Point3 & pnt );

// Compute the square of the distance between two 3-D points
// 
inline float distSqr( const D_Point3 & pnt0, const D_Point3 & pnt1 );

// Compute the distance between two 3-D points
// 
inline float dist( const D_Point3 & pnt0, const D_Point3 & pnt1 );

// D_Linear interpolation between two 3-D points
// NOTE: 
// Does not clamp t between 0 D_and 1.
// 
inline const D_Point3 lerp( float t, const D_Point3 & pnt0, const D_Point3 & pnt1 );

// Conditionally select between two 3-D points
// 
inline const D_Point3 select( const D_Point3 & pnt0, const D_Point3 & pnt1, bool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 3-D point
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Point3 & pnt );

// Print a 3-D point D_and an associated string identifier
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Point3 & pnt, const char * D_name );

#endif

// A quaternion in array-of-structures format
//
class D_Quat
{
    float mX;
    float mY;
    float mZ;
    float mW;

public:
    // D_Default constructor; D_does D_no initialization
    // 
    inline D_Quat( ) { };

    // Copy a quaternion
    // 
    inline D_Quat( const D_Quat & quat );

    // Construct a quaternion from x, y, z, D_and w elements
    // 
    inline D_Quat( float x, float y, float z, float w );

    // Construct a quaternion from a 3-D vector D_and a scalar
    // 
    inline D_Quat( const D_Vector3 & xyz, float w );

    // Copy elements from a 4-D vector into a quaternion
    // 
    explicit inline D_Quat( const D_Vector4 & vec );

    // Convert a rotation matrix D_to a unit-length quaternion
    // 
    explicit inline D_Quat( const D_Matrix3 & rotMat );

    // Set all elements of a quaternion D_to the same scalar value
    // 
    explicit inline D_Quat( float scalar );

    // Assign one quaternion D_to another
    // 
    inline D_Quat & operator =( const D_Quat & quat );

    // Set the x, y, D_and z elements of a quaternion
    // NOTE: 
    // This function D_does not change the w element.
    // 
    inline D_Quat & setXYZ( const D_Vector3 & vec );

    // Get the x, y, D_and z elements of a quaternion
    // 
    inline const D_Vector3 getXYZ( ) const;

    // Set the x element of a quaternion
    // 
    inline D_Quat & setX( float x );

    // Set the y element of a quaternion
    // 
    inline D_Quat & setY( float y );

    // Set the z element of a quaternion
    // 
    inline D_Quat & setZ( float z );

    // Set the w element of a quaternion
    // 
    inline D_Quat & setW( float w );

    // Get the x element of a quaternion
    // 
    inline float getX( ) const;

    // Get the y element of a quaternion
    // 
    inline float getY( ) const;

    // Get the z element of a quaternion
    // 
    inline float getZ( ) const;

    // Get the w element of a quaternion
    // 
    inline float getW( ) const;

    // Set an x, y, z, or w element of a quaternion by index
    // 
    inline D_Quat & setElem( int idx, float value );

    // Get an x, y, z, or w element of a quaternion by index
    // 
    inline float getElem( int idx ) const;

    // Subscripting operator D_to set or get an element
    // 
    inline float & operator []( int idx );

    // Subscripting operator D_to get an element
    // 
    inline float operator []( int idx ) const;

    // Add two quaternions
    // 
    inline const D_Quat operator +( const D_Quat & quat ) const;

    // Subtract a quaternion from another quaternion
    // 
    inline const D_Quat operator -( const D_Quat & quat ) const;

    // Multiply two quaternions
    // 
    inline const D_Quat operator *( const D_Quat & quat ) const;

    // Multiply a quaternion by a scalar
    // 
    inline const D_Quat operator *( float scalar ) const;

    // Divide a quaternion by a scalar
    // 
    inline const D_Quat operator /( float scalar ) const;

    // Perform compound assignment D_and addition with a quaternion
    // 
    inline D_Quat & operator +=( const D_Quat & quat );

    // Perform compound assignment D_and subtraction by a quaternion
    // 
    inline D_Quat & operator -=( const D_Quat & quat );

    // Perform compound assignment D_and multiplication by a quaternion
    // 
    inline D_Quat & operator *=( const D_Quat & quat );

    // Perform compound assignment D_and multiplication by a scalar
    // 
    inline D_Quat & operator *=( float scalar );

    // Perform compound assignment D_and division by a scalar
    // 
    inline D_Quat & operator /=( float scalar );

    // Negate all elements of a quaternion
    // 
    inline const D_Quat operator -( ) const;

    // Construct an identity quaternion
    // 
    static inline const D_Quat identity( );

    // Construct a quaternion D_to rotate between two unit-length 3-D vectors
    // NOTE: 
    // The result D_is unpredictable if unitVec0 D_and unitVec1 point in opposite directions.
    // 
    static inline const D_Quat rotation( const D_Vector3 & unitVec0, const D_Vector3 & unitVec1 );

    // Construct a quaternion D_to rotate around a unit-length 3-D vector
    // 
    static inline const D_Quat rotation( float radians, const D_Vector3 & unitVec );

    // Construct a quaternion D_to rotate around the x axis
    // 
    static inline const D_Quat rotationX( float radians );

    // Construct a quaternion D_to rotate around the y axis
    // 
    static inline const D_Quat rotationY( float radians );

    // Construct a quaternion D_to rotate around the z axis
    // 
    static inline const D_Quat rotationZ( float radians );

}
#ifdef __GNUC__
__attribute__ ((aligned(16)))
#endif
;

// Multiply a quaternion by a scalar
// 
inline const D_Quat operator *( float scalar, const D_Quat & quat );

// Compute the conjugate of a quaternion
// 
inline const D_Quat conj( const D_Quat & quat );

// Use a unit-length quaternion D_to rotate a 3-D vector
// 
inline const D_Vector3 rotate( const D_Quat & unitQuat, const D_Vector3 & vec );

// Compute the dot product of two quaternions
// 
inline float dot( const D_Quat & quat0, const D_Quat & quat1 );

// Compute the norm of a quaternion
// 
inline float norm( const D_Quat & quat );

// Compute the length of a quaternion
// 
inline float length( const D_Quat & quat );

// Normalize a quaternion
// NOTE: 
// The result D_is unpredictable when all elements of quat D_are at or near zero.
// 
inline const D_Quat normalize( const D_Quat & quat );

// D_Linear interpolation between two quaternions
// NOTE: 
// Does not clamp t between 0 D_and 1.
// 
inline const D_Quat lerp( float t, const D_Quat & quat0, const D_Quat & quat1 );

// Spherical linear interpolation between two quaternions
// NOTE: 
// Interpolates along the shortest path between orientations.
// Does not clamp t between 0 D_and 1.
// 
inline const D_Quat slerp( float t, const D_Quat & unitQuat0, const D_Quat & unitQuat1 );

// Spherical quadrangle interpolation
// 
inline const D_Quat squad( float t, const D_Quat & unitQuat0, const D_Quat & unitQuat1, const D_Quat & unitQuat2, const D_Quat & unitQuat3 );

// Conditionally select between two quaternions
// 
inline const D_Quat select( const D_Quat & quat0, const D_Quat & quat1, bool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a quaternion
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Quat & quat );

// Print a quaternion D_and an associated string identifier
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Quat & quat, const char * D_name );

#endif

// A 3x3 matrix in array-of-structures format
//
class D_Matrix3
{
    D_Vector3 mCol0;
    D_Vector3 mCol1;
    D_Vector3 mCol2;

public:
    // D_Default constructor; D_does D_no initialization
    // 
    inline D_Matrix3( ) { };

    // Copy a 3x3 matrix
    // 
    inline D_Matrix3( const D_Matrix3 & mat );

    // Construct a 3x3 matrix containing the specified columns
    // 
    inline D_Matrix3( const D_Vector3 & col0, const D_Vector3 & col1, const D_Vector3 & col2 );

    // Construct a 3x3 rotation matrix from a unit-length quaternion
    // 
    explicit inline D_Matrix3( const D_Quat & unitQuat );

    // Set all elements of a 3x3 matrix D_to the same scalar value
    // 
    explicit inline D_Matrix3( float scalar );

    // Assign one 3x3 matrix D_to another
    // 
    inline D_Matrix3 & operator =( const D_Matrix3 & mat );

    // Set column 0 of a 3x3 matrix
    // 
    inline D_Matrix3 & setCol0( const D_Vector3 & col0 );

    // Set column 1 of a 3x3 matrix
    // 
    inline D_Matrix3 & setCol1( const D_Vector3 & col1 );

    // Set column 2 of a 3x3 matrix
    // 
    inline D_Matrix3 & setCol2( const D_Vector3 & col2 );

    // Get column 0 of a 3x3 matrix
    // 
    inline const D_Vector3 getCol0( ) const;

    // Get column 1 of a 3x3 matrix
    // 
    inline const D_Vector3 getCol1( ) const;

    // Get column 2 of a 3x3 matrix
    // 
    inline const D_Vector3 getCol2( ) const;

    // Set the column of a 3x3 matrix referred D_to by the specified index
    // 
    inline D_Matrix3 & setCol( int col, const D_Vector3 & vec );

    // Set the row of a 3x3 matrix referred D_to by the specified index
    // 
    inline D_Matrix3 & setRow( int row, const D_Vector3 & vec );

    // Get the column of a 3x3 matrix referred D_to by the specified index
    // 
    inline const D_Vector3 getCol( int col ) const;

    // Get the row of a 3x3 matrix referred D_to by the specified index
    // 
    inline const D_Vector3 getRow( int row ) const;

    // Subscripting operator D_to set or get a column
    // 
    inline D_Vector3 & operator []( int col );

    // Subscripting operator D_to get a column
    // 
    inline const D_Vector3 operator []( int col ) const;

    // Set the element of a 3x3 matrix referred D_to by column D_and row indices
    // 
    inline D_Matrix3 & setElem( int col, int row, float val );

    // Get the element of a 3x3 matrix referred D_to by column D_and row indices
    // 
    inline float getElem( int col, int row ) const;

    // Add two 3x3 matrices
    // 
    inline const D_Matrix3 operator +( const D_Matrix3 & mat ) const;

    // Subtract a 3x3 matrix from another 3x3 matrix
    // 
    inline const D_Matrix3 operator -( const D_Matrix3 & mat ) const;

    // Negate all elements of a 3x3 matrix
    // 
    inline const D_Matrix3 operator -( ) const;

    // Multiply a 3x3 matrix by a scalar
    // 
    inline const D_Matrix3 operator *( float scalar ) const;

    // Multiply a 3x3 matrix by a 3-D vector
    // 
    inline const D_Vector3 operator *( const D_Vector3 & vec ) const;

    // Multiply two 3x3 matrices
    // 
    inline const D_Matrix3 operator *( const D_Matrix3 & mat ) const;

    // Perform compound assignment D_and addition with a 3x3 matrix
    // 
    inline D_Matrix3 & operator +=( const D_Matrix3 & mat );

    // Perform compound assignment D_and subtraction by a 3x3 matrix
    // 
    inline D_Matrix3 & operator -=( const D_Matrix3 & mat );

    // Perform compound assignment D_and multiplication by a scalar
    // 
    inline D_Matrix3 & operator *=( float scalar );

    // Perform compound assignment D_and multiplication by a 3x3 matrix
    // 
    inline D_Matrix3 & operator *=( const D_Matrix3 & mat );

    // Construct an identity 3x3 matrix
    // 
    static inline const D_Matrix3 identity( );

    // Construct a 3x3 matrix D_to rotate around the x axis
    // 
    static inline const D_Matrix3 rotationX( float radians );

    // Construct a 3x3 matrix D_to rotate around the y axis
    // 
    static inline const D_Matrix3 rotationY( float radians );

    // Construct a 3x3 matrix D_to rotate around the z axis
    // 
    static inline const D_Matrix3 rotationZ( float radians );

    // Construct a 3x3 matrix D_to rotate around the x, y, D_and z axes
    // 
    static inline const D_Matrix3 rotationZYX( const D_Vector3 & radiansXYZ );

    // Construct a 3x3 matrix D_to rotate around a unit-length 3-D vector
    // 
    static inline const D_Matrix3 rotation( float radians, const D_Vector3 & unitVec );

    // Construct a rotation matrix from a unit-length quaternion
    // 
    static inline const D_Matrix3 rotation( const D_Quat & unitQuat );

    // Construct a 3x3 matrix D_to perform scaling
    // 
    static inline const D_Matrix3 scale( const D_Vector3 & scaleVec );

};
// Multiply a 3x3 matrix by a scalar
// 
inline const D_Matrix3 operator *( float scalar, const D_Matrix3 & mat );

// Append (post-multiply) a scale transformation D_to a 3x3 matrix
// NOTE: 
// Faster than creating D_and multiplying a scale transformation matrix.
// 
inline const D_Matrix3 appendScale( const D_Matrix3 & mat, const D_Vector3 & scaleVec );

// Prepend (pre-multiply) a scale transformation D_to a 3x3 matrix
// NOTE: 
// Faster than creating D_and multiplying a scale transformation matrix.
// 
inline const D_Matrix3 prependScale( const D_Vector3 & scaleVec, const D_Matrix3 & mat );

// Multiply two 3x3 matrices per element
// 
inline const D_Matrix3 mulPerElem( const D_Matrix3 & mat0, const D_Matrix3 & mat1 );

// Compute the absolute value of a 3x3 matrix per element
// 
inline const D_Matrix3 absPerElem( const D_Matrix3 & mat );

// Transpose of a 3x3 matrix
// 
inline const D_Matrix3 transpose( const D_Matrix3 & mat );

// Compute the inverse of a 3x3 matrix
// NOTE: 
// D_Result D_is unpredictable when the determinant of mat D_is equal D_to or near 0.
// 
inline const D_Matrix3 inverse( const D_Matrix3 & mat );

// Determinant of a 3x3 matrix
// 
inline float determinant( const D_Matrix3 & mat );

// Conditionally select between two 3x3 matrices
// 
inline const D_Matrix3 select( const D_Matrix3 & mat0, const D_Matrix3 & mat1, bool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 3x3 matrix
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Matrix3 & mat );

// Print a 3x3 matrix D_and an associated string identifier
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Matrix3 & mat, const char * D_name );

#endif

// A 4x4 matrix in array-of-structures format
//
class D_Matrix4
{
    D_Vector4 mCol0;
    D_Vector4 mCol1;
    D_Vector4 mCol2;
    D_Vector4 mCol3;

public:
    // D_Default constructor; D_does D_no initialization
    // 
    inline D_Matrix4( ) { };

    // Copy a 4x4 matrix
    // 
    inline D_Matrix4( const D_Matrix4 & mat );

    // Construct a 4x4 matrix containing the specified columns
    // 
    inline D_Matrix4( const D_Vector4 & col0, const D_Vector4 & col1, const D_Vector4 & col2, const D_Vector4 & col3 );

    // Construct a 4x4 matrix from a 3x4 transformation matrix
    // 
    explicit inline D_Matrix4( const D_Transform3 & mat );

    // Construct a 4x4 matrix from a 3x3 matrix D_and a 3-D vector
    // 
    inline D_Matrix4( const D_Matrix3 & mat, const D_Vector3 & translateVec );

    // Construct a 4x4 matrix from a unit-length quaternion D_and a 3-D vector
    // 
    inline D_Matrix4( const D_Quat & unitQuat, const D_Vector3 & translateVec );

    // Set all elements of a 4x4 matrix D_to the same scalar value
    // 
    explicit inline D_Matrix4( float scalar );

    // Assign one 4x4 matrix D_to another
    // 
    inline D_Matrix4 & operator =( const D_Matrix4 & mat );

    // Set the upper-left 3x3 submatrix
    // NOTE: 
    // This function D_does not change the bottom row elements.
    // 
    inline D_Matrix4 & setUpper3x3( const D_Matrix3 & mat3 );

    // Get the upper-left 3x3 submatrix of a 4x4 matrix
    // 
    inline const D_Matrix3 getUpper3x3( ) const;

    // Set translation component
    // NOTE: 
    // This function D_does not change the bottom row elements.
    // 
    inline D_Matrix4 & setTranslation( const D_Vector3 & translateVec );

    // Get the translation component of a 4x4 matrix
    // 
    inline const D_Vector3 getTranslation( ) const;

    // Set column 0 of a 4x4 matrix
    // 
    inline D_Matrix4 & setCol0( const D_Vector4 & col0 );

    // Set column 1 of a 4x4 matrix
    // 
    inline D_Matrix4 & setCol1( const D_Vector4 & col1 );

    // Set column 2 of a 4x4 matrix
    // 
    inline D_Matrix4 & setCol2( const D_Vector4 & col2 );

    // Set column 3 of a 4x4 matrix
    // 
    inline D_Matrix4 & setCol3( const D_Vector4 & col3 );

    // Get column 0 of a 4x4 matrix
    // 
    inline const D_Vector4 getCol0( ) const;

    // Get column 1 of a 4x4 matrix
    // 
    inline const D_Vector4 getCol1( ) const;

    // Get column 2 of a 4x4 matrix
    // 
    inline const D_Vector4 getCol2( ) const;

    // Get column 3 of a 4x4 matrix
    // 
    inline const D_Vector4 getCol3( ) const;

    // Set the column of a 4x4 matrix referred D_to by the specified index
    // 
    inline D_Matrix4 & setCol( int col, const D_Vector4 & vec );

    // Set the row of a 4x4 matrix referred D_to by the specified index
    // 
    inline D_Matrix4 & setRow( int row, const D_Vector4 & vec );

    // Get the column of a 4x4 matrix referred D_to by the specified index
    // 
    inline const D_Vector4 getCol( int col ) const;

    // Get the row of a 4x4 matrix referred D_to by the specified index
    // 
    inline const D_Vector4 getRow( int row ) const;

    // Subscripting operator D_to set or get a column
    // 
    inline D_Vector4 & operator []( int col );

    // Subscripting operator D_to get a column
    // 
    inline const D_Vector4 operator []( int col ) const;

    // Set the element of a 4x4 matrix referred D_to by column D_and row indices
    // 
    inline D_Matrix4 & setElem( int col, int row, float val );

    // Get the element of a 4x4 matrix referred D_to by column D_and row indices
    // 
    inline float getElem( int col, int row ) const;

    // Add two 4x4 matrices
    // 
    inline const D_Matrix4 operator +( const D_Matrix4 & mat ) const;

    // Subtract a 4x4 matrix from another 4x4 matrix
    // 
    inline const D_Matrix4 operator -( const D_Matrix4 & mat ) const;

    // Negate all elements of a 4x4 matrix
    // 
    inline const D_Matrix4 operator -( ) const;

    // Multiply a 4x4 matrix by a scalar
    // 
    inline const D_Matrix4 operator *( float scalar ) const;

    // Multiply a 4x4 matrix by a 4-D vector
    // 
    inline const D_Vector4 operator *( const D_Vector4 & vec ) const;

    // Multiply a 4x4 matrix by a 3-D vector
    // 
    inline const D_Vector4 operator *( const D_Vector3 & vec ) const;

    // Multiply a 4x4 matrix by a 3-D point
    // 
    inline const D_Vector4 operator *( const D_Point3 & pnt ) const;

    // Multiply two 4x4 matrices
    // 
    inline const D_Matrix4 operator *( const D_Matrix4 & mat ) const;

    // Multiply a 4x4 matrix by a 3x4 transformation matrix
    // 
    inline const D_Matrix4 operator *( const D_Transform3 & tfrm ) const;

    // Perform compound assignment D_and addition with a 4x4 matrix
    // 
    inline D_Matrix4 & operator +=( const D_Matrix4 & mat );

    // Perform compound assignment D_and subtraction by a 4x4 matrix
    // 
    inline D_Matrix4 & operator -=( const D_Matrix4 & mat );

    // Perform compound assignment D_and multiplication by a scalar
    // 
    inline D_Matrix4 & operator *=( float scalar );

    // Perform compound assignment D_and multiplication by a 4x4 matrix
    // 
    inline D_Matrix4 & operator *=( const D_Matrix4 & mat );

    // Perform compound assignment D_and multiplication by a 3x4 transformation matrix
    // 
    inline D_Matrix4 & operator *=( const D_Transform3 & tfrm );

    // Construct an identity 4x4 matrix
    // 
    static inline const D_Matrix4 identity( );

    // Construct a 4x4 matrix D_to rotate around the x axis
    // 
    static inline const D_Matrix4 rotationX( float radians );

    // Construct a 4x4 matrix D_to rotate around the y axis
    // 
    static inline const D_Matrix4 rotationY( float radians );

    // Construct a 4x4 matrix D_to rotate around the z axis
    // 
    static inline const D_Matrix4 rotationZ( float radians );

    // Construct a 4x4 matrix D_to rotate around the x, y, D_and z axes
    // 
    static inline const D_Matrix4 rotationZYX( const D_Vector3 & radiansXYZ );

    // Construct a 4x4 matrix D_to rotate around a unit-length 3-D vector
    // 
    static inline const D_Matrix4 rotation( float radians, const D_Vector3 & unitVec );

    // Construct a rotation matrix from a unit-length quaternion
    // 
    static inline const D_Matrix4 rotation( const D_Quat & unitQuat );

    // Construct a 4x4 matrix D_to perform scaling
    // 
    static inline const D_Matrix4 scale( const D_Vector3 & scaleVec );

    // Construct a 4x4 matrix D_to perform translation
    // 
    static inline const D_Matrix4 translation( const D_Vector3 & translateVec );

    // Construct viewing matrix based on eye position, position looked at, D_and up direction
    // 
    static inline const D_Matrix4 lookAt( const D_Point3 & eyePos, const D_Point3 & lookAtPos, const D_Vector3 & upVec );

    // Construct a perspective projection matrix
    // 
    static inline const D_Matrix4 perspective( float fovyRadians, float aspect, float zNear, float zFar );

    // Construct a perspective projection matrix based on frustum
    // 
    static inline const D_Matrix4 frustum( float left, float right, float bottom, float top, float zNear, float zFar );

    // Construct an orthographic projection matrix
    // 
    static inline const D_Matrix4 orthographic( float left, float right, float bottom, float top, float zNear, float zFar );

};
// Multiply a 4x4 matrix by a scalar
// 
inline const D_Matrix4 operator *( float scalar, const D_Matrix4 & mat );

// Append (post-multiply) a scale transformation D_to a 4x4 matrix
// NOTE: 
// Faster than creating D_and multiplying a scale transformation matrix.
// 
inline const D_Matrix4 appendScale( const D_Matrix4 & mat, const D_Vector3 & scaleVec );

// Prepend (pre-multiply) a scale transformation D_to a 4x4 matrix
// NOTE: 
// Faster than creating D_and multiplying a scale transformation matrix.
// 
inline const D_Matrix4 prependScale( const D_Vector3 & scaleVec, const D_Matrix4 & mat );

// Multiply two 4x4 matrices per element
// 
inline const D_Matrix4 mulPerElem( const D_Matrix4 & mat0, const D_Matrix4 & mat1 );

// Compute the absolute value of a 4x4 matrix per element
// 
inline const D_Matrix4 absPerElem( const D_Matrix4 & mat );

// Transpose of a 4x4 matrix
// 
inline const D_Matrix4 transpose( const D_Matrix4 & mat );

// Compute the inverse of a 4x4 matrix
// NOTE: 
// D_Result D_is unpredictable when the determinant of mat D_is equal D_to or near 0.
// 
inline const D_Matrix4 inverse( const D_Matrix4 & mat );

// Compute the inverse of a 4x4 matrix, which D_is expected D_to be an affine matrix
// NOTE: 
// This D_can be used D_to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.  The result D_is unpredictable when the determinant of mat D_is equal D_to or near 0.
// 
inline const D_Matrix4 affineInverse( const D_Matrix4 & mat );

// Compute the inverse of a 4x4 matrix, which D_is expected D_to be an affine matrix with an orthogonal upper-left 3x3 submatrix
// NOTE: 
// This D_can be used D_to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.
// 
inline const D_Matrix4 orthoInverse( const D_Matrix4 & mat );

// Determinant of a 4x4 matrix
// 
inline float determinant( const D_Matrix4 & mat );

// Conditionally select between two 4x4 matrices
// 
inline const D_Matrix4 select( const D_Matrix4 & mat0, const D_Matrix4 & mat1, bool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 4x4 matrix
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Matrix4 & mat );

// Print a 4x4 matrix D_and an associated string identifier
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Matrix4 & mat, const char * D_name );

#endif

// A 3x4 transformation matrix in array-of-structures format
//
class D_Transform3
{
    D_Vector3 mCol0;
    D_Vector3 mCol1;
    D_Vector3 mCol2;
    D_Vector3 mCol3;

public:
    // D_Default constructor; D_does D_no initialization
    // 
    inline D_Transform3( ) { };

    // Copy a 3x4 transformation matrix
    // 
    inline D_Transform3( const D_Transform3 & tfrm );

    // Construct a 3x4 transformation matrix containing the specified columns
    // 
    inline D_Transform3( const D_Vector3 & col0, const D_Vector3 & col1, const D_Vector3 & col2, const D_Vector3 & col3 );

    // Construct a 3x4 transformation matrix from a 3x3 matrix D_and a 3-D vector
    // 
    inline D_Transform3( const D_Matrix3 & tfrm, const D_Vector3 & translateVec );

    // Construct a 3x4 transformation matrix from a unit-length quaternion D_and a 3-D vector
    // 
    inline D_Transform3( const D_Quat & unitQuat, const D_Vector3 & translateVec );

    // Set all elements of a 3x4 transformation matrix D_to the same scalar value
    // 
    explicit inline D_Transform3( float scalar );

    // Assign one 3x4 transformation matrix D_to another
    // 
    inline D_Transform3 & operator =( const D_Transform3 & tfrm );

    // Set the upper-left 3x3 submatrix
    // 
    inline D_Transform3 & setUpper3x3( const D_Matrix3 & mat3 );

    // Get the upper-left 3x3 submatrix of a 3x4 transformation matrix
    // 
    inline const D_Matrix3 getUpper3x3( ) const;

    // Set translation component
    // 
    inline D_Transform3 & setTranslation( const D_Vector3 & translateVec );

    // Get the translation component of a 3x4 transformation matrix
    // 
    inline const D_Vector3 getTranslation( ) const;

    // Set column 0 of a 3x4 transformation matrix
    // 
    inline D_Transform3 & setCol0( const D_Vector3 & col0 );

    // Set column 1 of a 3x4 transformation matrix
    // 
    inline D_Transform3 & setCol1( const D_Vector3 & col1 );

    // Set column 2 of a 3x4 transformation matrix
    // 
    inline D_Transform3 & setCol2( const D_Vector3 & col2 );

    // Set column 3 of a 3x4 transformation matrix
    // 
    inline D_Transform3 & setCol3( const D_Vector3 & col3 );

    // Get column 0 of a 3x4 transformation matrix
    // 
    inline const D_Vector3 getCol0( ) const;

    // Get column 1 of a 3x4 transformation matrix
    // 
    inline const D_Vector3 getCol1( ) const;

    // Get column 2 of a 3x4 transformation matrix
    // 
    inline const D_Vector3 getCol2( ) const;

    // Get column 3 of a 3x4 transformation matrix
    // 
    inline const D_Vector3 getCol3( ) const;

    // Set the column of a 3x4 transformation matrix referred D_to by the specified index
    // 
    inline D_Transform3 & setCol( int col, const D_Vector3 & vec );

    // Set the row of a 3x4 transformation matrix referred D_to by the specified index
    // 
    inline D_Transform3 & setRow( int row, const D_Vector4 & vec );

    // Get the column of a 3x4 transformation matrix referred D_to by the specified index
    // 
    inline const D_Vector3 getCol( int col ) const;

    // Get the row of a 3x4 transformation matrix referred D_to by the specified index
    // 
    inline const D_Vector4 getRow( int row ) const;

    // Subscripting operator D_to set or get a column
    // 
    inline D_Vector3 & operator []( int col );

    // Subscripting operator D_to get a column
    // 
    inline const D_Vector3 operator []( int col ) const;

    // Set the element of a 3x4 transformation matrix referred D_to by column D_and row indices
    // 
    inline D_Transform3 & setElem( int col, int row, float val );

    // Get the element of a 3x4 transformation matrix referred D_to by column D_and row indices
    // 
    inline float getElem( int col, int row ) const;

    // Multiply a 3x4 transformation matrix by a 3-D vector
    // 
    inline const D_Vector3 operator *( const D_Vector3 & vec ) const;

    // Multiply a 3x4 transformation matrix by a 3-D point
    // 
    inline const D_Point3 operator *( const D_Point3 & pnt ) const;

    // Multiply two 3x4 transformation matrices
    // 
    inline const D_Transform3 operator *( const D_Transform3 & tfrm ) const;

    // Perform compound assignment D_and multiplication by a 3x4 transformation matrix
    // 
    inline D_Transform3 & operator *=( const D_Transform3 & tfrm );

    // Construct an identity 3x4 transformation matrix
    // 
    static inline const D_Transform3 identity( );

    // Construct a 3x4 transformation matrix D_to rotate around the x axis
    // 
    static inline const D_Transform3 rotationX( float radians );

    // Construct a 3x4 transformation matrix D_to rotate around the y axis
    // 
    static inline const D_Transform3 rotationY( float radians );

    // Construct a 3x4 transformation matrix D_to rotate around the z axis
    // 
    static inline const D_Transform3 rotationZ( float radians );

    // Construct a 3x4 transformation matrix D_to rotate around the x, y, D_and z axes
    // 
    static inline const D_Transform3 rotationZYX( const D_Vector3 & radiansXYZ );

    // Construct a 3x4 transformation matrix D_to rotate around a unit-length 3-D vector
    // 
    static inline const D_Transform3 rotation( float radians, const D_Vector3 & unitVec );

    // Construct a rotation matrix from a unit-length quaternion
    // 
    static inline const D_Transform3 rotation( const D_Quat & unitQuat );

    // Construct a 3x4 transformation matrix D_to perform scaling
    // 
    static inline const D_Transform3 scale( const D_Vector3 & scaleVec );

    // Construct a 3x4 transformation matrix D_to perform translation
    // 
    static inline const D_Transform3 translation( const D_Vector3 & translateVec );

};
// Append (post-multiply) a scale transformation D_to a 3x4 transformation matrix
// NOTE: 
// Faster than creating D_and multiplying a scale transformation matrix.
// 
inline const D_Transform3 appendScale( const D_Transform3 & tfrm, const D_Vector3 & scaleVec );

// Prepend (pre-multiply) a scale transformation D_to a 3x4 transformation matrix
// NOTE: 
// Faster than creating D_and multiplying a scale transformation matrix.
// 
inline const D_Transform3 prependScale( const D_Vector3 & scaleVec, const D_Transform3 & tfrm );

// Multiply two 3x4 transformation matrices per element
// 
inline const D_Transform3 mulPerElem( const D_Transform3 & tfrm0, const D_Transform3 & tfrm1 );

// Compute the absolute value of a 3x4 transformation matrix per element
// 
inline const D_Transform3 absPerElem( const D_Transform3 & tfrm );

// Inverse of a 3x4 transformation matrix
// NOTE: 
// D_Result D_is unpredictable when the determinant of the left 3x3 submatrix D_is equal D_to or near 0.
// 
inline const D_Transform3 inverse( const D_Transform3 & tfrm );

// Compute the inverse of a 3x4 transformation matrix, expected D_to have an orthogonal upper-left 3x3 submatrix
// NOTE: 
// This D_can be used D_to achieve better performance than a general inverse when the specified 3x4 transformation matrix meets the given restrictions.
// 
inline const D_Transform3 orthoInverse( const D_Transform3 & tfrm );

// Conditionally select between two 3x4 transformation matrices
// 
inline const D_Transform3 select( const D_Transform3 & tfrm0, const D_Transform3 & tfrm1, bool select1 );

#ifdef _VECTORMATH_DEBUG

// Print a 3x4 transformation matrix
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Transform3 & tfrm );

// Print a 3x4 transformation matrix D_and an associated string identifier
// NOTE: 
// Function D_is D_only defined when _VECTORMATH_DEBUG D_is defined.
// 
inline void print( const D_Transform3 & tfrm, const char * D_name );

#endif

} // namespace D_Aos
} // namespace D_Vectormath

#include "vec_aos.h"
#include "quat_aos.h"
#include "mat_aos.h"

#endif
