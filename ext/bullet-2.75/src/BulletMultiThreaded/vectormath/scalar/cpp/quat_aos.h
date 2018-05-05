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

#ifndef _VECTORMATH_QUAT_AOS_CPP_H
#define _VECTORMATH_QUAT_AOS_CPP_H
//-----------------------------------------------------------------------------
// Definitions

#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

namespace D_Vectormath {
namespace D_Aos {

inline D_Quat::D_Quat( const D_Quat & quat )
{
    mX = quat.mX;
    mY = quat.mY;
    mZ = quat.mZ;
    mW = quat.mW;
}

inline D_Quat::D_Quat( float _x, float _y, float _z, float _w )
{
    mX = _x;
    mY = _y;
    mZ = _z;
    mW = _w;
}

inline D_Quat::D_Quat( const D_Vector3 & xyz, float _w )
{
    this->setXYZ( xyz );
    this->setW( _w );
}

inline D_Quat::D_Quat( const D_Vector4 & vec )
{
    mX = vec.getX();
    mY = vec.getY();
    mZ = vec.getZ();
    mW = vec.getW();
}

inline D_Quat::D_Quat( float scalar )
{
    mX = scalar;
    mY = scalar;
    mZ = scalar;
    mW = scalar;
}

inline const D_Quat D_Quat::identity( )
{
    return D_Quat( 0.0f, 0.0f, 0.0f, 1.0f );
}

inline const D_Quat lerp( float t, const D_Quat & quat0, const D_Quat & quat1 )
{
    return ( quat0 + ( ( quat1 - quat0 ) * t ) );
}

inline const D_Quat slerp( float t, const D_Quat & unitQuat0, const D_Quat & unitQuat1 )
{
    D_Quat start;
    float recipSinAngle, scale0, scale1, cosAngle, angle;
    cosAngle = dot( unitQuat0, unitQuat1 );
    if ( cosAngle < 0.0f ) {
        cosAngle = -cosAngle;
        start = ( -unitQuat0 );
    } else {
        start = unitQuat0;
    }
    if ( cosAngle < D__VECTORMATH_SLERP_TOL ) {
        angle = acosf( cosAngle );
        recipSinAngle = ( 1.0f / sinf( angle ) );
        scale0 = ( sinf( ( ( 1.0f - t ) * angle ) ) * recipSinAngle );
        scale1 = ( sinf( ( t * angle ) ) * recipSinAngle );
    } else {
        scale0 = ( 1.0f - t );
        scale1 = t;
    }
    return ( ( start * scale0 ) + ( unitQuat1 * scale1 ) );
}

inline const D_Quat squad( float t, const D_Quat & unitQuat0, const D_Quat & unitQuat1, const D_Quat & unitQuat2, const D_Quat & unitQuat3 )
{
    D_Quat tmp0, tmp1;
    tmp0 = slerp( t, unitQuat0, unitQuat3 );
    tmp1 = slerp( t, unitQuat1, unitQuat2 );
    return slerp( ( ( 2.0f * t ) * ( 1.0f - t ) ), tmp0, tmp1 );
}

inline D_Quat & D_Quat::operator =( const D_Quat & quat )
{
    mX = quat.mX;
    mY = quat.mY;
    mZ = quat.mZ;
    mW = quat.mW;
    return *this;
}

inline D_Quat & D_Quat::setXYZ( const D_Vector3 & vec )
{
    mX = vec.getX();
    mY = vec.getY();
    mZ = vec.getZ();
    return *this;
}

inline const D_Vector3 D_Quat::getXYZ( ) const
{
    return D_Vector3( mX, mY, mZ );
}

inline D_Quat & D_Quat::setX( float _x )
{
    mX = _x;
    return *this;
}

inline float D_Quat::getX( ) const
{
    return mX;
}

inline D_Quat & D_Quat::setY( float _y )
{
    mY = _y;
    return *this;
}

inline float D_Quat::getY( ) const
{
    return mY;
}

inline D_Quat & D_Quat::setZ( float _z )
{
    mZ = _z;
    return *this;
}

inline float D_Quat::getZ( ) const
{
    return mZ;
}

inline D_Quat & D_Quat::setW( float _w )
{
    mW = _w;
    return *this;
}

inline float D_Quat::getW( ) const
{
    return mW;
}

inline D_Quat & D_Quat::setElem( int idx, float value )
{
    *(&mX + idx) = value;
    return *this;
}

inline float D_Quat::getElem( int idx ) const
{
    return *(&mX + idx);
}

inline float & D_Quat::operator []( int idx )
{
    return *(&mX + idx);
}

inline float D_Quat::operator []( int idx ) const
{
    return *(&mX + idx);
}

inline const D_Quat D_Quat::operator +( const D_Quat & quat ) const
{
    return D_Quat(
        ( mX + quat.mX ),
        ( mY + quat.mY ),
        ( mZ + quat.mZ ),
        ( mW + quat.mW )
    );
}

inline const D_Quat D_Quat::operator -( const D_Quat & quat ) const
{
    return D_Quat(
        ( mX - quat.mX ),
        ( mY - quat.mY ),
        ( mZ - quat.mZ ),
        ( mW - quat.mW )
    );
}

inline const D_Quat D_Quat::operator *( float scalar ) const
{
    return D_Quat(
        ( mX * scalar ),
        ( mY * scalar ),
        ( mZ * scalar ),
        ( mW * scalar )
    );
}

inline D_Quat & D_Quat::operator +=( const D_Quat & quat )
{
    *this = *this + quat;
    return *this;
}

inline D_Quat & D_Quat::operator -=( const D_Quat & quat )
{
    *this = *this - quat;
    return *this;
}

inline D_Quat & D_Quat::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

inline const D_Quat D_Quat::operator /( float scalar ) const
{
    return D_Quat(
        ( mX / scalar ),
        ( mY / scalar ),
        ( mZ / scalar ),
        ( mW / scalar )
    );
}

inline D_Quat & D_Quat::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

inline const D_Quat D_Quat::operator -( ) const
{
    return D_Quat(
        -mX,
        -mY,
        -mZ,
        -mW
    );
}

inline const D_Quat operator *( float scalar, const D_Quat & quat )
{
    return quat * scalar;
}

inline float dot( const D_Quat & quat0, const D_Quat & quat1 )
{
    float result;
    result = ( quat0.getX() * quat1.getX() );
    result = ( result + ( quat0.getY() * quat1.getY() ) );
    result = ( result + ( quat0.getZ() * quat1.getZ() ) );
    result = ( result + ( quat0.getW() * quat1.getW() ) );
    return result;
}

inline float norm( const D_Quat & quat )
{
    float result;
    result = ( quat.getX() * quat.getX() );
    result = ( result + ( quat.getY() * quat.getY() ) );
    result = ( result + ( quat.getZ() * quat.getZ() ) );
    result = ( result + ( quat.getW() * quat.getW() ) );
    return result;
}

inline float length( const D_Quat & quat )
{
    return sqrtf( norm( quat ) );
}

inline const D_Quat normalize( const D_Quat & quat )
{
    float lenSqr, lenInv;
    lenSqr = norm( quat );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    return D_Quat(
        ( quat.getX() * lenInv ),
        ( quat.getY() * lenInv ),
        ( quat.getZ() * lenInv ),
        ( quat.getW() * lenInv )
    );
}

inline const D_Quat D_Quat::rotation( const D_Vector3 & unitVec0, const D_Vector3 & unitVec1 )
{
    float cosHalfAngleX2, recipCosHalfAngleX2;
    cosHalfAngleX2 = sqrtf( ( 2.0f * ( 1.0f + dot( unitVec0, unitVec1 ) ) ) );
    recipCosHalfAngleX2 = ( 1.0f / cosHalfAngleX2 );
    return D_Quat( ( cross( unitVec0, unitVec1 ) * recipCosHalfAngleX2 ), ( cosHalfAngleX2 * 0.5f ) );
}

inline const D_Quat D_Quat::rotation( float radians, const D_Vector3 & unitVec )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return D_Quat( ( unitVec * s ), c );
}

inline const D_Quat D_Quat::rotationX( float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return D_Quat( s, 0.0f, 0.0f, c );
}

inline const D_Quat D_Quat::rotationY( float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return D_Quat( 0.0f, s, 0.0f, c );
}

inline const D_Quat D_Quat::rotationZ( float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return D_Quat( 0.0f, 0.0f, s, c );
}

inline const D_Quat D_Quat::operator *( const D_Quat & quat ) const
{
    return D_Quat(
        ( ( ( ( mW * quat.mX ) + ( mX * quat.mW ) ) + ( mY * quat.mZ ) ) - ( mZ * quat.mY ) ),
        ( ( ( ( mW * quat.mY ) + ( mY * quat.mW ) ) + ( mZ * quat.mX ) ) - ( mX * quat.mZ ) ),
        ( ( ( ( mW * quat.mZ ) + ( mZ * quat.mW ) ) + ( mX * quat.mY ) ) - ( mY * quat.mX ) ),
        ( ( ( ( mW * quat.mW ) - ( mX * quat.mX ) ) - ( mY * quat.mY ) ) - ( mZ * quat.mZ ) )
    );
}

inline D_Quat & D_Quat::operator *=( const D_Quat & quat )
{
    *this = *this * quat;
    return *this;
}

inline const D_Vector3 rotate( const D_Quat & quat, const D_Vector3 & vec )
{
    float tmpX, tmpY, tmpZ, tmpW;
    tmpX = ( ( ( quat.getW() * vec.getX() ) + ( quat.getY() * vec.getZ() ) ) - ( quat.getZ() * vec.getY() ) );
    tmpY = ( ( ( quat.getW() * vec.getY() ) + ( quat.getZ() * vec.getX() ) ) - ( quat.getX() * vec.getZ() ) );
    tmpZ = ( ( ( quat.getW() * vec.getZ() ) + ( quat.getX() * vec.getY() ) ) - ( quat.getY() * vec.getX() ) );
    tmpW = ( ( ( quat.getX() * vec.getX() ) + ( quat.getY() * vec.getY() ) ) + ( quat.getZ() * vec.getZ() ) );
    return D_Vector3(
        ( ( ( ( tmpW * quat.getX() ) + ( tmpX * quat.getW() ) ) - ( tmpY * quat.getZ() ) ) + ( tmpZ * quat.getY() ) ),
        ( ( ( ( tmpW * quat.getY() ) + ( tmpY * quat.getW() ) ) - ( tmpZ * quat.getX() ) ) + ( tmpX * quat.getZ() ) ),
        ( ( ( ( tmpW * quat.getZ() ) + ( tmpZ * quat.getW() ) ) - ( tmpX * quat.getY() ) ) + ( tmpY * quat.getX() ) )
    );
}

inline const D_Quat conj( const D_Quat & quat )
{
    return D_Quat( -quat.getX(), -quat.getY(), -quat.getZ(), quat.getW() );
}

inline const D_Quat select( const D_Quat & quat0, const D_Quat & quat1, bool select1 )
{
    return D_Quat(
        ( select1 )? quat1.getX() : quat0.getX(),
        ( select1 )? quat1.getY() : quat0.getY(),
        ( select1 )? quat1.getZ() : quat0.getZ(),
        ( select1 )? quat1.getW() : quat0.getW()
    );
}

#ifdef _VECTORMATH_DEBUG

inline void print( const D_Quat & quat )
{
    printf( "( %f %f %f %f )\n", quat.getX(), quat.getY(), quat.getZ(), quat.getW() );
}

inline void print( const D_Quat & quat, const char * D_name )
{
    printf( "%s: ( %f %f %f %f )\n", D_name, quat.getX(), quat.getY(), quat.getZ(), quat.getW() );
}

#endif

} // namespace D_Aos
} // namespace D_Vectormath

#endif
