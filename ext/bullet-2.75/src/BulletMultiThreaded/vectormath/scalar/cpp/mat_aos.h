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

#ifndef _VECTORMATH_MAT_AOS_CPP_H
#define _VECTORMATH_MAT_AOS_CPP_H

namespace D_Vectormath {
namespace D_Aos {

//-----------------------------------------------------------------------------
// Constants

#define D__VECTORMATH_PI_OVER_2 1.570796327f

//-----------------------------------------------------------------------------
// Definitions

inline D_Matrix3::D_Matrix3( const D_Matrix3 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
}

inline D_Matrix3::D_Matrix3( float scalar )
{
    mCol0 = D_Vector3( scalar );
    mCol1 = D_Vector3( scalar );
    mCol2 = D_Vector3( scalar );
}

inline D_Matrix3::D_Matrix3( const D_Quat & unitQuat )
{
    float qx, qy, qz, qw, qx2, qy2, qz2, qxqx2, qyqy2, qzqz2, qxqy2, qyqz2, qzqw2, qxqz2, qyqw2, qxqw2;
    qx = unitQuat.getX();
    qy = unitQuat.getY();
    qz = unitQuat.getZ();
    qw = unitQuat.getW();
    qx2 = ( qx + qx );
    qy2 = ( qy + qy );
    qz2 = ( qz + qz );
    qxqx2 = ( qx * qx2 );
    qxqy2 = ( qx * qy2 );
    qxqz2 = ( qx * qz2 );
    qxqw2 = ( qw * qx2 );
    qyqy2 = ( qy * qy2 );
    qyqz2 = ( qy * qz2 );
    qyqw2 = ( qw * qy2 );
    qzqz2 = ( qz * qz2 );
    qzqw2 = ( qw * qz2 );
    mCol0 = D_Vector3( ( ( 1.0f - qyqy2 ) - qzqz2 ), ( qxqy2 + qzqw2 ), ( qxqz2 - qyqw2 ) );
    mCol1 = D_Vector3( ( qxqy2 - qzqw2 ), ( ( 1.0f - qxqx2 ) - qzqz2 ), ( qyqz2 + qxqw2 ) );
    mCol2 = D_Vector3( ( qxqz2 + qyqw2 ), ( qyqz2 - qxqw2 ), ( ( 1.0f - qxqx2 ) - qyqy2 ) );
}

inline D_Matrix3::D_Matrix3( const D_Vector3 & _col0, const D_Vector3 & _col1, const D_Vector3 & _col2 )
{
    mCol0 = _col0;
    mCol1 = _col1;
    mCol2 = _col2;
}

inline D_Matrix3 & D_Matrix3::setCol0( const D_Vector3 & _col0 )
{
    mCol0 = _col0;
    return *this;
}

inline D_Matrix3 & D_Matrix3::setCol1( const D_Vector3 & _col1 )
{
    mCol1 = _col1;
    return *this;
}

inline D_Matrix3 & D_Matrix3::setCol2( const D_Vector3 & _col2 )
{
    mCol2 = _col2;
    return *this;
}

inline D_Matrix3 & D_Matrix3::setCol( int col, const D_Vector3 & vec )
{
    *(&mCol0 + col) = vec;
    return *this;
}

inline D_Matrix3 & D_Matrix3::setRow( int row, const D_Vector3 & vec )
{
    mCol0.setElem( row, vec.getElem( 0 ) );
    mCol1.setElem( row, vec.getElem( 1 ) );
    mCol2.setElem( row, vec.getElem( 2 ) );
    return *this;
}

inline D_Matrix3 & D_Matrix3::setElem( int col, int row, float val )
{
    D_Vector3 tmpV3_0;
    tmpV3_0 = this->getCol( col );
    tmpV3_0.setElem( row, val );
    this->setCol( col, tmpV3_0 );
    return *this;
}

inline float D_Matrix3::getElem( int col, int row ) const
{
    return this->getCol( col ).getElem( row );
}

inline const D_Vector3 D_Matrix3::getCol0( ) const
{
    return mCol0;
}

inline const D_Vector3 D_Matrix3::getCol1( ) const
{
    return mCol1;
}

inline const D_Vector3 D_Matrix3::getCol2( ) const
{
    return mCol2;
}

inline const D_Vector3 D_Matrix3::getCol( int col ) const
{
    return *(&mCol0 + col);
}

inline const D_Vector3 D_Matrix3::getRow( int row ) const
{
    return D_Vector3( mCol0.getElem( row ), mCol1.getElem( row ), mCol2.getElem( row ) );
}

inline D_Vector3 & D_Matrix3::operator []( int col )
{
    return *(&mCol0 + col);
}

inline const D_Vector3 D_Matrix3::operator []( int col ) const
{
    return *(&mCol0 + col);
}

inline D_Matrix3 & D_Matrix3::operator =( const D_Matrix3 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
    return *this;
}

inline const D_Matrix3 transpose( const D_Matrix3 & mat )
{
    return D_Matrix3(
        D_Vector3( mat.getCol0().getX(), mat.getCol1().getX(), mat.getCol2().getX() ),
        D_Vector3( mat.getCol0().getY(), mat.getCol1().getY(), mat.getCol2().getY() ),
        D_Vector3( mat.getCol0().getZ(), mat.getCol1().getZ(), mat.getCol2().getZ() )
    );
}

inline const D_Matrix3 inverse( const D_Matrix3 & mat )
{
    D_Vector3 tmp0, tmp1, tmp2;
    float detinv;
    tmp0 = cross( mat.getCol1(), mat.getCol2() );
    tmp1 = cross( mat.getCol2(), mat.getCol0() );
    tmp2 = cross( mat.getCol0(), mat.getCol1() );
    detinv = ( 1.0f / dot( mat.getCol2(), tmp2 ) );
    return D_Matrix3(
        D_Vector3( ( tmp0.getX() * detinv ), ( tmp1.getX() * detinv ), ( tmp2.getX() * detinv ) ),
        D_Vector3( ( tmp0.getY() * detinv ), ( tmp1.getY() * detinv ), ( tmp2.getY() * detinv ) ),
        D_Vector3( ( tmp0.getZ() * detinv ), ( tmp1.getZ() * detinv ), ( tmp2.getZ() * detinv ) )
    );
}

inline float determinant( const D_Matrix3 & mat )
{
    return dot( mat.getCol2(), cross( mat.getCol0(), mat.getCol1() ) );
}

inline const D_Matrix3 D_Matrix3::operator +( const D_Matrix3 & mat ) const
{
    return D_Matrix3(
        ( mCol0 + mat.mCol0 ),
        ( mCol1 + mat.mCol1 ),
        ( mCol2 + mat.mCol2 )
    );
}

inline const D_Matrix3 D_Matrix3::operator -( const D_Matrix3 & mat ) const
{
    return D_Matrix3(
        ( mCol0 - mat.mCol0 ),
        ( mCol1 - mat.mCol1 ),
        ( mCol2 - mat.mCol2 )
    );
}

inline D_Matrix3 & D_Matrix3::operator +=( const D_Matrix3 & mat )
{
    *this = *this + mat;
    return *this;
}

inline D_Matrix3 & D_Matrix3::operator -=( const D_Matrix3 & mat )
{
    *this = *this - mat;
    return *this;
}

inline const D_Matrix3 D_Matrix3::operator -( ) const
{
    return D_Matrix3(
        ( -mCol0 ),
        ( -mCol1 ),
        ( -mCol2 )
    );
}

inline const D_Matrix3 absPerElem( const D_Matrix3 & mat )
{
    return D_Matrix3(
        absPerElem( mat.getCol0() ),
        absPerElem( mat.getCol1() ),
        absPerElem( mat.getCol2() )
    );
}

inline const D_Matrix3 D_Matrix3::operator *( float scalar ) const
{
    return D_Matrix3(
        ( mCol0 * scalar ),
        ( mCol1 * scalar ),
        ( mCol2 * scalar )
    );
}

inline D_Matrix3 & D_Matrix3::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

inline const D_Matrix3 operator *( float scalar, const D_Matrix3 & mat )
{
    return mat * scalar;
}

inline const D_Vector3 D_Matrix3::operator *( const D_Vector3 & vec ) const
{
    return D_Vector3(
        ( ( ( mCol0.getX() * vec.getX() ) + ( mCol1.getX() * vec.getY() ) ) + ( mCol2.getX() * vec.getZ() ) ),
        ( ( ( mCol0.getY() * vec.getX() ) + ( mCol1.getY() * vec.getY() ) ) + ( mCol2.getY() * vec.getZ() ) ),
        ( ( ( mCol0.getZ() * vec.getX() ) + ( mCol1.getZ() * vec.getY() ) ) + ( mCol2.getZ() * vec.getZ() ) )
    );
}

inline const D_Matrix3 D_Matrix3::operator *( const D_Matrix3 & mat ) const
{
    return D_Matrix3(
        ( *this * mat.mCol0 ),
        ( *this * mat.mCol1 ),
        ( *this * mat.mCol2 )
    );
}

inline D_Matrix3 & D_Matrix3::operator *=( const D_Matrix3 & mat )
{
    *this = *this * mat;
    return *this;
}

inline const D_Matrix3 mulPerElem( const D_Matrix3 & mat0, const D_Matrix3 & mat1 )
{
    return D_Matrix3(
        mulPerElem( mat0.getCol0(), mat1.getCol0() ),
        mulPerElem( mat0.getCol1(), mat1.getCol1() ),
        mulPerElem( mat0.getCol2(), mat1.getCol2() )
    );
}

inline const D_Matrix3 D_Matrix3::identity( )
{
    return D_Matrix3(
        D_Vector3::xAxis( ),
        D_Vector3::yAxis( ),
        D_Vector3::zAxis( )
    );
}

inline const D_Matrix3 D_Matrix3::rotationX( float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    return D_Matrix3(
        D_Vector3::xAxis( ),
        D_Vector3( 0.0f, c, s ),
        D_Vector3( 0.0f, -s, c )
    );
}

inline const D_Matrix3 D_Matrix3::rotationY( float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    return D_Matrix3(
        D_Vector3( c, 0.0f, -s ),
        D_Vector3::yAxis( ),
        D_Vector3( s, 0.0f, c )
    );
}

inline const D_Matrix3 D_Matrix3::rotationZ( float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    return D_Matrix3(
        D_Vector3( c, s, 0.0f ),
        D_Vector3( -s, c, 0.0f ),
        D_Vector3::zAxis( )
    );
}

inline const D_Matrix3 D_Matrix3::rotationZYX( const D_Vector3 & radiansXYZ )
{
    float D_sX, cX, D_sY, cY, D_sZ, cZ, tmp0, tmp1;
    D_sX = sinf( radiansXYZ.getX() );
    cX = cosf( radiansXYZ.getX() );
    D_sY = sinf( radiansXYZ.getY() );
    cY = cosf( radiansXYZ.getY() );
    D_sZ = sinf( radiansXYZ.getZ() );
    cZ = cosf( radiansXYZ.getZ() );
    tmp0 = ( cZ * D_sY );
    tmp1 = ( D_sZ * D_sY );
    return D_Matrix3(
        D_Vector3( ( cZ * cY ), ( D_sZ * cY ), -D_sY ),
        D_Vector3( ( ( tmp0 * D_sX ) - ( D_sZ * cX ) ), ( ( tmp1 * D_sX ) + ( cZ * cX ) ), ( cY * D_sX ) ),
        D_Vector3( ( ( tmp0 * cX ) + ( D_sZ * D_sX ) ), ( ( tmp1 * cX ) - ( cZ * D_sX ) ), ( cY * cX ) )
    );
}

inline const D_Matrix3 D_Matrix3::rotation( float radians, const D_Vector3 & unitVec )
{
    float x, y, z, s, c, oneMinusC, xy, yz, zx;
    s = sinf( radians );
    c = cosf( radians );
    x = unitVec.getX();
    y = unitVec.getY();
    z = unitVec.getZ();
    xy = ( x * y );
    yz = ( y * z );
    zx = ( z * x );
    oneMinusC = ( 1.0f - c );
    return D_Matrix3(
        D_Vector3( ( ( ( x * x ) * oneMinusC ) + c ), ( ( xy * oneMinusC ) + ( z * s ) ), ( ( zx * oneMinusC ) - ( y * s ) ) ),
        D_Vector3( ( ( xy * oneMinusC ) - ( z * s ) ), ( ( ( y * y ) * oneMinusC ) + c ), ( ( yz * oneMinusC ) + ( x * s ) ) ),
        D_Vector3( ( ( zx * oneMinusC ) + ( y * s ) ), ( ( yz * oneMinusC ) - ( x * s ) ), ( ( ( z * z ) * oneMinusC ) + c ) )
    );
}

inline const D_Matrix3 D_Matrix3::rotation( const D_Quat & unitQuat )
{
    return D_Matrix3( unitQuat );
}

inline const D_Matrix3 D_Matrix3::scale( const D_Vector3 & scaleVec )
{
    return D_Matrix3(
        D_Vector3( scaleVec.getX(), 0.0f, 0.0f ),
        D_Vector3( 0.0f, scaleVec.getY(), 0.0f ),
        D_Vector3( 0.0f, 0.0f, scaleVec.getZ() )
    );
}

inline const D_Matrix3 appendScale( const D_Matrix3 & mat, const D_Vector3 & scaleVec )
{
    return D_Matrix3(
        ( mat.getCol0() * scaleVec.getX( ) ),
        ( mat.getCol1() * scaleVec.getY( ) ),
        ( mat.getCol2() * scaleVec.getZ( ) )
    );
}

inline const D_Matrix3 prependScale( const D_Vector3 & scaleVec, const D_Matrix3 & mat )
{
    return D_Matrix3(
        mulPerElem( mat.getCol0(), scaleVec ),
        mulPerElem( mat.getCol1(), scaleVec ),
        mulPerElem( mat.getCol2(), scaleVec )
    );
}

inline const D_Matrix3 select( const D_Matrix3 & mat0, const D_Matrix3 & mat1, bool select1 )
{
    return D_Matrix3(
        select( mat0.getCol0(), mat1.getCol0(), select1 ),
        select( mat0.getCol1(), mat1.getCol1(), select1 ),
        select( mat0.getCol2(), mat1.getCol2(), select1 )
    );
}

#ifdef _VECTORMATH_DEBUG

inline void print( const D_Matrix3 & mat )
{
    print( mat.getRow( 0 ) );
    print( mat.getRow( 1 ) );
    print( mat.getRow( 2 ) );
}

inline void print( const D_Matrix3 & mat, const char * D_name )
{
    printf("%s:\n", D_name);
    print( mat );
}

#endif

inline D_Matrix4::D_Matrix4( const D_Matrix4 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
    mCol3 = mat.mCol3;
}

inline D_Matrix4::D_Matrix4( float scalar )
{
    mCol0 = D_Vector4( scalar );
    mCol1 = D_Vector4( scalar );
    mCol2 = D_Vector4( scalar );
    mCol3 = D_Vector4( scalar );
}

inline D_Matrix4::D_Matrix4( const D_Transform3 & mat )
{
    mCol0 = D_Vector4( mat.getCol0(), 0.0f );
    mCol1 = D_Vector4( mat.getCol1(), 0.0f );
    mCol2 = D_Vector4( mat.getCol2(), 0.0f );
    mCol3 = D_Vector4( mat.getCol3(), 1.0f );
}

inline D_Matrix4::D_Matrix4( const D_Vector4 & _col0, const D_Vector4 & _col1, const D_Vector4 & _col2, const D_Vector4 & _col3 )
{
    mCol0 = _col0;
    mCol1 = _col1;
    mCol2 = _col2;
    mCol3 = _col3;
}

inline D_Matrix4::D_Matrix4( const D_Matrix3 & mat, const D_Vector3 & translateVec )
{
    mCol0 = D_Vector4( mat.getCol0(), 0.0f );
    mCol1 = D_Vector4( mat.getCol1(), 0.0f );
    mCol2 = D_Vector4( mat.getCol2(), 0.0f );
    mCol3 = D_Vector4( translateVec, 1.0f );
}

inline D_Matrix4::D_Matrix4( const D_Quat & unitQuat, const D_Vector3 & translateVec )
{
    D_Matrix3 mat;
    mat = D_Matrix3( unitQuat );
    mCol0 = D_Vector4( mat.getCol0(), 0.0f );
    mCol1 = D_Vector4( mat.getCol1(), 0.0f );
    mCol2 = D_Vector4( mat.getCol2(), 0.0f );
    mCol3 = D_Vector4( translateVec, 1.0f );
}

inline D_Matrix4 & D_Matrix4::setCol0( const D_Vector4 & _col0 )
{
    mCol0 = _col0;
    return *this;
}

inline D_Matrix4 & D_Matrix4::setCol1( const D_Vector4 & _col1 )
{
    mCol1 = _col1;
    return *this;
}

inline D_Matrix4 & D_Matrix4::setCol2( const D_Vector4 & _col2 )
{
    mCol2 = _col2;
    return *this;
}

inline D_Matrix4 & D_Matrix4::setCol3( const D_Vector4 & _col3 )
{
    mCol3 = _col3;
    return *this;
}

inline D_Matrix4 & D_Matrix4::setCol( int col, const D_Vector4 & vec )
{
    *(&mCol0 + col) = vec;
    return *this;
}

inline D_Matrix4 & D_Matrix4::setRow( int row, const D_Vector4 & vec )
{
    mCol0.setElem( row, vec.getElem( 0 ) );
    mCol1.setElem( row, vec.getElem( 1 ) );
    mCol2.setElem( row, vec.getElem( 2 ) );
    mCol3.setElem( row, vec.getElem( 3 ) );
    return *this;
}

inline D_Matrix4 & D_Matrix4::setElem( int col, int row, float val )
{
    D_Vector4 tmpV3_0;
    tmpV3_0 = this->getCol( col );
    tmpV3_0.setElem( row, val );
    this->setCol( col, tmpV3_0 );
    return *this;
}

inline float D_Matrix4::getElem( int col, int row ) const
{
    return this->getCol( col ).getElem( row );
}

inline const D_Vector4 D_Matrix4::getCol0( ) const
{
    return mCol0;
}

inline const D_Vector4 D_Matrix4::getCol1( ) const
{
    return mCol1;
}

inline const D_Vector4 D_Matrix4::getCol2( ) const
{
    return mCol2;
}

inline const D_Vector4 D_Matrix4::getCol3( ) const
{
    return mCol3;
}

inline const D_Vector4 D_Matrix4::getCol( int col ) const
{
    return *(&mCol0 + col);
}

inline const D_Vector4 D_Matrix4::getRow( int row ) const
{
    return D_Vector4( mCol0.getElem( row ), mCol1.getElem( row ), mCol2.getElem( row ), mCol3.getElem( row ) );
}

inline D_Vector4 & D_Matrix4::operator []( int col )
{
    return *(&mCol0 + col);
}

inline const D_Vector4 D_Matrix4::operator []( int col ) const
{
    return *(&mCol0 + col);
}

inline D_Matrix4 & D_Matrix4::operator =( const D_Matrix4 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
    mCol3 = mat.mCol3;
    return *this;
}

inline const D_Matrix4 transpose( const D_Matrix4 & mat )
{
    return D_Matrix4(
        D_Vector4( mat.getCol0().getX(), mat.getCol1().getX(), mat.getCol2().getX(), mat.getCol3().getX() ),
        D_Vector4( mat.getCol0().getY(), mat.getCol1().getY(), mat.getCol2().getY(), mat.getCol3().getY() ),
        D_Vector4( mat.getCol0().getZ(), mat.getCol1().getZ(), mat.getCol2().getZ(), mat.getCol3().getZ() ),
        D_Vector4( mat.getCol0().getW(), mat.getCol1().getW(), mat.getCol2().getW(), mat.getCol3().getW() )
    );
}

inline const D_Matrix4 inverse( const D_Matrix4 & mat )
{
    D_Vector4 res0, res1, res2, res3;
    float mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, detInv;
    mA = mat.getCol0().getX();
    mB = mat.getCol0().getY();
    mC = mat.getCol0().getZ();
    mD = mat.getCol0().getW();
    mE = mat.getCol1().getX();
    mF = mat.getCol1().getY();
    mG = mat.getCol1().getZ();
    mH = mat.getCol1().getW();
    mI = mat.getCol2().getX();
    mJ = mat.getCol2().getY();
    mK = mat.getCol2().getZ();
    mL = mat.getCol2().getW();
    mM = mat.getCol3().getX();
    mN = mat.getCol3().getY();
    mO = mat.getCol3().getZ();
    mP = mat.getCol3().getW();
    tmp0 = ( ( mK * mD ) - ( mC * mL ) );
    tmp1 = ( ( mO * mH ) - ( mG * mP ) );
    tmp2 = ( ( mB * mK ) - ( mJ * mC ) );
    tmp3 = ( ( mF * mO ) - ( mN * mG ) );
    tmp4 = ( ( mJ * mD ) - ( mB * mL ) );
    tmp5 = ( ( mN * mH ) - ( mF * mP ) );
    res0.setX( ( ( ( mJ * tmp1 ) - ( mL * tmp3 ) ) - ( mK * tmp5 ) ) );
    res0.setY( ( ( ( mN * tmp0 ) - ( mP * tmp2 ) ) - ( mO * tmp4 ) ) );
    res0.setZ( ( ( ( mD * tmp3 ) + ( mC * tmp5 ) ) - ( mB * tmp1 ) ) );
    res0.setW( ( ( ( mH * tmp2 ) + ( mG * tmp4 ) ) - ( mF * tmp0 ) ) );
    detInv = ( 1.0f / ( ( ( ( mA * res0.getX() ) + ( mE * res0.getY() ) ) + ( mI * res0.getZ() ) ) + ( mM * res0.getW() ) ) );
    res1.setX( ( mI * tmp1 ) );
    res1.setY( ( mM * tmp0 ) );
    res1.setZ( ( mA * tmp1 ) );
    res1.setW( ( mE * tmp0 ) );
    res3.setX( ( mI * tmp3 ) );
    res3.setY( ( mM * tmp2 ) );
    res3.setZ( ( mA * tmp3 ) );
    res3.setW( ( mE * tmp2 ) );
    res2.setX( ( mI * tmp5 ) );
    res2.setY( ( mM * tmp4 ) );
    res2.setZ( ( mA * tmp5 ) );
    res2.setW( ( mE * tmp4 ) );
    tmp0 = ( ( mI * mB ) - ( mA * mJ ) );
    tmp1 = ( ( mM * mF ) - ( mE * mN ) );
    tmp2 = ( ( mI * mD ) - ( mA * mL ) );
    tmp3 = ( ( mM * mH ) - ( mE * mP ) );
    tmp4 = ( ( mI * mC ) - ( mA * mK ) );
    tmp5 = ( ( mM * mG ) - ( mE * mO ) );
    res2.setX( ( ( ( mL * tmp1 ) - ( mJ * tmp3 ) ) + res2.getX() ) );
    res2.setY( ( ( ( mP * tmp0 ) - ( mN * tmp2 ) ) + res2.getY() ) );
    res2.setZ( ( ( ( mB * tmp3 ) - ( mD * tmp1 ) ) - res2.getZ() ) );
    res2.setW( ( ( ( mF * tmp2 ) - ( mH * tmp0 ) ) - res2.getW() ) );
    res3.setX( ( ( ( mJ * tmp5 ) - ( mK * tmp1 ) ) + res3.getX() ) );
    res3.setY( ( ( ( mN * tmp4 ) - ( mO * tmp0 ) ) + res3.getY() ) );
    res3.setZ( ( ( ( mC * tmp1 ) - ( mB * tmp5 ) ) - res3.getZ() ) );
    res3.setW( ( ( ( mG * tmp0 ) - ( mF * tmp4 ) ) - res3.getW() ) );
    res1.setX( ( ( ( mK * tmp3 ) - ( mL * tmp5 ) ) - res1.getX() ) );
    res1.setY( ( ( ( mO * tmp2 ) - ( mP * tmp4 ) ) - res1.getY() ) );
    res1.setZ( ( ( ( mD * tmp5 ) - ( mC * tmp3 ) ) + res1.getZ() ) );
    res1.setW( ( ( ( mH * tmp4 ) - ( mG * tmp2 ) ) + res1.getW() ) );
    return D_Matrix4(
        ( res0 * detInv ),
        ( res1 * detInv ),
        ( res2 * detInv ),
        ( res3 * detInv )
    );
}

inline const D_Matrix4 affineInverse( const D_Matrix4 & mat )
{
    D_Transform3 affineMat;
    affineMat.setCol0( mat.getCol0().getXYZ( ) );
    affineMat.setCol1( mat.getCol1().getXYZ( ) );
    affineMat.setCol2( mat.getCol2().getXYZ( ) );
    affineMat.setCol3( mat.getCol3().getXYZ( ) );
    return D_Matrix4( inverse( affineMat ) );
}

inline const D_Matrix4 orthoInverse( const D_Matrix4 & mat )
{
    D_Transform3 affineMat;
    affineMat.setCol0( mat.getCol0().getXYZ( ) );
    affineMat.setCol1( mat.getCol1().getXYZ( ) );
    affineMat.setCol2( mat.getCol2().getXYZ( ) );
    affineMat.setCol3( mat.getCol3().getXYZ( ) );
    return D_Matrix4( orthoInverse( affineMat ) );
}

inline float determinant( const D_Matrix4 & mat )
{
    float dx, dy, dz, dw, mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5;
    mA = mat.getCol0().getX();
    mB = mat.getCol0().getY();
    mC = mat.getCol0().getZ();
    mD = mat.getCol0().getW();
    mE = mat.getCol1().getX();
    mF = mat.getCol1().getY();
    mG = mat.getCol1().getZ();
    mH = mat.getCol1().getW();
    mI = mat.getCol2().getX();
    mJ = mat.getCol2().getY();
    mK = mat.getCol2().getZ();
    mL = mat.getCol2().getW();
    mM = mat.getCol3().getX();
    mN = mat.getCol3().getY();
    mO = mat.getCol3().getZ();
    mP = mat.getCol3().getW();
    tmp0 = ( ( mK * mD ) - ( mC * mL ) );
    tmp1 = ( ( mO * mH ) - ( mG * mP ) );
    tmp2 = ( ( mB * mK ) - ( mJ * mC ) );
    tmp3 = ( ( mF * mO ) - ( mN * mG ) );
    tmp4 = ( ( mJ * mD ) - ( mB * mL ) );
    tmp5 = ( ( mN * mH ) - ( mF * mP ) );
    dx = ( ( ( mJ * tmp1 ) - ( mL * tmp3 ) ) - ( mK * tmp5 ) );
    dy = ( ( ( mN * tmp0 ) - ( mP * tmp2 ) ) - ( mO * tmp4 ) );
    dz = ( ( ( mD * tmp3 ) + ( mC * tmp5 ) ) - ( mB * tmp1 ) );
    dw = ( ( ( mH * tmp2 ) + ( mG * tmp4 ) ) - ( mF * tmp0 ) );
    return ( ( ( ( mA * dx ) + ( mE * dy ) ) + ( mI * dz ) ) + ( mM * dw ) );
}

inline const D_Matrix4 D_Matrix4::operator +( const D_Matrix4 & mat ) const
{
    return D_Matrix4(
        ( mCol0 + mat.mCol0 ),
        ( mCol1 + mat.mCol1 ),
        ( mCol2 + mat.mCol2 ),
        ( mCol3 + mat.mCol3 )
    );
}

inline const D_Matrix4 D_Matrix4::operator -( const D_Matrix4 & mat ) const
{
    return D_Matrix4(
        ( mCol0 - mat.mCol0 ),
        ( mCol1 - mat.mCol1 ),
        ( mCol2 - mat.mCol2 ),
        ( mCol3 - mat.mCol3 )
    );
}

inline D_Matrix4 & D_Matrix4::operator +=( const D_Matrix4 & mat )
{
    *this = *this + mat;
    return *this;
}

inline D_Matrix4 & D_Matrix4::operator -=( const D_Matrix4 & mat )
{
    *this = *this - mat;
    return *this;
}

inline const D_Matrix4 D_Matrix4::operator -( ) const
{
    return D_Matrix4(
        ( -mCol0 ),
        ( -mCol1 ),
        ( -mCol2 ),
        ( -mCol3 )
    );
}

inline const D_Matrix4 absPerElem( const D_Matrix4 & mat )
{
    return D_Matrix4(
        absPerElem( mat.getCol0() ),
        absPerElem( mat.getCol1() ),
        absPerElem( mat.getCol2() ),
        absPerElem( mat.getCol3() )
    );
}

inline const D_Matrix4 D_Matrix4::operator *( float scalar ) const
{
    return D_Matrix4(
        ( mCol0 * scalar ),
        ( mCol1 * scalar ),
        ( mCol2 * scalar ),
        ( mCol3 * scalar )
    );
}

inline D_Matrix4 & D_Matrix4::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

inline const D_Matrix4 operator *( float scalar, const D_Matrix4 & mat )
{
    return mat * scalar;
}

inline const D_Vector4 D_Matrix4::operator *( const D_Vector4 & vec ) const
{
    return D_Vector4(
        ( ( ( ( mCol0.getX() * vec.getX() ) + ( mCol1.getX() * vec.getY() ) ) + ( mCol2.getX() * vec.getZ() ) ) + ( mCol3.getX() * vec.getW() ) ),
        ( ( ( ( mCol0.getY() * vec.getX() ) + ( mCol1.getY() * vec.getY() ) ) + ( mCol2.getY() * vec.getZ() ) ) + ( mCol3.getY() * vec.getW() ) ),
        ( ( ( ( mCol0.getZ() * vec.getX() ) + ( mCol1.getZ() * vec.getY() ) ) + ( mCol2.getZ() * vec.getZ() ) ) + ( mCol3.getZ() * vec.getW() ) ),
        ( ( ( ( mCol0.getW() * vec.getX() ) + ( mCol1.getW() * vec.getY() ) ) + ( mCol2.getW() * vec.getZ() ) ) + ( mCol3.getW() * vec.getW() ) )
    );
}

inline const D_Vector4 D_Matrix4::operator *( const D_Vector3 & vec ) const
{
    return D_Vector4(
        ( ( ( mCol0.getX() * vec.getX() ) + ( mCol1.getX() * vec.getY() ) ) + ( mCol2.getX() * vec.getZ() ) ),
        ( ( ( mCol0.getY() * vec.getX() ) + ( mCol1.getY() * vec.getY() ) ) + ( mCol2.getY() * vec.getZ() ) ),
        ( ( ( mCol0.getZ() * vec.getX() ) + ( mCol1.getZ() * vec.getY() ) ) + ( mCol2.getZ() * vec.getZ() ) ),
        ( ( ( mCol0.getW() * vec.getX() ) + ( mCol1.getW() * vec.getY() ) ) + ( mCol2.getW() * vec.getZ() ) )
    );
}

inline const D_Vector4 D_Matrix4::operator *( const D_Point3 & pnt ) const
{
    return D_Vector4(
        ( ( ( ( mCol0.getX() * pnt.getX() ) + ( mCol1.getX() * pnt.getY() ) ) + ( mCol2.getX() * pnt.getZ() ) ) + mCol3.getX() ),
        ( ( ( ( mCol0.getY() * pnt.getX() ) + ( mCol1.getY() * pnt.getY() ) ) + ( mCol2.getY() * pnt.getZ() ) ) + mCol3.getY() ),
        ( ( ( ( mCol0.getZ() * pnt.getX() ) + ( mCol1.getZ() * pnt.getY() ) ) + ( mCol2.getZ() * pnt.getZ() ) ) + mCol3.getZ() ),
        ( ( ( ( mCol0.getW() * pnt.getX() ) + ( mCol1.getW() * pnt.getY() ) ) + ( mCol2.getW() * pnt.getZ() ) ) + mCol3.getW() )
    );
}

inline const D_Matrix4 D_Matrix4::operator *( const D_Matrix4 & mat ) const
{
    return D_Matrix4(
        ( *this * mat.mCol0 ),
        ( *this * mat.mCol1 ),
        ( *this * mat.mCol2 ),
        ( *this * mat.mCol3 )
    );
}

inline D_Matrix4 & D_Matrix4::operator *=( const D_Matrix4 & mat )
{
    *this = *this * mat;
    return *this;
}

inline const D_Matrix4 D_Matrix4::operator *( const D_Transform3 & tfrm ) const
{
    return D_Matrix4(
        ( *this * tfrm.getCol0() ),
        ( *this * tfrm.getCol1() ),
        ( *this * tfrm.getCol2() ),
        ( *this * D_Point3( tfrm.getCol3() ) )
    );
}

inline D_Matrix4 & D_Matrix4::operator *=( const D_Transform3 & tfrm )
{
    *this = *this * tfrm;
    return *this;
}

inline const D_Matrix4 mulPerElem( const D_Matrix4 & mat0, const D_Matrix4 & mat1 )
{
    return D_Matrix4(
        mulPerElem( mat0.getCol0(), mat1.getCol0() ),
        mulPerElem( mat0.getCol1(), mat1.getCol1() ),
        mulPerElem( mat0.getCol2(), mat1.getCol2() ),
        mulPerElem( mat0.getCol3(), mat1.getCol3() )
    );
}

inline const D_Matrix4 D_Matrix4::identity( )
{
    return D_Matrix4(
        D_Vector4::xAxis( ),
        D_Vector4::yAxis( ),
        D_Vector4::zAxis( ),
        D_Vector4::wAxis( )
    );
}

inline D_Matrix4 & D_Matrix4::setUpper3x3( const D_Matrix3 & mat3 )
{
    mCol0.setXYZ( mat3.getCol0() );
    mCol1.setXYZ( mat3.getCol1() );
    mCol2.setXYZ( mat3.getCol2() );
    return *this;
}

inline const D_Matrix3 D_Matrix4::getUpper3x3( ) const
{
    return D_Matrix3(
        mCol0.getXYZ( ),
        mCol1.getXYZ( ),
        mCol2.getXYZ( )
    );
}

inline D_Matrix4 & D_Matrix4::setTranslation( const D_Vector3 & translateVec )
{
    mCol3.setXYZ( translateVec );
    return *this;
}

inline const D_Vector3 D_Matrix4::getTranslation( ) const
{
    return mCol3.getXYZ( );
}

inline const D_Matrix4 D_Matrix4::rotationX( float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    return D_Matrix4(
        D_Vector4::xAxis( ),
        D_Vector4( 0.0f, c, s, 0.0f ),
        D_Vector4( 0.0f, -s, c, 0.0f ),
        D_Vector4::wAxis( )
    );
}

inline const D_Matrix4 D_Matrix4::rotationY( float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    return D_Matrix4(
        D_Vector4( c, 0.0f, -s, 0.0f ),
        D_Vector4::yAxis( ),
        D_Vector4( s, 0.0f, c, 0.0f ),
        D_Vector4::wAxis( )
    );
}

inline const D_Matrix4 D_Matrix4::rotationZ( float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    return D_Matrix4(
        D_Vector4( c, s, 0.0f, 0.0f ),
        D_Vector4( -s, c, 0.0f, 0.0f ),
        D_Vector4::zAxis( ),
        D_Vector4::wAxis( )
    );
}

inline const D_Matrix4 D_Matrix4::rotationZYX( const D_Vector3 & radiansXYZ )
{
    float D_sX, cX, D_sY, cY, D_sZ, cZ, tmp0, tmp1;
    D_sX = sinf( radiansXYZ.getX() );
    cX = cosf( radiansXYZ.getX() );
    D_sY = sinf( radiansXYZ.getY() );
    cY = cosf( radiansXYZ.getY() );
    D_sZ = sinf( radiansXYZ.getZ() );
    cZ = cosf( radiansXYZ.getZ() );
    tmp0 = ( cZ * D_sY );
    tmp1 = ( D_sZ * D_sY );
    return D_Matrix4(
        D_Vector4( ( cZ * cY ), ( D_sZ * cY ), -D_sY, 0.0f ),
        D_Vector4( ( ( tmp0 * D_sX ) - ( D_sZ * cX ) ), ( ( tmp1 * D_sX ) + ( cZ * cX ) ), ( cY * D_sX ), 0.0f ),
        D_Vector4( ( ( tmp0 * cX ) + ( D_sZ * D_sX ) ), ( ( tmp1 * cX ) - ( cZ * D_sX ) ), ( cY * cX ), 0.0f ),
        D_Vector4::wAxis( )
    );
}

inline const D_Matrix4 D_Matrix4::rotation( float radians, const D_Vector3 & unitVec )
{
    float x, y, z, s, c, oneMinusC, xy, yz, zx;
    s = sinf( radians );
    c = cosf( radians );
    x = unitVec.getX();
    y = unitVec.getY();
    z = unitVec.getZ();
    xy = ( x * y );
    yz = ( y * z );
    zx = ( z * x );
    oneMinusC = ( 1.0f - c );
    return D_Matrix4(
        D_Vector4( ( ( ( x * x ) * oneMinusC ) + c ), ( ( xy * oneMinusC ) + ( z * s ) ), ( ( zx * oneMinusC ) - ( y * s ) ), 0.0f ),
        D_Vector4( ( ( xy * oneMinusC ) - ( z * s ) ), ( ( ( y * y ) * oneMinusC ) + c ), ( ( yz * oneMinusC ) + ( x * s ) ), 0.0f ),
        D_Vector4( ( ( zx * oneMinusC ) + ( y * s ) ), ( ( yz * oneMinusC ) - ( x * s ) ), ( ( ( z * z ) * oneMinusC ) + c ), 0.0f ),
        D_Vector4::wAxis( )
    );
}

inline const D_Matrix4 D_Matrix4::rotation( const D_Quat & unitQuat )
{
    return D_Matrix4( D_Transform3::rotation( unitQuat ) );
}

inline const D_Matrix4 D_Matrix4::scale( const D_Vector3 & scaleVec )
{
    return D_Matrix4(
        D_Vector4( scaleVec.getX(), 0.0f, 0.0f, 0.0f ),
        D_Vector4( 0.0f, scaleVec.getY(), 0.0f, 0.0f ),
        D_Vector4( 0.0f, 0.0f, scaleVec.getZ(), 0.0f ),
        D_Vector4::wAxis( )
    );
}

inline const D_Matrix4 appendScale( const D_Matrix4 & mat, const D_Vector3 & scaleVec )
{
    return D_Matrix4(
        ( mat.getCol0() * scaleVec.getX( ) ),
        ( mat.getCol1() * scaleVec.getY( ) ),
        ( mat.getCol2() * scaleVec.getZ( ) ),
        mat.getCol3()
    );
}

inline const D_Matrix4 prependScale( const D_Vector3 & scaleVec, const D_Matrix4 & mat )
{
    D_Vector4 scale4;
    scale4 = D_Vector4( scaleVec, 1.0f );
    return D_Matrix4(
        mulPerElem( mat.getCol0(), scale4 ),
        mulPerElem( mat.getCol1(), scale4 ),
        mulPerElem( mat.getCol2(), scale4 ),
        mulPerElem( mat.getCol3(), scale4 )
    );
}

inline const D_Matrix4 D_Matrix4::translation( const D_Vector3 & translateVec )
{
    return D_Matrix4(
        D_Vector4::xAxis( ),
        D_Vector4::yAxis( ),
        D_Vector4::zAxis( ),
        D_Vector4( translateVec, 1.0f )
    );
}

inline const D_Matrix4 D_Matrix4::lookAt( const D_Point3 & eyePos, const D_Point3 & lookAtPos, const D_Vector3 & upVec )
{
    D_Matrix4 m4EyeFrame;
    D_Vector3 v3X, v3Y, v3Z;
    v3Y = normalize( upVec );
    v3Z = normalize( ( eyePos - lookAtPos ) );
    v3X = normalize( cross( v3Y, v3Z ) );
    v3Y = cross( v3Z, v3X );
    m4EyeFrame = D_Matrix4( D_Vector4( v3X ), D_Vector4( v3Y ), D_Vector4( v3Z ), D_Vector4( eyePos ) );
    return orthoInverse( m4EyeFrame );
}

inline const D_Matrix4 D_Matrix4::perspective( float fovyRadians, float aspect, float zNear, float zFar )
{
    float f, rangeInv;
    f = tanf( ( (float)( D__VECTORMATH_PI_OVER_2 ) - ( 0.5f * fovyRadians ) ) );
    rangeInv = ( 1.0f / ( zNear - zFar ) );
    return D_Matrix4(
        D_Vector4( ( f / aspect ), 0.0f, 0.0f, 0.0f ),
        D_Vector4( 0.0f, f, 0.0f, 0.0f ),
        D_Vector4( 0.0f, 0.0f, ( ( zNear + zFar ) * rangeInv ), -1.0f ),
        D_Vector4( 0.0f, 0.0f, ( ( ( zNear * zFar ) * rangeInv ) * 2.0f ), 0.0f )
    );
}

inline const D_Matrix4 D_Matrix4::frustum( float left, float right, float bottom, float top, float zNear, float zFar )
{
    float sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf, n2;
    sum_rl = ( right + left );
    sum_tb = ( top + bottom );
    sum_nf = ( zNear + zFar );
    inv_rl = ( 1.0f / ( right - left ) );
    inv_tb = ( 1.0f / ( top - bottom ) );
    inv_nf = ( 1.0f / ( zNear - zFar ) );
    n2 = ( zNear + zNear );
    return D_Matrix4(
        D_Vector4( ( n2 * inv_rl ), 0.0f, 0.0f, 0.0f ),
        D_Vector4( 0.0f, ( n2 * inv_tb ), 0.0f, 0.0f ),
        D_Vector4( ( sum_rl * inv_rl ), ( sum_tb * inv_tb ), ( sum_nf * inv_nf ), -1.0f ),
        D_Vector4( 0.0f, 0.0f, ( ( n2 * inv_nf ) * zFar ), 0.0f )
    );
}

inline const D_Matrix4 D_Matrix4::orthographic( float left, float right, float bottom, float top, float zNear, float zFar )
{
    float sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf;
    sum_rl = ( right + left );
    sum_tb = ( top + bottom );
    sum_nf = ( zNear + zFar );
    inv_rl = ( 1.0f / ( right - left ) );
    inv_tb = ( 1.0f / ( top - bottom ) );
    inv_nf = ( 1.0f / ( zNear - zFar ) );
    return D_Matrix4(
        D_Vector4( ( inv_rl + inv_rl ), 0.0f, 0.0f, 0.0f ),
        D_Vector4( 0.0f, ( inv_tb + inv_tb ), 0.0f, 0.0f ),
        D_Vector4( 0.0f, 0.0f, ( inv_nf + inv_nf ), 0.0f ),
        D_Vector4( ( -sum_rl * inv_rl ), ( -sum_tb * inv_tb ), ( sum_nf * inv_nf ), 1.0f )
    );
}

inline const D_Matrix4 select( const D_Matrix4 & mat0, const D_Matrix4 & mat1, bool select1 )
{
    return D_Matrix4(
        select( mat0.getCol0(), mat1.getCol0(), select1 ),
        select( mat0.getCol1(), mat1.getCol1(), select1 ),
        select( mat0.getCol2(), mat1.getCol2(), select1 ),
        select( mat0.getCol3(), mat1.getCol3(), select1 )
    );
}

#ifdef _VECTORMATH_DEBUG

inline void print( const D_Matrix4 & mat )
{
    print( mat.getRow( 0 ) );
    print( mat.getRow( 1 ) );
    print( mat.getRow( 2 ) );
    print( mat.getRow( 3 ) );
}

inline void print( const D_Matrix4 & mat, const char * D_name )
{
    printf("%s:\n", D_name);
    print( mat );
}

#endif

inline D_Transform3::D_Transform3( const D_Transform3 & tfrm )
{
    mCol0 = tfrm.mCol0;
    mCol1 = tfrm.mCol1;
    mCol2 = tfrm.mCol2;
    mCol3 = tfrm.mCol3;
}

inline D_Transform3::D_Transform3( float scalar )
{
    mCol0 = D_Vector3( scalar );
    mCol1 = D_Vector3( scalar );
    mCol2 = D_Vector3( scalar );
    mCol3 = D_Vector3( scalar );
}

inline D_Transform3::D_Transform3( const D_Vector3 & _col0, const D_Vector3 & _col1, const D_Vector3 & _col2, const D_Vector3 & _col3 )
{
    mCol0 = _col0;
    mCol1 = _col1;
    mCol2 = _col2;
    mCol3 = _col3;
}

inline D_Transform3::D_Transform3( const D_Matrix3 & tfrm, const D_Vector3 & translateVec )
{
    this->setUpper3x3( tfrm );
    this->setTranslation( translateVec );
}

inline D_Transform3::D_Transform3( const D_Quat & unitQuat, const D_Vector3 & translateVec )
{
    this->setUpper3x3( D_Matrix3( unitQuat ) );
    this->setTranslation( translateVec );
}

inline D_Transform3 & D_Transform3::setCol0( const D_Vector3 & _col0 )
{
    mCol0 = _col0;
    return *this;
}

inline D_Transform3 & D_Transform3::setCol1( const D_Vector3 & _col1 )
{
    mCol1 = _col1;
    return *this;
}

inline D_Transform3 & D_Transform3::setCol2( const D_Vector3 & _col2 )
{
    mCol2 = _col2;
    return *this;
}

inline D_Transform3 & D_Transform3::setCol3( const D_Vector3 & _col3 )
{
    mCol3 = _col3;
    return *this;
}

inline D_Transform3 & D_Transform3::setCol( int col, const D_Vector3 & vec )
{
    *(&mCol0 + col) = vec;
    return *this;
}

inline D_Transform3 & D_Transform3::setRow( int row, const D_Vector4 & vec )
{
    mCol0.setElem( row, vec.getElem( 0 ) );
    mCol1.setElem( row, vec.getElem( 1 ) );
    mCol2.setElem( row, vec.getElem( 2 ) );
    mCol3.setElem( row, vec.getElem( 3 ) );
    return *this;
}

inline D_Transform3 & D_Transform3::setElem( int col, int row, float val )
{
    D_Vector3 tmpV3_0;
    tmpV3_0 = this->getCol( col );
    tmpV3_0.setElem( row, val );
    this->setCol( col, tmpV3_0 );
    return *this;
}

inline float D_Transform3::getElem( int col, int row ) const
{
    return this->getCol( col ).getElem( row );
}

inline const D_Vector3 D_Transform3::getCol0( ) const
{
    return mCol0;
}

inline const D_Vector3 D_Transform3::getCol1( ) const
{
    return mCol1;
}

inline const D_Vector3 D_Transform3::getCol2( ) const
{
    return mCol2;
}

inline const D_Vector3 D_Transform3::getCol3( ) const
{
    return mCol3;
}

inline const D_Vector3 D_Transform3::getCol( int col ) const
{
    return *(&mCol0 + col);
}

inline const D_Vector4 D_Transform3::getRow( int row ) const
{
    return D_Vector4( mCol0.getElem( row ), mCol1.getElem( row ), mCol2.getElem( row ), mCol3.getElem( row ) );
}

inline D_Vector3 & D_Transform3::operator []( int col )
{
    return *(&mCol0 + col);
}

inline const D_Vector3 D_Transform3::operator []( int col ) const
{
    return *(&mCol0 + col);
}

inline D_Transform3 & D_Transform3::operator =( const D_Transform3 & tfrm )
{
    mCol0 = tfrm.mCol0;
    mCol1 = tfrm.mCol1;
    mCol2 = tfrm.mCol2;
    mCol3 = tfrm.mCol3;
    return *this;
}

inline const D_Transform3 inverse( const D_Transform3 & tfrm )
{
    D_Vector3 tmp0, tmp1, tmp2, inv0, inv1, inv2;
    float detinv;
    tmp0 = cross( tfrm.getCol1(), tfrm.getCol2() );
    tmp1 = cross( tfrm.getCol2(), tfrm.getCol0() );
    tmp2 = cross( tfrm.getCol0(), tfrm.getCol1() );
    detinv = ( 1.0f / dot( tfrm.getCol2(), tmp2 ) );
    inv0 = D_Vector3( ( tmp0.getX() * detinv ), ( tmp1.getX() * detinv ), ( tmp2.getX() * detinv ) );
    inv1 = D_Vector3( ( tmp0.getY() * detinv ), ( tmp1.getY() * detinv ), ( tmp2.getY() * detinv ) );
    inv2 = D_Vector3( ( tmp0.getZ() * detinv ), ( tmp1.getZ() * detinv ), ( tmp2.getZ() * detinv ) );
    return D_Transform3(
        inv0,
        inv1,
        inv2,
        D_Vector3( ( -( ( inv0 * tfrm.getCol3().getX() ) + ( ( inv1 * tfrm.getCol3().getY() ) + ( inv2 * tfrm.getCol3().getZ() ) ) ) ) )
    );
}

inline const D_Transform3 orthoInverse( const D_Transform3 & tfrm )
{
    D_Vector3 inv0, inv1, inv2;
    inv0 = D_Vector3( tfrm.getCol0().getX(), tfrm.getCol1().getX(), tfrm.getCol2().getX() );
    inv1 = D_Vector3( tfrm.getCol0().getY(), tfrm.getCol1().getY(), tfrm.getCol2().getY() );
    inv2 = D_Vector3( tfrm.getCol0().getZ(), tfrm.getCol1().getZ(), tfrm.getCol2().getZ() );
    return D_Transform3(
        inv0,
        inv1,
        inv2,
        D_Vector3( ( -( ( inv0 * tfrm.getCol3().getX() ) + ( ( inv1 * tfrm.getCol3().getY() ) + ( inv2 * tfrm.getCol3().getZ() ) ) ) ) )
    );
}

inline const D_Transform3 absPerElem( const D_Transform3 & tfrm )
{
    return D_Transform3(
        absPerElem( tfrm.getCol0() ),
        absPerElem( tfrm.getCol1() ),
        absPerElem( tfrm.getCol2() ),
        absPerElem( tfrm.getCol3() )
    );
}

inline const D_Vector3 D_Transform3::operator *( const D_Vector3 & vec ) const
{
    return D_Vector3(
        ( ( ( mCol0.getX() * vec.getX() ) + ( mCol1.getX() * vec.getY() ) ) + ( mCol2.getX() * vec.getZ() ) ),
        ( ( ( mCol0.getY() * vec.getX() ) + ( mCol1.getY() * vec.getY() ) ) + ( mCol2.getY() * vec.getZ() ) ),
        ( ( ( mCol0.getZ() * vec.getX() ) + ( mCol1.getZ() * vec.getY() ) ) + ( mCol2.getZ() * vec.getZ() ) )
    );
}

inline const D_Point3 D_Transform3::operator *( const D_Point3 & pnt ) const
{
    return D_Point3(
        ( ( ( ( mCol0.getX() * pnt.getX() ) + ( mCol1.getX() * pnt.getY() ) ) + ( mCol2.getX() * pnt.getZ() ) ) + mCol3.getX() ),
        ( ( ( ( mCol0.getY() * pnt.getX() ) + ( mCol1.getY() * pnt.getY() ) ) + ( mCol2.getY() * pnt.getZ() ) ) + mCol3.getY() ),
        ( ( ( ( mCol0.getZ() * pnt.getX() ) + ( mCol1.getZ() * pnt.getY() ) ) + ( mCol2.getZ() * pnt.getZ() ) ) + mCol3.getZ() )
    );
}

inline const D_Transform3 D_Transform3::operator *( const D_Transform3 & tfrm ) const
{
    return D_Transform3(
        ( *this * tfrm.mCol0 ),
        ( *this * tfrm.mCol1 ),
        ( *this * tfrm.mCol2 ),
        D_Vector3( ( *this * D_Point3( tfrm.mCol3 ) ) )
    );
}

inline D_Transform3 & D_Transform3::operator *=( const D_Transform3 & tfrm )
{
    *this = *this * tfrm;
    return *this;
}

inline const D_Transform3 mulPerElem( const D_Transform3 & tfrm0, const D_Transform3 & tfrm1 )
{
    return D_Transform3(
        mulPerElem( tfrm0.getCol0(), tfrm1.getCol0() ),
        mulPerElem( tfrm0.getCol1(), tfrm1.getCol1() ),
        mulPerElem( tfrm0.getCol2(), tfrm1.getCol2() ),
        mulPerElem( tfrm0.getCol3(), tfrm1.getCol3() )
    );
}

inline const D_Transform3 D_Transform3::identity( )
{
    return D_Transform3(
        D_Vector3::xAxis( ),
        D_Vector3::yAxis( ),
        D_Vector3::zAxis( ),
        D_Vector3( 0.0f )
    );
}

inline D_Transform3 & D_Transform3::setUpper3x3( const D_Matrix3 & tfrm )
{
    mCol0 = tfrm.getCol0();
    mCol1 = tfrm.getCol1();
    mCol2 = tfrm.getCol2();
    return *this;
}

inline const D_Matrix3 D_Transform3::getUpper3x3( ) const
{
    return D_Matrix3( mCol0, mCol1, mCol2 );
}

inline D_Transform3 & D_Transform3::setTranslation( const D_Vector3 & translateVec )
{
    mCol3 = translateVec;
    return *this;
}

inline const D_Vector3 D_Transform3::getTranslation( ) const
{
    return mCol3;
}

inline const D_Transform3 D_Transform3::rotationX( float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    return D_Transform3(
        D_Vector3::xAxis( ),
        D_Vector3( 0.0f, c, s ),
        D_Vector3( 0.0f, -s, c ),
        D_Vector3( 0.0f )
    );
}

inline const D_Transform3 D_Transform3::rotationY( float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    return D_Transform3(
        D_Vector3( c, 0.0f, -s ),
        D_Vector3::yAxis( ),
        D_Vector3( s, 0.0f, c ),
        D_Vector3( 0.0f )
    );
}

inline const D_Transform3 D_Transform3::rotationZ( float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    return D_Transform3(
        D_Vector3( c, s, 0.0f ),
        D_Vector3( -s, c, 0.0f ),
        D_Vector3::zAxis( ),
        D_Vector3( 0.0f )
    );
}

inline const D_Transform3 D_Transform3::rotationZYX( const D_Vector3 & radiansXYZ )
{
    float D_sX, cX, D_sY, cY, D_sZ, cZ, tmp0, tmp1;
    D_sX = sinf( radiansXYZ.getX() );
    cX = cosf( radiansXYZ.getX() );
    D_sY = sinf( radiansXYZ.getY() );
    cY = cosf( radiansXYZ.getY() );
    D_sZ = sinf( radiansXYZ.getZ() );
    cZ = cosf( radiansXYZ.getZ() );
    tmp0 = ( cZ * D_sY );
    tmp1 = ( D_sZ * D_sY );
    return D_Transform3(
        D_Vector3( ( cZ * cY ), ( D_sZ * cY ), -D_sY ),
        D_Vector3( ( ( tmp0 * D_sX ) - ( D_sZ * cX ) ), ( ( tmp1 * D_sX ) + ( cZ * cX ) ), ( cY * D_sX ) ),
        D_Vector3( ( ( tmp0 * cX ) + ( D_sZ * D_sX ) ), ( ( tmp1 * cX ) - ( cZ * D_sX ) ), ( cY * cX ) ),
        D_Vector3( 0.0f )
    );
}

inline const D_Transform3 D_Transform3::rotation( float radians, const D_Vector3 & unitVec )
{
    return D_Transform3( D_Matrix3::rotation( radians, unitVec ), D_Vector3( 0.0f ) );
}

inline const D_Transform3 D_Transform3::rotation( const D_Quat & unitQuat )
{
    return D_Transform3( D_Matrix3( unitQuat ), D_Vector3( 0.0f ) );
}

inline const D_Transform3 D_Transform3::scale( const D_Vector3 & scaleVec )
{
    return D_Transform3(
        D_Vector3( scaleVec.getX(), 0.0f, 0.0f ),
        D_Vector3( 0.0f, scaleVec.getY(), 0.0f ),
        D_Vector3( 0.0f, 0.0f, scaleVec.getZ() ),
        D_Vector3( 0.0f )
    );
}

inline const D_Transform3 appendScale( const D_Transform3 & tfrm, const D_Vector3 & scaleVec )
{
    return D_Transform3(
        ( tfrm.getCol0() * scaleVec.getX( ) ),
        ( tfrm.getCol1() * scaleVec.getY( ) ),
        ( tfrm.getCol2() * scaleVec.getZ( ) ),
        tfrm.getCol3()
    );
}

inline const D_Transform3 prependScale( const D_Vector3 & scaleVec, const D_Transform3 & tfrm )
{
    return D_Transform3(
        mulPerElem( tfrm.getCol0(), scaleVec ),
        mulPerElem( tfrm.getCol1(), scaleVec ),
        mulPerElem( tfrm.getCol2(), scaleVec ),
        mulPerElem( tfrm.getCol3(), scaleVec )
    );
}

inline const D_Transform3 D_Transform3::translation( const D_Vector3 & translateVec )
{
    return D_Transform3(
        D_Vector3::xAxis( ),
        D_Vector3::yAxis( ),
        D_Vector3::zAxis( ),
        translateVec
    );
}

inline const D_Transform3 select( const D_Transform3 & tfrm0, const D_Transform3 & tfrm1, bool select1 )
{
    return D_Transform3(
        select( tfrm0.getCol0(), tfrm1.getCol0(), select1 ),
        select( tfrm0.getCol1(), tfrm1.getCol1(), select1 ),
        select( tfrm0.getCol2(), tfrm1.getCol2(), select1 ),
        select( tfrm0.getCol3(), tfrm1.getCol3(), select1 )
    );
}

#ifdef _VECTORMATH_DEBUG

inline void print( const D_Transform3 & tfrm )
{
    print( tfrm.getRow( 0 ) );
    print( tfrm.getRow( 1 ) );
    print( tfrm.getRow( 2 ) );
}

inline void print( const D_Transform3 & tfrm, const char * D_name )
{
    printf("%s:\n", D_name);
    print( tfrm );
}

#endif

inline D_Quat::D_Quat( const D_Matrix3 & tfrm )
{
    float trace, radicand, scale, xx, yx, zx, xy, yy, zy, xz, yz, zz, tmpx, tmpy, tmpz, tmpw, qx, qy, qz, qw;
    int negTrace, ZgtX, ZgtY, YgtX;
    int largestXorY, largestYorZ, largestZorX;

    xx = tfrm.getCol0().getX();
    yx = tfrm.getCol0().getY();
    zx = tfrm.getCol0().getZ();
    xy = tfrm.getCol1().getX();
    yy = tfrm.getCol1().getY();
    zy = tfrm.getCol1().getZ();
    xz = tfrm.getCol2().getX();
    yz = tfrm.getCol2().getY();
    zz = tfrm.getCol2().getZ();

    trace = ( ( xx + yy ) + zz );

    negTrace = ( trace < 0.0f );
    ZgtX = zz > xx;
    ZgtY = zz > yy;
    YgtX = yy > xx;
    largestXorY = ( !ZgtX || !ZgtY ) && negTrace;
    largestYorZ = ( YgtX || ZgtX ) && negTrace;
    largestZorX = ( ZgtY || !YgtX ) && negTrace;
    
    if ( largestXorY )
    {
        zz = -zz;
        xy = -xy;
    }
    if ( largestYorZ )
    {
        xx = -xx;
        yz = -yz;
    }
    if ( largestZorX )
    {
        yy = -yy;
        zx = -zx;
    }

    radicand = ( ( ( xx + yy ) + zz ) + 1.0f );
    scale = ( 0.5f * ( 1.0f / sqrtf( radicand ) ) );

    tmpx = ( ( zy - yz ) * scale );
    tmpy = ( ( xz - zx ) * scale );
    tmpz = ( ( yx - xy ) * scale );
    tmpw = ( radicand * scale );
    qx = tmpx;
    qy = tmpy;
    qz = tmpz;
    qw = tmpw;

    if ( largestXorY )
    {
        qx = tmpw;
        qy = tmpz;
        qz = tmpy;
        qw = tmpx;
    }
    if ( largestYorZ )
    {
        tmpx = qx;
        tmpz = qz;
        qx = qy;
        qy = tmpx;
        qz = qw;
        qw = tmpz;
    }

    mX = qx;
    mY = qy;
    mZ = qz;
    mW = qw;
}

inline const D_Matrix3 outer( const D_Vector3 & tfrm0, const D_Vector3 & tfrm1 )
{
    return D_Matrix3(
        ( tfrm0 * tfrm1.getX( ) ),
        ( tfrm0 * tfrm1.getY( ) ),
        ( tfrm0 * tfrm1.getZ( ) )
    );
}

inline const D_Matrix4 outer( const D_Vector4 & tfrm0, const D_Vector4 & tfrm1 )
{
    return D_Matrix4(
        ( tfrm0 * tfrm1.getX( ) ),
        ( tfrm0 * tfrm1.getY( ) ),
        ( tfrm0 * tfrm1.getZ( ) ),
        ( tfrm0 * tfrm1.getW( ) )
    );
}

inline const D_Vector3 rowMul( const D_Vector3 & vec, const D_Matrix3 & mat )
{
    return D_Vector3(
        ( ( ( vec.getX() * mat.getCol0().getX() ) + ( vec.getY() * mat.getCol0().getY() ) ) + ( vec.getZ() * mat.getCol0().getZ() ) ),
        ( ( ( vec.getX() * mat.getCol1().getX() ) + ( vec.getY() * mat.getCol1().getY() ) ) + ( vec.getZ() * mat.getCol1().getZ() ) ),
        ( ( ( vec.getX() * mat.getCol2().getX() ) + ( vec.getY() * mat.getCol2().getY() ) ) + ( vec.getZ() * mat.getCol2().getZ() ) )
    );
}

inline const D_Matrix3 crossMatrix( const D_Vector3 & vec )
{
    return D_Matrix3(
        D_Vector3( 0.0f, vec.getZ(), -vec.getY() ),
        D_Vector3( -vec.getZ(), 0.0f, vec.getX() ),
        D_Vector3( vec.getY(), -vec.getX(), 0.0f )
    );
}

inline const D_Matrix3 crossMatrixMul( const D_Vector3 & vec, const D_Matrix3 & mat )
{
    return D_Matrix3( cross( vec, mat.getCol0() ), cross( vec, mat.getCol1() ), cross( vec, mat.getCol2() ) );
}

} // namespace D_Aos
} // namespace D_Vectormath

#endif
