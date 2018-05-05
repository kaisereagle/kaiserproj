//************************************
// ベクトル、行列、クォータニオン演算
//************************************

#ifndef		_VECMATQUAT_H_
#define		_VECMATQUAT_H_

#pragma pack( push, 1 )

// 3要素ベクトル
typedef struct Vector3
{
	float	x, y, z;
} Vector3;

// 4要素ベクトル(クォータニオン兼用)
typedef struct Vector4
{
	float	x, y, z, w;
} Vector4;

// テクスチャUV
typedef struct TexUV
{
	float	u, v;
} TexUV;

// アルファ無しカラー
typedef struct Color3
{
	float	r, g, b;
} Color3;

// アルファありカラー
typedef struct Color4
{
	float	r, g, b, a;
} Color4;

// 行列
typedef float	Matrix[4][4];
#pragma pack( pop )

class VecMatQuat{
public:
 static void Vector3Add( Vector3 *pvec3Out, const Vector3 *pvec3Add1, const Vector3 *pvec3Add2 );
 static void Vector3Sub( Vector3 *pvec3Dst, const Vector3 *pvec3Sub1, const Vector3 *pvec3Sub2 );
 static void Vector3MulAdd( Vector3 *pvec3Out, const Vector3 *pvec3Add1, const Vector3 *pvec3Add2, float fRate );
 static void Vector3Normalize( Vector3 *pvec3Out, const Vector3 *pvec3Src );
 static float Vector3DotProduct( const Vector3 *pvec3Src1, const Vector3 *pvec3Src2 ); 
 static void Vector3CrossProduct( Vector3 *pvec3Out, const Vector3 *pvec3Src1, const Vector3 *pvec3Src2 );
 static void Vector3Lerp( Vector3 *pvec3Out, const Vector3 *pvec3Src1, const Vector3 *pvec3Src2, float fLerpValue );
 static void Vector3Transform( Vector3 *pvec3Out, const Vector3 *pVec3In, const Matrix matTransform );
 static void Vector3Rotate( Vector3 *pvec3Out, const Vector3 *pVec3In, const Matrix matRotate );

 static void MatrixIdentity( Matrix matOut );
 static void MatrixRotationX( Matrix matOut, float fAngle );
 static void MatrixMultiply( Matrix matOut, const Matrix matSrc1, const Matrix matSrc2 );
 static void MatrixInverse( Matrix matOut, const Matrix matSrc );
 static void MatrixLerp( Matrix matOut, Matrix matSrc1, Matrix matSrc2, float fLerpValue );
 static void MatrixPlanarProjection( Matrix matOut, const Vector4 *pvec4Plane, const Vector3 *pvec3LightPos );

 static void QuaternionCreateAxis( Vector4 *pvec4Out, const Vector3 *pvec3Axis, float fRotAngle );
 static void QuaternionCreateEuler( Vector4 *pvec4Out, const Vector3 *pvec3EulerAngle );
 static void QuaternionNormalize( Vector4 *pvec4Out, const Vector4 *pvec4Src );
 static void QuaternionMultiply( Vector4 *pvec4Out, const Vector4 *pvec4Src1, const Vector4 *pvec4Src2 );
 static void QuaternionSlerp( Vector4 *pvec4Out, const Vector4 *pvec4Src1, const Vector4 *pvec4Src2, float fLerpValue );
 static void QuaternionToMatrix( Matrix matOut, const Vector4 *pvec4Quat );
 static void QuaternionToEuler( Vector3 *pvecAngle, const Vector4 *pvec4Quat );
};

	
#endif	// _VECMATQUAT_H_
