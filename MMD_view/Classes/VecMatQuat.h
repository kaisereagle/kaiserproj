//************************************
// ベクトル、行列、クォータニオン演算
//************************************

#ifndef		_VECMATQUAT_H_
#define		_VECMATQUAT_H_

#pragma pack( push, 1 )

// 3要素ベクトル
typedef struct vec3
{
	float	x, y, z;
} vec3;

// 4要素ベクトル(クォータニオン兼用)
typedef struct vec4
{
	float	x, y, z, w;
} vec4;

// テクスチャUV
typedef struct tex
{
	float	u, v;
} tex;

// アルファ無しカラー
typedef struct color3
{
	float	r, g, b;
} color3;

// アルファありカラー
typedef struct color4
{
	float	r, g, b, a;
} color4;

// 行列
typedef float	matrix[4][4];
#pragma pack( pop )

class VecMatQuat{
public:
 static void vec_add( vec3 *pvec3Out, const vec3 *pvec3Add1, const vec3 *pvec3Add2 );
 static void vec_sub( vec3 *pvec3Dst, const vec3 *pvec3Sub1, const vec3 *pvec3Sub2 );
 static void vec_add_mul( vec3 *pvec3Out, const vec3 *pvec3Add1, const vec3 *pvec3Add2, float fRate );
 static void vec_nomalize( vec3 *pvec3Out, const vec3 *pvec3Src );
 static float vec_dot_product( const vec3 *pvec3Src1, const vec3 *pvec3Src2 ); 
 static void vec_cross_product( vec3 *pvec3Out, const vec3 *pvec3Src1, const vec3 *pvec3Src2 );
 static void vec_lerp( vec3 *pvec3Out, const vec3 *pvec3Src1, const vec3 *pvec3Src2, float fLerpValue );
 static void vec_transform( vec3 *pvec3Out, const vec3 *pVec3In, const matrix matTransform );
 static void vec_rotate( vec3 *pvec3Out, const vec3 *pVec3In, const matrix matRotate );

 static void mat_identity( matrix matOut );
 static void mat_rotation( matrix matOut, float fAngle );
 static void mat_mul( matrix matOut, const matrix matSrc1, const matrix matSrc2 );
 static void mat_inverse( matrix matOut, const matrix matSrc );
 static void mat_lerp( matrix matOut, matrix matSrc1, matrix matSrc2, float fLerpValue );
 static void mat_planar_projection( matrix matOut, const vec4 *pvec4Plane, const vec3 *pvec3LightPos );

 static void quat_create_axis( vec4 *pvec4Out, const vec3 *pvec3Axis, float fRotAngle );
 static void quat_create_euler( vec4 *pvec4Out, const vec3 *pvec3EulerAngle );
 static void quat_nomalize( vec4 *pvec4Out, const vec4 *pvec4Src );
 static void quat_mul( vec4 *pvec4Out, const vec4 *pvec4Src1, const vec4 *pvec4Src2 );
 static void quat_slerp( vec4 *pvec4Out, const vec4 *pvec4Src1, const vec4 *pvec4Src2, float fLerpValue );
 static void quat_to_mat( matrix matOut, const vec4 *pvec4Quat );
 static void quat_to_euler( vec3 *pvecAngle, const vec4 *pvec4Quat );
};

	
#endif	// _VECMATQUAT_H_
