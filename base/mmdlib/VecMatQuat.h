//************************************
// ベクトル、行列、クォータニオン演算
//************************************

#ifndef		_VECMATQUAT_H_
#define		_VECMATQUAT_H_

#pragma pack( push, 1 )

// 3要素ベクトル
struct vec3
{
	float	x, y, z;
};

// 4要素ベクトル(クォータニオン兼用)
struct vec4
{
	float	x, y, z, w;
};

// テクスチャUV
struct tex
{
	float	u, v;
};

// アルファ無しカラー
struct color3
{
	float	r, g, b;
};

// アルファありカラー
struct color4
{
	float	r, g, b, a;
};

// 行列
typedef float	mtrx[4][4];

#pragma pack( pop )


extern void vec_add( vec3 *pvec3Out, const vec3 *pvec3Add1, const vec3 *pvec3Add2 );
extern void vec_sub( vec3 *pvec3Dst, const vec3 *pvec3Sub1, const vec3 *pvec3Sub2 );
extern void vec_add_mul( vec3 *pvec3Out, const vec3 *pvec3Add1, const vec3 *pvec3Add2, float fRate );
extern void vec_nomalize( vec3 *pvec3Out, const vec3 *pvec3Src );
extern float vec_dot_product( const vec3 *pvec3Src1, const vec3 *pvec3Src2 );
extern void vec_cross_product( vec3 *pvec3Out, const vec3 *pvec3Src1, const vec3 *pvec3Src2 );
extern void vec_lerp( vec3 *pvec3Out, const vec3 *pvec3Src1, const vec3 *pvec3Src2, float fLerpValue );
extern void vec_transform( vec3 *pvec3Out, const vec3 *pVec3In, const mtrx matTransform );
extern void vec_rotate( vec3 *pvec3Out, const vec3 *pVec3In, const mtrx matRotate );

extern void mat_identity( mtrx matOut );
extern void mat_rotation( mtrx matOut, float fAngle );
extern void mat_mul( mtrx matOut, const mtrx matSrc1, const mtrx matSrc2 );
extern void mat_inverse( mtrx matOut, const mtrx matSrc );
extern void mat_lerp( mtrx matOut, mtrx matSrc1, mtrx matSrc2, float fLerpValue );
extern void mat_planar_projection( mtrx matOut, const vec4 *pvec4Plane, const vec3 *pvec3LightPos );

extern void quat_create_axis( vec4 *pvec4Out, const vec3 *pvec3Axis, float fRotAngle );
extern void quat_create_euler( vec4 *pvec4Out, const vec3 *pvec3EulerAngle );
extern void quat_nomalize( vec4 *pvec4Out, const vec4 *pvec4Src );
extern void quat_mul( vec4 *pvec4Out, const vec4 *pvec4Src1, const vec4 *pvec4Src2 );
extern void quat_slerp( vec4 *pvec4Out, const vec4 *pvec4Src1, const vec4 *pvec4Src2, float fLerpValue );
extern void quat_to_mat( mtrx matOut, const vec4 *pvec4Quat );
extern void quat_to_euler( vec3 *pvecAngle, const vec4 *pvec4Quat );

#endif	// _VECMATQUAT_H_
