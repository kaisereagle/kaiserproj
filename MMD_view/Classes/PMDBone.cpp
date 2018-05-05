//***********
// PMDボーン
//***********

//#include <OpenGLES/EAGL.h>
#ifdef WIN32
#include <windows.h>
#include "windowsx.h"
#include <winuser.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include    <stdio.h>
#include    <stdlib.h>
#include    <stdbool.h>
#include    <assert.h>
#include    <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#else
#include <OpenGLES/ES1/gl.h>
#include <OpenGLES/ES1/glext.h>
#endif
#include <string.h>
#include <math.h>
#include "PMDBone.h"

#include "GlTrans.h"
#include "GlutTrans.h"


//================
// コンストラクタ
//================
cPMDBone::cPMDBone( void ) : m_pParentBone( NULL ), m_pChildBone( NULL )
{
}

//==============
// デストラクタ
//==============
cPMDBone::~cPMDBone( void )
{
}

//========
// 初期化
//========
void cPMDBone::initialize( const PMD_Bone *pPMDBoneData, const cPMDBone pBoneArray[] )
{
	// ボーン名のコピー
	strncpy( m_szName, pPMDBoneData->szName, 20 );
	m_szName[20] = '\0';

	// 位置のコピー
	m_vec3OrgPosition = pPMDBoneData->vec3Position;

	// 親ボーンの設定
	if( pPMDBoneData->nParentNo != -1 )
	{
		m_pParentBone = &(pBoneArray[pPMDBoneData->nParentNo]);
		VecMatQuat::vec_sub( &m_vec3Offset, &m_vec3OrgPosition, &m_pParentBone->m_vec3OrgPosition );
	}
	else
	{
		// 親なし
		m_vec3Offset = m_vec3OrgPosition;
	}

	// 子ボーンの設定
	if( pPMDBoneData->nChildNo != -1 )
	{
		m_pChildBone = (cPMDBone *)&(pBoneArray[pPMDBoneData->nChildNo]);
	}

	VecMatQuat::mat_identity(	m_matInvTransform );
	m_matInvTransform[3][0] = -m_vec3OrgPosition.x; 
	m_matInvTransform[3][1] = -m_vec3OrgPosition.y; 
	m_matInvTransform[3][2] = -m_vec3OrgPosition.z; 

	m_bIKLimitAngle = false;

	// 各変数の初期値を設定
	reset();
}

//============================================================================
// 親のボーン番号のほうが大きい場合におかしくなるので初期化終了後に再計算する
//============================================================================
void cPMDBone::recalcOffset( void )
{
	if( m_pParentBone )	VecMatQuat::vec_sub( &m_vec3Offset, &m_vec3OrgPosition, &m_pParentBone->m_vec3OrgPosition );
}

//======================
// 各変数の初期値を設定
//======================
void cPMDBone::reset( void )
{
	m_vec3Position.x = m_vec3Position.y = m_vec3Position.z = 0.0f;
	m_vec4Rotation.x = m_vec4Rotation.y = m_vec4Rotation.z = 0.0f; m_vec4Rotation.w = 1.0f;
	m_vec4LookRotation = m_vec4Rotation;

	VecMatQuat::mat_identity(	m_matLocal );
	m_matLocal[3][0] = m_vec3OrgPosition.x; 
	m_matLocal[3][1] = m_vec3OrgPosition.y; 
	m_matLocal[3][2] = m_vec3OrgPosition.z; 
}

//====================
// ボーンの行列を更新
//====================
void cPMDBone::updateMatrix( void )
{
	// クォータニオンと移動値からボーンのローカルマトリックスを作成
	VecMatQuat::quat_to_mat( m_matLocal, &m_vec4Rotation );
	m_matLocal[3][0] = m_vec3Position.x + m_vec3Offset.x; 
	m_matLocal[3][1] = m_vec3Position.y + m_vec3Offset.y; 
	m_matLocal[3][2] = m_vec3Position.z + m_vec3Offset.z; 

	// 親があるなら親の回転を受け継ぐ
	if( m_pParentBone )	VecMatQuat::mat_mul( m_matLocal, m_matLocal, m_pParentBone->m_matLocal );
}

#define		RAD(a)	(a * (3.1415926f / 180.0f))
#define		DEG(a)	(a * (180.0f / 3.1415926f))

//==========================
// ボーンを指定座標へ向ける
//==========================
void cPMDBone::lookAt( const vec3 *pvecTargetPos, float fLimitXD, float fLimitXU, float fLimitY  )
{
	// どうもおかしいので要調整

	matrix	matTemp;

	VecMatQuat::mat_identity(	matTemp );
	matTemp[3][0] = m_vec3Position.x + m_vec3Offset.x; 
	matTemp[3][1] = m_vec3Position.y + m_vec3Offset.y; 
	matTemp[3][2] = m_vec3Position.z + m_vec3Offset.z;

	if( m_pParentBone )
	{
		matrix	matInvTemp;
		VecMatQuat::mat_inverse( matInvTemp, m_pParentBone->m_matLocal );
		matInvTemp[3][0] =  m_pParentBone->m_matLocal[3][0]; 
		matInvTemp[3][1] =  m_pParentBone->m_matLocal[3][1]; 
		matInvTemp[3][2] = -m_pParentBone->m_matLocal[3][2];
		VecMatQuat::mat_mul( matTemp, matTemp, matInvTemp );
	}
	VecMatQuat::mat_inverse( matTemp, matTemp );

	vec3		vec3LocalTgtPosZY;
	vec3		vec3LocalTgtPosXZ;

	VecMatQuat::vec_transform( &vec3LocalTgtPosZY, pvecTargetPos, matTemp );

	vec3LocalTgtPosXZ = vec3LocalTgtPosZY;
	vec3LocalTgtPosXZ.y = 0.0f;
	VecMatQuat::vec_nomalize( &vec3LocalTgtPosXZ, &vec3LocalTgtPosXZ );

	vec3LocalTgtPosZY.x = 0.0f;
	VecMatQuat::vec_nomalize( &vec3LocalTgtPosZY, &vec3LocalTgtPosZY );

	vec3		vec3Angle = { 0.0f, 0.0f, 0.0f };

	vec3Angle.x =  asinf( vec3LocalTgtPosZY.y );
	if( vec3LocalTgtPosXZ.x < 0.0f )	vec3Angle.y =  acosf( vec3LocalTgtPosXZ.z );
	else								vec3Angle.y = -acosf( vec3LocalTgtPosXZ.z );

	if( vec3Angle.x < RAD(fLimitXD) )	vec3Angle.x = RAD(fLimitXD);
	if( RAD(fLimitXU) < vec3Angle.x )	vec3Angle.x = RAD(fLimitXU);

	if( vec3Angle.y < RAD(-fLimitY) )	vec3Angle.y = RAD(-fLimitY);
	if( RAD(fLimitY) < vec3Angle.y  )	vec3Angle.y = RAD( fLimitY);

	vec4		vec4RotTemp;

	VecMatQuat::quat_create_euler( &vec4RotTemp, &vec3Angle );
	VecMatQuat::quat_slerp( &m_vec4LookRotation, &m_vec4LookRotation, &vec4RotTemp, 0.3f );

	m_vec4Rotation = m_vec4LookRotation;
}

//========================
// スキニング用行列を更新
//========================
void cPMDBone::updateSkinningMat( void )
{
	VecMatQuat::mat_mul( m_matSkinning, m_matInvTransform, m_matLocal );
}

//==============
// デバッグ描画
//==============
void cPMDBone::debugDraw( void )
{
	glDisable( GL_TEXTURE_2D );

	glPushMatrix();

		glMultMatrixf( (const float *)m_matLocal );

		glColor4f( 1.0f, 0.0f, 1.0f, 1.0f );

	GlutTrans::glutSolidCube( 0.3f );

	glPopMatrix();

	glPushMatrix();

		glColor4f( 1.0f, 1.0f, 1.0f, 1.0f );

		if( m_pParentBone )
		{
			GlTrans te;
			te.glBegin( GL_LINES );
				te.glVertex3f( m_pParentBone->m_matLocal[3][0], m_pParentBone->m_matLocal[3][1], m_pParentBone->m_matLocal[3][2] );
				te.glVertex3f(                m_matLocal[3][0],                m_matLocal[3][1],                m_matLocal[3][2] );
			te.glEnd();
		}

	glPopMatrix();
}
