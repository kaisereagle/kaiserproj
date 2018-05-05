//***********
// PMD�{�[��
//***********

#ifdef	_WIN32
#include	<gl/glut.h>
#else
#ifdef __APPLE__
#include	<GLUT/glut.h>
#else
#include	<GL/glut.h>
#endif
#endif
#include	<string.h>
#include	<math.h>
#include	"PMDBone.h"


//================
// �R���X�g���N�^
//================
cPMDBone::cPMDBone( void ) : m_pParentBone( NULL ), m_pChildBone( NULL )
{
}

//==============
// �f�X�g���N�^
//==============
cPMDBone::~cPMDBone( void )
{
}

//========
// ������
//========
void cPMDBone::initialize( const PMD_Bone *pPMDBoneData, const cPMDBone pBoneArray[] )
{
	// �{�[�����̃R�s�[
	strncpy( m_szName, pPMDBoneData->szName, 20 );
	m_szName[20] = '\0';

	// �ʒu�̃R�s�[
	m_vec3OrgPosition = pPMDBoneData->vec3Position;

	// �e�{�[���̐ݒ�
	if( pPMDBoneData->nParentNo != -1 )
	{
		m_pParentBone = &(pBoneArray[pPMDBoneData->nParentNo]);
		vec_sub( &m_vec3Offset, &m_vec3OrgPosition, &m_pParentBone->m_vec3OrgPosition );
	}
	else
	{
		// �e�Ȃ�
		m_vec3Offset = m_vec3OrgPosition;
	}

	// �q�{�[���̐ݒ�
	if( pPMDBoneData->nChildNo != -1 )
	{
		m_pChildBone = (cPMDBone *)&(pBoneArray[pPMDBoneData->nChildNo]);
	}

	mat_identity(	m_matInvTransform );
	m_matInvTransform[3][0] = -m_vec3OrgPosition.x; 
	m_matInvTransform[3][1] = -m_vec3OrgPosition.y; 
	m_matInvTransform[3][2] = -m_vec3OrgPosition.z; 

	m_bIKLimitAngle = false;

	// �e�ϐ��̏����l��ݒ�
	reset();
}

//============================================================================
// �e�̃{�[���ԍ��̂ق����傫���ꍇ�ɂ��������Ȃ�̂ŏ������I����ɍČv�Z����
//============================================================================
void cPMDBone::recalcOffset( void )
{
	if( m_pParentBone )	vec_sub( &m_vec3Offset, &m_vec3OrgPosition, &m_pParentBone->m_vec3OrgPosition );
}

//======================
// �e�ϐ��̏����l��ݒ�
//======================
void cPMDBone::reset( void )
{
	m_vec3Position.x = m_vec3Position.y = m_vec3Position.z = 0.0f;
	m_vec4Rotation.x = m_vec4Rotation.y = m_vec4Rotation.z = 0.0f; m_vec4Rotation.w = 1.0f;
	m_vec4LookRotation = m_vec4Rotation;

	mat_identity(	m_matLocal );
	m_matLocal[3][0] = m_vec3OrgPosition.x; 
	m_matLocal[3][1] = m_vec3OrgPosition.y; 
	m_matLocal[3][2] = m_vec3OrgPosition.z; 
}

//====================
// �{�[���̍s����X�V
//====================
void cPMDBone::updateMatrix( void )
{
	// �N�H�[�^�j�I���ƈړ��l����{�[���̃��[�J���}�g���b�N�X���쐬
	quat_to_mat( m_matLocal, &m_vec4Rotation );
	m_matLocal[3][0] = m_vec3Position.x + m_vec3Offset.x; 
	m_matLocal[3][1] = m_vec3Position.y + m_vec3Offset.y; 
	m_matLocal[3][2] = m_vec3Position.z + m_vec3Offset.z; 

	// �e������Ȃ�e�̉�]���󂯌p��
	if( m_pParentBone )	mat_mul( m_matLocal, m_matLocal, m_pParentBone->m_matLocal );
}

#define		RAD(a)	(a * (3.1415926f / 180.0f))
#define		DEG(a)	(a * (180.0f / 3.1415926f))

//==========================
// �{�[�����w����W�֌�����
//==========================
void cPMDBone::lookAt( const vec3 *pvecTargetPos, float fLimitXD, float fLimitXU, float fLimitY  )
{
	// �ǂ������������̂ŗv����

	mtrx	matTemp;

	mat_identity(	matTemp );
	matTemp[3][0] = m_vec3Position.x + m_vec3Offset.x; 
	matTemp[3][1] = m_vec3Position.y + m_vec3Offset.y; 
	matTemp[3][2] = m_vec3Position.z + m_vec3Offset.z;

	if( m_pParentBone )
	{
		mtrx	matInvTemp;
		mat_inverse( matInvTemp, m_pParentBone->m_matLocal );
		matInvTemp[3][0] =  m_pParentBone->m_matLocal[3][0]; 
		matInvTemp[3][1] =  m_pParentBone->m_matLocal[3][1]; 
		matInvTemp[3][2] = -m_pParentBone->m_matLocal[3][2];
		mat_mul( matTemp, matTemp, matInvTemp );
	}
	mat_inverse( matTemp, matTemp );

	vec3		vec3LocalTgtPosZY;
	vec3		vec3LocalTgtPosXZ;

	vec_transform( &vec3LocalTgtPosZY, pvecTargetPos, matTemp );

	vec3LocalTgtPosXZ = vec3LocalTgtPosZY;
	vec3LocalTgtPosXZ.y = 0.0f;
	vec_nomalize( &vec3LocalTgtPosXZ, &vec3LocalTgtPosXZ );

	vec3LocalTgtPosZY.x = 0.0f;
	vec_nomalize( &vec3LocalTgtPosZY, &vec3LocalTgtPosZY );

	vec3		vec3Angle = { 0.0f, 0.0f, 0.0f };

	vec3Angle.x =  asinf( vec3LocalTgtPosZY.y );
	if( vec3LocalTgtPosXZ.x < 0.0f )	vec3Angle.y =  acosf( vec3LocalTgtPosXZ.z );
	else								vec3Angle.y = -acosf( vec3LocalTgtPosXZ.z );

	if( vec3Angle.x < RAD(fLimitXD) )	vec3Angle.x = RAD(fLimitXD);
	if( RAD(fLimitXU) < vec3Angle.x )	vec3Angle.x = RAD(fLimitXU);

	if( vec3Angle.y < RAD(-fLimitY) )	vec3Angle.y = RAD(-fLimitY);
	if( RAD(fLimitY) < vec3Angle.y  )	vec3Angle.y = RAD( fLimitY);

	vec4		vec4RotTemp;

	quat_create_euler( &vec4RotTemp, &vec3Angle );
	quat_slerp( &m_vec4LookRotation, &m_vec4LookRotation, &vec4RotTemp, 0.3f );

	m_vec4Rotation = m_vec4LookRotation;
}

//========================
// �X�L�j���O�p�s����X�V
//========================
void cPMDBone::updateSkinningMat( void )
{
	mat_mul( m_matSkinning, m_matInvTransform, m_matLocal );
}

//==============
// �f�o�b�O�`��
//==============
void cPMDBone::debugDraw( void )
{
	glDisable( GL_TEXTURE_2D );

	glPushMatrix();

		glMultMatrixf( (const float *)m_matLocal );

		glColor4f( 1.0f, 0.0f, 1.0f, 1.0f );

		glutSolidCube( 0.3f );

	glPopMatrix();

	glPushMatrix();

		glColor4f( 1.0f, 1.0f, 1.0f, 1.0f );

		if( m_pParentBone )
		{
			glBegin( GL_LINES );
				glVertex3f( m_pParentBone->m_matLocal[3][0], m_pParentBone->m_matLocal[3][1], m_pParentBone->m_matLocal[3][2] );
				glVertex3f(                m_matLocal[3][0],                m_matLocal[3][1],                m_matLocal[3][2] );
			glEnd();
		}

	glPopMatrix();
}
