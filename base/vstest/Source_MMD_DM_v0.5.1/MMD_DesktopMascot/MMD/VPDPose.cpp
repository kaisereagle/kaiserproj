//***************
// VMD���[�V����
//***************

#include	<stdio.h>
#include	<stdlib.h>
#include	<malloc.h>
#include	<string.h>
#include	"VPDPose.h"

//================
// �R���X�g���N�^
//================
cVPDPose::cVPDPose( void ) : m_pPoseDataList( NULL )
{
}

//==============
// �f�X�g���N�^
//==============
cVPDPose::~cVPDPose( void )
{
	release();
}

//================
// �t�@�C���̓Ǎ�
//================
bool cVPDPose::load( const wchar_t *wszFilePath )
{
	FILE	*pFile;

	if (_wfopen_s( &pFile, wszFilePath, L"rt" )) {
		return false;	// �t�@�C�����J���Ȃ�
	}

	// �ǂݍ���
	bool	bRet = initialize( pFile );
	fclose( pFile );

	return bRet;
}

//=======================================
// �f�[�^���̋󔒕����ƃR�����g����菜��
//=======================================
void cVPDPose::trim( char *szStr )
{
	char szRet[256];
	int  iR = 0;
	memset( szRet, '\0', sizeof( szRet ) );

	for ( int i = 0; i < sizeof( szRet ); i++ ) {
		if ( szStr[i] == '\0' || szStr[i] == '\r' || szStr[i] == '\n' ) break;	// ������̍Ōォ���s
		if ( szStr[i] == ';' ) break;											// �Z�~�R������������̍Ō�ƌ��Ȃ�
		if ( szStr[i] == '/' && szStr[i+1] == '/' ) break;						// ����ȍ~�R�����g
		if ( iR < 1 && ( szStr[i] == ' ' || szStr[i] == '\t' ) ) continue;		// �擪�̋󔒕����͔�΂�

		// ����ȊO�̕����͎c��
		szRet[iR] = szStr[i];
		iR++;
	}
	
	strcpy( szStr, szRet );
}

//====================
// �|�[�Y�f�[�^�̓Ǎ�
//====================
bool cVPDPose::initialize( FILE *pFile )
{
	char buf[256];
	int  nBone = 0;					// �Ǎ��ς݃{�[���ԍ�
	int  phase = 0;					// ���݉���Ǎ���ł��邩��\��
	PoseDataList *pBone = NULL;

	// �w�b�_�̃`�F�b�N
	fgets( buf, 256, pFile );
	trim( buf );

	if( strncmp( buf, "Vocaloid Pose Data file", 30 ) != 0 )
		return false;	// �t�@�C���`�����Ⴄ

	// �V�f�[�^���ǂݍ��߂����Ȃ�Ήߋ��̃f�[�^���
	release();

	//-----------------------------------------------------
	// �Ǎ�
	while ( !feof( pFile ) ) {
		fgets( buf, 256, pFile );
		trim( buf );

		if ( strlen( buf ) < 1 ) continue;	// ��s�Ȃ��΂�

		if ( phase == 0 ) {
			// �e�t�@�C����
			//strcpy( szBaseFile, buf );
			phase++;
			continue;
		} else if ( phase == 1 ) {
			// �{�[����
			m_unNumPoseBones = (unsigned int) atoi( buf );
			phase++;
			continue;
		} else if ( phase == 2 ) {
			// �Ǎ��ݍς݃{�[�������w�b�_�[�̐��ȏ�ɂȂ�����A�I��
			if (nBone >= m_unNumPoseBones) {
				phase++;
				break;	// continue;
			}

			// �{�[���\���̍쐬
			if (pBone == NULL) {
				// �ŏ��̃{�[���������ꍇ
				pBone = new PoseDataList();
				m_pPoseDataList = pBone;
			} else {
				// 2�Ԗڈȍ~�������ꍇ
				pBone->pNext = new PoseDataList();
				pBone = pBone->pNext;
			}

			// �{�[����
			char* pName = strchr( buf, '{' );
			memset( pBone->szBoneName, '\0', sizeof( pBone->szBoneName ) );
			strncpy_s( pBone->szBoneName, pName + 1, sizeof( pBone->szBoneName ));

			// �{�[�����W
			fgets( buf, 256, pFile );
			trim( buf );
			sscanf_s(buf, "%f,%f,%f", &(pBone->vecPosition.x), &(pBone->vecPosition.y), &(pBone->vecPosition.z));

			// �{�[����]
			fgets( buf, 256, pFile );
			trim( buf );
			sscanf_s(buf, "%f,%f,%f,%f", &(pBone->vecRotation.x), &(pBone->vecRotation.y), &(pBone->vecRotation.z), &(pBone->vecRotation.w));

			// ������
			fgets( buf, 256, pFile );

			nBone++;
		} else {
			break;
		}
	}
	return true;
}

//======
// ���
//======
void cVPDPose::release( void )
{
	// �|�[�Y�f�[�^�̉��
	PoseDataList	*pPoseTemp = m_pPoseDataList,
					*pNextPoseTemp;

	while( pPoseTemp )
	{
		pNextPoseTemp = pPoseTemp->pNext;

		delete pPoseTemp;

		pPoseTemp = pNextPoseTemp;
	}

	m_pPoseDataList = NULL;
}
