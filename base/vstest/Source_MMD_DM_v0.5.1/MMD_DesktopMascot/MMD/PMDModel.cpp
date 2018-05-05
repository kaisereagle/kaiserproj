//***********
// PMD���f��
//***********

#include	<windows.h>
#include	<GL/gl.h>
#include	<GL/glu.h>

#include	<stdio.h>
#include	<stdlib.h>
#include	<malloc.h>
#include	<string.h>
//#include	<gl/glut.h>
#include	"PMDModel.h"
#include	"TextureList.h"

#define	_DBG_BONE_DRAW		(0)
#define	_DBG_IK_DRAW		(0)
#define	_DBG_TEXTURE_DRAW	(0)
#define	_DBG_RIGIDBODY_DRAW	(0)

extern cTextureList		g_clsTextureList;

//--------------------------
// IK�f�[�^�\�[�g�p��r�֐�
//--------------------------
static int compareFunc( const void *pA, const void *pB )
{
	return (int)(((cPMDIK *)pA)->getSortVal() - ((cPMDIK *)pB)->getSortVal());
}

//--------------------------
// �F��RGBA���������ꂼ��1�𒴂��Ȃ��悤�ɂ���
//--------------------------
static void saturateColor( float *pfCol )
{
	if (pfCol[0] > 1.0f) pfCol[0] = 1.0f;
	if (pfCol[1] > 1.0f) pfCol[1] = 1.0f;
	if (pfCol[2] > 1.0f) pfCol[2] = 1.0f;
	if (pfCol[3] > 1.0f) pfCol[3] = 1.0f;
	return;
}

//--------------------------
//// �F��RGB�ǂꂩ��1�𒴂��Ă����ꍇ�A�m�[�}���C�Y���s��
////--------------------------
//static void normalizeColor( float *pfCol )
//{
//	if (pfCol[3] > 1.0f) pfCol[3] = 1.0f;		// ���l��RGB�Ƃ͕ʂɁA�ő�1�ɑ�����
//
//	float len = 0.0f;
//	if (pfCol[0] > 1.0f) len = pfCol[0];
//	if (pfCol[1] > 1.0f) len = max(len, pfCol[1]);
//	if (pfCol[2] > 1.0f) len = max(len, pfCol[2]);
//
//	if (len == 0.0f) return;	// RGB�������1�𒴂��Ă��Ȃ���΁A���̂܂܏I��
//
//	pfCol[0] /= len;
//	pfCol[1] /= len;
//	pfCol[2] /= len;
//	return;
//}


//================
// �R���X�g���N�^
//================
cPMDModel::cPMDModel( void ) : m_pvec3OrgPositionArray( NULL ), m_pvec3OrgNormalArray( NULL ), m_puvOrgTexureUVArray( NULL ),
									m_pOrgSkinInfoArray( NULL ), m_pvec3PositionArray( NULL ), m_pvec3NormalArray( NULL ), m_pIndices( NULL ),
										m_pMaterials( NULL ), m_pBoneArray( NULL ), m_pNeckBone( NULL ), m_pCenterBone( NULL ), m_pIKArray( NULL ), m_pFaceArray( NULL ),
											m_pRigidBodyArray( NULL ), m_pConstraintArray( NULL ), m_iDebug( NULL ), m_bPhysics( TRUE )
{
}

//==============
// �f�X�g���N�^
//==============
cPMDModel::~cPMDModel( void )
{
	release();
}

//====================
// �t�@�C���̓ǂݍ���
//====================
bool cPMDModel::load( const wchar_t *wszFilePath )
{
	FILE	*pFile;
	fpos_t	fposFileSize;
	unsigned char
			*pData;

	pFile = _wfopen( wszFilePath, L"rb" );
	if( !pFile )	return false;	// �t�@�C�����J���Ȃ�

	// �t�@�C���T�C�Y�擾
	fseek( pFile, 0, SEEK_END );
	fgetpos( pFile, &fposFileSize );

	// �������m��
	pData = (unsigned char *)malloc( (size_t)fposFileSize );

	// �ǂݍ���
	fseek( pFile, 0, SEEK_SET );
	fread( pData, 1, (size_t)fposFileSize, pFile );

	fclose( pFile );

	// ���f���f�[�^������
	bool	bRet = initialize( pData, fposFileSize );

	free( pData );

	return bRet;
}

//======================
// ���f���f�[�^�̏�����
//======================
bool cPMDModel::initialize( unsigned char *pData, unsigned long ulDataSize )
{
	// �w�b�_�̃`�F�b�N
	PMD_Header	*pPMDHeader = (PMD_Header *)pData;
	if( pPMDHeader->szMagic[0] != 'P' || pPMDHeader->szMagic[1] != 'm' || pPMDHeader->szMagic[2] != 'd' || pPMDHeader->fVersion != 1.0f )
		return false;	// �t�@�C���`�����Ⴄ

	// 2009/07/15 Ru--en	�V�f�[�^��ǂݍ��߂����Ȃ�΂���܂ł̃f�[�^���
	release();
	m_clMotionPlayer.clear();

	m_bLookAt = false;

	unsigned char *pDataTop = pData;

	// ���f�����̃R�s�[
	strncpy_s( m_szModelName, pPMDHeader->szName, 20 );
	m_szModelName[20] = '\0';

	pData += sizeof( PMD_Header );

	unsigned long	ulSize;

	//-----------------------------------------------------
	// ���_���擾
	m_ulNumVertices = *((unsigned long *)pData);
	pData += sizeof( unsigned long );

	// ���_�z����R�s�[
	m_pvec3OrgPositionArray =  (Vector3 *)malloc( sizeof(Vector3)  * m_ulNumVertices );
	m_pvec3OrgNormalArray   =  (Vector3 *)malloc( sizeof(Vector3)  * m_ulNumVertices );
	m_puvOrgTexureUVArray   =    (TexUV *)malloc( sizeof(TexUV)    * m_ulNumVertices );
	m_pOrgSkinInfoArray     = (SkinInfo *)malloc( sizeof(SkinInfo) * m_ulNumVertices ); 

	PMD_Vertex	*pVertex = (PMD_Vertex *)pData;
	for( unsigned long i = 0 ; i < m_ulNumVertices ; i++, pVertex++ )
	{
		m_pvec3OrgPositionArray[i] =  pVertex->vec3Pos;
		m_pvec3OrgNormalArray[i]   =  pVertex->vec3Normal;
		m_puvOrgTexureUVArray[i]   =  pVertex->uvTex;

		m_pOrgSkinInfoArray[i].fWeight     = pVertex->cbWeight / 100.0f; 
		m_pOrgSkinInfoArray[i].unBoneNo[0] = pVertex->unBoneNo[0]; 
		m_pOrgSkinInfoArray[i].unBoneNo[1] = pVertex->unBoneNo[1]; 
	}

	ulSize = sizeof( PMD_Vertex ) * m_ulNumVertices;
	pData += ulSize;

	// �X�L�j���O�p���_�z��쐬
	m_pvec3PositionArray = (Vector3 *)malloc( sizeof(Vector3) * m_ulNumVertices );
	m_pvec3NormalArray   = (Vector3 *)malloc( sizeof(Vector3) * m_ulNumVertices );

	//-----------------------------------------------------
	// ���_�C���f�b�N�X���擾
	m_ulNumIndices = *((unsigned long *)pData);
	pData += sizeof( unsigned long );

	// ���_�C���f�b�N�X�z����R�s�[
	ulSize = sizeof( unsigned short ) * m_ulNumIndices;
	m_pIndices = (unsigned short *)malloc( ulSize );

	memcpy( m_pIndices, pData, ulSize );
	pData += ulSize;

	//-----------------------------------------------------
	// �}�e���A�����擾
	m_ulNumMaterials = *((unsigned long *)pData);
	pData += sizeof( unsigned long );

	// �}�e���A���z����R�s�[
	m_pMaterials = (Material *)malloc( sizeof(Material) * m_ulNumMaterials );

	PMD_Material	*pPMDMaterial = (PMD_Material *)pData;
	for( unsigned long i = 0 ; i < m_ulNumMaterials ; i++ )
	{
		m_pMaterials[i].col4Diffuse = pPMDMaterial->col4Diffuse;

		m_pMaterials[i].col4Specular.r = pPMDMaterial->col3Specular.r;
		m_pMaterials[i].col4Specular.g = pPMDMaterial->col3Specular.g;
		m_pMaterials[i].col4Specular.b = pPMDMaterial->col3Specular.b;
		m_pMaterials[i].col4Specular.a = 1.0f;

		m_pMaterials[i].col4Ambient.r = pPMDMaterial->col3Ambient.r;
		m_pMaterials[i].col4Ambient.g = pPMDMaterial->col3Ambient.g;
		m_pMaterials[i].col4Ambient.b = pPMDMaterial->col3Ambient.b;
		m_pMaterials[i].col4Ambient.a = 1.0f;

		m_pMaterials[i].fShininess = pPMDMaterial->fShininess;
		m_pMaterials[i].ulNumIndices = pPMDMaterial->ulNumIndices;

		m_pMaterials[i].bEdgeFlag = (pPMDMaterial->cbEdgeFlag == 1);

		m_pMaterials[i].uiTexID = 0xFFFFFFFF;
		m_pMaterials[i].uiMapID = 0xFFFFFFFF;
		if( pPMDMaterial->szTextureFileName[0] )
		{
			char szTexName[20];
			char szMapName[20];

			// �e�N�X�`���g�p�t���O
			type_TexFlags flag = parseTexName( pPMDMaterial->szTextureFileName, szTexName, szMapName );
			m_pMaterials[i].unTexFlag = flag;
			
			if ( (flag & TEXFLAG_TEXTURE) != TEXFLAG_NONE )
			{
				// �e�N�X�`������Ȃ�Ǎ�
				m_pMaterials[i].uiTexID = g_clsTextureList.getTexture( szTexName );
			}

			if ( (flag & ( TEXFLAG_MAP | TEXFLAG_ADDMAP )) != TEXFLAG_NONE )
			{
				// �ʏ�X�t�B�A�}�b�v�܂��͉��Z�X�t�B�A�}�b�v����Ȃ�Ǎ�
				m_pMaterials[i].uiMapID = g_clsTextureList.getTexture( szMapName );
			}
		}
		else
		{
			m_pMaterials[i].unTexFlag = TEXFLAG_NONE;
		}

		pPMDMaterial++;
	}

	pData += sizeof(PMD_Material) * m_ulNumMaterials;

	//-----------------------------------------------------
	// �{�[�����擾
	m_unNumBones = *((unsigned short *)pData);
	pData += sizeof( unsigned short );

	// �{�[���z����쐬
	m_pBoneArray = new cPMDBone[m_unNumBones];

	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
	{
		m_pBoneArray[i].initialize( (const PMD_Bone *)pData, m_pBoneArray );

		// ��̃{�[����ۑ�
		if( strncmp( m_pBoneArray[i].getName(), "��", 20 ) == 0 )	m_pNeckBone = &m_pBoneArray[i];

		// �Z���^�[�̃{�[����ۑ�
		if( strncmp( m_pBoneArray[i].getName(), "�Z���^�[", 20 ) == 0 ) m_pCenterBone = &m_pBoneArray[i];

		pData += sizeof( PMD_Bone );	
	}
	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )	m_pBoneArray[i].recalcOffset();

	//-----------------------------------------------------
	// IK���擾
	m_unNumIK = *((unsigned short *)pData);
	pData += sizeof( unsigned short );

	// IK�z����쐬
	if( m_unNumIK )
	{
		m_pIKArray = new cPMDIK[m_unNumIK];

		for( unsigned short i = 0 ; i < m_unNumIK ; i++ )
		{
			m_pIKArray[i].initialize( (const PMD_IK *)pData, m_pBoneArray );
			pData += sizeof( PMD_IK ) + sizeof(unsigned short) * (((PMD_IK *)pData)->cbNumLink - 1);
		}
		qsort( m_pIKArray, m_unNumIK, sizeof(cPMDIK), compareFunc );
	}

	//-----------------------------------------------------
	// �\��擾
	m_unNumFaces = *((unsigned short *)pData);
	pData += sizeof( unsigned short );

	// �\��z����쐬
	if( m_unNumFaces )
	{
		m_pFaceArray = new cPMDFace[m_unNumFaces];

		for( unsigned short i = 0 ; i < m_unNumFaces ; i++ )
		{
			m_pFaceArray[i].initialize( (const PMD_Face *)pData, &m_pFaceArray[0] );
			pData += sizeof( PMD_Face ) + sizeof(PMD_FaceVtx) * (((PMD_Face *)pData)->ulNumVertices - 1);
		}
	}

	//-----------------------------------------------------
	// �������獄�̏��܂œǂݔ�΂�

	// �\��g�p�\�����X�g
	unsigned char	cbFaceDispNum = *((unsigned char *)pData);
	pData += sizeof( unsigned char );

	pData += sizeof( unsigned short ) * cbFaceDispNum;

	// �{�[���g�p�g�����X�g
	unsigned char	cbBoneDispNum = *((unsigned char *)pData);
	pData += sizeof( unsigned char );

	pData += sizeof( char ) * 50 * cbBoneDispNum;

	// �{�[���g�p�\�����X�g
	unsigned long	ulBoneFrameDispNum = *((unsigned long *)pData);
	pData += sizeof( unsigned long );

	pData += 3 * ulBoneFrameDispNum;

	// �㑱�f�[�^�̗L�����`�F�b�N
	if( (unsigned long)pData - (unsigned long)pDataTop >= ulDataSize )	return true;

	// �p�ꖼ�Ή�
	unsigned char	cbEnglishNameExist = *((unsigned char *)pData);
	pData += sizeof( unsigned char );

	if( cbEnglishNameExist )
	{
		// ���f�����ƃR�����g(�p��)
		pData += sizeof( char ) * 276;

		// �{�[����(�p��)
		pData += sizeof( char ) * 20 * m_unNumBones;

		// �\�(�p��)	�c "base"�͉p�ꖼ���X�g�ɂ͊܂܂�Ȃ��̂�-1
		if (m_unNumFaces > 0)	pData += sizeof( char ) * 20 * (m_unNumFaces - 1);

		// �{�[���g�p�g�����X�g(�p��)
		pData += sizeof( char ) * 50 * (cbBoneDispNum);
	}

	// �㑱�f�[�^�̗L�����`�F�b�N(�����̃`�F�b�N�͕s�v����)
	if( (unsigned long)pData - (unsigned long)pDataTop >= ulDataSize )	return true;

	// �g�D�[���e�N�X�`�����X�g
	pData += 100 * 10;		// �t�@�C���� 100Byte * 10��

	// �����܂œǂݔ�΂�
	//-----------------------------------------------------

	// �㑱�f�[�^�̗L�����`�F�b�N
	if( (unsigned long)pData - (unsigned long)pDataTop >= ulDataSize )	return true;

	//-----------------------------------------------------
	// ���̐��擾
	m_ulRigidBodyNum = *((unsigned long *)pData);
	pData += sizeof( unsigned long );

	// ���̔z����쐬
	if( m_ulRigidBodyNum )
	{
		m_pRigidBodyArray = new cPMDRigidBody[m_ulRigidBodyNum];

		for( unsigned long i = 0 ; i < m_ulRigidBodyNum ; i++ )
		{
			const PMD_RigidBody *pPMDRigidBody = (const PMD_RigidBody *)pData;

			cPMDBone	*pBone = NULL;
			if( pPMDRigidBody->unBoneIndex == 0xFFFF )	pBone = getBoneByName( "�Z���^�[" );
			else										pBone = &m_pBoneArray[pPMDRigidBody->unBoneIndex];

			m_pRigidBodyArray[i].initialize( pPMDRigidBody, pBone );
			pData += sizeof( PMD_RigidBody );
		}
	}

	//-----------------------------------------------------
	// �R���X�g���C���g���擾
	m_ulConstraintNum = *((unsigned long *)pData);
	pData += sizeof( unsigned long );

	// �R���X�g���C���g�z����쐬
	if( m_ulConstraintNum )
	{
		m_pConstraintArray = new cPMDConstraint[m_ulConstraintNum];

		for( unsigned long i = 0 ; i < m_ulConstraintNum ; i++ )
		{
			const PMD_Constraint *pPMDConstraint = (const PMD_Constraint *)pData;

			cPMDRigidBody	*pRigidBodyA = &m_pRigidBodyArray[pPMDConstraint->ulRigidA],
							*pRigidBodyB = &m_pRigidBodyArray[pPMDConstraint->ulRigidB];

			m_pConstraintArray[i].initialize( pPMDConstraint, pRigidBodyA, pRigidBodyB );
			pData += sizeof( PMD_Constraint );
		}
	}

	return true;
}

////----------------------------------------------
//// �X�t�B�A�}�b�v�p�̃e�N�X�`�������ǂ�����Ԃ�
////----------------------------------------------
//bool cPMDModel::isSphereMapTexName( const char *szTextureName )
//{
//	int		iLen = strlen( szTextureName );
//	bool	bRet = false;
//
//	if( 	(szTextureName[iLen - 3] == 's' || szTextureName[iLen - 3] == 'S') &&
//			(szTextureName[iLen - 2] == 'p' || szTextureName[iLen - 2] == 'P') &&
//			(szTextureName[iLen - 1] == 'h' || szTextureName[iLen - 1] == 'H')		)
//	{
//		bRet = true;
//	}
//
//	return bRet;
//}

//----------------------------------------------
// �X�t�B�A�}�b�v�p�̃e�N�X�`�������ǂ�����Ԃ�
//----------------------------------------------
cPMDModel::type_TexFlags cPMDModel::parseTexName( const char *szTextureName, char *szOutTexName, char *szOutMapName )
{
	type_TexFlags	fRet = TEXFLAG_NONE;

	memset( szOutTexName, '\0', 20 );
	memset( szOutMapName, '\0', 20 );

	const char *szSeparator = strchr( szTextureName, '*' );		// ��؂蕶���̃A�X�^���X�N��T��
	if( szSeparator == NULL )
	{
		// �A�X�^���X�N���Ȃ������ꍇ

		int iLen = strlen( szTextureName );
		if ( iLen < 1 )	return fRet;						// �e�N�X�`���w��Ȃ�

		// �g���q���X�t�B�A�}�b�v�̂��̂����ׂ�
		if(		(iLen >= 4) &&
				(szTextureName[iLen - 4] == '.') &&
				(szTextureName[iLen - 3] == 's' || szTextureName[iLen - 3] == 'S') &&
				(szTextureName[iLen - 2] == 'p' || szTextureName[iLen - 2] == 'P')		)
		{
			if ( (szTextureName[iLen - 1] == 'h' || szTextureName[iLen - 1] == 'H') )		// �ʏ�i��Z�j�X�t�B�A�}�b�v
			{
				strcpy( szOutMapName, szTextureName );
				fRet = TEXFLAG_MAP;
			}
			else if ( (szTextureName[iLen - 1] == 'a' || szTextureName[iLen - 1] == 'A') )	// ���Z�X�t�B�A�}�b�v
			{
				strcpy( szOutMapName, szTextureName );
				fRet = TEXFLAG_ADDMAP;
			}
			else
			{
				fRet = TEXFLAG_NONE;		// ��L�ȊO�Ŋg���q ".sp?" �Ȃ�΃e�N�X�`���Ȃ������ɂ��Ă���
			}
		}
		else
		{
			// �X�t�B�A�}�b�v�łȂ���΃e�N�X�`���Ɖ���
			strcpy( szOutTexName, szTextureName );
			fRet = TEXFLAG_TEXTURE;
		}
	}
	else
	{
		// �A�X�^���X�N���������ꍇ

		strncpy( szOutTexName, szTextureName, ( szSeparator - szTextureName ) );	// �e�N�X�`���t�@�C����
		strcpy( szOutMapName, szSeparator + 1 );									// �X�t�B�A�}�b�v�t�@�C����

		int iTexLen = strlen( szOutTexName );
		if ( iTexLen > 0 )	fRet = (type_TexFlags)(fRet | TEXFLAG_TEXTURE);	// �e�N�X�`������

		int iMapLen = strlen( szOutMapName );
		if(		(iMapLen >= 4) &&
				(szTextureName[iMapLen - 4] == '.') &&
				(szTextureName[iMapLen - 3] == 's' || szTextureName[iMapLen - 3] == 'S') &&
				(szTextureName[iMapLen - 2] == 'p' || szTextureName[iMapLen - 2] == 'P') &&
				(szTextureName[iMapLen - 1] == 'a' || szTextureName[iMapLen - 1] == 'A')		)
		{
			// ���Z�X�t�B�A�}�b�v����
			fRet = (type_TexFlags)(fRet | TEXFLAG_ADDMAP);
		}
		else if ( iMapLen > 0 )
		{
			// �ʏ�i��Z�j�X�t�B�A�}�b�v����
			//				�A�X�^���X�N�Ȃ��ƈقȂ�A".spa"�ȊO�Ȃ炷�ׂĒʏ�X�t�B�A�}�b�v�����ɂ��Ă���B�ʓ|�Ȃ̂ŁB
			fRet = (type_TexFlags)(fRet | TEXFLAG_MAP);
		}
	}

	return fRet;
}


//======================
// �w�薼�̃{�[����Ԃ�
//======================
cPMDBone *cPMDModel::getBoneByName( const char *szBoneName )
{
	cPMDBone *pBoneArray = m_pBoneArray;
	for( unsigned long i = 0 ; i < m_unNumBones ; i++, pBoneArray++ )
	{
		if( strncmp( pBoneArray->getName(), szBoneName, 20 ) == 0 )
			return pBoneArray;
	}

	return NULL;
}

//====================
// �w�薼�̕\���Ԃ�
//====================
cPMDFace *cPMDModel::getFaceByName( const char *szFaceName )
{
	cPMDFace *pFaceArray = m_pFaceArray;
	for( unsigned long i = 0 ; i < m_unNumFaces ; i++, pFaceArray++ )
	{
		if( strncmp( pFaceArray->getName(), szFaceName, 20 ) == 0 )
			return pFaceArray;
	}

	return NULL;
}

//====================
// ���[�V�����̃Z�b�g
//====================
void cPMDModel::setMotion( cVMDMotion *pVMDMotion, bool bLoop, float fInterpolateFrame )
{
	m_clMotionPlayer.setup( this, pVMDMotion, bLoop, fInterpolateFrame );
}

//==========================================
// ���̂̈ʒu�����݂̃{�[���̈ʒu�ɂ��킹��
//==========================================
void cPMDModel::resetRigidBodyPos( void )
{
	for( unsigned long i = 0 ; i < m_ulRigidBodyNum ; i++ )
	{
		m_pRigidBodyArray[i].moveToBonePos();
	}
}

//==============================
// ���[�V�����ɂ��{�[���̍X�V
//==============================
bool cPMDModel::updateMotion( float fElapsedFrame )
{
	bool	bMotionEnd = false;

	// ���[�V�����X�V�O�ɕ\������Z�b�g
	if( m_pFaceArray )	m_pFaceArray[0].setFace( m_pvec3OrgPositionArray );

	// ���[�V�����X�V
	bMotionEnd = m_clMotionPlayer.update( fElapsedFrame );

	// �{�[���s��̍X�V
	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
	{
		m_pBoneArray[i].updateMatrix();
	}

	//// �{�[���ʒu���킹
	//for( unsigned long i = 0 ; i < m_ulRigidBodyNum ; i++ )
	//{
	//	m_pRigidBodyArray[i].fixPosition( fElapsedFrame );
	//}

	// IK�̍X�V
	for( unsigned short i = 0 ; i < m_unNumIK ; i++ )
	{
		m_pIKArray[i].update();
	}

	return bMotionEnd;
}

//================================
// ��̃{�[�����^�[�Q�b�g�Ɍ�����
//================================
void cPMDModel::updateNeckBone( const Vector3 *pvec3LookTarget, float fLimitXD, float fLimitXU, float fLimitY )
{
	if( m_pNeckBone && m_bLookAt )
	{
		m_pNeckBone->lookAt( pvec3LookTarget, fLimitXD, fLimitXU, fLimitY );

		unsigned short i;
		for( i = 0 ; i < m_unNumBones ; i++ )
		{
			if( m_pNeckBone == &m_pBoneArray[i] )	break;
		}

		for( ; i < m_unNumBones ; i++ )
		{
			m_pBoneArray[i].updateMatrix();
		}
	}
}

//================================
// ���ׂĂ̍��̂ɑ��x��������i�}�E�X�Œ͂�œ��������Ƃ��j
//================================
void cPMDModel::applyCentralImpulse( float fVelX, float fVelY, float fVelZ )
{
	if ( m_bPhysics ) {
		btVector3 vec3 = btVector3( fVelX, fVelY, fVelZ );
		for( unsigned long i = 0 ; i < m_ulRigidBodyNum; i++ )
		{
			m_pRigidBodyArray[i].getRigidBody()->applyCentralImpulse( vec3 );
		}
	}
}

//====================
// ���_�X�L�j���O����
//====================
void cPMDModel::updateSkinning( void )
{
	// �������Z���f
	if ( m_bPhysics ) {
		for( unsigned long i = 0 ; i < m_ulRigidBodyNum ; i++ )
		{
			m_pRigidBodyArray[i].updateBoneTransform();
		}
	}

	// �X�L�j���O�p�s��̍X�V
	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
	{
		m_pBoneArray[i].updateSkinningMat();
	}

	// ���_�X�L�j���O
	Matrix	matTemp;
	for( unsigned long i = 0 ; i < m_ulNumVertices ; i++ )
	{
		if( m_pOrgSkinInfoArray[i].fWeight == 0.0f )
		{
			Vector3Transform( &m_pvec3PositionArray[i], &m_pvec3OrgPositionArray[i], m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[1]].m_matSkinning );
			Vector3Rotate( &m_pvec3NormalArray[i], &m_pvec3OrgNormalArray[i], m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[1]].m_matSkinning );
		}
		else if( m_pOrgSkinInfoArray[i].fWeight >= 0.9999f )
		{
			Vector3Transform( &m_pvec3PositionArray[i], &m_pvec3OrgPositionArray[i], m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[0]].m_matSkinning );
			Vector3Rotate( &m_pvec3NormalArray[i], &m_pvec3OrgNormalArray[i], m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[0]].m_matSkinning );
		}
		else
		{
			MatrixLerp( matTemp,	m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[0]].m_matSkinning,
									m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[1]].m_matSkinning,		m_pOrgSkinInfoArray[i].fWeight );

			Vector3Transform( &m_pvec3PositionArray[i], &m_pvec3OrgPositionArray[i], matTemp );
			Vector3Rotate( &m_pvec3NormalArray[i], &m_pvec3OrgNormalArray[i], matTemp );
		}
	}
}

//====================
// ���f���f�[�^�̕`��
//====================
void cPMDModel::render( void )
{
	if( !m_pvec3OrgPositionArray )	return;

	// �Ɩ��𖳌��ɂ����ۂɎg���W��
	static const float	fLightDif[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	static const float	fLightAmb[] = { 0.5f, 0.5f, 0.5f, 1.0f };

	unsigned short	*pIndices = m_pIndices;

	glEnableClientState( GL_VERTEX_ARRAY );
	glEnableClientState( GL_NORMAL_ARRAY );
	glEnableClientState( GL_TEXTURE_COORD_ARRAY );

	// ���_���W�A�@���A�e�N�X�`�����W�̊e�z����Z�b�g
	glVertexPointer( 3, GL_FLOAT, 0, m_pvec3PositionArray );
	glNormalPointer( GL_FLOAT, 0, m_pvec3NormalArray );
	glTexCoordPointer( 2, GL_FLOAT, 0, m_puvOrgTexureUVArray );

	glTexGeni( GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP );
	glTexGeni( GL_T, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP );
	glDisable( GL_CULL_FACE );

	for( unsigned long i = 0 ; i < m_ulNumMaterials ; i++ )
	{
		// �֊s�E�e�L���ŐF�w����@��ς���
		if ( m_pMaterials[i].bEdgeFlag )
		{
			// �֊s�E�e�L��̂Ƃ��͏Ɩ���L���ɂ���
			glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE,  (float *)&(m_pMaterials[i].col4Diffuse)  );
			glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT,  (float *)&(m_pMaterials[i].col4Ambient)  );
			glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, (float *)&(m_pMaterials[i].col4Specular) );
			glMaterialf(  GL_FRONT_AND_BACK, GL_SHININESS, m_pMaterials[i].fShininess );

			glEnable( GL_LIGHTING );
		}
		else
		{
			// �������������āA1�𒴂��镪��1�ɍ��킹��
			float color[4];
			color[0] = m_pMaterials[i].col4Ambient.r + fLightAmb[0];
			color[1] = m_pMaterials[i].col4Ambient.g + fLightAmb[1];
			color[2] = m_pMaterials[i].col4Ambient.b + fLightAmb[2];
			color[3] = m_pMaterials[i].col4Ambient.a + fLightAmb[3];
			saturateColor(color);

			// �������𔽉f
			color[0] *= m_pMaterials[i].col4Diffuse.r * fLightDif[0];
			color[1] *= m_pMaterials[i].col4Diffuse.g * fLightDif[1];
			color[2] *= m_pMaterials[i].col4Diffuse.b * fLightDif[2];
			color[3] *= m_pMaterials[i].col4Diffuse.a * fLightDif[3];
			glColor4fv(color);

			// �֊s�E�e�����̂Ƃ��͏Ɩ��𖳌��ɂ���
			glDisable( GL_LIGHTING );
		}

		// �������łȂ���΃|���S�����ʂ𖳌��ɂ���
		if( m_pMaterials[i].col4Diffuse.a < 1.0f )	glDisable( GL_CULL_FACE );
		else										glEnable( GL_CULL_FACE );

		// �e�N�X�`���E�X�t�B�A�}�b�v�̏���
		type_TexFlags fTexFlag = m_pMaterials[i].unTexFlag;

		if( (fTexFlag & TEXFLAG_TEXTURE) && m_pMaterials[i].uiTexID != 0xFFFFFFFF )
		{
			// �e�N�X�`������Ȃ�Bind����
			glBindTexture( GL_TEXTURE_2D, m_pMaterials[i].uiTexID );

			glEnable( GL_TEXTURE_2D );
			glDisable( GL_TEXTURE_GEN_S );
			glDisable( GL_TEXTURE_GEN_T );
		}
		else if( (fTexFlag & (TEXFLAG_MAP | TEXFLAG_ADDMAP)) && m_pMaterials[i].uiMapID != 0xFFFFFFFF )
		{
			// �X�t�B�A�}�b�v����Ȃ�Bind����
			glBindTexture( GL_TEXTURE_2D, m_pMaterials[i].uiMapID );

			glEnable( GL_TEXTURE_2D );
			glEnable( GL_TEXTURE_GEN_S );
			glEnable( GL_TEXTURE_GEN_T );
		}
		else
		{
			// �e�N�X�`�����X�t�B�A�}�b�v���Ȃ�
			glDisable( GL_TEXTURE_2D );
			glDisable( GL_TEXTURE_GEN_S );
			glDisable( GL_TEXTURE_GEN_T );
		}

		// ���_�C���f�b�N�X���w�肵�ă|���S���`��
		glDrawElements( GL_TRIANGLES, m_pMaterials[i].ulNumIndices, GL_UNSIGNED_SHORT, pIndices );

		pIndices += m_pMaterials[i].ulNumIndices;
	}

	glDisableClientState( GL_VERTEX_ARRAY );
	glDisableClientState( GL_NORMAL_ARRAY );
	glDisableClientState( GL_TEXTURE_COORD_ARRAY );

	glDisable( GL_TEXTURE_GEN_S );
	glDisable( GL_TEXTURE_GEN_T );
	glDisable( GL_TEXTURE_2D );

//#if	_DBG_BONE_DRAW
	if (m_iDebug == 1) {
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
	{
		m_pBoneArray[i].debugDraw();
	}

	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	}
//#endif

//#if	_DBG_IK_DRAW
	if (m_iDebug == 2) {
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	for( unsigned short i = 0 ; i < m_unNumIK ; i++ )
	{
		m_pIKArray[i].debugDraw();
	}

	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	}
//#endif

//#if	_DBG_TEXTURE_DRAW
	if (m_iDebug == 3) {
	g_clsTextureList.debugDraw();
	}
//#endif

//#if _DBG_RIGIDBODY_DRAW
	if (m_iDebug == 4) {
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	glLineWidth( 1.0f );

	GLUquadric *quad = gluNewQuadric();
	gluQuadricDrawStyle( quad, GLU_LINE );

	for( unsigned short i = 0 ; i < m_ulRigidBodyNum ; i++ )
	{
		glPushMatrix();
		btRigidBody *pbtRigidBody = m_pRigidBodyArray[i].getRigidBody();
		btCollisionShape *pbtColShape = pbtRigidBody->getCollisionShape();

		btTransform btTrans = pbtRigidBody->getCenterOfMassTransform();
		btScalar transMat[16];
		btTrans.getOpenGLMatrix( transMat );
		glMultMatrixf( transMat );

		glDisable( GL_TEXTURE_2D );

		// ���̂̎�ނɂ���ĐF����
		switch (m_pRigidBodyArray[i].m_iType) {
			case 0:		// �{�[���Ǐ]
				glColor4f( 0.0f, 1.0f, 0.0f, 1.0f );
				break;
			case 1:		// �������Z
				glColor4f( 1.0f, 0.0f, 0.0f, 1.0f );
				break;
			case 2:		// �������Z�i�{�[���ʒu���킹�j
				glColor4f( 1.0f, 1.0f, 0.0f, 1.0f );
				break;
			default:	// �s��
				glColor4f( 1.0f, 1.0f, 1.0f, 1.0f );
				break;
		}

		switch ( pbtColShape->getShapeType()) {
			case SPHERE_SHAPE_PROXYTYPE:
				gluSphere( quad, ((btSphereShape *)pbtColShape)->getRadius(), 6, 6 );
				break;

			case BOX_SHAPE_PROXYTYPE:
				{
				btVector3 boxHalf = ((btBoxShape *)pbtColShape)->getHalfExtentsWithoutMargin();
				glBegin( GL_LINE_LOOP );
					glVertex3f( -boxHalf.x(),  boxHalf.y(), -boxHalf.z() );
					glVertex3f(  boxHalf.x(),  boxHalf.y(), -boxHalf.z() );
					glVertex3f(  boxHalf.x(), -boxHalf.y(), -boxHalf.z() );
					glVertex3f( -boxHalf.x(), -boxHalf.y(), -boxHalf.z() );
				glEnd();
				glBegin( GL_LINE_LOOP );
					glVertex3f(  boxHalf.x(),  boxHalf.y(),  boxHalf.z() );
					glVertex3f( -boxHalf.x(),  boxHalf.y(),  boxHalf.z() );
					glVertex3f( -boxHalf.x(), -boxHalf.y(),  boxHalf.z() );
					glVertex3f(  boxHalf.x(), -boxHalf.y(),  boxHalf.z() );
				glEnd();
				glBegin( GL_LINES );
					glVertex3f(  boxHalf.x(),  boxHalf.y(), -boxHalf.z() );
					glVertex3f(  boxHalf.x(),  boxHalf.y(),  boxHalf.z() );
					glVertex3f(  boxHalf.x(), -boxHalf.y(), -boxHalf.z() );
					glVertex3f(  boxHalf.x(), -boxHalf.y(),  boxHalf.z() );
					glVertex3f( -boxHalf.x(),  boxHalf.y(), -boxHalf.z() );
					glVertex3f( -boxHalf.x(),  boxHalf.y(),  boxHalf.z() );
					glVertex3f( -boxHalf.x(), -boxHalf.y(), -boxHalf.z() );
					glVertex3f( -boxHalf.x(), -boxHalf.y(),  boxHalf.z() );
				glEnd();
				}
				break;

			case CAPSULE_SHAPE_PROXYTYPE:
				//glScaled( 1.0, ((btCapsuleShape *)pbtColShape)->getHalfHeight() * 2.0, 1.0 );
				//gluSphere( quad, ((btCapsuleShape *)pbtColShape)->getRadius(), 6, 6 );
				glRotated( -90.0, 1, 0, 0 );
				glTranslated( 0.0, 0.0, -((btCapsuleShape *)pbtColShape)->getHalfHeight() );
				gluCylinder(
					quad,
					((btCapsuleShape *)pbtColShape)->getRadius(),
					((btCapsuleShape *)pbtColShape)->getRadius(),
					((btCapsuleShape *)pbtColShape)->getHalfHeight() * 2.0,
					6,
					1
					);

				break;
		}

		glPopMatrix();
	}

	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	}
//#endif
}

//========================
// ���f���f�[�^�̉e�p�`��
//========================
void cPMDModel::renderForShadow( void )
{
	if( !m_pvec3OrgPositionArray )	return;

	glEnableClientState( GL_VERTEX_ARRAY );

	glVertexPointer( 3, GL_FLOAT, 0, m_pvec3PositionArray );

	glDrawElements( GL_TRIANGLES, m_ulNumIndices, GL_UNSIGNED_SHORT, m_pIndices );

	glDisableClientState( GL_VERTEX_ARRAY );
}

//====================
// ���f���f�[�^�̉��
//====================
void cPMDModel::release( void )
{
	if( m_pvec3OrgPositionArray )
	{
		free( m_pvec3OrgPositionArray );
		m_pvec3OrgPositionArray = NULL;
	}

	if( m_pvec3OrgNormalArray )
	{
		free( m_pvec3OrgNormalArray );
		m_pvec3OrgNormalArray = NULL;
	}

	if( m_puvOrgTexureUVArray )
	{
		free( m_puvOrgTexureUVArray );
		m_puvOrgTexureUVArray = NULL;
	}

	if( m_pOrgSkinInfoArray )
	{
		free( m_pOrgSkinInfoArray );
		m_pOrgSkinInfoArray = NULL;
	}

	if( m_pvec3PositionArray )
	{
		free( m_pvec3PositionArray );
		m_pvec3PositionArray = NULL;
	}

	if( m_pvec3NormalArray )
	{
		free( m_pvec3NormalArray );
		m_pvec3NormalArray = NULL;
	}

	if( m_pIndices )
	{
		free( m_pIndices );
		m_pIndices = NULL;
	}

	if( m_pMaterials )
	{
		for( unsigned long i = 0 ; i < m_ulNumMaterials ; i++ )
		{
			if( m_pMaterials[i].uiTexID != 0xFFFFFFFF )
				g_clsTextureList.releaseTexture( m_pMaterials[i].uiTexID );

			if( m_pMaterials[i].uiMapID != 0xFFFFFFFF )
				g_clsTextureList.releaseTexture( m_pMaterials[i].uiMapID );
		}

		free( m_pMaterials );
		m_pMaterials = NULL;
	}

	if( m_pBoneArray )
	{
		delete [] m_pBoneArray;
		m_pBoneArray = NULL;
		m_pNeckBone = NULL;
		m_pCenterBone = NULL;
	}

	if( m_pIKArray )
	{
		delete [] m_pIKArray;
		m_pIKArray = NULL;
	}

	if( m_pFaceArray )
	{
		delete [] m_pFaceArray;
		m_pFaceArray = NULL;
	}

	if( m_pConstraintArray )
	{
		delete [] m_pConstraintArray;
		m_pConstraintArray = NULL;
		m_ulConstraintNum = 0;
	}

	if( m_pRigidBodyArray )
	{
		delete [] m_pRigidBodyArray;
		m_pRigidBodyArray = NULL;
		m_ulRigidBodyNum = 0;
	}
}

// ----------------------------------------------------------------------------
//Ru--en	�ǉ��֐���

//==================================
// ���[�v���邩�ǂ�����ݒ�
//==================================
void cPMDModel::setLoop( bool bLoop )
{
	m_clMotionPlayer.setLoop( bLoop );
}

//==================================
// ���[�V�������ŏ��̃t���[���ɖ߂�
//==================================
void cPMDModel::rewind( void )
{
	m_clMotionPlayer.rewind();
}

//==================================
// �������Z���s�����ǂ�����ݒ�
//==================================
void cPMDModel::enablePhysics( bool bEnabled )
{
	m_bPhysics = bEnabled;
}

//==================================
// �p���̃��Z�b�g
//==================================
void cPMDModel::resetBones( void )
{
	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
	{
		m_pBoneArray[i].reset();
	}
}

//==================================
// �Z���^�[�{�[���̍��W�ψʂ�Ԃ�
//==================================
void cPMDModel::getModelPosition( Vector3* pvec3Pos )
{
	if (m_pCenterBone != NULL)
	{
		Vector3 vec3OrgPos;
		m_pCenterBone->getOrgPos( &vec3OrgPos );
		m_pCenterBone->getPos( pvec3Pos );
		pvec3Pos->x -= vec3OrgPos.x;
		pvec3Pos->y -= vec3OrgPos.y;
		pvec3Pos->z -= vec3OrgPos.z;
	}
	else
	{
		// �Z���^�[�s���Ȃ�΃[���ɂ��Ă���
		pvec3Pos->x = pvec3Pos->y = pvec3Pos->z = 0.0f;
	}
}

