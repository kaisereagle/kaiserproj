//***********
// PMD���f��
//***********

#ifndef	_PMDMODEL_H_
#define	_PMDMODEL_H_

#include	"PMDTypes.h"
#include	"PMDBone.h"
#include	"PMDIK.h"
#include	"PMDFace.h"
#include	"PMDRigidBody.h"
#include	"PMDConstraint.h"
#include	"MotionPlayer.h"

class cPMDModel
{
	private :
		char			m_szModelName[21];	// ���f����

		unsigned long	m_ulNumVertices;	// ���_��

		Vector3			*m_pvec3OrgPositionArray;	// ���W�z��
		Vector3			*m_pvec3OrgNormalArray;		// �@���z��
		TexUV			*m_puvOrgTexureUVArray;		// �e�N�X�`�����W�z��

		struct SkinInfo
		{
			float			fWeight;		// �E�F�C�g
			unsigned short	unBoneNo[2];	// �{�[���ԍ�
		};
		SkinInfo		*m_pOrgSkinInfoArray;

		Vector3			*m_pvec3PositionArray;
		Vector3			*m_pvec3NormalArray;

		unsigned long	m_ulNumIndices;		// ���_�C���f�b�N�X��
		unsigned short	*m_pIndices;		// ���_�C���f�b�N�X�z��

		enum type_TexFlags					// �e�N�X�`���ƃX�t�B�A�}�b�v�̎g�p�������t���O
		{
			TEXFLAG_NONE	= 0,			// �e�N�X�`�����X�t�B�A�}�b�v������
			TEXFLAG_TEXTURE	= 1,			// �e�N�X�`������
			TEXFLAG_MAP		= 2,			// �X�t�B�A�}�b�v����
			TEXFLAG_ADDMAP	= 4				// ���Z�^�X�t�B�A�}�b�v����iTEXFLAG_MAP�ƕ��p�s�j
		};

		struct Material
		{
			Color4			col4Diffuse,
							col4Specular,
							col4Ambient;
			float			fShininess;

			unsigned long	ulNumIndices;
			unsigned int	uiTexID;		// �e�N�X�`��ID
			unsigned int	uiMapID;		// �X�t�B�A�}�b�v�e�N�X�`��ID
			bool			bEdgeFlag;		// �֊s�E�e�̗L��
			type_TexFlags	unTexFlag;		// �e�N�X�`���ƃX�t�B�A�}�b�v�̗L��
		};

		unsigned long	m_ulNumMaterials;	// �}�e���A����
		Material		*m_pMaterials;		// �}�e���A���z��

		unsigned short	m_unNumBones;		// �{�[����
		cPMDBone		*m_pBoneArray;		// �{�[���z��

		cPMDBone		*m_pNeckBone;		// ��̃{�[��
		bool			m_bLookAt;			// ����^�[�Q�b�g�֌����邩�ǂ���

		cPMDBone		*m_pCenterBone;		// �Z���^�[�̃{�[��

		unsigned short	m_unNumIK;			// IK��
		cPMDIK			*m_pIKArray;		// IK�z��

		unsigned short	m_unNumFaces;		// �\�
		cPMDFace		*m_pFaceArray;		// �\��z��

		unsigned long	m_ulRigidBodyNum;	// ���̐�
		cPMDRigidBody	*m_pRigidBodyArray;	// ���̔z��

		unsigned long	m_ulConstraintNum;	// �R���X�g���C���g��
		cPMDConstraint	*m_pConstraintArray;// �R���X�g���C���g�z��

		cMotionPlayer	m_clMotionPlayer;

		//bool			isSphereMapTexName( const char *szTextureName );	// �X�t�B�A�}�b�v�Ȃ��true
		type_TexFlags	parseTexName( const char *szTextureName, char *szOutTexName, char *szOutMapName );		// �e�N�X�`�������e�N�X�`���ƃX�t�B�A�}�b�v�ɕ���

	public :
		cPMDModel( void );
		~cPMDModel( void );

		bool load( const wchar_t *szFilePath );
		bool initialize( unsigned char *pData, unsigned long ulDataSize );

		cPMDBone *getBoneByName( const char *szBoneName );
		cPMDFace *getFaceByName( const char *szFaceName );

		void setMotion( cVMDMotion *pVMDMotion, bool bLoop = false, float fInterpolateFrame = 0.0f );
		void resetRigidBodyPos( void );

		bool updateMotion( float fElapsedFrame );
		void updateNeckBone( const Vector3 *pvec3LookTarget, float fLimitXD = -20.0f, float fLimitXU = 45.0f, float fLimitY = 80.0f );
		void updateSkinning( void );

		void render( void );
		void renderForShadow( void );

		void release( void );

		inline void toggleLookAtFlag( void ){ m_bLookAt = !m_bLookAt; } 
		inline void setLookAtFlag( bool bFlag ){ m_bLookAt = bFlag; } 

		inline const char *getModelName( void ){ return m_szModelName; } 

		// Ru--en added
		void rewind( void );												// ���[�V�����̐擪�ɖ߂�
		void setLoop( bool bLoop );											// ���[�V�����J��Ԃ� On/Off
		void enablePhysics( bool bEnabled );								// �������Z On/Off
		void resetBones( void );											// �{�[���ʒu���Đݒ�
		void applyCentralImpulse( float fVelX, float fVelY, float fVelZ );	// �e���̂ɑ��x��������
		bool m_bPhysics;													// �������Z���s����
		int  m_iDebug;														// �f�o�b�O�\���ؑ�
		void getModelPosition( Vector3* pvec3Pos );							// �Z���^�[�{�[���̍��W���擾

	friend class cMotionPlayer;
};

#endif	// _PMDMODEL_H_
