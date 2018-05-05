//***********
// PMDモデル
//***********

#ifndef	_PMDMODEL_H_
#define	_PMDMODEL_H_

#include	"PMDTypes.h"
#include	"PMDBone.h"
#include	"PMDIK.h"
#include	"PMDFace.h"
#include	"MotionPlayer.h"
#include    "PMDRigidBody.h"
#include    "PMDConstraint.h"
class cPMDModel
{
	private :
		char			m_szModelName[21];	// モデル名

		unsigned int	m_ulNumVertices;	// 頂点数

		vec3			*m_pvec3OrgPositionArray;	// 座標配列
		vec3			*m_pvec3OrgNormalArray;		// 法線配列
		tex			*m_puvOrgTexureUVArray;		// テクスチャ座標配列

		struct SkinInfo
		{
			float			fWeight;		// ウェイト
			unsigned short	unBoneNo[2];	// ボーン番号
		};
		SkinInfo		*m_pOrgSkinInfoArray;

		vec3			*m_pvec3PositionArray;
		vec3			*m_pvec3NormalArray;

		unsigned int	m_ulNumIndices;		// 頂点インデックス数
		unsigned short	*m_pIndices;		// 頂点インデックス配列

		struct Material
		{
			color4			col4Diffuse,
							col4Specular,
							col4Ambient;
			float			fShininess;

			unsigned int	ulNumIndices;
			unsigned int	uiTexID;		// テクスチャID
		};
		unsigned int	m_ulNumMaterials;	// マテリアル数
		Material		*m_pMaterials;		// マテリアル配列

		unsigned short	m_unNumBones;		// ボーン数
		cPMDBone		*m_pBoneArray;		// ボーン配列

		cPMDBone		*m_pNeckBone;		// 首のボーン
		bool			m_bLookAt;			// 首をターゲットへ向けるかどうか

		unsigned short	m_unNumIK;			// IK数
		cPMDIK			*m_pIKArray;		// IK配列

		unsigned short	m_unNumFaces;		// 表情数
		cPMDFace		*m_pFaceArray;		// 表情配列

    unsigned long    m_ulRigidBodyNum;    // 剛体数
    cPMDRigidBody    *m_pRigidBodyArray;    // 剛体配列
    
    unsigned long    m_ulConstraintNum;    // コンストレイント数
    cPMDConstraint    *m_pConstraintArray;// コンストレイント配列
    
		cMotionPlayer	m_clMotionPlayer;

	public :
		cPMDModel( void );
		~cPMDModel( void );

		bool load( const char *szFilePath );
		bool initialize( unsigned char *pData, unsigned long ulDataSize );
    bool initbullet( unsigned char *pData, unsigned long ulDataSize );
    cPMDBone *getBoneByName( const char *szBoneName );
    cPMDBone *getBoneByName_( const char *szBoneName );
		cPMDFace *getFaceByName( const char *szFaceName );

		void setMotion( cVMDMotion *pVMDMotion, bool bLoop = false, float fInterpolateFrame = 0.0f );
        void resetRigidBodyPos( void );
		bool updateMotion( float fElapsedFrame );
		void updateNeckBone( const vec3 *pvec3LookTarget, float fLimitXD = -20.0f, float fLimitXU = 45.0f, float fLimitY = 80.0f );
		void updateSkinning( void );
		void render( void );

    void updateSkinning_( void );
    void render_( void );

    void renderForShadow( void );

		void release( void );

		inline void toggleLookAtFlag( void ){ m_bLookAt = !m_bLookAt; } 
		inline void setLookAtFlag( bool bFlag ){ m_bLookAt = bFlag; } 

		inline const char *getModelName( void ){ return m_szModelName; } 

	friend class cMotionPlayer;
};

#endif	// _PMDMODEL_H_
