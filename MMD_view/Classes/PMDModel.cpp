//***********
// PMDモデル
//***********

#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
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
#include	"PMDModel.h"
#include	"TextureList.h"
#include "MMDFiles_utils.h"
#define	_DBG_BONE_DRAW		(0)
#define	_DBG_IK_DRAW		(0)
#define	_DBG_TEXTURE_DRAW	(0)

extern cTextureList		g_clsTextureList;

extern void __log(char *in,int num);

//--------------------------
// IKデータソート用比較関数
//--------------------------
static int compareFunc( const void *pA, const void *pB )
{
	return (int)(((cPMDIK *)pA)->getSortVal() - ((cPMDIK *)pB)->getSortVal());
}

//================
// コンストラクタ
//================
cPMDModel::cPMDModel( void ) : m_pvec3OrgPositionArray( NULL ), m_pvec3OrgNormalArray( NULL ), m_puvOrgTexureUVArray( NULL ),
									m_pOrgSkinInfoArray( NULL ), m_pvec3PositionArray( NULL ), m_pvec3NormalArray( NULL ), m_pIndices( NULL ),
										m_pMaterials( NULL ), m_pBoneArray( NULL ), m_pNeckBone( NULL), m_pIKArray( NULL ), m_pFaceArray( NULL )
{
}

//==============
// デストラクタ
//==============
cPMDModel::~cPMDModel( void )
{
	release();
}

//====================
// ファイルの読み込み
//====================
bool cPMDModel::load( const char *szFilePath )
{
	FILE	*pFile;
	fpos_t	fposFileSize;
	unsigned char
			*pData;

	pFile = fopen( szFilePath, "rb" );
	if( !pFile )	return false;	// ファイルが開けない

	// ファイルサイズ取得
	fseek( pFile, 0, SEEK_END );
	fgetpos( pFile, &fposFileSize );

	// メモリ確保
	pData = (unsigned char *)malloc( (size_t)fposFileSize );

	// 読み込み
	fseek( pFile, 0, SEEK_SET );
	fread( pData, 1, (size_t)fposFileSize, pFile );

	fclose( pFile );

	// モデルデータ初期化
	bool	bRet = initialize( pData, fposFileSize );

	free( pData );

	return bRet;
}
//typedef unsigned long UINT16;
//======================
// モデルデータの初期化
//======================
bool cPMDModel::initialize( unsigned char *pData, unsigned long ulDataSize )
{
	release();
	m_clMotionPlayer.clear();

	m_bLookAt = false;

	unsigned char *pDataTop = pData;

	// ヘッダのチェック
	mmd_head	*pPMDHeader = (mmd_head *)pData;
	if( pPMDHeader->szMagic[0] != 'P' || pPMDHeader->szMagic[1] != 'm' || pPMDHeader->szMagic[2] != 'd' || pPMDHeader->fVersion != 1.0f )
		return false;	// ファイル形式が違う

	// モデル名のコピー
	strncpy( m_szModelName, pPMDHeader->szName, 20 );
	m_szModelName[20] = '\0';

	pData += sizeof( mmd_head );

	unsigned long	ulSize;

	//-----------------------------------------------------
	// 頂点数取得
    m_ulNumVertices = *((unsigned int *)pData);
    pData += sizeof( unsigned int );
//    m_ulNumVertices = *((unsigned long *)pData);
//    pData += sizeof( unsigned long );
//NSLog(@"... PMDModel::parsePMD m_ulNumVertices = [%d]", m_ulNumVertices);
	// 頂点配列をコピー
	m_pvec3OrgPositionArray =  (vec3 *)malloc( sizeof(vec3)  * m_ulNumVertices );
	m_pvec3OrgNormalArray   =  (vec3 *)malloc( sizeof(vec3)  * m_ulNumVertices );
	m_puvOrgTexureUVArray   =    (tex *)malloc( sizeof(tex)    * m_ulNumVertices );
	m_pOrgSkinInfoArray     = (SkinInfo *)malloc( sizeof(SkinInfo) * m_ulNumVertices ); 

    unsigned char *point =pData;
	PMD_Vertex	*pVertex = (PMD_Vertex *)pData;
	for( unsigned int i = 0 ; i < m_ulNumVertices ; i++, pVertex++ )
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
    point+= ulSize;;
	// スキニング用頂点配列作成
	m_pvec3PositionArray = (vec3 *)malloc( sizeof(vec3) * m_ulNumVertices );
	m_pvec3NormalArray   = (vec3 *)malloc( sizeof(vec3) * m_ulNumVertices );

	//-----------------------------------------------------
	// 頂点インデックス数取得
    m_ulNumIndices = *((unsigned int *)pData);
    pData += sizeof( unsigned int );
    point+= sizeof( unsigned int );
    //NSLog(@"... PMDModel::parsePMDm_ulNumIndices = [%d]", m_ulNumIndices);
//    m_ulNumIndices = *((unsigned long *)pData);
//    pData += sizeof( unsigned long );

	// 頂点インデックス配列をコピー
	ulSize = sizeof( unsigned short ) * m_ulNumIndices;
	m_pIndices = (unsigned short *)malloc( ulSize );

	memcpy( m_pIndices, pData, ulSize );
	pData += ulSize;
point+= ulSize;
	//-----------------------------------------------------
	// マテリアル数取得
	m_ulNumMaterials = *((unsigned int *)pData);
	pData += sizeof( unsigned int);
    point+= sizeof( unsigned int);
   // NSLog(@"... PMDModel::m_ulNumMaterials = [%d]", m_ulNumMaterials);
	// マテリアル配列をコピー
	m_pMaterials = (Material *)malloc( sizeof(Material) * m_ulNumMaterials );

	PMD_Material	*pPMDMaterial = (PMD_Material *)pData;
	for( unsigned short i = 0 ; i < m_ulNumMaterials ; i++ )
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

		if( pPMDMaterial->szTextureFileName[0] )
			m_pMaterials[i].uiTexID = g_clsTextureList.getTexture( pPMDMaterial->szTextureFileName );
		else
			m_pMaterials[i].uiTexID = 0xFFFFFFFF;

		pPMDMaterial++;
	}
    
    
    int msize=sizeof(PMD_Material) * m_ulNumMaterials;
    
    pData += sizeof(PMD_Material) * m_ulNumMaterials;
    //pData += 1120;
point+= 1120;
	//-----------------------------------------------------
	// ボーン数取得
        m_unNumBones = *((unsigned short *)pData);
    int bo=*((unsigned short *)point);
//    m_unNumBones = 174;

	pData += sizeof( unsigned short );
 //   NSLog(@"... PMDModel::parsePMD m_numBone = [%d] %d", m_unNumBones,bo);
    char sjisBuff[21];
    char *m_name;
	// ボーン配列を作成
	m_pBoneArray = new cPMDBone[m_unNumBones];
	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
	{
		m_pBoneArray[i].initialize( (const PMD_Bone *)pData, m_pBoneArray );
        strncpy(sjisBuff, "頭", 20);
        sjisBuff[20] = '\0';
        m_name =MMDFiles_strdup_from_utf8_to_sjis(sjisBuff);
		// 首のボーンを保存
		if( strncmp( m_pBoneArray[i].getName(), m_name, 20 ) == 0 )	m_pNeckBone = &m_pBoneArray[i];

		pData += sizeof( PMD_Bone );
       // NSLog(@"bone = i[%d]", i);
	}
	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )	m_pBoneArray[i].recalcOffset();

	//-----------------------------------------------------
	// IK数取得
    m_unNumIK = *((unsigned short *)pData);
//    m_unNumIK = 7;

	pData += sizeof( unsigned short );

	// IK配列を作成
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
	// 表情数取得
    m_unNumFaces = *((unsigned short *)pData);
    pData += sizeof( unsigned short );
__log("num:%d",m_unNumFaces);
      __log("size:%d",sizeof( unsigned short ));
    // 表情配列を作成
    if( m_unNumFaces )
    {
        m_pFaceArray = new cPMDFace[m_unNumFaces];

        for( unsigned short i = 0 ; i < m_unNumFaces ; i++ )
        {
            m_pFaceArray[i].initialize( (const PMD_Face *)pData, &m_pFaceArray[0] );
            pData += sizeof( PMD_Face ) + sizeof(PMD_FaceVtx) * (((PMD_Face *)pData)->ulNumVertices - 1);
            __log("i:%d",i);
            __log("face:%d",((PMD_Face *)pData)->ulNumVertices);
        }
    }
    return initbullet(pData, ulDataSize);
	//return true;
}
void cPMDModel::resetRigidBodyPos( void )
{
    for( unsigned long i = 0 ; i < m_ulRigidBodyNum ; i++ )
    {
        m_pRigidBodyArray[i].moveToBonePos();
    }
}
//======================
// 指定名のボーンを返す
//======================
cPMDBone *cPMDModel::getBoneByName_( const char *szBoneName )
{
	cPMDBone *pBoneArray = m_pBoneArray;
    char sjisBuff[21];
    char *m_name;
    strncpy(sjisBuff, szBoneName, 20);
    sjisBuff[20] = '\0';
    m_name =MMDFiles_strdup_from_sjis_to_utf8(pBoneArray->getName());
  //  __log((char*)szBoneName,0);

	for( unsigned long i = 0 ; i < m_unNumBones ; i++, pBoneArray++ )
	{
       // __log("bone name i:%d",i);

        //__log((char *)pBoneArray->getName(),i);
        
  //      if( strncmp( m_name, szBoneName, 20 ) == 0 )
            if( strncmp( pBoneArray->getName(), szBoneName, 20 ) == 0 )
			return pBoneArray;
	}

	return NULL;
}
cPMDBone *cPMDModel::getBoneByName( const char *szBoneName )
{
    cPMDBone *pBoneArray = m_pBoneArray;
    char sjisBuff[21];
    char *m_name;
    strncpy(sjisBuff, szBoneName, 20);
    sjisBuff[20] = '\0';
    m_name =MMDFiles_strdup_from_sjis_to_utf8(pBoneArray->getName());
    //  __log((char*)szBoneName,0);
    
    for( unsigned long i = 0 ; i < m_unNumBones ; i++, pBoneArray++ )
    {
        // __log("bone name i:%d",i);
        
        //__log((char *)pBoneArray->getName(),i);
        
              if( strncmp( m_name, szBoneName, 20 ) == 0 )
        //if( strncmp( pBoneArray->getName(), szBoneName, 20 ) == 0 )
            return pBoneArray;
    }
    
    return NULL;
}

//====================
// 指定名の表情を返す
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
// モーションのセット
//====================
void cPMDModel::setMotion( cVMDMotion *pVMDMotion, bool bLoop, float fInterpolateFrame )
{
	m_clMotionPlayer.setup( this, pVMDMotion, bLoop, fInterpolateFrame );
}

//==============================
// モーションによるボーンの更新
//==============================
bool cPMDModel::updateMotion( float fElapsedFrame )
{
	bool	bMotionEnd = false;

	// モーション更新前に表情をリセット
	if( m_pFaceArray )	m_pFaceArray[0].setFace( m_pvec3OrgPositionArray );

	// モーション更新
	bMotionEnd = m_clMotionPlayer.update( fElapsedFrame );

	// ボーン行列の更新
	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
	{
		m_pBoneArray[i].updateMatrix();
	}

	return bMotionEnd;
}
bool cPMDModel::initbullet( unsigned char *pData, unsigned long ulDataSize )
{
    unsigned char *pDataTop = pData;
    //-----------------------------------------------------
    // ここから剛体情報まで読み飛ばし
    
        // 表情枠用表示リスト
        unsigned char    cbFaceDispNum = *((unsigned char *)pData);
        pData += sizeof( unsigned char );
    
        pData += sizeof( unsigned short ) * cbFaceDispNum;
    
        // ボーン枠用枠名リスト
        unsigned char    cbBoneDispNum = *((unsigned char *)pData);
        pData += sizeof( unsigned char );
    
        pData += sizeof( char ) * 50 * cbBoneDispNum;
    
        // ボーン枠用表示リスト
        unsigned int    ulBoneFrameDispNum = *((unsigned int *)pData);
        pData += sizeof( unsigned int );
    
        pData += 3 * ulBoneFrameDispNum;
    
        // 後続データの有無をチェック
        if( (unsigned long)pData - (unsigned long)pDataTop >= ulDataSize )    return true;
    
        // 英語名対応
        unsigned char    cbEnglishNameExist = *((unsigned char *)pData);
        pData += sizeof( unsigned char );
    
        if( cbEnglishNameExist )
        {
            // モデル名とコメント(英語)
            pData += sizeof( char ) * 276;
    
            // ボーン名(英語)
            pData += sizeof( char ) * 20 * m_unNumBones;
    
            // 表情名(英語)
            pData += sizeof( char ) * 20 * (m_unNumFaces - 1);    // "base"は英語名リストには含まれない
    
            // ボーン枠用枠名リスト(英語)
            pData += sizeof( char ) * 50 * (cbBoneDispNum);
        }
    
        // 後続データの有無をチェック(ここのチェックは不要かも)
        if( (unsigned long)pData - (unsigned long)pDataTop >= ulDataSize )    return true;
    
        // トゥーンテクスチャリスト
        pData += 100 * 10;        // ファイル名 100Byte * 10個
    
        // ここまで読み飛ばし
        //-----------------------------------------------------
    
        // 後続データの有無をチェック
        if( (unsigned long)pData - (unsigned long)pDataTop >= ulDataSize )    return true;
    
        //-----------------------------------------------------
        // 剛体数取得
        m_ulRigidBodyNum = *((unsigned int *)pData);
        pData += sizeof( unsigned int );
    
        // 剛体配列を作成
        if( m_ulRigidBodyNum )
        {
            m_pRigidBodyArray = new cPMDRigidBody[m_ulRigidBodyNum];
    
            for( unsigned long i = 0 ; i < m_ulRigidBodyNum ; i++ )
            {
                const PMD_RigidBody *pPMDRigidBody = (const PMD_RigidBody *)pData;
    
                cPMDBone    *pBone = NULL;
                if( pPMDRigidBody->unBoneIndex == 0xFFFF )    pBone = getBoneByName( "センター" );
                else                                        pBone = &m_pBoneArray[pPMDRigidBody->unBoneIndex];
                __log("i:%d",i);
                __log("bone index:%d",pPMDRigidBody->unBoneIndex);

                m_pRigidBodyArray[i].initialize( pPMDRigidBody, pBone );
                pData += sizeof( PMD_RigidBody );
            }
        }
    
        //-----------------------------------------------------
        // コンストレイント数取得
        m_ulConstraintNum = *((unsigned int *)pData);
        pData += sizeof( unsigned int );
    
        // コンストレイント配列を作成
        if( m_ulConstraintNum )
        {
            m_pConstraintArray = new cPMDConstraint[m_ulConstraintNum];
    
            for( unsigned long i = 0 ; i < m_ulConstraintNum ; i++ )
            {
                const PMD_Constraint *pPMDConstraint = (const PMD_Constraint *)pData;
    
                cPMDRigidBody    *pRigidBodyA = &m_pRigidBodyArray[pPMDConstraint->ulRigidA],
                *pRigidBodyB = &m_pRigidBodyArray[pPMDConstraint->ulRigidB];
    
                m_pConstraintArray[i].initialize( pPMDConstraint, pRigidBodyA, pRigidBodyB );
                pData += sizeof( PMD_Constraint );
            }
        }
    
    return true;
}
//================================
// 首のボーンをターゲットに向ける
//================================
void cPMDModel::updateNeckBone( const vec3 *pvec3LookTarget, float fLimitXD, float fLimitXU, float fLimitY )
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

//====================
// 頂点スキニング処理
//====================
void cPMDModel::updateSkinning( void )
{
	// IKの更新
    for( unsigned short i = 0 ; i < m_unNumIK ; i++ )
    {
        m_pIKArray[i].update();
    }
    // 物理演算反映
    for( unsigned long i = 0 ; i < m_ulRigidBodyNum ; i++ )
    {
        m_pRigidBodyArray[i].updateBoneTransform();
    }
	// スキニング用行列の更新
	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
	{
		m_pBoneArray[i].updateSkinningMat();
	}

	// 頂点スキニング
	matrix	matTemp;
	for( unsigned long i = 0 ; i < m_ulNumVertices ; i++ )
	{
		if( m_pOrgSkinInfoArray[i].fWeight == 0.0f )
		{
			VecMatQuat::vec_transform( &m_pvec3PositionArray[i], &m_pvec3OrgPositionArray[i], m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[1]].m_matSkinning );
			VecMatQuat::vec_rotate( &m_pvec3NormalArray[i], &m_pvec3OrgNormalArray[i], m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[1]].m_matSkinning );
		}
		else if( m_pOrgSkinInfoArray[i].fWeight >= 0.9999f )
		{
			VecMatQuat::vec_transform( &m_pvec3PositionArray[i], &m_pvec3OrgPositionArray[i], m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[0]].m_matSkinning );
			VecMatQuat::vec_rotate( &m_pvec3NormalArray[i], &m_pvec3OrgNormalArray[i], m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[0]].m_matSkinning );
		}
		else
		{
			VecMatQuat::mat_lerp( matTemp,	m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[0]].m_matSkinning,
									m_pBoneArray[m_pOrgSkinInfoArray[i].unBoneNo[1]].m_matSkinning,		m_pOrgSkinInfoArray[i].fWeight );

			VecMatQuat::vec_transform( &m_pvec3PositionArray[i], &m_pvec3OrgPositionArray[i], matTemp );
			VecMatQuat::vec_rotate( &m_pvec3NormalArray[i], &m_pvec3OrgNormalArray[i], matTemp );
		}
	}
}

//====================
// モデルデータの描画
//====================
void cPMDModel::render( void )
{
	if( !m_pvec3OrgPositionArray )	return;

	unsigned short	*pIndices = m_pIndices;

	glEnableClientState( GL_VERTEX_ARRAY );
	glEnableClientState( GL_NORMAL_ARRAY );
	glEnableClientState( GL_TEXTURE_COORD_ARRAY );

	// 頂点座標、法線、テクスチャ座標の各配列をセット
	glVertexPointer( 3, GL_FLOAT, 0, m_pvec3PositionArray );
	glNormalPointer( GL_FLOAT, 0, m_pvec3NormalArray );
	glTexCoordPointer( 2, GL_FLOAT, 0, m_puvOrgTexureUVArray );

	for( unsigned long i = 0 ; i < m_ulNumMaterials ; i++ )
	{
		// マテリアル設定
		glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE,  (float *)&(m_pMaterials[i].col4Diffuse)  );
		glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT,  (float *)&(m_pMaterials[i].col4Ambient)  );
		glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, (float *)&(m_pMaterials[i].col4Specular) );
		glMaterialf(  GL_FRONT_AND_BACK, GL_SHININESS, m_pMaterials[i].fShininess );

		if( m_pMaterials[i].col4Diffuse.a < 1.0f )	glDisable( GL_CULL_FACE );
		else										glEnable( GL_CULL_FACE );

		if( m_pMaterials[i].uiTexID != 0xFFFFFFFF )
		{
			// テクスチャありならBindする
			glEnable( GL_TEXTURE_2D );
			glBindTexture( GL_TEXTURE_2D, m_pMaterials[i].uiTexID );
		}
		else
		{
			// テクスチャなし
			glDisable( GL_TEXTURE_2D );
		}	

		// 頂点インデックスを指定してポリゴン描画
		glDrawElements( GL_TRIANGLES, m_pMaterials[i].ulNumIndices, GL_UNSIGNED_SHORT, pIndices );

		pIndices += m_pMaterials[i].ulNumIndices;
	}

	glDisableClientState( GL_VERTEX_ARRAY );
	glDisableClientState( GL_NORMAL_ARRAY );
	glDisableClientState( GL_TEXTURE_COORD_ARRAY );

#if	_DBG_BONE_DRAW
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
	{
		m_pBoneArray[i].debugDraw();
	}

	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
#endif

#if	_DBG_IK_DRAW
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	for( unsigned short i = 0 ; i < m_unNumIK ; i++ )
	{
		m_pIKArray[i].debugDraw();
	}

	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
#endif

#if	_DBG_TEXTURE_DRAW
	g_clsTextureList.debugDraw();
#endif
}

//========================
// モデルデータの影用描画
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
// モデルデータの解放
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
		}

		free( m_pMaterials );
		m_pMaterials = NULL;
	}

	if( m_pBoneArray )
	{
		delete [] m_pBoneArray;
		m_pBoneArray = NULL;
		m_pNeckBone = NULL;
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
}
