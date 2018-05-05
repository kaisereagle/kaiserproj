//***********
// PMDモデル
//***********

#include	<stdio.h>
#include	<stdlib.h>
#include <OpenGLES/ES1/gl.h>
#include <OpenGLES/ES1/glext.h>
#include <OpenGLES/ES2/gl.h>
#include <OpenGLES/ES2/glext.h>
#include	<string.h>
//#include	<gl/glut.h>
#include	"PMDModel.h"
#include	"TextureList.h"
#include "common.h"
#define	_DBG_BONE_DRAW		(0)
#define	_DBG_IK_DRAW		(0)
#define	_DBG_TEXTURE_DRAW	(0)

extern cTextureList		g_clsTextureList;
extern "C"{
extern void ES20_sjis2utf8(GLchar *str, const int array_length);
}

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
/**
 * Little Endian格納の32bit整数を読み込む
 */

int32_t RawData_read32(unsigned char *rawData) {
    int32_t w0 = (int16_t) (rawData)[0] & 0xFF;
    int32_t w1 = (int16_t) (rawData)[1] & 0xFF;
    int32_t w2 = (int16_t) (rawData)[2] & 0xFF;
    int32_t w3 = (int16_t) (rawData)[3] & 0xFF;
    
    rawData += 4;
    return (w3 << 24) | (w2 << 16) | (w1 << 8) | w0;
}
int32_t RawData_read16(unsigned char* rawData) {
    int32_t w0 = (int16_t) (rawData)[0] & 0xFF;
    int32_t w1 = (int16_t) (rawData)[1] & 0xFF;
    
    rawData += 2;
    return  (w1 << 8) | w0;
}
/**
 * 指定バイト数の情報を読み込む
 */
void RawData_readBytes(unsigned char * rawData, void* result, int bytes) {
    memcpy(result, rawData, bytes);
    rawData += bytes;
}
/**
 * PMDファイルが格納する頂点情報
 */
typedef struct PmdVertex {
    /**
     * 位置
     */
    Vector3 position;
    
    /**
     * テクスチャUV
     */
    Vector3 uv;
    
    /**
     * 法線
     */
    TexUV normal;
    
    /**
     * サンプルでは使用しないが、PMDファイル仕様的に含まれている情報
     */
    struct {
        
        /**
         * 関連付けられたボーン番号
         */
        GLshort bone_num[2];
        
        /**
         * ボーンの重み情報（最大100）
         * bone_num[0]の影響度がbone_weight、bone_num[1]の影響度が100 - bone_weightで表される
         */
        GLbyte bone_weight;
        
        /**
         * エッジ表示フラグ
         * 0=無効
         * 1=有効
         */
        GLbyte edge_flag;
    } extra;
} PmdVertex;
/**
 * PMD材質情報
 */
typedef struct PmdMaterial {
    /**
     * 拡散反射光
     */
    Vector4 diffuse;
    
    /**
     * 頂点数
     */
    GLint indices_num;
    
    /**
     * テクスチャファイル名
     */
    GLchar diffuse_texture_name[20 + 12];
    
    /**
     * サンプルでは使用しないが、PMDファイル仕様的に含まれている情報
     */
    struct {
        
        /**
         * 輪郭フラグ
         */
        GLbyte edge_flag;
        
        /**
         *
         */
        GLfloat shininess;
        
        /**
         * スペキュラ色
         */
        Vector3 specular_color;
        
        /**
         * 環境光
         */
        Vector3 ambient_color;
        /**
         *
         */
        GLbyte toon_index;
        
        /**
         * エフェクト用テクスチャ名
         * .sph/.spaファイルが指定される。サンプル内では利用しない
         */
        GLchar effect_texture_name[20 + 12];
    } extra;
    
} PmdMaterial;

/**
 * ボーン情報
 */
typedef struct PmdBone {
    /**
     * ボーン名
     */
    GLchar name[20 + 12];
    
    /**
     * 親ボーン番号
     * 無い場合は0xFFFF = -1
     */
    GLshort parent_bone_index;
    
    /**
     * ボーンの位置
     */
    Vector3 position;
    
    /**
     * サンプルでは使用しないが、PMDファイル仕様的に含まれている情報
     */
    struct {
        /**
         * 制御ボーン
         */
        GLshort tail_pos_bone_index;
        
        /**
         * ボーンの種類
         */
        GLbyte type;
        
        /**
         * IKボーン
         * サンプルではベイク済みもしくは手動制御を前提とする
         */
        GLshort ik_parent_bone_index;
    } extra;
} PmdBone;

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
	PMD_Header	*pPMDHeader = (PMD_Header *)pData;
	if( pPMDHeader->szMagic[0] != 'P' || pPMDHeader->szMagic[1] != 'm' || pPMDHeader->szMagic[2] != 'd' || pPMDHeader->fVersion != 1.0f )
		return false;	// ファイル形式が違う

	// モデル名のコピー
	strncpy( m_szModelName, pPMDHeader->szName, 20 );
	m_szModelName[20] = '\0';

	pData += sizeof( PMD_Header );

	unsigned long	ulSize;
//        m_ulNumVertices = *((unsigned short *)pData);
//        pData += sizeof( unsigned long );
m_ulNumVertices = RawData_read32(pData);
    pData +=4;
        m_pvec3OrgPositionArray =  (Vector3 *)malloc( sizeof(Vector3)  * m_ulNumVertices );
        m_pvec3OrgNormalArray   =  (Vector3 *)malloc( sizeof(Vector3)  * m_ulNumVertices );
        m_puvOrgTexureUVArray   =    (TexUV *)malloc( sizeof(TexUV)    * m_ulNumVertices );
        m_pOrgSkinInfoArray     = (SkinInfo *)malloc( sizeof(SkinInfo) * m_ulNumVertices );
int vsize=0;
    int i = 0;
    for (i = 0; i < m_ulNumVertices; ++i)
    {
        
        PmdVertex *v = new PmdVertex;
        
        // 頂点情報ロード
        RawData_readBytes(pData, &v->position, sizeof(Vector3)); // 位置
        pData+=sizeof(Vector3);
        RawData_readBytes(pData, &v->normal, sizeof(Vector3)); // 法線
        pData+=sizeof(Vector3);
        RawData_readBytes(pData, &v->uv, sizeof(TexUV)); // UV
        pData+=sizeof(TexUV);
        RawData_readBytes(pData, &v->extra.bone_num, sizeof(GLushort) * 2); // ボーン設定
        pData+=sizeof(GLushort)*2;
        RawData_readBytes(pData, &v->extra.bone_weight, sizeof(GLbyte)); // ボーン重み
        pData+=sizeof(GLbyte);
        RawData_readBytes(pData, &v->extra.edge_flag, sizeof(GLbyte)); // 輪郭フラグ
        pData+=sizeof(GLbyte);
                m_pvec3OrgPositionArray[i] =  v->position;
                m_pvec3OrgNormalArray[i]   =  v->uv;
                m_puvOrgTexureUVArray[i]   =  v->normal;
        
                m_pOrgSkinInfoArray[i].fWeight     = v->extra.bone_weight/ 100.0f;
                m_pOrgSkinInfoArray[i].unBoneNo[0] = v->extra.bone_num[0];
                m_pOrgSkinInfoArray[i].unBoneNo[1] = v->extra.bone_num[1];

vsize +=sizeof(Vector3)+sizeof(Vector3)+sizeof(TexUV)+(sizeof(GLushort) * 2)+sizeof(GLbyte)+sizeof(GLbyte);
    }
    int32_t w0 = (int16_t) (pData)[0] & 0xFF;
    //read_head    uint8_t *    "\x84\x0e\x01"    0x000000012467f169
    //w0    int32_t    132
//
//    //-----------------------------------------------------
//    // 頂点数取得
//    m_ulNumVertices = *((unsigned short *)pData);
//    pData += sizeof( unsigned short );
//
//    // 頂点配列をコピー
//    m_pvec3OrgPositionArray =  (Vector3 *)malloc( sizeof(Vector3)  * m_ulNumVertices );
//    m_pvec3OrgNormalArray   =  (Vector3 *)malloc( sizeof(Vector3)  * m_ulNumVertices );
//    m_puvOrgTexureUVArray   =    (TexUV *)malloc( sizeof(TexUV)    * m_ulNumVertices );
//    m_pOrgSkinInfoArray     = (SkinInfo *)malloc( sizeof(SkinInfo) * m_ulNumVertices );
//
//    PMD_Vertex    *pVertex = (PMD_Vertex *)pData;
//    for( unsigned long i = 0 ; i < m_ulNumVertices ; i++, pVertex++ )
//    {
//        m_pvec3OrgPositionArray[i] =  pVertex->vec3Pos;
//        m_pvec3OrgNormalArray[i]   =  pVertex->vec3Normal;
//        m_puvOrgTexureUVArray[i]   =  pVertex->uvTex;
//
//        m_pOrgSkinInfoArray[i].fWeight     = pVertex->cbWeight / 100.0f;
//        m_pOrgSkinInfoArray[i].unBoneNo[0] = pVertex->unBoneNo[0];
//        m_pOrgSkinInfoArray[i].unBoneNo[1] = pVertex->unBoneNo[1];
//    }
//    int pvsize=sizeof( PMD_Vertex );
//
    ulSize = sizeof( PMD_Vertex ) * m_ulNumVertices;
    //pData += ulSize;
//    pData += 475410;602186

	// スキニング用頂点配列作成
	m_pvec3PositionArray = (Vector3 *)malloc( sizeof(Vector3) * m_ulNumVertices );
	m_pvec3NormalArray   = (Vector3 *)malloc( sizeof(Vector3) * m_ulNumVertices );

	//-----------------------------------------------------
	// 頂点インデックス数取得
   // pData += sizeof( unsigned short );
   // const GLuint numIndices=  RawData_read32(pData);
	//m_ulNumIndices = *((unsigned short *)pData);
	//pData += sizeof( unsigned short );
m_ulNumIndices = RawData_read32(pData);
    //69252
    pData+=4;
	// 頂点インデックス配列をコピー
	ulSize = sizeof( unsigned short ) * m_ulNumIndices;
	m_pIndices = (unsigned short *)malloc( ulSize );
    //

	memcpy( m_pIndices, pData, ulSize );
	pData += ulSize;

	//-----------------------------------------------------
	// マテリアル数取得
//	m_ulNumMaterials = *((unsigned char *)pData);
    m_ulNumMaterials = RawData_read32(pData);
    pData+=4;
//	pData += sizeof( unsigned char );

	// マテリアル配列をコピー
	m_pMaterials = (Material *)malloc( sizeof(Material) * m_ulNumMaterials );

	PMD_Material	*pPMDMaterial = (PMD_Material *)pData;
    int sumVert = 0;
    for (i = 0; i < m_ulNumMaterials; ++i) {
        PmdMaterial *m = new PmdMaterial;
        
        RawData_readBytes(pData, &m->diffuse, sizeof(Vector4));
        pData+=sizeof(Vector4);
        RawData_readBytes(pData, &m->extra.shininess, sizeof(GLfloat));
        pData+=sizeof(GLfloat);
        RawData_readBytes(pData, &m->extra.specular_color, sizeof(Vector3));
        pData+=sizeof(Vector3);
        RawData_readBytes(pData, &m->extra.ambient_color, sizeof(Vector3));
        pData+=sizeof(Vector3);
        RawData_readBytes(pData, &m->extra.toon_index, sizeof(GLubyte));
        pData+=sizeof(GLubyte);
        RawData_readBytes(pData, &m->extra.edge_flag, sizeof(GLubyte));
        pData+=sizeof(GLubyte);
        RawData_readBytes(pData, &m->indices_num, sizeof(GLuint));
        pData+=sizeof(GLuint);
        // テクスチャ名を読み込む
        {
            RawData_readBytes(pData, m->diffuse_texture_name, 20);
            
            // エフェクトテクスチャが含まれていれば文字列を分離する
            // diffuse.png*effect.spaのように"*"で区切られている
            GLchar *p = strchr(m->diffuse_texture_name, '*');
            if (p) {
                // diffuseは"*"を無効化することで切る
                *p = '\0';
                // エフェクトは文字列コピーする
                strcpy(m->extra.effect_texture_name, p + 1);
            } else {
                // エフェクトが無いなら最初の文字でターミネートする
                m->extra.effect_texture_name[0] = '\0';
            }
            
            // SJISで文字列が格納されているため、UTF-8に変換をかける
//            ES20_sjis2utf8(m->diffuse_texture_name, sizeof(m->diffuse_texture_name));
  //          ES20_sjis2utf8(m->extra.effect_texture_name, sizeof(m->extra.effect_texture_name));
            
        }
        m_pMaterials[i].col4Diffuse.a = m->diffuse.w;
        m_pMaterials[i].col4Diffuse.b = m->diffuse.x;
        m_pMaterials[i].col4Diffuse.g = m->diffuse.y;
        m_pMaterials[i].col4Diffuse.r = m->diffuse.z;

                m_pMaterials[i].col4Specular.r = m->extra.specular_color.x;
                m_pMaterials[i].col4Specular.g =m->extra.specular_color.y;
                m_pMaterials[i].col4Specular.b = m->extra.specular_color.z;
m_pMaterials[i].col4Specular.a = 1.0f;
        
               m_pMaterials[i].col4Ambient.r = m->extra.ambient_color.x;
                m_pMaterials[i].col4Ambient.g = m->extra.ambient_color.y;
                m_pMaterials[i].col4Ambient.b = m->extra.ambient_color.z;
                m_pMaterials[i].col4Ambient.a = 1.0f;
        //
                m_pMaterials[i].fShininess = m->extra.shininess;
        
                m_pMaterials[i].ulNumIndices = m->indices_num;

        //__logf("material[%d] tex(%s) effect(%s) vert(%d) face(%d) toon(%d)", i, m->diffuse_texture_name, m->extra.effect_texture_name, m->indices_num, m->indices_num / 3, (int )m->extra.toon_index);
        pData+=20;
        sumVert += m->indices_num;
    }

//    for( unsigned long i = 0 ; i < m_ulNumMaterials ; i++ )
//    {
//        m_pMaterials[i].col4Diffuse = pPMDMaterial->col4Diffuse;
//
//        m_pMaterials[i].col4Specular.r = pPMDMaterial->col3Specular.r;
//        m_pMaterials[i].col4Specular.g = pPMDMaterial->col3Specular.g;
//        m_pMaterials[i].col4Specular.b = pPMDMaterial->col3Specular.b;
//        m_pMaterials[i].col4Specular.a = 1.0f;
//
//        m_pMaterials[i].col4Ambient.r = pPMDMaterial->col3Ambient.r;
//        m_pMaterials[i].col4Ambient.g = pPMDMaterial->col3Ambient.g;
//        m_pMaterials[i].col4Ambient.b = pPMDMaterial->col3Ambient.b;
//        m_pMaterials[i].col4Ambient.a = 1.0f;
//
//        m_pMaterials[i].fShininess = pPMDMaterial->fShininess;
//        m_pMaterials[i].ulNumIndices = pPMDMaterial->ulNumIndices;
//
//        if( pPMDMaterial->szTextureFileName[0] )
//            m_pMaterials[i].uiTexID = g_clsTextureList.getTexture( pPMDMaterial->szTextureFileName );
//        else
//            m_pMaterials[i].uiTexID = 0xFFFFFFFF;
//
//        pPMDMaterial++;
//    }
//
//    pData += sizeof(PMD_Material) * m_ulNumMaterials;

	//-----------------------------------------------------
	// ボーン数取得
	m_unNumBones = *((unsigned char *)pData);
	pData += 2;

	// ボーン配列を作成
	m_pBoneArray = new cPMDBone[m_unNumBones];
    //int i;
    int bsize=0;
    for (i = 0; i < m_unNumBones; ++i) {
        PmdBone *bone = new PmdBone;
        
        RawData_readBytes(pData, bone->name, 20);
        pData+=20;
        // SJISで文字列が格納されているため、UTF-8に変換をかける
        ES20_sjis2utf8(bone->name, sizeof(bone->name));
        
        RawData_readBytes(pData, &bone->parent_bone_index, sizeof(GLshort));
        pData+=sizeof(GLshort);
        RawData_readBytes(pData, &bone->extra.tail_pos_bone_index, sizeof(GLshort));
        pData+=sizeof(GLshort);
        RawData_readBytes(pData, &bone->extra.type, sizeof(GLbyte));
        pData+=sizeof(GLbyte);
        RawData_readBytes(pData, &bone->extra.ik_parent_bone_index, sizeof(GLshort));
        pData+=sizeof(GLshort);
        RawData_readBytes(pData, &bone->position, sizeof(Vector3));
        pData+=sizeof(Vector3);
        bsize+=sizeof(GLshort)+sizeof(GLshort)+sizeof(GLbyte)+sizeof(GLshort)+sizeof(Vector3)+20;
        __logf("bone[%d] name(%s) size [%d]", i, bone->name,bsize);
        m_pBoneArray[i].initialize( (const PMD_Bone *)pData, m_pBoneArray );
    }
    
//    for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
//    {
//        m_pBoneArray[i].initialize( (const PMD_Bone *)pData, m_pBoneArray );
//
//        // 首のボーンを保存
//        if( strncmp( m_pBoneArray[i].getName(), "頭", 20 ) == 0 )    m_pNeckBone = &m_pBoneArray[i];
//
//        pData += sizeof( PMD_Bone );
//    }
//    for( unsigned short i = 0 ; i < m_unNumBones ; i++ )    m_pBoneArray[i].recalcOffset();

    
	//-----------------------------------------------------
	// IK数取得

    
    m_unNumIK = RawData_read16(pData);
    pData +=2;
//    m_unNumIK = *((unsigned short *)pData);
//    pData += sizeof( unsigned short );
    __logf("ik[%d]",m_unNumIK);
	// IK配列を作成
    int ikbyte=0;
    int bornnum=0;
//    if( m_unNumIK )
//    {
//        m_pIKArray = new cPMDIK[m_unNumIK];
//
//        for( unsigned short i = 0 ; i < m_unNumIK ; i++ )
//        {
//            PMD_IK *ik=new PMD_IK;
//
//            //ES20_sjis2utf8(ik->nTargetNo, sizeof(bone->name));
//
//            RawData_readBytes(pData, &ik->nTargetNo, sizeof(GLshort));
//            pData+=sizeof(GLshort);
//            RawData_readBytes(pData, &ik->nEffNo, sizeof(GLshort));
//            pData+=sizeof(GLshort);
//            RawData_readBytes(pData, &ik->cbNumLink, sizeof(GLbyte));
//            pData+=sizeof(GLbyte);
//            RawData_readBytes(pData, &ik->unCount, sizeof(GLshort));
//            pData+=sizeof(GLshort);
//            RawData_readBytes(pData, &ik->fFact, sizeof(GLfloat));
//            pData+=sizeof(GLfloat);
//            ikbyte +=sizeof(GLshort)+sizeof(GLshort)+sizeof(GLbyte)+sizeof(GLshort)+sizeof(GLfloat);
//            __logf("target[%d] sentann[%d] bone[%d] count[%d] ", ik->nTargetNo,ik->nEffNo,ik->cbNumLink,ik->unCount);
//            bornnum=(int)ik->cbNumLink;
//            for(i=0;i < bornnum;i++)
//            {
//                RawData_readBytes(pData, &ik->punLinkNo[0], sizeof(GLshort));
//                pData+=sizeof(GLshort);
//            }
//            ikbyte +=sizeof(GLshort)*(bornnum-1);
//
//            __logf("ik byte[%d] ",ikbyte);
//
//
//
//
//            m_pIKArray[i].initialize( (const PMD_IK *)&ik, m_pBoneArray );
//
//
    
			//pData += sizeof( PMD_IK ) + sizeof(unsigned short) * (((PMD_IK *)pData)->cbNumLink - 1);
//            if(((PMD_IK *)pData)->cbNumLink == 0)
//            {
//                pData +=2+2+1+2+4;
//            }else{
//                pData +=2+2+1+2+4+2;
//
//            }
//        }
//        qsort( m_pIKArray, m_unNumIK, sizeof(cPMDIK), compareFunc );
	//}

	//-----------------------------------------------------
	// 表情数取得
//    m_unNumFaces = *((unsigned short *)pData);
//    pData += sizeof( unsigned short );
//
//    // 表情配列を作成
//    if( m_unNumFaces )
//    {
//        m_pFaceArray = new cPMDFace[m_unNumFaces];
//
//        for( unsigned short i = 0 ; i < m_unNumFaces ; i++ )
//        {
//            m_pFaceArray[i].initialize( (const PMD_Face *)pData, &m_pFaceArray[0] );
//            pData += sizeof( PMD_Face ) + sizeof(PMD_FaceVtx) * (((PMD_Face *)pData)->ulNumVertices - 1);
//        }
//    }

	return true;
}

//======================
// 指定名のボーンを返す
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

//================================
// 首のボーンをターゲットに向ける
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

	// スキニング用行列の更新
	for( unsigned short i = 0 ; i < m_unNumBones ; i++ )
	{
		m_pBoneArray[i].updateSkinningMat();
	}

	// 頂点スキニング
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
