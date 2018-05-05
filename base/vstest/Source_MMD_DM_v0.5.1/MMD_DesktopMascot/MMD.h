// ---------------------------------------------------------------------------
//
// ARTK_MMD の MMD データアクセス部分とのつなぎ
//
// このファイル中の関数の多くは PY さん作の ARTK_MMD を元にしています。
//
// 2010/01/20	Ver. 0.0.9	ウィンドウ追従で奥行きは残すよう修正
// 2010/01/14	Ver. 0.0.8	ウィンドウのモデル追従への対応
// 2009/10/18	Ver. 0.0.7	パスからのディレクトリ抽出を変更
// 2009/08/30	Ver. 0.0.6	VPD対応や物理演算フラグに関する修正
// 2009/08/07	Ver. 0.0.5	FPS表示など
// 2009/07/16	Ver. 0.0.4	モデル読込時にマウス目線モード設定引継
// 2009/07/09	Ver. 0.0.3	ファイル読込時のカレントディレクトリ変更を修正
// 2009/07/08	Ver. 0.0.2	マウス目線モード実装
// 2009/07/01	Ver. 0.0.1	整理
// 2009/06/30	Ver. 0.0.0	何とか動作する段階
//
// ---------------------------------------------------------------------------

#include <windows.h>
#include <locale.h>
#include <stdio.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "BulletPhysics/BulletPhysics.h"
#include "MMD/PMDModel.h"
#include "MMD/VMDMotion.h"

#define	MMD_FRUSTUM_NEAR	5.0				// glFrustum() の zNear
#define MMD_FRUSTUM_FAR		1000.0			// glFrustum() の zFar
#define MMD_CAMERA_Z		(-100.0)		// モデル原点に対するカメラz座標
#define MMD_MODEL_Y			(-10.0)			// モデル原点のオフセット（表示中心と回転操作時原点に影響）
#define MMD_BG_COLOR		(0x00FF00FF)	// 透過色として背景にする色（ARGB）

extern cBulletPhysics	g_clBulletPhysics;

static cPMDModel	g_clPMDModel;
static cVMDMotion	g_clVMDMotion;
static cVPDPose		g_clVPDPose;
static bool			g_bDispFPS;				// FPS表示フラグ
static bool			g_bLookAtCusor = FALSE;	// カーソル目線モードにするか
static bool			g_bFollowModel = TRUE;	// カメラが常にモデルを追うか
static bool			g_bPhysics	= FALSE;	// 物理演算を行うか
static float		g_fpFps		= 0.0f;		// 計算された現在のFPSが入る
static bool			g_bBlendPose = FALSE;	// trueだとVPD読込時にそれまでのモーションを消さない
static float		g_fpScale	= 1.0f;		// 拡大率
static Vector3		g_tagModelPosVec;		// モデル配置
static Vector4		g_tagModelRotQuat;		// モデル回転
static GLfloat		g_pfModelRotMat[16];	// モデル回転のOpenGL行列
static Vector3		g_tagFaceVec= {0};		// 顔を向ける先のベクトル
static Matrix		g_mTransform;			// 変換行列（GL_PROJECTION 行列と GL_MODELVIEW 行列の積のコピー）
static Matrix		g_mInvProj;				// GL_PROJECTION の逆行列
static Matrix		g_mCenterOffset;		// ウィンドウ移動モードでセンターへのX,Y移動量を計算するための行列

static void init( void );
static void initGL( void );
static void display( void );
static void resize( int w, int h );
static bool openNewModel( void );
static bool openNewModel( const wchar_t *wszFileName );
static bool openNewMotion( void );
static bool openNewMotion( const wchar_t *wszFileName );
static void RotateModel( const Vector4* quat );
static void TranslateModel( const Vector3 vec );
static void ResetModelPosition();
static void GetTransformMatrix( Matrix matOut );
static void GetInvProjMatrix( Matrix matOut );
static void calcRotationMatrix( GLfloat *pDestMat, Vector4 *pQuat );
static void drawString( const char *szStr, float x, float y );
static void drawChar( char cChar );


//--------
// 初期化
//--------
void init( void )
{
	initGL();

	g_clBulletPhysics.initialize();

	g_bDispFPS = false;
	ResetModelPosition();
}

//--------
// OpenGL関連初期化
//--------
void initGL( void )
{
	// 背景色の設定
	glClearColor(
		(float)((MMD_BG_COLOR & 0x00FF0000) >> 16) / 255.0f,
		(float)((MMD_BG_COLOR & 0x0000FF00) >> 8) / 255.0f,
		(float)((MMD_BG_COLOR & 0x000000FF)) / 255.0f,
		(float)((MMD_BG_COLOR & 0xFF000000) >> 24) / 255.0f
		);

	glEnable( GL_DEPTH_TEST );

	glEnable( GL_TEXTURE_2D );
	glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	glEnable( GL_CULL_FACE );
	glCullFace( GL_FRONT );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	glEnable( GL_ALPHA_TEST );
	glAlphaFunc( GL_GEQUAL, 0.05f );

	const float	fLightPos[] = { -1.0f, 2.0f , 5.0f, 0.0f };
	const float	fLightDif[] = { 0.5f, 0.5f, 0.5f, 1.0f },
				fLightSpc[] = { 0.4f, 0.4f, 0.4f, 1.0f },
				fLightAmb[] = { 1.0f, 1.0f, 1.0f, 1.0f };
				//fLightAmb[] = { 0.9f, 0.9f, 0.9f, 1.0f };

	glLightfv( GL_LIGHT0, GL_POSITION, fLightPos );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, fLightDif );
	glLightfv( GL_LIGHT0, GL_AMBIENT, fLightAmb );
	glLightfv( GL_LIGHT0, GL_SPECULAR, fLightSpc );
	glEnable( GL_LIGHT0 );

	glEnable( GL_LIGHTING );

	// 現時点の変換行列を取得
	GetTransformMatrix( g_mTransform );
}

//----------
// 表示関数
//----------
void display()
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glEnable( GL_CULL_FACE );
	glEnable( GL_ALPHA_TEST );
	glEnable( GL_BLEND );

	glPushMatrix();

		// モデルを追うモードのときはセンターボーンを基準にする
		if ( g_bFollowModel )
		{
			Vector3 vecCenter, vecOffset;
			g_clPMDModel.getModelPosition( &vecCenter);
			vecCenter.z = -vecCenter.z;						// モデル座標系はOpenGL座標系とZ軸が逆
			Vector3Transform( &vecOffset, &vecCenter, g_mCenterOffset );
			glMatrixMode( GL_MODELVIEW );
			glLoadIdentity();
			glTranslatef( g_tagModelPosVec.x - vecOffset.x, g_tagModelPosVec.y - vecOffset.y, g_tagModelPosVec.z );
			//glTranslatef( g_tagModelPosVec.x, g_tagModelPosVec.y, g_tagModelPosVec.z );
			glMultMatrixf( g_pfModelRotMat );
			glTranslatef( 0.0f, MMD_MODEL_Y, 0.0f );		// 目線を原点付近に合わせるため、デフォルトで下げる
		}
		glScalef( 1.0f, 1.0f, -1.0f);	// 左手系 → 右手系

#ifdef _DEBUG
	// 目線デバッグ表示
	glPushMatrix();
		glDisable( GL_LIGHTING );
		glBegin(GL_LINES);
		glColor3f( 1.0f, 0.0f, 0.0f );
		glVertex3f(0.0f, -MMD_MODEL_Y, 0.0f);
		glVertex3f(g_tagFaceVec.x, g_tagFaceVec.y, -g_tagFaceVec.z);
		glColor3f( 0.0f, 1.0f, 0.0f );
		glVertex3f(1.0f, -MMD_MODEL_Y, 0.0f);
		glVertex3f(0.0f, -MMD_MODEL_Y, 0.0f);
		glEnd();
	glPopMatrix();
#endif

		// モデル描画
		g_clPMDModel.render();
	glPopMatrix();

	// FPSの表示
	if( g_bDispFPS )
	{
		char szStr[16];
		sprintf_s( szStr, "%3.2f", g_fpFps );
		drawString( szStr, 0.2f, 0.8f );
	}

	glFlush();
}

//-------
// クォータニオンから OpenGL の回転行列を作成
//-------
void calcRotationMatrix(GLfloat *pDestMat, Vector4 *pQuat)
{
	Matrix mat;
	QuaternionToMatrix(mat, pQuat);
	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 4; i++) {
			pDestMat[i * 4 + j] = mat[j][i];
		}
	}
}

//--------------------------------
// ウインドウがリサイズされたとき
//--------------------------------
void resize( int w, int h )
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glViewport( 0, 0, w, h );
	
	// カメラ設定
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	glFrustum(-1, 1, -(double)h / (double)w, (double)h / (double)w, MMD_FRUSTUM_NEAR, MMD_FRUSTUM_FAR);
	glTranslatef(0.0f, 0.0f, MMD_CAMERA_Z);
	glScalef(g_fpScale, g_fpScale, g_fpScale);

	// モデルの配置
	calcRotationMatrix( g_pfModelRotMat, &g_tagModelRotQuat );
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glTranslatef( g_tagModelPosVec.x, g_tagModelPosVec.y, g_tagModelPosVec.z );
	glMultMatrixf( g_pfModelRotMat );
	glTranslatef( 0.0f, MMD_MODEL_Y, 0.0f );		// 目線を原点付近に合わせるため、デフォルトで下げる

	// 更新された変換行列を取得
	GetTransformMatrix( g_mTransform );

	// GL_PROJECTION の逆行列計算
	GetInvProjMatrix( g_mInvProj ); 

	// センターをウィンドウ中心に表示させるための補正量計算用行列
	MatrixIdentity( g_mCenterOffset );
	g_mCenterOffset[0][0] = g_pfModelRotMat[0];
	g_mCenterOffset[0][1] = g_pfModelRotMat[1];
	g_mCenterOffset[0][2] = 0.0f;
	g_mCenterOffset[0][3] = 0.0f;
	g_mCenterOffset[1][0] = g_pfModelRotMat[4];
	g_mCenterOffset[1][1] = g_pfModelRotMat[5];
	g_mCenterOffset[1][2] = 0.0f;
	g_mCenterOffset[1][3] = 0.0f;
	g_mCenterOffset[2][0] = g_pfModelRotMat[8];
	g_mCenterOffset[2][1] = g_pfModelRotMat[9];
	g_mCenterOffset[2][2] = 0.0f;
	g_mCenterOffset[2][3] = 0.0f;
	g_mCenterOffset[3][0] = 0.0f;
	g_mCenterOffset[3][1] = 0.0f;
	g_mCenterOffset[3][2] = 0.0f;
	g_mCenterOffset[3][3] = 1.0f;
}

//-------
// モデル回転を加える
//-------
void RotateModel(const Vector4* quat)
{
	QuaternionMultiply(&g_tagModelRotQuat, &g_tagModelRotQuat, quat);
}

//-------
// モデル平行移動を加える
//-------
void TranslateModel(const Vector3 vec)
{
	g_tagModelPosVec.x += vec.x;
	g_tagModelPosVec.y += vec.y;
	g_tagModelPosVec.z += vec.z;
}

//-------
// モデル配置を元に戻す
//-------
void ResetModelPosition()
{
	// モデル回転
	g_tagModelRotQuat.w = 1.0f;
	g_tagModelRotQuat.x = 0.0f;
	g_tagModelRotQuat.y = 0.0f;
	g_tagModelRotQuat.z = 0.0f;

	// モデル座標
	g_tagModelPosVec.x = 0.0f;
	g_tagModelPosVec.y = 0.0f;
	g_tagModelPosVec.z = 0.0f;
}

//-------
// 現時点の変換行列を取得
//-------
void GetTransformMatrix( Matrix matOut )
{
	GLfloat fProj[16], fModel[16];
	glGetFloatv( GL_PROJECTION_MATRIX, fProj );
	glGetFloatv( GL_MODELVIEW_MATRIX, fModel );

	// matOut に GL_PROJECTION * GL_MODEL_VIEW を代入
	int n = 0;
	for( int i = 0 ; i < 4 ; i++ )
	{
		matOut[i][0] = (float)(fProj[0] * fModel[n] + fProj[4] * fModel[n+1] + fProj[ 8] * fModel[n+2] + fProj[12] * fModel[n+3]);
		matOut[i][1] = (float)(fProj[1] * fModel[n] + fProj[5] * fModel[n+1] + fProj[ 9] * fModel[n+2] + fProj[13] * fModel[n+3]);
		matOut[i][2] = (float)(fProj[2] * fModel[n] + fProj[6] * fModel[n+1] + fProj[10] * fModel[n+2] + fProj[14] * fModel[n+3]);
		matOut[i][3] = (float)(fProj[3] * fModel[n] + fProj[7] * fModel[n+1] + fProj[11] * fModel[n+2] + fProj[15] * fModel[n+3]);
		n += 4;
	}
}

//-------
// 現時点の GL_PROJECTION の逆行列を取得
//-------
void GetInvProjMatrix( Matrix matOut )
{
	Matrix matProj;
	GLfloat fProj[16];
	glGetFloatv( GL_PROJECTION_MATRIX, fProj );

	for( int i = 0 ; i < 4 ; i++ )
	{
		for( int j = 0; j < 4; j++ )
		{
			matProj[i][j] = (float)fProj[i * 4 + j];		// i列j行
		}
	}
	MatrixInverse( matOut, matProj );
}

//--------
// パスをディレクトリ部分とファイル名に分離
//--------
bool SplitPath(wchar_t *wszDir, wchar_t *wszFile, const wchar_t *wszPath)
{
	bool found = FALSE;
	int  nIndex = 0;
	wszDir[0] = NULL;
	wszFile[0] = NULL;
	for (int i = wcslen(wszPath); i >= 0; i--) {
		wchar_t ch = wszPath[i];
		if ( found ) {
			wszDir[i] = wszPath[i];
		} else if ( ch == '\\' || ch == '/' || ch == ':' ) {
			found = TRUE;
			wszDir[i+1] = NULL;
			wszDir[i] = wszPath[i];
			nIndex = i + 1;
		}
	}
	wcsncpy_s( wszFile, MAX_PATH, wszPath + nIndex, MAX_PATH );
	return found;
}

//--------
// ファイルの拡張子（正しくは最後4文字）を返す
//--------
inline void GetFileExtension(wchar_t *wszExt, const wchar_t *wszFileName)
{
	int nLen = wcslen( wszFileName );
	wszExt[0] = NULL;
	// ダブルクォーテーションで囲まれていそうなら、それを避ける
	if ( nLen > 0 && wszFileName[nLen - 1] == L'"' ) nLen--;

	// ラスト4文字をコピー
	for ( int j = 0; j < 5 && (nLen - j) >= 0; j++ ) {
		wszExt[4 - j] = wszFileName[nLen - j];
	}
}


//----------------------
// モデルファイルを開く
//----------------------
bool openNewModel( const wchar_t *wszPath )
{
	bool bOpened = FALSE;
	wchar_t	wszDirName[MAX_PATH];
	wchar_t	wszFileName[MAX_PATH];
	wchar_t wszCurrentDir[MAX_PATH];
	const wchar_t *wszFile;
	DWORD nCurDirBufSize = 0;

	// カレントディレクトリを変更
	if ( SplitPath( wszDirName, wszFileName, wszPath)) {
		nCurDirBufSize = GetCurrentDirectory( MAX_PATH, wszCurrentDir );
		SetCurrentDirectory( wszDirName );
		wszFile = wszFileName;
	} else {
		wszFile = wszPath;
	}

	if ( g_clPMDModel.load( wszFile) ) {
		g_clPMDModel.setLookAtFlag(g_bLookAtCusor);		// マウス目線モード設定
		g_clPMDModel.enablePhysics( g_bPhysics );		// 物理演算設定
		bOpened = TRUE;
	}

	// カレントディレクトリが変更されていれば元に戻す
	if ( nCurDirBufSize != 0 ) {
		SetCurrentDirectory( wszCurrentDir );
	}
	return bOpened;
}
bool openNewModel( HWND hWnd )
{
	OPENFILENAME	ofn;
	wchar_t			wszFileName[MAX_PATH],
					wszDirName[MAX_PATH],
					wszFile[64];

	ZeroMemory( &wszFileName, MAX_PATH );
	ZeroMemory( &wszDirName,  MAX_PATH );
	ZeroMemory( &wszFile,      64 );
	ZeroMemory( &ofn, sizeof(OPENFILENAME) );
	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = hWnd;
	ofn.lpstrFilter = L"PMD File(*.pmd)\0*.pmd\0\0";
	ofn.lpstrFile = wszFileName;
	ofn.lpstrFileTitle = wszFile;
	ofn.nMaxFile = MAX_PATH;
	ofn.nMaxFileTitle = sizeof(wszFile);
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
	ofn.lpstrDefExt = L"pmd";
	ofn.lpstrTitle = L"Open PMD";

	if( GetOpenFileName( &ofn ) )
	{
		return openNewModel( wszFileName );
	}
	return FALSE;
}

//--------------------------
// モーションファイルを開く
//--------------------------
bool openNewMotion( const wchar_t *wszPath )
{
	bool bOpened = FALSE;
	wchar_t	wszDirName[MAX_PATH];
	wchar_t	wszFileName[MAX_PATH];
	wchar_t wszCurrentDir[MAX_PATH];
	const wchar_t *wszFile;
	wchar_t wszExt[5];
	DWORD nCurDirBufSize = 0;

	// カレントディレクトリを変更
	if ( SplitPath( wszDirName, wszFileName, wszPath)) {
		nCurDirBufSize = GetCurrentDirectory( MAX_PATH, wszCurrentDir );
		SetCurrentDirectory( wszDirName );
		wszFile = wszFileName;
	} else {
		wszFile = wszPath;
	}
	// 拡張子を取得
	GetFileExtension(wszExt, wszFileName);

	if (_wcsnicmp(wszExt, L".vmd", 4) == 0) {
		// VMDファイルならばモーションとして開く
		if ( g_clVMDMotion.load( wszFile ) ) {
			g_clPMDModel.setMotion( &g_clVMDMotion, TRUE );
			g_clPMDModel.updateMotion( 0.0f );
			g_clPMDModel.resetRigidBodyPos();
			bOpened = TRUE;
		}
	} else if (_wcsnicmp(wszExt, L".vpd", 4) == 0) {
		// VPDファイルでもモーションに変換して開く
		if ( g_clVPDPose.load( wszFile ) ) {
			if (!g_bBlendPose) {
				g_clVMDMotion.release();	// それまでのモーションを消去
				g_clPMDModel.resetBones();
			}
			g_clVMDMotion.import( &g_clVPDPose );		// モーションにインポート
			g_clPMDModel.setMotion( &g_clVMDMotion, TRUE );
			g_clPMDModel.updateMotion( 0.0f );
			g_clPMDModel.resetRigidBodyPos();
			bOpened = TRUE;
		}
	}

	// カレントディレクトリが変更されていれば元に戻す
	if ( nCurDirBufSize != 0 ) {
		SetCurrentDirectory( wszCurrentDir );
	}
	return bOpened;
}

bool openNewMotion( HWND hWnd )
{
	OPENFILENAME	ofn;
	wchar_t			wszFileName[MAX_PATH],
					wszDirName[MAX_PATH],
					wszFile[64];

	ZeroMemory( &wszFileName, MAX_PATH );
	ZeroMemory( &wszDirName,  MAX_PATH );
	ZeroMemory( &wszFile,      64 );
	ZeroMemory( &ofn, sizeof(OPENFILENAME) );
	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = hWnd;
	ofn.lpstrFilter = L"VMD Motion/VPD Pose(*.vmd;*.vpd)\0*.vmd;*.vpd\0\0";
	ofn.lpstrFile = wszFileName;
	ofn.lpstrFileTitle = wszFile;
	ofn.nMaxFile = MAX_PATH;
	ofn.nMaxFileTitle = sizeof(wszFile);
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
	ofn.lpstrDefExt = L"vmd";
	ofn.lpstrTitle = L"Open VMD/VPD";

	if( GetOpenFileName( &ofn ) )
	{
		return openNewMotion( wszFileName );
	}
	return FALSE;
}

// 数字を描画
void drawString( const char *szStr, float x, float y )
{
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );
	glDisable( GL_TEXTURE_2D );

	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0.0f, 1.0f, 0.0f, 1.0f );

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	glTranslatef( x, y, 0.0f );
	glScalef( 0.02f, 0.02f, 1.0f );

	glColor4f( 0.0f, 1.0f, 0.0f, 1.0f );
	glLineWidth( 2.0f );

	int i = 0;
	for ( int i = 0; ; i++ ) {
		char c = szStr[i];
		if ( c == NULL ) break;

		drawChar( c );
		glTranslatef( 1.5f, 0.0f, 0.0f );
	}

	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();

	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
}

// 1文字描画　※ただし数字に限る
void drawChar( char cChar )
{
	unsigned char pat = 0;		// 7セグメントパターン
	switch ( cChar ) {
		case '0':	pat = 0x3F; break;
		case '1':	pat = 0x06;	break;
		case '2':	pat = 0x5B;	break;
		case '3':	pat = 0x4F;	break;
		case '4':	pat = 0x66;	break;
		case '5':	pat = 0x6D;	break;
		case '6':	pat = 0x7D;	break;
		case '7':	pat = 0x07;	break;
		case '8':	pat = 0x7F;	break;
		case '9':	pat = 0x6F;	break;
		case '.':	pat = 0x80;	break;
		case ' ':	pat = 0x00;	break;
		default:	pat = 0x00;	break;
	}

	glBegin( GL_LINES );
		if ( pat & 0x01 ) {
			glVertex2d(0, 2);
			glVertex2d(1, 2);
		}
		if ( pat & 0x02 ) {
			glVertex2d(1, 2);
			glVertex2d(1, 1);
		}
		if ( pat & 0x04 ) {
			glVertex2d(1, 1);
			glVertex2d(1, 0);
		}
		if ( pat & 0x08 ) {
			glVertex2d(0, 0);
			glVertex2d(1, 0);
		}
		if ( pat & 0x10 ) {
			glVertex2d(0, 0);
			glVertex2d(0, 1);
		}
		if ( pat & 0x20 ) {
			glVertex2d(0, 1);
			glVertex2d(0, 2);
		}
		if ( pat & 0x40 ) {
			glVertex2d(0, 1);
			glVertex2d(1, 1);
		}
		if ( pat & 0x80 ) {
			glVertex2d(0.5, 0.0);
			glVertex2d(0.5, 0.1);
		}
	glEnd();
}