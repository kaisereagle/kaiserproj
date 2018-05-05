//#pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")

#ifdef _DEBUG
//#pragma comment(lib,"../../freeglut-2.8.1/lib/x86/Debug/freeglut.lib")
//#pragma comment(lib,"../../glew-1.11.0/lib/Debug/Win32/glew32d.lib")
//#pragma comment(lib,"../../bullet3-master/bin/BulletCollision_vs2010_debug.lib")
//#pragma comment(lib,"../../bullet3-master/bin/BulletDynamics_vs2010_debug.lib")
//#pragma comment(lib,"../../bullet3-master/bin/LinearMath_vs2010_debug.lib")
#else
#pragma comment(lib,"../../freeglut-2.8.1/lib/x86/freeglut.lib")
#pragma comment(lib,"../../glew-1.11.0/lib/Release/Win32/glew32.lib")
#pragma comment(lib,"../../bullet3-master/bin/BulletCollision_vs2010.lib")
#pragma comment(lib,"../../bullet3-master/bin/BulletDynamics_vs2010.lib")
#pragma comment(lib,"../../bullet3-master/bin/LinearMath_vs2010.lib")
#endif

#include <GL/glut.h>
//#include "../BulletPhysics/BulletPhysics.h"
#include "../../MMD_view/Classes/PMDModel.h"
#include "../../MMD_view/Classes/VMDMotion.h"
//#include "../mmdlib/PMDModel.h"
//#include "../mmdlib/VMDMotion.h"

//extern cBulletPhysics g_clBulletPhysics;
static cPMDModel g_clPMDModel;
static cVMDMotion g_clVMDMotion;

//----------------------------------------------------
void Initialize(void)
{
	glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);
	const float fLightPos[] = { 0.45f, 0.55f, 1.0f, 0.0f };
	const float fLightDif[] = { 0.9f, 0.9f, 0.9f, 1.0f };
	const float fLightAmb[] = { 0.9f, 0.9f, 0.9f, 1.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, fLightPos);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, fLightDif);
	glLightfv(GL_LIGHT0, GL_AMBIENT, fLightAmb);
	glLightfv(GL_LIGHT0, GL_SPECULAR, fLightAmb);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

	//g_clBulletPhysics.initialize();
	// SetCurrentDirectory("./");             // ファイルをアクセスためのパスを設定。Release/***.exeから考える。
	g_clPMDModel.load("./miku.pmd");              // PMDファイルを登録
	g_clVMDMotion.load("./1.vmd");          // VMDファイルを登録
	g_clPMDModel.setMotion(&g_clVMDMotion, false); // モデルをモーションに紐付け
	g_clPMDModel.updateMotion(0.0f);               // プレイ時間を初期化
	//  g_clPMDModel.resetRigidBodyPos();              // モデルを初期化

	glClearColor(0.0, 1.0, 1.0, 1.0); //背景色
}
//----------------------------------------------------
static float lscalef = 1.0f;
void Display(void)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective( /* field of view in degree */ 40.0,/* aspect ratio */ 1.0,/* Z near */ 1.0, /* Z far */ 500.0);
	gluLookAt(0, 20, 50,  /* eye is at (0,0,5) */ 0.0, 10.0, 0.0, /* center is at (0,0,0) */ 0.0, 1.0, 0.); /* up is in positive Y direction */
	glDepthFunc(GL_LEQUAL);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glScalef(1.0f*lscalef, 1.0f*lscalef, -1.0f*lscalef);
		g_clPMDModel.render();
	}
	glutSwapBuffers(); // ダブルバッファリング
}
//----------------------------------------------------
float getElapsedFrame(void)
{
	static int s_iPrevTime = 0;
	int   iTime = glutGet(GLUT_ELAPSED_TIME);
	float  fDiffTime;
	if (s_iPrevTime == 0) s_iPrevTime = iTime;
	fDiffTime = (float)(iTime - s_iPrevTime) * (30.0f / 1000.0f);
	s_iPrevTime = iTime;
	return fDiffTime;
}
void Idle(void)
{
	vec3 vecCamPos;
	float  fElapsedFrame = getElapsedFrame();
	if (fElapsedFrame > 10.0f) fElapsedFrame = 10.0f;
	g_clPMDModel.updateMotion(fElapsedFrame);
	g_clPMDModel.updateNeckBone(&vecCamPos);
	//g_clBulletPhysics.update(fElapsedFrame);
	g_clPMDModel.updateSkinning();
	glutPostRedisplay();
}
//----------------------------------------
void Visibility(int visible)
{
	if (visible == GLUT_VISIBLE) glutIdleFunc(Idle);
	else       glutIdleFunc(NULL);
}
//----------------------------------------------------
void Cleanup(void)
{
	g_clPMDModel.release();       // PMD,VMDが取得したメモリを解放
	//g_clBulletPhysics.release();  // Bulletが取得したメモリを解放
}
//----------------------------------------------------
int main(int argc, char *argv[])
{
	glutInit(&argc, argv);               // glut の初期化
	glutInitWindowPosition(100, 100); // ウィンドウの位置
	glutInitWindowSize(640, 480);      // ウィンドウサイズ
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE); // ディスプレイモード
	glutCreateWindow("MMDapr0");         // ウィンドウ作成
	glutDisplayFunc(Display);            // 描画時コールバック
	// glutVisibilityFunc(Visibility);
	glutIdleFunc(Idle);                  // アイドル時コールバック
	//  atexit(Cleanup);                     // 終了時コールバック（メモリ解放など
	Initialize();                        // 初期設定
	glutMainLoop();
	return 0;
}