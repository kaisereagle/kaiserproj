// ---------------------------------------------------------------------------
//
// MMD_DesktopMascot
//   PMD, VMD, VPD をデスクトップマスコット風に表示します
//
// Copyright (c) 2009-2011  Ru--en
// 
// 
// 2011/04/30	Ver.0.5.1	PNG,JPEG等のテクスチャをGDI+で対応
// 2011/03/27	Ver.0.5.0	エッジ無し指定材質の色計算を修正（主にLat式対策）
// 2010/01/20	Ver.0.4.9	ウィンドウ移動奥行き方向修正
// 2010/01/15	Ver.0.4.8	センターボーンに合わせたウィンドウ移動実装、メニュー２カ国語化
// 2010/01/07	Ver.0.4.7	輪郭・影なし材質に対応、読込エラー修正
// 2010/01/06	Ver.0.4.6	アスタリスク区切りのスフィアマップ指定に部分対応
// 2009/10/18	Ver.0.4.5	Bullet 2.75 使用、コマンドライン引数対応修正
// 2009/08/31	Ver.0.4.4	ARTK_MMD v0.7 取込（スフィアマップ対応）
// 2009/08/30	Ver.0.4.3	メニューにチェックを表示
// 2009/08/07	Ver.0.4.2	モーションを常に60fpsで計算、FPS表示追加
// 2009/08/03	Ver.0.4.1	物理演算のためフレームレート修正
// 2009/08/02	Ver.0.4.0	ARTK_MMD v0.5 取込（物理演算対応）
// 2009/07/15	Ver.0.3.7	ファイルドロップ対応、BGM再生、
//							タスクバー右クリック対応、コマンドライン対応
// 2009/07/15	Ver.0.3.6	サイズ変更修正、ファイル名のワイド文字化、
//							メニュー英語化、マウス目線修正
// 2009/07/14	Ver.0.3.5	再生速度修正、キャンバスサイズ変更追加
// 2009/07/09	Ver.0.3.4	テクスチャ読込修正
// 2009/07/08	Ver.0.3.3	FPS制限の10FPSが機能しなかったので修正
// 2009/07/08	Ver.0.3.2	FPS制限追加、マウス目線追加
// 2009/07/01	Ver.0.3.1	ソース公開用に整理、回転/移動機能追加
// 2009/06/30	Ver.0.3.0	初回公開版
// 2009/06/29	Ver.0.2.0	内輪公開版
// 2009/06/29	Ver.0.1.0	コンセプト提示
//
// ---------------------------------------------------------------------------
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
// ---------------------------------------------------------------------------

#include "stdafx.h"
#include "MMD_DesktopMascot.h"

//#define MMDDM_WINDOW			1				// 有効にするとウィンドウ枠表示モード(デバッグ用)

// 定数
#define MMDDM_MAX_LOADSTRING	100
#define MMDDM_DEFAULT_WIDTH		400				// 初期サイズ 横幅[pixel]
#define MMDDM_DEFAULT_HEIGHT	400				// 初期サイズ 高さ[pixel]
#define MMDDM_WAIT				1				// ループ処理のウェイト[msec]
#define MMDDM_ROTATION_STEP		0.08726646256f	// 回転操作単位[rad] （0.087rad ≒ 5度）
#define MMDDM_MOVE_STEP			0.5f			// 並進操作単位
#define MMDDM_MOTION_SPEED		1.0f			// アニメーション速度調整[倍]
#define MMDDM_TIMER_ID			1				// SetTimer()に渡すID

// グローバル変数:
HINSTANCE	g_hInst = NULL;						// 現在のインターフェイス
HWND		g_hWnd = NULL;						// 現在のウィンドウハンドル
TCHAR	szTitle[MMDDM_MAX_LOADSTRING];			// タイトル バーのテキスト
TCHAR	szWindowClass[MMDDM_MAX_LOADSTRING];	// メイン ウィンドウ クラス名

OffscreenGL	*g_clGl = NULL;						// オフスクリーンOpenGLクラスのインスタンス
bool		g_bTopMost	= FALSE;				// 最前面に表示するか
float		g_fpFrameSkip = 0.5f;				// 表示FPS制限．例えば2にすると15fpsになる
float		g_fpLastDispFrame = 0.0f;			// 最後に表示したフレーム番号
float		g_fpLastCalcFrame = 0.0f;			// 最後に更新されたモーションのフレーム番号
DWORD		g_dwStartTime = 0;					// モーション再生開始時の GetTickCount() 値
bool		g_bBGM		= FALSE;				// BGMを流したか
POINT		g_tagLastCurPos;					// 直前のマウスカーソル座標
POINT		g_tagWindowOffset;					// 画面中央からウィンドウが移動された量

ULONG_PTR pGdiToken;
Gdiplus::GdiplusStartupInput gdiInput;



//--------
// メイン
//--------
int APIENTRY _tWinMain( HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPTSTR    lpCmdLine,
                     int       nCmdShow )
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

 	// TODO: ここにコードを挿入してください。
	MSG msg;
	HACCEL hAccelTable;

	// グローバル文字列を初期化しています。
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MMDDM_MAX_LOADSTRING);
	LoadString(hInstance, IDS_CLASSNAME, szWindowClass, MMDDM_MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// アプリケーションの初期化を実行します:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}
	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_MENU));

	// メイン メッセージ ループ:
	while (GetMessage(&msg, NULL, 0, 0)) {
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	// 終了時の解放処理
	Release();

	return (int) msg.wParam;
}

//
//  関数: MyRegisterClass()
//
//  目的: ウィンドウ クラスを登録します。
//
//  コメント:
//
//    この関数および使い方は、'RegisterClassEx' 関数が追加された
//    Windows 95 より前の Win32 システムと互換させる場合にのみ必要です。
//    アプリケーションが、関連付けられた
//    正しい形式の小さいアイコンを取得できるようにするには、
//    この関数を呼び出してください。
//
ATOM MyRegisterClass( HINSTANCE hInstance )
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_MMDDM_ICON));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= 0;	//MAKEINTRESOURCE(IDC_MENU);
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_MMDDM_ICON));

	return RegisterClassEx(&wcex);
}

//
//   関数: InitInstance(HINSTANCE, int)
//
//   目的: インスタンス ハンドルを保存して、メイン ウィンドウを作成します。
//
//   コメント:
//
//        この関数で、グローバル変数でインスタンス ハンドルを保存し、
//        メイン プログラム ウィンドウを作成および表示します。
//
BOOL InitInstance( HINSTANCE hInstance, int nCmdShow )
{
	g_hInst = hInstance; // グローバル変数にインスタンス処理を格納します。

	// 画面中央にウィンドウを合わせる
	g_tagWindowOffset.x = (GetSystemMetrics( SM_CXSCREEN ) - MMDDM_DEFAULT_WIDTH) / 2;
	g_tagWindowOffset.y = (GetSystemMetrics( SM_CYSCREEN ) - MMDDM_DEFAULT_HEIGHT) / 2;

	g_hWnd = CreateWindowEx(
#ifdef MMDDM_WINDOW
		(g_bTopMost ? WS_EX_TOPMOST : 0),					// レイヤードウィンドウにしない
#else
		WS_EX_LAYERED | (g_bTopMost ? WS_EX_TOPMOST : 0),	// レイヤードウィンドウにする
#endif
		szWindowClass,
		szTitle,
		WS_VISIBLE,	// | WS_OVERLAPPEDWINDOW,
		g_tagWindowOffset.x,
		g_tagWindowOffset.y,
		MMDDM_DEFAULT_WIDTH, //CW_USEDEFAULT,
		MMDDM_DEFAULT_HEIGHT, //0,
		NULL,
		NULL,
		hInstance,
		NULL
		);

	if (!g_hWnd)
	{
		return FALSE;
	}

	// ドラッグアンドドロップ有効
	DragAcceptFiles(g_hWnd, TRUE);

    // 現在のウィンドウサイズを記憶
	UpdateWindowSize(g_hWnd);

	// 半透明のテスト
	//   これを使った場合、カラーキーは使用できないみたい
	//SetLayeredWindowAttributes(g_hWnd, 0, 0x8f, ULW_ALPHA);

	// ビットマップを用意
	g_clGl = new OffscreenGL(MMDDM_DEFAULT_WIDTH, MMDDM_DEFAULT_HEIGHT);
	g_clGl->SetCurrent();

	// OpenGL 初期設定
	init();
	resize(MMDDM_DEFAULT_WIDTH, MMDDM_DEFAULT_HEIGHT);

	// GDI+ 初期化
	Gdiplus::GdiplusStartup( &pGdiToken, &gdiInput, NULL );

	// コマンドライン引数があればそのファイルを開く
	LPTSTR lpCmdLine = GetCommandLine();
	if (!OpenCommandLineFiles(lpCmdLine)) {
		// デフォルトデータ自動読込
		if (!openNewModel( L"default.pmd" )) {
			if (!openNewModel( g_hWnd )) {
				// デフォルトのモデルが読み込めず，指定ファイルも開けなければ強制終了
				return FALSE;
			}
		}
		openNewMotion( L"default.vmd" );
		g_clPMDModel.setMotion( &g_clVMDMotion, true );
		g_clPMDModel.updateMotion( 0 );
		g_clPMDModel.updateSkinning();
	} else {
		// コマンドライン引数で最前面表示が変えられるので、再設定
		SetWindowPos(g_hWnd, (g_bTopMost ? HWND_TOPMOST : HWND_NOTOPMOST), NULL, NULL, NULL, NULL, SWP_NOMOVE | SWP_NOSIZE);
	}

	ResetTime();			// モーション再生時刻リセット
	ResetCursor();			// マウスカーソル位置リセット

	ShowWindow(g_hWnd, nCmdShow);
	UpdateWindow(g_hWnd);
	LocateWindow(g_hWnd);	// ウィンドウ位置調整

	// 描画更新のためのタイマー開始
	SetTimer( g_hWnd, MMDDM_TIMER_ID, MMDDM_WAIT, NULL );

	return TRUE;
}

//
//  関数: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  目的:  メイン ウィンドウのメッセージを処理します。
//
//  WM_COMMAND	- アプリケーション メニューの処理
//  WM_PAINT	- メイン ウィンドウの描画
//  WM_DESTROY	- 中止メッセージを表示して戻る
//
//
LRESULT CALLBACK WndProc( HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam )
{
	int wmId, wmEvent;
	//PAINTSTRUCT ps;
	//HDC hdc;

	switch (message)
	{
	case WM_COMMAND:
		KillTimer( hWnd, MMDDM_TIMER_ID );			// モデル更新処理を中断するためタイマーを切る
		wmId    = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// 選択されたメニューの解析:
		switch (wmId)
		{
		case IDM_POPUP_MENU:						// キーボードの右クリックメニューボタンでポップアップを開く
			PopupMenu(hWnd, lParam);
			break;
		case IDM_ABOUT:
			DialogBox(g_hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_OPEN_MODEL:						// モデル(.pmd)を開く
			if (openNewModel( hWnd )) {
				ResetTime();
			}
			break;
		case IDM_OPEN_MOTION:						// モーション(.vmd/.vpd)を開く
			if (openNewMotion( hWnd )) {
				ResetTime();
			}
			break;
		case IDM_LOOKATME:							// マウス目線
			g_bLookAtCusor = !g_bLookAtCusor;
			if (g_bLookAtCusor) {
				// マウス目線モードOnなら視線ベクトルを更新しておく
				UpdateMousePosition(hWnd);
			}
			g_clPMDModel.setLookAtFlag(g_bLookAtCusor);
			break;
		case IDM_ENABLE_PHYSICS:					// 物理演算 On/Off
			g_bPhysics = !g_bPhysics;
			g_clPMDModel.enablePhysics( g_bPhysics );
			g_clPMDModel.resetRigidBodyPos();
			break;
		case IDM_SHOW_FPS:							// マウス目線 On/Off
			g_bDispFPS = !g_bDispFPS;
			break;
		case IDM_POSEBLEND:							// VPD読込時の追加モード On/Off
			g_bBlendPose = !g_bBlendPose;
			break;
		case IDM_MOVINGWINDOW:						// モデルのセンターに合わせてウィンドウ移動
			g_bFollowModel = !g_bFollowModel;
			LocateWindow( hWnd );
			break;
		case IDM_SCALE_100:
			Scale(hWnd, 1.0f);
			break;
		case IDM_SCALE_50:
			Scale(hWnd, 0.5f);
			break;
		case IDM_SCALE_200:
			Scale(hWnd, 2.0f);
			break;
		case IDM_SCLAE_UP:
			Scale(hWnd, g_fpScale * 1.2f);							// 120%に拡大
			break;
		case IDM_SCALE_DOWN:
			Scale(hWnd, g_fpScale * 0.8f);							// 80%に縮小
			break;
		case IDM_ROT_LEFT:
			Rotate(hWnd, 0.0f, 1.0f, 0.0f, MMDDM_ROTATION_STEP);	// 時計回り5度回転
			break;
		case IDM_ROT_RIGHT:
			Rotate(hWnd, 0.0f, 1.0f, 0.0f, -MMDDM_ROTATION_STEP);	// 反時計回り5度回転
			break;
		case IDM_ROT_UP:
			Rotate(hWnd, 1.0f, 0.0f, 0.0f, MMDDM_ROTATION_STEP);	// 上向き5度回転
			break;
		case IDM_ROT_DOWN:
			Rotate(hWnd, 1.0f, 0.0f, 0.0f, -MMDDM_ROTATION_STEP);	// 下向き5度回転
			break;
		case IDM_MOV_LEFT:
			Translate(hWnd, -MMDDM_MOVE_STEP, 0.0f, 0.0f);			// 向かって左に移動
			break;
		case IDM_MOV_RIGHT:
			Translate(hWnd, MMDDM_MOVE_STEP, 0.0f, 0.0f);			// 向かって右に移動
			break;
		case IDM_MOV_UP:
			Translate(hWnd, 0.0f, MMDDM_MOVE_STEP, 0.0f);			// 上に移動
			break;
		case IDM_MOV_DOWN:
			Translate(hWnd, 0.0f, -MMDDM_MOVE_STEP, 0.0f);			// 下に移動
			break;
		case IDM_POS_RESET:
			ResetPosition(hWnd);									// 配置をリセット
			break;
		case IDM_TOPMOST:
			ToggleTopMost(hWnd);									// 常に最前面に表示
			break;
		case IDM_FPS_60:
			g_fpFrameSkip = 0.5f;									// 0.5フレームごとに描画させる
			break;
		case IDM_FPS_30:
			g_fpFrameSkip = 1.0f;									// 1フレームごとに描画させる
			break;
		case IDM_FPS_15:
			g_fpFrameSkip = 2.0f;									// 2フレームごとに描画させる
			break;
		case IDM_FPS_10:
			g_fpFrameSkip = 3.0f;									// 3フレームごとに描画させる
			break;
		case IDM_FPS_5:
			g_fpFrameSkip = 6.0f;									// 6フレームごとに描画させる
			break;
		case IDM_CANVAS_FULL:
			ResizeWindow(hWnd, NULL);
			break;
		case IDM_CANVAS_800:
			ResizeWindow(hWnd, 800);
			break;
		case IDM_CANVAS_600:
			ResizeWindow(hWnd, 600);
			break;
		case IDM_CANVAS_400:
			ResizeWindow(hWnd, 400);
			break;
		case IDM_CANVAS_200:
			ResizeWindow(hWnd, 200);
			break;
		case IDM_DEBUG:												// デバッグ用表示切替
			g_clPMDModel.m_iDebug++;
			if (g_clPMDModel.m_iDebug > 4) g_clPMDModel.m_iDebug = 0;
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		SetTimer( hWnd, MMDDM_TIMER_ID, MMDDM_WAIT, NULL);			// タイマー復帰
		break;
	case WM_LBUTTONDOWN:
		//// ↓左ボタンが押されたとき，タイトルバーだと誤認させることでドラッグ移動を可能にする方法
		//PostMessage(hWnd, WM_NCLBUTTONDOWN, HTCAPTION, lParam);

		// ↓ノーマルな方法
		ResetCursor();
		SetCapture( hWnd );
		break;
	case WM_LBUTTONUP:
		if (GetCapture() == hWnd) {
			ReleaseCapture();
			ResetCursor();
		}
		break;
	case WM_RBUTTONDOWN:
	case 0x0313:						// タスクバー右クリックで発生（隠し機能？）
		PopupMenu(hWnd, lParam);
		break;
	case WM_MOUSEMOVE:
		if (wParam & MK_LBUTTON) {
			UpdateCursor(hWnd);
		}
		break;
	case WM_PAINT:
		//hdc = BeginPaint(hWnd, &ps);	// レイヤードウィンドウなので不要？
		calcFps();						// FPS計算
		Render(hWnd);					// 描画処理
		//EndPaint(hWnd, &ps);			// レイヤードウィンドウなので不要？
		ValidateRect(hWnd, NULL);		// 描画完了
		break;
	case WM_DROPFILES:
		OpenDraggedFiles(hWnd, (HDROP)wParam);
		break;
	case WM_TIMER:
		UpdateMotion( hWnd );
		if ( g_bFollowModel )
			LocateWindow(hWnd);			// モデルのセンターに合わせたウィンドウ移動
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	//case MM_MCINOTIFY:		// 思ったように動作しない？
	//	g_clPMDModel.rewind();
	//	ResetTime();
	//	break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// バージョン情報ボックスのメッセージ ハンドラです。
INT_PTR CALLBACK About( HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam )
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

//--------
// 描画処理
//--------
void Render( HWND hWnd )
{
	if (!g_clGl) return;

	if (g_bLookAtCusor) {
		UpdateMousePosition(hWnd);
	}

	g_clGl->BeginRender();
	display();
	g_clGl->EndRender();

	HDC hBmpDC = g_clGl->GetDC();

#ifdef MMDDM_WINDOW
	HDC dc = ::GetDC(hWnd);
	RECT rc;
	GetWindowRect(hWnd, &rc);
	BitBlt(dc, 0, 0, (rc.right - rc.left), (rc.bottom - rc.top), hBmpDC, 0, 0, SRCCOPY);
	return;
#else
	HDC hScreenDC = GetDC(NULL);

	POINT wndPos, surPos;
	RECT rc;
	SIZE wndSize;

	GetWindowRect(hWnd, &rc);
	wndPos.x = rc.left;
	wndPos.y = rc.top;
	surPos.x = surPos.y = 0;
	wndSize.cx = rc.right - rc.left;
	wndSize.cy = rc.bottom - rc.top;

	BLENDFUNCTION bf;
	bf.BlendOp = AC_SRC_OVER;
	bf.BlendFlags = 0;
	bf.AlphaFormat = AC_SRC_ALPHA;
	bf.SourceConstantAlpha = 0xFF;

	//// アルファ値を適用
	//UpdateLayeredWindow(hWnd, hScreenDC, &wndPos, &wndSize, hBmpDC, &surPos, 0, &bf, ULW_ALPHA);

	// カラーキーを適用
	UpdateLayeredWindow(hWnd, hScreenDC, &wndPos, &wndSize, hBmpDC, &surPos, MMD_BG_COLOR, &bf, ULW_COLORKEY);

	ReleaseDC(0, hScreenDC);
#endif 
}

//--------
// モーションを更新して描画指示
//--------
void UpdateMotion( HWND hWnd )
{
	float fCurrentFrame;

	// 再生開始からの経過フレーム数
	fCurrentFrame = (float)GetElapsedTime(g_dwStartTime) * MMDDM_MOTION_SPEED * 0.03f;

	// 演算を行う最小フレーム単位
	float frameStep = 0.5f;

	// 物理演算を飛ばすか。また、描画が5フレーム以上発生するようなら、強制的に物理演算をスキップ
	bool  bSkipPhysics = ( !g_bPhysics || (fCurrentFrame - g_fpLastCalcFrame ) > 5.0f );

	// モーション更新
	if ( bSkipPhysics ) {
		frameStep = floor((float)(fCurrentFrame - g_fpLastCalcFrame) / frameStep) * frameStep;	// frameStep単位で端数を無くした経過フレーム数
		if ( frameStep < g_fpFrameSkip ) return;							// 描画のFPS制限に合わせる

		g_clPMDModel.updateMotion( frameStep );
		if ( g_bLookAtCusor ) g_clPMDModel.updateNeckBone( &g_tagFaceVec );	// マウス目線反映
		g_clPMDModel.resetRigidBodyPos();									// 物理演算をスキップする場合は剛体をボーンに合わせる
		g_fpLastCalcFrame += frameStep;
	} else {
		while ( ( fCurrentFrame - g_fpLastCalcFrame ) >= frameStep ) {
			g_clPMDModel.updateMotion( frameStep );
			if ( g_bLookAtCusor ) g_clPMDModel.updateNeckBone( &g_tagFaceVec );	// マウス目線反映
			g_clBulletPhysics.update( frameStep );								// 物理演算処理
			g_fpLastCalcFrame += frameStep;
		}
	}

	// 描画更新指示
	if ( ( fCurrentFrame - g_fpLastDispFrame ) < g_fpFrameSkip) return;		// FPS制限

	g_clPMDModel.updateSkinning();											// スキニング更新
	InvalidateRect( hWnd, NULL, TRUE );										// 描画指示

	g_fpLastDispFrame = fCurrentFrame;										// 描画したフレーム番号を記録（誤差防止のため整数単位）
}

//--------
// マウスカーソル位置更新
//--------
void ResetCursor()
{
	// カーソル位置取得
	GetCursorPos( &g_tagLastCurPos);
}


//--------
// マウスカーソル位置更新
//--------
void UpdateCursor( HWND hWnd )
{
	static const float fMaxVelocity = 5.0f;		// 最大速度
	static const float fCoefVelocity = 0.002f;	// 速度に掛ける適当な係数（マウスの動きに比例させる係数）

	static DWORD lastTime = GetElapsedTime(0);
	DWORD elapsedTime;
	float sec;
	POINT tagCurrentPos;
	float fVelocity[2];

	// カーソル位置取得
	GetCursorPos( &tagCurrentPos );

	// ウィンドウ位置更新
	g_tagWindowOffset.x += (tagCurrentPos.x - g_tagLastCurPos.x);
	g_tagWindowOffset.y += (tagCurrentPos.y - g_tagLastCurPos.y);
	LocateWindow( hWnd );

	//// ウィンドウ位置更新
	//RECT  rect;
	//GetWindowRect( hWnd, &rect );
	//SetWindowPos(
	//	hWnd,
	//	NULL,
	//	rect.left + (tagCurrentPos.x - g_tagLastCurPos.x),
	//	rect.top + (tagCurrentPos.y - g_tagLastCurPos.y),
	//	NULL,
	//	NULL,
	//	SWP_NOOWNERZORDER | SWP_NOSIZE | SWP_SHOWWINDOW
	//	);

	// 前回からの経過時間
	elapsedTime = GetElapsedTime(lastTime);
	sec = (float)elapsedTime / 1000.0f;

	if ( sec > 0.0f ) {
		// カーソル速度計算
		fVelocity[0] = (float)(tagCurrentPos.x - g_tagLastCurPos.x) / sec * fCoefVelocity;
		fVelocity[1] = (float)(tagCurrentPos.y - g_tagLastCurPos.y) / sec * fCoefVelocity;

		// 速度制限
		if (fVelocity[0] < -fMaxVelocity)	fVelocity[0] = -fMaxVelocity;
		if (fVelocity[0] > fMaxVelocity)	fVelocity[0] = fMaxVelocity;
		if (fVelocity[1] < -fMaxVelocity)	fVelocity[1] = -fMaxVelocity;
		if (fVelocity[1] > fMaxVelocity)	fVelocity[1] = fMaxVelocity;

		// モデルの回転を反映
		Vector4 vec4, invModelRotQuat;
		invModelRotQuat.w = g_tagModelRotQuat.w;
		invModelRotQuat.x = -g_tagModelRotQuat.x;
		invModelRotQuat.y = -g_tagModelRotQuat.y;
		invModelRotQuat.z = -g_tagModelRotQuat.z;
		vec4.w = 0.0f;
		vec4.x = -fVelocity[0];
		vec4.y = fVelocity[1];
		vec4.z = 0.0f;
		QuaternionMultiply( &vec4, &invModelRotQuat, &vec4 );
		QuaternionMultiply( &vec4, &vec4, &g_tagModelRotQuat );

		// マウスによる速度を全剛体に加える
		g_clPMDModel.applyCentralImpulse( vec4.x, vec4.y, vec4.z );
	}

	g_tagLastCurPos = tagCurrentPos;
	lastTime = GetElapsedTime(0);
}

//--------
// 描画範囲のリサイズ
//--------
void UpdateWindowSize( HWND hWnd )
{
	static SIZE lastSize = {0};
	
	RECT rc;
	GetWindowRect(hWnd, &rc);
	if (lastSize.cx != (rc.right - rc.left) || lastSize.cy != (rc.bottom - rc.top)) {
		lastSize.cx = rc.right - rc.left;
		lastSize.cy = rc.bottom - rc.top;

		if (g_clGl) {
			// ビットマップのサイズを変更
			g_clGl->ResizeBitmap(lastSize.cx, lastSize.cy);

			// OpenGL 再初期化
			g_clGl->SetCurrent();
			//g_clGl->BeginRender();
			initGL();
			resize(lastSize.cx, lastSize.cy);
			//g_clGl->EndRender();
		}
	}
}

//--------
// 回転操作
//--------
void Rotate( HWND hWnd, float x, float y, float z, float angle )
{
	SIZE size = g_clGl->GetSize();
	Vector4 quat;
	Vector3 vec;
	vec.x = x;
	vec.y = y;
	vec.z = z;

	QuaternionCreateAxis(&quat, &vec, angle);
	RotateModel(&quat);

	g_clGl->BeginRender();
	resize((int)size.cx, (int)size.cy);
	g_clGl->EndRender();

	InvalidateRect(hWnd, NULL, TRUE);
}

//--------
// 並進操作
//--------
void Translate( HWND hWnd, float x, float y, float z )
{
	SIZE size = g_clGl->GetSize();
	Vector3 vec;
	vec.x = x;
	vec.y = y;
	vec.z = z;

	TranslateModel(vec);

	g_clGl->BeginRender();
	resize((int)size.cx, (int)size.cy);
	g_clGl->EndRender();

	InvalidateRect(hWnd, NULL, TRUE);
}

//--------
// 拡大縮小操作
//--------
void Scale( HWND hWnd, float scale )
{
	SIZE size = g_clGl->GetSize();

	g_fpScale = scale;

	g_clGl->BeginRender();
	resize((int)size.cx, (int)size.cy);
	g_clGl->EndRender();

	InvalidateRect(hWnd, NULL, TRUE);
}

//--------
// 配置をリセット
//--------
void ResetPosition( HWND hWnd )
{
	SIZE size = g_clGl->GetSize();

	ResetModelPosition();

	g_clGl->BeginRender();
	resize((int)size.cx, (int)size.cy);
	g_clGl->EndRender();

	InvalidateRect(hWnd, NULL, TRUE);
}

//--------
// 右クリックメニューを開く
// </sumamry>
void PopupMenu( HWND hWnd, LPARAM lParam )
{
	POINT mousepos;
	HMENU hMenu, hSubmenu;
	MENUITEMINFO info;

	// メニュー定義読込
	hMenu = LoadMenu((HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE), MAKEINTRESOURCE(IDC_MENU));
	hSubmenu = GetSubMenu(hMenu, 0);

	// チェックマークを設定するためのMENUITEMINFO準備
	info.cbSize = sizeof( info );
	info.fMask = MIIM_STATE;

	// カーソル目線モードのチェックマーク
	GetMenuItemInfo( hMenu, IDM_LOOKATME, FALSE, &info );
	info.fState = (g_bLookAtCusor ? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_LOOKATME, FALSE, &info );

	// 常に最前面のチェックマーク
	GetMenuItemInfo( hMenu, IDM_TOPMOST, FALSE, &info );
	info.fState = (g_bTopMost ? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_TOPMOST, FALSE, &info );

	// 物理演算のチェックマーク
	GetMenuItemInfo( hMenu, IDM_ENABLE_PHYSICS, FALSE, &info );
	info.fState = (g_bPhysics ? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_ENABLE_PHYSICS, FALSE, &info );

	// ポーズブレンドのチェックマーク
	GetMenuItemInfo( hMenu, IDM_POSEBLEND, FALSE, &info );
	info.fState = (g_bBlendPose ? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_POSEBLEND, FALSE, &info );

	// ウィンドウ移動のチェックマーク
	GetMenuItemInfo( hMenu, IDM_MOVINGWINDOW, FALSE, &info );
	info.fState = (g_bFollowModel? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_MOVINGWINDOW, FALSE, &info );

	// FPS表示のチェックマーク
	GetMenuItemInfo( hMenu, IDM_SHOW_FPS, FALSE, &info );
	info.fState = (g_bDispFPS ? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_SHOW_FPS, FALSE, &info );

	// FPS制限のチェックマーク
	UINT item = 0;
	if ( g_fpFrameSkip == 0.5f )		item = IDM_FPS_60;
	else if ( g_fpFrameSkip == 1.0f )	item = IDM_FPS_30;
	else if ( g_fpFrameSkip == 2.0f )	item = IDM_FPS_15;
	else if ( g_fpFrameSkip == 3.0f )	item = IDM_FPS_10;
	else if ( g_fpFrameSkip == 6.0f )	item = IDM_FPS_5;
	if (item != 0) {
		GetMenuItemInfo( hMenu, item, FALSE, &info );
		info.fState = MFS_CHECKED;
		SetMenuItemInfo( hMenu, item, FALSE, &info );
	}

	// メニューを開く座標を決める
	//// これはタスクバー右クリックではうまくいかなかった
	//mousepos.x = LOWORD(lParam);
	//mousepos.y = HIWORD(lParam);
	RECT rc;
	if (!GetWindowRect(hWnd, &rc)) return;
	if (!GetCursorPos(&mousepos)) return;
	mousepos.x -= rc.left;
	mousepos.y -= rc.top;
	ClientToScreen(hWnd, &mousepos);

	// メニューを開く
	TrackPopupMenu(hSubmenu, TPM_LEFTALIGN, mousepos.x, mousepos.y, 0, hWnd, NULL);

	DestroyMenu(hMenu);
}

//--------
// カーソル目線モード時に目線ベクトルを計算
//--------
void UpdateMousePosition( HWND hWnd )
{
	RECT rc;
	POINT pos;
	if (!GetWindowRect(hWnd, &rc)) return;
	if (!GetCursorPos(&pos)) return;

	Vector4 vecGm, vecRot, vecRotCon;
	vecGm.w = 0;
	vecGm.z = 20.0f;		// モデルに対するカーソルのz座標

	// 画面の座標から MMD_FRUSTUM_NEAR 位置での座標へ
	vecGm.x = ((float)(pos.x - rc.left) / (float)(rc.right - rc.left) - 0.5f) * 2.0f;
	vecGm.y = -((float)(pos.y - rc.top) / (float)(rc.bottom - rc.top) - 0.5f) * 2.0f;
	vecGm.y *= (float)(rc.bottom - rc.top) / (float)(rc.right - rc.left);	// ウィンドウ縦横比補正

	// カメラパースの逆反映
	vecGm.x = (float)(vecGm.x * (-vecGm.z - MMD_CAMERA_Z) / MMD_FRUSTUM_NEAR);
	vecGm.y = (float)(vecGm.y * (-vecGm.z - MMD_CAMERA_Z) / MMD_FRUSTUM_NEAR);

	// 拡大率の逆反映
	vecGm.x /= g_fpScale;
	vecGm.y /= g_fpScale;
	vecGm.z /= g_fpScale;

	// 並進の逆反映
	vecGm.x -= g_tagModelPosVec.x;
	vecGm.y -= g_tagModelPosVec.y;
	vecGm.z += g_tagModelPosVec.z;

	// 回転の逆反映
	vecRot = g_tagModelRotQuat;
	vecRotCon.w = vecRot.w = -vecRot.w;					// グローバル座標からモデル座標にするため、逆回転とする
	vecRotCon.x = -vecRot.x;
	vecRotCon.y = -vecRot.y;
	vecRotCon.z = -vecRot.z;
	QuaternionMultiply(&vecGm, &vecRotCon, &vecGm);		// クォータニオンによる回転
	QuaternionMultiply(&vecGm, &vecGm, &vecRot);		// クォータニオンによる回転
	
	vecGm.y -= (float)MMD_MODEL_Y;						// デフォルトの座標引き上げ

	if (g_bFollowModel)									// ウィンドウ移動モードなら、センター位置を補正
	{
		Vector3 vecPos;
		g_clPMDModel.getModelPosition(&vecPos);
		vecGm.x += vecPos.x;
		vecGm.y += vecPos.y;
		vecGm.z -= vecPos.z;
	}

	g_tagFaceVec.x = vecGm.x;
	g_tagFaceVec.y = vecGm.y;
	g_tagFaceVec.z = vecGm.z;

}

//--------
// 最前面に表示を切り替え
//--------
void ToggleTopMost( HWND hWnd )
{
	g_bTopMost = !g_bTopMost;
	if (g_bTopMost) {
		SetWindowPos(hWnd, HWND_TOPMOST, NULL, NULL, NULL, NULL, SWP_NOMOVE | SWP_NOSIZE);
	} else {
		SetWindowPos(hWnd, HWND_NOTOPMOST, NULL, NULL, NULL, NULL, SWP_NOMOVE | SWP_NOSIZE);
	}
}

//--------
// モデルのセンターに合わせてウィンドウを移動
//--------
void LocateWindow( HWND hWnd )
{
	Vector3 vec3ModelPos, vec3Pos;
	g_clPMDModel.getModelPosition( &vec3ModelPos );				// モデル座標取得
	vec3ModelPos.z = -vec3ModelPos.z;							// PMDと座標系が違う
	Vector3Transform( &vec3Pos, &vec3ModelPos, g_mTransform );	// 座標変換

	// ウィンドウ位置更新
	int x = (int)(vec3Pos.x * 2.0f) + g_tagWindowOffset.x;		// glFrustum() で-1から1としているため、×2.0する
	int y = (int)(-vec3Pos.y * 2.0f) + g_tagWindowOffset.y;

	//// はみ出しチェック（マルチモニタ用にMonitorFromWindow()を…と思ったがうまくいかなかった）
	//if ( !MonitorFromWindow( hWnd, MONITOR_DEFAULTTONULL ) )
	//{
	//	// はみ出していたら強制的にプライマリモニタに納める
	//RECT rect;
	//GetWindowRect( hWnd, &rect );
	//int w = rect.right - rect.left;
	//int h = rect.bottom - rect.top;
	//int sw = GetSystemMetrics( SM_CXSCREEN );
	//int sh = GetSystemMetrics( SM_CYSCREEN );
	//if (x <= -w) x = 1 - w;
	//if (y <= -h) y = 1 - h;
	//if (x > sw) x = sw - 1;
	//if (y > sh) y = sh - 1;
	//}

	SetWindowPos(
		hWnd,
		NULL,
		x,
		y,
		NULL,
		NULL,
		SWP_NOOWNERZORDER | SWP_NOSIZE | SWP_SHOWWINDOW
		);
}

//--------
// ドラッグアンドドロップされたファイルを開く
//--------
void OpenDraggedFiles( HWND hWnd, HDROP hDrop )
{
	WCHAR wszFileName[MAX_PATH];
	WCHAR wszPMDFileName[MAX_PATH];
	WCHAR wszVMDFileName[MAX_PATH];
	WCHAR wszVPDFileName[MAX_PATH];
	WCHAR wszBGMFileName[MAX_PATH];
	WCHAR wszExt[5];
	bool bIsPMDFound = false;
	bool bIsVMDFound = false;
	bool bIsVPDFound = false;
	bool bIsBGMFound = false;

	// ドロップされたファイル数を調べる
	UINT nFiles = DragQueryFile(hDrop, 0xFFFFFFFF, NULL, NULL);

	// 対応ファイルがあるかどうか調べる
	//   複数同種のファイルがあった場合は最後の物のみ有効
	for (UINT i= 0; i < nFiles; i++) {
		// ファイル名取得
		DragQueryFile(hDrop, i, wszFileName, MAX_PATH);

		// 拡張子を取得
		GetFileExtension(wszExt, wszFileName);

		if (_wcsnicmp(wszExt, L".pmd", 4) == 0) {
			// PMDファイルがあった
			wcscpy_s(wszPMDFileName, MAX_PATH, wszFileName);
			bIsPMDFound = true;
		} else if (_wcsnicmp(wszExt, L".vmd", 4) == 0) {
			// VMDファイルがあった
			wcscpy_s(wszVMDFileName, wszFileName);
			bIsVMDFound = true;
		} else if (_wcsnicmp(wszExt, L".vpd", 4) == 0) {
			// VPDファイルがあった
			wcscpy_s(wszVPDFileName, wszFileName);
			bIsVPDFound = true;
		} else if ((_wcsnicmp(wszExt, L".mp3", 4) == 0)
			|| (_wcsnicmp(wszExt, L".wav", 4) == 0)
			|| (_wcsnicmp(wszExt, L".wma", 4) == 0)
			) {
			// 音楽ファイルがあった
			wcscpy_s(wszBGMFileName, MAX_PATH, wszFileName);
			bIsBGMFound = true;
		}
	}

	// PMDファイルがドラッグされていれば開く
	if (bIsPMDFound) {
		if (openNewModel(wszPMDFileName)) {
			ResetTime();
		}
	}

	// VPDファイルがドラッグされていれば開く
	if (bIsVPDFound) {
		if (openNewMotion(wszVPDFileName)) {
			ResetTime();
		}
	}

	// VMDファイルがドラッグされていれば開く
	if (bIsVMDFound) {
		if (openNewMotion(wszVMDFileName)) {
			ResetTime();
		}
	}

	// BGMファイルがドラッグされていれば開く
	if (bIsBGMFound) {
		if (OpenBGM(wszBGMFileName)) {
			g_clPMDModel.setLoop(FALSE);		// BGM再生させるとループ解除する

			//// 巻き戻しは再生のMM_MCINOTIFYを検知したときにするため，ここでしない
			//g_clPMDModel.rewind();
			//ResetTime();
		}
	}

	// ドロップの解放
	DragFinish(hDrop);
}

//--------
// コマンドライン引数解釈
//--------
// <return>モデルが開かれたら TRUE を返す</return>
bool OpenCommandLineFiles( LPTSTR lpCmdLine )
{
	LPWSTR *lpArgv;
	int nArgc;
	LPWSTR wszFileName;
	WCHAR wszPMDFileName[MAX_PATH];
	WCHAR wszVMDFileName[MAX_PATH];
	WCHAR wszVPDFileName[MAX_PATH];
	WCHAR wszBGMFileName[MAX_PATH];
	WCHAR wszExt[5];
	bool bIsPMDFound = false;
	bool bIsVMDFound = false;
	bool bIsVPDFound = false;
	bool bIsBGMFound = false;

	// コマンドライン引数解析
	lpArgv = CommandLineToArgvW(lpCmdLine, &nArgc);
	if (lpArgv == NULL) return FALSE;

	// 対応ファイルがあるかどうか調べる
	//   複数同種のファイルがあった場合は最後の物のみ有効
	for (int i = 1; i < nArgc; i++) {
		wszFileName = lpArgv[i];

		// コマンドラインパラメータの解釈
		if (wszFileName[0] == L'/' || wszFileName[0] == '-') {
			switch (wszFileName[1]) {
				case 'F':
				case 'f':
					// "-F1"でFPS表示をON、"-F0"でOFF（デフォルトはOFF）
					g_bDispFPS = (wszFileName[2] == '1');
					break;

				case 'T':
				case 't':
					// "-T1"で最前面をON、"-T0"でOFF（デフォルトはOFF）
					g_bTopMost = (wszFileName[2] == '1');
					break;

				case 'B':
				case 'b':
					// "-B1"で物理演算をON、"-B0"でOFF（デフォルトはOFF）
					g_bPhysics = (wszFileName[2] == '1');
					break;

				case 'L':
				case 'l':
					// "-L1"でカーソル目線ON、"-L0"でOFF（デフォルトはOFF）
					g_bLookAtCusor = (wszFileName[2] == '1');
					break;

				case 'W':
				case 'w':
					// "-W1"でウィンドウ自動移動ON、"-W0"でOFF（デフォルトはON）
					g_bFollowModel = (wszFileName[2] == '1');
					break;
			}

			continue;	// '/'または'-'で始まる場合はファイル名と解釈させない
		}

		// 拡張子を取得
		GetFileExtension(wszExt, wszFileName);

		if (_wcsnicmp(wszExt, L".pmd", 4) == 0) {
			// PMDファイルがあった
			wcscpy_s(wszPMDFileName, MAX_PATH, wszFileName);
			bIsPMDFound = true;
		} else if (_wcsnicmp(wszExt, L".vmd", 4) == 0) {
			// VMDファイルがあった
			wcscpy_s(wszVMDFileName, wszFileName);
			bIsVMDFound = true;
		} else if (_wcsnicmp(wszExt, L".vpd", 4) == 0) {
			// VPDファイルがあった
			wcscpy_s(wszVPDFileName, wszFileName);
			bIsVPDFound = true;
		} else if ((_wcsnicmp(wszExt, L".mp3", 4) == 0)
			|| (_wcsnicmp(wszExt, L".wav", 4) == 0)
			|| (_wcsnicmp(wszExt, L".wma", 4) == 0)
			) {
			// 音楽ファイルがあった
			wcscpy_s(wszBGMFileName, MAX_PATH, wszFileName);
			bIsBGMFound = true;
		}
	}
	// PMDファイルがドラッグされていれば開く
	if (bIsPMDFound) {
		if (openNewModel(wszPMDFileName)) {
			ResetTime();
		}
	}

	// VPDファイルがドラッグされていれば開く
	if (bIsVPDFound) {
		if (openNewMotion(wszVPDFileName)) {
			ResetTime();
		}
	}

	// VMDファイルがドラッグされていれば開く
	if (bIsVMDFound) {
		if (openNewMotion(wszVMDFileName)) {
			ResetTime();
		}
	}

	// BGMファイルがドラッグされていれば開く
	if (bIsBGMFound) {
		if (OpenBGM(wszBGMFileName)) {
			g_clPMDModel.setLoop(FALSE);		// BGM再生させるとループ解除する
		}
	}

	LocalFree(lpArgv);

	return bIsPMDFound;
}

//--------
// キャンバスサイズ選択
//--------
void ResizeWindow( HWND hWnd, int size )
{
	if (size == NULL) {
		// size == NULL だとフルスクリーンサイズ
		//  ついでにウィンドウ位置を調整
		HWND hDesktop = GetDesktopWindow();
		RECT rc;
		GetWindowRect(hDesktop, &rc);
		SetWindowPos(
			hWnd, 
			NULL,
			0,
			0,
			rc.right,
			rc.bottom,
			SWP_NOZORDER
			);
	} else {
		// size == NULL だとフルスクリーンサイズ
		SetWindowPos(
			hWnd, 
			NULL,
			NULL,
			NULL,
			size,
			size,
			SWP_NOZORDER | SWP_NOMOVE
			);
	}
	UpdateWindowSize(hWnd);
}

//--------
// dwStartTime からの時間をミリ秒で返す
//--------
DWORD GetElapsedTime( DWORD dwStartTime )
{
	DWORD	time = GetTickCount();
	DWORD	interval;

	if ( dwStartTime > time ) {
		// 本プログラム使用中に GetTickCount() が1周した場合(システム起動から約49日経過)
		//   2周以降はダメなので、そんなに長く起動しないでね。
		interval = (DWORD)(0x100000000 + time - dwStartTime);
	} else {
		interval = time - dwStartTime;
	}

	return interval;
}

//--------
// モーション開始時刻をリセット
//--------
void ResetTime()
{
	g_dwStartTime = GetTickCount();		// 再生開始時刻リセット
	g_fpLastDispFrame = 0.0f;			// 描画フレーム番号リセット
	g_fpLastCalcFrame = 0.0f;			// モーション更新フレーム番号リセット
}

//-------------------
// FPSの計測
//-------------------
void calcFps()
{
	static DWORD	startTime = GetElapsedTime( 0 ),
					elapsedTime = 0;
	static int		iCnt = 0;
	static float	fFps = 0.0f;

	elapsedTime = GetElapsedTime( startTime );
	if( elapsedTime >= 1000 )
	{
		g_fpFps = (1000.0f * iCnt) / (float)(elapsedTime);
		startTime += elapsedTime;
		iCnt = 0;
	}
	iCnt++; 
}

//--------
// 終了時の処理
//--------
void Release()
{
	g_clVMDMotion.release();
	g_clPMDModel.release();
	g_clBulletPhysics.release();

	if (g_clGl) {
		free(g_clGl);
	}

	CloseBGM();

	// GDI+ 終了時処理
	Gdiplus::GdiplusShutdown( pGdiToken );
}

//--------
// ファイルパスをエスケープし、ダブルクォーテーションで囲む
//--------
void EscapeFilePath( LPWSTR wszDest, LPCWSTR wszSrc )
{
	int nLen = wcslen(wszSrc);
	int i, j;

	// ダブルクォーテーションがついていなければ、付ける
	if (wszSrc[0] != L'"') {
		wcscpy_s(wszDest, 2, L"\"");
		j = 1;		// wszDest のインデックス。ダブルクォーテーションの分で1から開始
	} else {
		wcscpy_s(wszDest, 1, L"");
	}
	
	// エスケープしながらパスの内容をコピー
	for (i = 0; i < nLen; i++) {
		if (wszSrc[i] == L'\\') {
			wszDest[j++] = L'\\';
		}
		wszDest[j++] = wszSrc[i];
	}

	// 最後にダブルクォーテーションがついていなければ、付ける
	if (wszSrc[i] != L'"') {
		wszDest[j++] = L'"';
	}

	// 終端
	wszDest[j++] = '\0';
}

//--------
// BGMの停止とクローズ
//--------
void CloseBGM()
{
	if (g_bBGM) {
		MCIERROR res;
		res = mciSendString(L"stop bgm", NULL, 0, NULL);
		res = mciSendString(L"close bgm", NULL, 0, NULL);
		g_bBGM = FALSE;
	}
}

//--------
// BGMを読込
//--------
bool OpenBGM( LPWSTR wszFileName )
{
	WCHAR wszPath[394];
	WCHAR wszCommand[512];
	MCIERROR res;

	// ファイルパスのエスケープ
	EscapeFilePath(wszPath, wszFileName);

	// （すでに開かれていたら）BGMをクローズ
	CloseBGM();

	// BGMを開く
	wcscpy_s(wszCommand, L"open ");
	wcscat_s(wszCommand, 394, wszPath);
	wcscat_s(wszCommand, L" alias bgm");
	res = mciSendString(wszCommand, NULL, 0, NULL);
	if (res != 0) return FALSE;

	// 再生
	//res = mciSendString(L"play bgm notify", NULL, 0, g_hWnd);
	res = mciSendString(L"play bgm", NULL, 0, g_hWnd);
	if (res != 0) return FALSE;

	// 再生したらモーションを先頭に巻き戻し
	g_clPMDModel.rewind();
	ResetTime();

	g_bBGM = TRUE;

	return TRUE;
}
