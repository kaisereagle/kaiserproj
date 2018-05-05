// main.cpp : アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "main.h"

#define WINDOW_WIDTH  640
#define WINDOW_HEIGHT 480

extern int WGLCreate(HWND window);
extern void WGLDestroy();
extern int Initialize(int width, int height);
extern void DrawScene(int width, int height);

HWND ghWnd;

#define MAX_LOADSTRING 100

// グローバル変数:
HINSTANCE hInst;								// 現在のインターフェイス
TCHAR szTitle[MAX_LOADSTRING];					// タイトル バーのテキスト
TCHAR szWindowClass[MAX_LOADSTRING];			// メイン ウィンドウ クラス名

// このコード モジュールに含まれる関数の宣言を転送します:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY _tWinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPTSTR    lpCmdLine,
                     int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

 	// TODO: ここにコードを挿入してください。
	MSG msg;
	HACCEL hAccelTable;

	// グローバル文字列を初期化しています。
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_LESSON1_1, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// アプリケーションの初期化を実行します:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_LESSON1_1));

	// メイン メッセージ ループ:
	while(1) {
		while(PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE) == TRUE) {
			if (GetMessage(&msg, NULL, 0, 0))
			{
				if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
				{
					TranslateMessage(&msg);
					DispatchMessage(&msg);
				}
			} else {
				return (int) msg.wParam;
			}
		}
		DrawScene(WINDOW_WIDTH, WINDOW_HEIGHT);
		InvalidateRect(ghWnd, NULL, FALSE);
	}

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
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_LESSON1_1));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_LESSON1_1);
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

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
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	hInst = hInstance; // グローバル変数にインスタンス処理を格納します。

	ghWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, CW_USEDEFAULT, WINDOW_WIDTH, WINDOW_HEIGHT, NULL, NULL, hInstance, NULL);

	if (!ghWnd) {
		return FALSE;
	}

	{
		RECT	rc;
		DWORD	dwStyle;
		int		xp;
		int		yp;
		int		menuFlag;

		/* change window style to popup */
		dwStyle = GetWindowLong(ghWnd, GWL_STYLE);

		GetWindowRect(ghWnd, &rc);
		xp = rc.left;
		yp = rc.top;

		menuFlag	= GetMenu(ghWnd) != NULL;
		rc.top		= rc.left = 0;
		rc.right	= WINDOW_WIDTH;
		rc.bottom	= WINDOW_HEIGHT;
		AdjustWindowRect(&rc, dwStyle, menuFlag);

		SetWindowPos(ghWnd, HWND_NOTOPMOST, xp, yp, rc.right - rc.left, rc.bottom - rc.top,
					 SWP_NOZORDER | SWP_NOACTIVATE);
	}

	ShowWindow(ghWnd, nCmdShow);
	UpdateWindow(ghWnd);

    if (!WGLCreate(ghWnd)) {
		return FALSE;
	}
	if (!Initialize(WINDOW_WIDTH, WINDOW_HEIGHT)) {
		return FALSE;
	}

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
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	PAINTSTRUCT ps;
	HDC hdc;

	switch (message)
	{
	case WM_COMMAND:
		wmId    = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// 選択されたメニューの解析:
		switch (wmId)
		{
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;
	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
		// TODO: 描画コードをここに追加してください...
		EndPaint(hWnd, &ps);
		break;
	case WM_SIZE:
		{
			RECT	rc;
			DWORD	dwStyle;
			int		xp;
			int		yp;
			int		menuFlag;

			/* change window style to popup */
			dwStyle = GetWindowLong(ghWnd, GWL_STYLE);

			GetWindowRect(ghWnd, &rc);
			xp = rc.left;
			yp = rc.top;

			menuFlag	= GetMenu(ghWnd) != NULL;
			rc.top		= rc.left = 0;
			rc.right	= WINDOW_WIDTH;
			rc.bottom	= WINDOW_HEIGHT;
			AdjustWindowRect(&rc, dwStyle, menuFlag);

			SetWindowPos(ghWnd, HWND_NOTOPMOST, xp, yp, rc.right - rc.left, rc.bottom - rc.top,
						 SWP_NOZORDER | SWP_NOACTIVATE);
		}
		break;
	case WM_ERASEBKGND:
		return TRUE;
	case WM_DESTROY:
		WGLDestroy();
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// バージョン情報ボックスのメッセージ ハンドラです。
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
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


#if defined(_DEBUG) || defined(DEBUG)

void TRACE(LPCSTR pszFormat, ...)
{
	va_list	argp;
    int len;
	char *pszBuf;

	va_start(argp, pszFormat);
    len = _vscprintf(pszFormat, argp) + 1;
    if ((pszBuf = (char *)malloc(sizeof(char) * len)) != NULL) {
	    vsprintf_s(pszBuf, len, pszFormat, argp);
	    va_end(argp);
	    OutputDebugStringA(pszBuf);
        free(pszBuf);
    }
    else {
	    va_end(argp);
        OutputDebugStringA("ERROR: out of memory.\n");
    }
}

#endif