// main.cpp : �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
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

// �O���[�o���ϐ�:
HINSTANCE hInst;								// ���݂̃C���^�[�t�F�C�X
TCHAR szTitle[MAX_LOADSTRING];					// �^�C�g�� �o�[�̃e�L�X�g
TCHAR szWindowClass[MAX_LOADSTRING];			// ���C�� �E�B���h�E �N���X��

// ���̃R�[�h ���W���[���Ɋ܂܂��֐��̐錾��]�����܂�:
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

 	// TODO: �����ɃR�[�h��}�����Ă��������B
	MSG msg;
	HACCEL hAccelTable;

	// �O���[�o������������������Ă��܂��B
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_LESSON1_1, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// �A�v���P�[�V�����̏����������s���܂�:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_LESSON1_1));

	// ���C�� ���b�Z�[�W ���[�v:
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
//  �֐�: MyRegisterClass()
//
//  �ړI: �E�B���h�E �N���X��o�^���܂��B
//
//  �R�����g:
//
//    ���̊֐�����юg�����́A'RegisterClassEx' �֐����ǉ����ꂽ
//    Windows 95 ���O�� Win32 �V�X�e���ƌ݊�������ꍇ�ɂ̂ݕK�v�ł��B
//    �A�v���P�[�V�������A�֘A�t����ꂽ
//    �������`���̏������A�C�R�����擾�ł���悤�ɂ���ɂ́A
//    ���̊֐����Ăяo���Ă��������B
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
//   �֐�: InitInstance(HINSTANCE, int)
//
//   �ړI: �C���X�^���X �n���h����ۑ����āA���C�� �E�B���h�E���쐬���܂��B
//
//   �R�����g:
//
//        ���̊֐��ŁA�O���[�o���ϐ��ŃC���X�^���X �n���h����ۑ����A
//        ���C�� �v���O���� �E�B���h�E���쐬����ѕ\�����܂��B
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	hInst = hInstance; // �O���[�o���ϐ��ɃC���X�^���X�������i�[���܂��B

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
//  �֐�: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  �ړI:  ���C�� �E�B���h�E�̃��b�Z�[�W���������܂��B
//
//  WM_COMMAND	- �A�v���P�[�V���� ���j���[�̏���
//  WM_PAINT	- ���C�� �E�B���h�E�̕`��
//  WM_DESTROY	- ���~���b�Z�[�W��\�����Ė߂�
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
		// �I�����ꂽ���j���[�̉��:
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
		// TODO: �`��R�[�h�������ɒǉ����Ă�������...
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

// �o�[�W�������{�b�N�X�̃��b�Z�[�W �n���h���ł��B
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