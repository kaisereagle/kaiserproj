// ---------------------------------------------------------------------------
//
// MMD_DesktopMascot
//   PMD, VMD, VPD ���f�X�N�g�b�v�}�X�R�b�g���ɕ\�����܂�
//
// Copyright (c) 2009-2011  Ru--en
// 
// 
// 2011/04/30	Ver.0.5.1	PNG,JPEG���̃e�N�X�`����GDI+�őΉ�
// 2011/03/27	Ver.0.5.0	�G�b�W�����w��ގ��̐F�v�Z���C���i���Lat���΍�j
// 2010/01/20	Ver.0.4.9	�E�B���h�E�ړ����s�������C��
// 2010/01/15	Ver.0.4.8	�Z���^�[�{�[���ɍ��킹���E�B���h�E�ړ������A���j���[�Q�J���ꉻ
// 2010/01/07	Ver.0.4.7	�֊s�E�e�Ȃ��ގ��ɑΉ��A�Ǎ��G���[�C��
// 2010/01/06	Ver.0.4.6	�A�X�^���X�N��؂�̃X�t�B�A�}�b�v�w��ɕ����Ή�
// 2009/10/18	Ver.0.4.5	Bullet 2.75 �g�p�A�R�}���h���C�������Ή��C��
// 2009/08/31	Ver.0.4.4	ARTK_MMD v0.7 �捞�i�X�t�B�A�}�b�v�Ή��j
// 2009/08/30	Ver.0.4.3	���j���[�Ƀ`�F�b�N��\��
// 2009/08/07	Ver.0.4.2	���[�V���������60fps�Ōv�Z�AFPS�\���ǉ�
// 2009/08/03	Ver.0.4.1	�������Z�̂��߃t���[�����[�g�C��
// 2009/08/02	Ver.0.4.0	ARTK_MMD v0.5 �捞�i�������Z�Ή��j
// 2009/07/15	Ver.0.3.7	�t�@�C���h���b�v�Ή��ABGM�Đ��A
//							�^�X�N�o�[�E�N���b�N�Ή��A�R�}���h���C���Ή�
// 2009/07/15	Ver.0.3.6	�T�C�Y�ύX�C���A�t�@�C�����̃��C�h�������A
//							���j���[�p�ꉻ�A�}�E�X�ڐ��C��
// 2009/07/14	Ver.0.3.5	�Đ����x�C���A�L�����o�X�T�C�Y�ύX�ǉ�
// 2009/07/09	Ver.0.3.4	�e�N�X�`���Ǎ��C��
// 2009/07/08	Ver.0.3.3	FPS������10FPS���@�\���Ȃ������̂ŏC��
// 2009/07/08	Ver.0.3.2	FPS�����ǉ��A�}�E�X�ڐ��ǉ�
// 2009/07/01	Ver.0.3.1	�\�[�X���J�p�ɐ����A��]/�ړ��@�\�ǉ�
// 2009/06/30	Ver.0.3.0	������J��
// 2009/06/29	Ver.0.2.0	���֌��J��
// 2009/06/29	Ver.0.1.0	�R���Z�v�g��
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

//#define MMDDM_WINDOW			1				// �L���ɂ���ƃE�B���h�E�g�\�����[�h(�f�o�b�O�p)

// �萔
#define MMDDM_MAX_LOADSTRING	100
#define MMDDM_DEFAULT_WIDTH		400				// �����T�C�Y ����[pixel]
#define MMDDM_DEFAULT_HEIGHT	400				// �����T�C�Y ����[pixel]
#define MMDDM_WAIT				1				// ���[�v�����̃E�F�C�g[msec]
#define MMDDM_ROTATION_STEP		0.08726646256f	// ��]����P��[rad] �i0.087rad �� 5�x�j
#define MMDDM_MOVE_STEP			0.5f			// ���i����P��
#define MMDDM_MOTION_SPEED		1.0f			// �A�j���[�V�������x����[�{]
#define MMDDM_TIMER_ID			1				// SetTimer()�ɓn��ID

// �O���[�o���ϐ�:
HINSTANCE	g_hInst = NULL;						// ���݂̃C���^�[�t�F�C�X
HWND		g_hWnd = NULL;						// ���݂̃E�B���h�E�n���h��
TCHAR	szTitle[MMDDM_MAX_LOADSTRING];			// �^�C�g�� �o�[�̃e�L�X�g
TCHAR	szWindowClass[MMDDM_MAX_LOADSTRING];	// ���C�� �E�B���h�E �N���X��

OffscreenGL	*g_clGl = NULL;						// �I�t�X�N���[��OpenGL�N���X�̃C���X�^���X
bool		g_bTopMost	= FALSE;				// �őO�ʂɕ\�����邩
float		g_fpFrameSkip = 0.5f;				// �\��FPS�����D�Ⴆ��2�ɂ����15fps�ɂȂ�
float		g_fpLastDispFrame = 0.0f;			// �Ō�ɕ\�������t���[���ԍ�
float		g_fpLastCalcFrame = 0.0f;			// �Ō�ɍX�V���ꂽ���[�V�����̃t���[���ԍ�
DWORD		g_dwStartTime = 0;					// ���[�V�����Đ��J�n���� GetTickCount() �l
bool		g_bBGM		= FALSE;				// BGM�𗬂�����
POINT		g_tagLastCurPos;					// ���O�̃}�E�X�J�[�\�����W
POINT		g_tagWindowOffset;					// ��ʒ�������E�B���h�E���ړ����ꂽ��

ULONG_PTR pGdiToken;
Gdiplus::GdiplusStartupInput gdiInput;



//--------
// ���C��
//--------
int APIENTRY _tWinMain( HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPTSTR    lpCmdLine,
                     int       nCmdShow )
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

 	// TODO: �����ɃR�[�h��}�����Ă��������B
	MSG msg;
	HACCEL hAccelTable;

	// �O���[�o������������������Ă��܂��B
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MMDDM_MAX_LOADSTRING);
	LoadString(hInstance, IDS_CLASSNAME, szWindowClass, MMDDM_MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// �A�v���P�[�V�����̏����������s���܂�:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}
	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_MENU));

	// ���C�� ���b�Z�[�W ���[�v:
	while (GetMessage(&msg, NULL, 0, 0)) {
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	// �I�����̉������
	Release();

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
//   �֐�: InitInstance(HINSTANCE, int)
//
//   �ړI: �C���X�^���X �n���h����ۑ����āA���C�� �E�B���h�E���쐬���܂��B
//
//   �R�����g:
//
//        ���̊֐��ŁA�O���[�o���ϐ��ŃC���X�^���X �n���h����ۑ����A
//        ���C�� �v���O���� �E�B���h�E���쐬����ѕ\�����܂��B
//
BOOL InitInstance( HINSTANCE hInstance, int nCmdShow )
{
	g_hInst = hInstance; // �O���[�o���ϐ��ɃC���X�^���X�������i�[���܂��B

	// ��ʒ����ɃE�B���h�E�����킹��
	g_tagWindowOffset.x = (GetSystemMetrics( SM_CXSCREEN ) - MMDDM_DEFAULT_WIDTH) / 2;
	g_tagWindowOffset.y = (GetSystemMetrics( SM_CYSCREEN ) - MMDDM_DEFAULT_HEIGHT) / 2;

	g_hWnd = CreateWindowEx(
#ifdef MMDDM_WINDOW
		(g_bTopMost ? WS_EX_TOPMOST : 0),					// ���C���[�h�E�B���h�E�ɂ��Ȃ�
#else
		WS_EX_LAYERED | (g_bTopMost ? WS_EX_TOPMOST : 0),	// ���C���[�h�E�B���h�E�ɂ���
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

	// �h���b�O�A���h�h���b�v�L��
	DragAcceptFiles(g_hWnd, TRUE);

    // ���݂̃E�B���h�E�T�C�Y���L��
	UpdateWindowSize(g_hWnd);

	// �������̃e�X�g
	//   ������g�����ꍇ�A�J���[�L�[�͎g�p�ł��Ȃ��݂���
	//SetLayeredWindowAttributes(g_hWnd, 0, 0x8f, ULW_ALPHA);

	// �r�b�g�}�b�v��p��
	g_clGl = new OffscreenGL(MMDDM_DEFAULT_WIDTH, MMDDM_DEFAULT_HEIGHT);
	g_clGl->SetCurrent();

	// OpenGL �����ݒ�
	init();
	resize(MMDDM_DEFAULT_WIDTH, MMDDM_DEFAULT_HEIGHT);

	// GDI+ ������
	Gdiplus::GdiplusStartup( &pGdiToken, &gdiInput, NULL );

	// �R�}���h���C������������΂��̃t�@�C�����J��
	LPTSTR lpCmdLine = GetCommandLine();
	if (!OpenCommandLineFiles(lpCmdLine)) {
		// �f�t�H���g�f�[�^�����Ǎ�
		if (!openNewModel( L"default.pmd" )) {
			if (!openNewModel( g_hWnd )) {
				// �f�t�H���g�̃��f�����ǂݍ��߂��C�w��t�@�C�����J���Ȃ���΋����I��
				return FALSE;
			}
		}
		openNewMotion( L"default.vmd" );
		g_clPMDModel.setMotion( &g_clVMDMotion, true );
		g_clPMDModel.updateMotion( 0 );
		g_clPMDModel.updateSkinning();
	} else {
		// �R�}���h���C�������ōőO�ʕ\�����ς�����̂ŁA�Đݒ�
		SetWindowPos(g_hWnd, (g_bTopMost ? HWND_TOPMOST : HWND_NOTOPMOST), NULL, NULL, NULL, NULL, SWP_NOMOVE | SWP_NOSIZE);
	}

	ResetTime();			// ���[�V�����Đ��������Z�b�g
	ResetCursor();			// �}�E�X�J�[�\���ʒu���Z�b�g

	ShowWindow(g_hWnd, nCmdShow);
	UpdateWindow(g_hWnd);
	LocateWindow(g_hWnd);	// �E�B���h�E�ʒu����

	// �`��X�V�̂��߂̃^�C�}�[�J�n
	SetTimer( g_hWnd, MMDDM_TIMER_ID, MMDDM_WAIT, NULL );

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
LRESULT CALLBACK WndProc( HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam )
{
	int wmId, wmEvent;
	//PAINTSTRUCT ps;
	//HDC hdc;

	switch (message)
	{
	case WM_COMMAND:
		KillTimer( hWnd, MMDDM_TIMER_ID );			// ���f���X�V�����𒆒f���邽�߃^�C�}�[��؂�
		wmId    = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// �I�����ꂽ���j���[�̉��:
		switch (wmId)
		{
		case IDM_POPUP_MENU:						// �L�[�{�[�h�̉E�N���b�N���j���[�{�^���Ń|�b�v�A�b�v���J��
			PopupMenu(hWnd, lParam);
			break;
		case IDM_ABOUT:
			DialogBox(g_hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_OPEN_MODEL:						// ���f��(.pmd)���J��
			if (openNewModel( hWnd )) {
				ResetTime();
			}
			break;
		case IDM_OPEN_MOTION:						// ���[�V����(.vmd/.vpd)���J��
			if (openNewMotion( hWnd )) {
				ResetTime();
			}
			break;
		case IDM_LOOKATME:							// �}�E�X�ڐ�
			g_bLookAtCusor = !g_bLookAtCusor;
			if (g_bLookAtCusor) {
				// �}�E�X�ڐ����[�hOn�Ȃ王���x�N�g�����X�V���Ă���
				UpdateMousePosition(hWnd);
			}
			g_clPMDModel.setLookAtFlag(g_bLookAtCusor);
			break;
		case IDM_ENABLE_PHYSICS:					// �������Z On/Off
			g_bPhysics = !g_bPhysics;
			g_clPMDModel.enablePhysics( g_bPhysics );
			g_clPMDModel.resetRigidBodyPos();
			break;
		case IDM_SHOW_FPS:							// �}�E�X�ڐ� On/Off
			g_bDispFPS = !g_bDispFPS;
			break;
		case IDM_POSEBLEND:							// VPD�Ǎ����̒ǉ����[�h On/Off
			g_bBlendPose = !g_bBlendPose;
			break;
		case IDM_MOVINGWINDOW:						// ���f���̃Z���^�[�ɍ��킹�ăE�B���h�E�ړ�
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
			Scale(hWnd, g_fpScale * 1.2f);							// 120%�Ɋg��
			break;
		case IDM_SCALE_DOWN:
			Scale(hWnd, g_fpScale * 0.8f);							// 80%�ɏk��
			break;
		case IDM_ROT_LEFT:
			Rotate(hWnd, 0.0f, 1.0f, 0.0f, MMDDM_ROTATION_STEP);	// ���v���5�x��]
			break;
		case IDM_ROT_RIGHT:
			Rotate(hWnd, 0.0f, 1.0f, 0.0f, -MMDDM_ROTATION_STEP);	// �����v���5�x��]
			break;
		case IDM_ROT_UP:
			Rotate(hWnd, 1.0f, 0.0f, 0.0f, MMDDM_ROTATION_STEP);	// �����5�x��]
			break;
		case IDM_ROT_DOWN:
			Rotate(hWnd, 1.0f, 0.0f, 0.0f, -MMDDM_ROTATION_STEP);	// ������5�x��]
			break;
		case IDM_MOV_LEFT:
			Translate(hWnd, -MMDDM_MOVE_STEP, 0.0f, 0.0f);			// �������č��Ɉړ�
			break;
		case IDM_MOV_RIGHT:
			Translate(hWnd, MMDDM_MOVE_STEP, 0.0f, 0.0f);			// �������ĉE�Ɉړ�
			break;
		case IDM_MOV_UP:
			Translate(hWnd, 0.0f, MMDDM_MOVE_STEP, 0.0f);			// ��Ɉړ�
			break;
		case IDM_MOV_DOWN:
			Translate(hWnd, 0.0f, -MMDDM_MOVE_STEP, 0.0f);			// ���Ɉړ�
			break;
		case IDM_POS_RESET:
			ResetPosition(hWnd);									// �z�u�����Z�b�g
			break;
		case IDM_TOPMOST:
			ToggleTopMost(hWnd);									// ��ɍőO�ʂɕ\��
			break;
		case IDM_FPS_60:
			g_fpFrameSkip = 0.5f;									// 0.5�t���[�����Ƃɕ`�悳����
			break;
		case IDM_FPS_30:
			g_fpFrameSkip = 1.0f;									// 1�t���[�����Ƃɕ`�悳����
			break;
		case IDM_FPS_15:
			g_fpFrameSkip = 2.0f;									// 2�t���[�����Ƃɕ`�悳����
			break;
		case IDM_FPS_10:
			g_fpFrameSkip = 3.0f;									// 3�t���[�����Ƃɕ`�悳����
			break;
		case IDM_FPS_5:
			g_fpFrameSkip = 6.0f;									// 6�t���[�����Ƃɕ`�悳����
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
		case IDM_DEBUG:												// �f�o�b�O�p�\���ؑ�
			g_clPMDModel.m_iDebug++;
			if (g_clPMDModel.m_iDebug > 4) g_clPMDModel.m_iDebug = 0;
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		SetTimer( hWnd, MMDDM_TIMER_ID, MMDDM_WAIT, NULL);			// �^�C�}�[���A
		break;
	case WM_LBUTTONDOWN:
		//// �����{�^���������ꂽ�Ƃ��C�^�C�g���o�[���ƌ�F�����邱�ƂŃh���b�O�ړ����\�ɂ�����@
		//PostMessage(hWnd, WM_NCLBUTTONDOWN, HTCAPTION, lParam);

		// ���m�[�}���ȕ��@
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
	case 0x0313:						// �^�X�N�o�[�E�N���b�N�Ŕ����i�B���@�\�H�j
		PopupMenu(hWnd, lParam);
		break;
	case WM_MOUSEMOVE:
		if (wParam & MK_LBUTTON) {
			UpdateCursor(hWnd);
		}
		break;
	case WM_PAINT:
		//hdc = BeginPaint(hWnd, &ps);	// ���C���[�h�E�B���h�E�Ȃ̂ŕs�v�H
		calcFps();						// FPS�v�Z
		Render(hWnd);					// �`�揈��
		//EndPaint(hWnd, &ps);			// ���C���[�h�E�B���h�E�Ȃ̂ŕs�v�H
		ValidateRect(hWnd, NULL);		// �`�抮��
		break;
	case WM_DROPFILES:
		OpenDraggedFiles(hWnd, (HDROP)wParam);
		break;
	case WM_TIMER:
		UpdateMotion( hWnd );
		if ( g_bFollowModel )
			LocateWindow(hWnd);			// ���f���̃Z���^�[�ɍ��킹���E�B���h�E�ړ�
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	//case MM_MCINOTIFY:		// �v�����悤�ɓ��삵�Ȃ��H
	//	g_clPMDModel.rewind();
	//	ResetTime();
	//	break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// �o�[�W�������{�b�N�X�̃��b�Z�[�W �n���h���ł��B
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
// �`�揈��
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

	//// �A���t�@�l��K�p
	//UpdateLayeredWindow(hWnd, hScreenDC, &wndPos, &wndSize, hBmpDC, &surPos, 0, &bf, ULW_ALPHA);

	// �J���[�L�[��K�p
	UpdateLayeredWindow(hWnd, hScreenDC, &wndPos, &wndSize, hBmpDC, &surPos, MMD_BG_COLOR, &bf, ULW_COLORKEY);

	ReleaseDC(0, hScreenDC);
#endif 
}

//--------
// ���[�V�������X�V���ĕ`��w��
//--------
void UpdateMotion( HWND hWnd )
{
	float fCurrentFrame;

	// �Đ��J�n����̌o�߃t���[����
	fCurrentFrame = (float)GetElapsedTime(g_dwStartTime) * MMDDM_MOTION_SPEED * 0.03f;

	// ���Z���s���ŏ��t���[���P��
	float frameStep = 0.5f;

	// �������Z���΂����B�܂��A�`�悪5�t���[���ȏ㔭������悤�Ȃ�A�����I�ɕ������Z���X�L�b�v
	bool  bSkipPhysics = ( !g_bPhysics || (fCurrentFrame - g_fpLastCalcFrame ) > 5.0f );

	// ���[�V�����X�V
	if ( bSkipPhysics ) {
		frameStep = floor((float)(fCurrentFrame - g_fpLastCalcFrame) / frameStep) * frameStep;	// frameStep�P�ʂŒ[���𖳂������o�߃t���[����
		if ( frameStep < g_fpFrameSkip ) return;							// �`���FPS�����ɍ��킹��

		g_clPMDModel.updateMotion( frameStep );
		if ( g_bLookAtCusor ) g_clPMDModel.updateNeckBone( &g_tagFaceVec );	// �}�E�X�ڐ����f
		g_clPMDModel.resetRigidBodyPos();									// �������Z���X�L�b�v����ꍇ�͍��̂��{�[���ɍ��킹��
		g_fpLastCalcFrame += frameStep;
	} else {
		while ( ( fCurrentFrame - g_fpLastCalcFrame ) >= frameStep ) {
			g_clPMDModel.updateMotion( frameStep );
			if ( g_bLookAtCusor ) g_clPMDModel.updateNeckBone( &g_tagFaceVec );	// �}�E�X�ڐ����f
			g_clBulletPhysics.update( frameStep );								// �������Z����
			g_fpLastCalcFrame += frameStep;
		}
	}

	// �`��X�V�w��
	if ( ( fCurrentFrame - g_fpLastDispFrame ) < g_fpFrameSkip) return;		// FPS����

	g_clPMDModel.updateSkinning();											// �X�L�j���O�X�V
	InvalidateRect( hWnd, NULL, TRUE );										// �`��w��

	g_fpLastDispFrame = fCurrentFrame;										// �`�悵���t���[���ԍ����L�^�i�덷�h�~�̂��ߐ����P�ʁj
}

//--------
// �}�E�X�J�[�\���ʒu�X�V
//--------
void ResetCursor()
{
	// �J�[�\���ʒu�擾
	GetCursorPos( &g_tagLastCurPos);
}


//--------
// �}�E�X�J�[�\���ʒu�X�V
//--------
void UpdateCursor( HWND hWnd )
{
	static const float fMaxVelocity = 5.0f;		// �ő呬�x
	static const float fCoefVelocity = 0.002f;	// ���x�Ɋ|����K���ȌW���i�}�E�X�̓����ɔ�Ⴓ����W���j

	static DWORD lastTime = GetElapsedTime(0);
	DWORD elapsedTime;
	float sec;
	POINT tagCurrentPos;
	float fVelocity[2];

	// �J�[�\���ʒu�擾
	GetCursorPos( &tagCurrentPos );

	// �E�B���h�E�ʒu�X�V
	g_tagWindowOffset.x += (tagCurrentPos.x - g_tagLastCurPos.x);
	g_tagWindowOffset.y += (tagCurrentPos.y - g_tagLastCurPos.y);
	LocateWindow( hWnd );

	//// �E�B���h�E�ʒu�X�V
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

	// �O�񂩂�̌o�ߎ���
	elapsedTime = GetElapsedTime(lastTime);
	sec = (float)elapsedTime / 1000.0f;

	if ( sec > 0.0f ) {
		// �J�[�\�����x�v�Z
		fVelocity[0] = (float)(tagCurrentPos.x - g_tagLastCurPos.x) / sec * fCoefVelocity;
		fVelocity[1] = (float)(tagCurrentPos.y - g_tagLastCurPos.y) / sec * fCoefVelocity;

		// ���x����
		if (fVelocity[0] < -fMaxVelocity)	fVelocity[0] = -fMaxVelocity;
		if (fVelocity[0] > fMaxVelocity)	fVelocity[0] = fMaxVelocity;
		if (fVelocity[1] < -fMaxVelocity)	fVelocity[1] = -fMaxVelocity;
		if (fVelocity[1] > fMaxVelocity)	fVelocity[1] = fMaxVelocity;

		// ���f���̉�]�𔽉f
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

		// �}�E�X�ɂ�鑬�x��S���̂ɉ�����
		g_clPMDModel.applyCentralImpulse( vec4.x, vec4.y, vec4.z );
	}

	g_tagLastCurPos = tagCurrentPos;
	lastTime = GetElapsedTime(0);
}

//--------
// �`��͈͂̃��T�C�Y
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
			// �r�b�g�}�b�v�̃T�C�Y��ύX
			g_clGl->ResizeBitmap(lastSize.cx, lastSize.cy);

			// OpenGL �ď�����
			g_clGl->SetCurrent();
			//g_clGl->BeginRender();
			initGL();
			resize(lastSize.cx, lastSize.cy);
			//g_clGl->EndRender();
		}
	}
}

//--------
// ��]����
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
// ���i����
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
// �g��k������
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
// �z�u�����Z�b�g
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
// �E�N���b�N���j���[���J��
// </sumamry>
void PopupMenu( HWND hWnd, LPARAM lParam )
{
	POINT mousepos;
	HMENU hMenu, hSubmenu;
	MENUITEMINFO info;

	// ���j���[��`�Ǎ�
	hMenu = LoadMenu((HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE), MAKEINTRESOURCE(IDC_MENU));
	hSubmenu = GetSubMenu(hMenu, 0);

	// �`�F�b�N�}�[�N��ݒ肷�邽�߂�MENUITEMINFO����
	info.cbSize = sizeof( info );
	info.fMask = MIIM_STATE;

	// �J�[�\���ڐ����[�h�̃`�F�b�N�}�[�N
	GetMenuItemInfo( hMenu, IDM_LOOKATME, FALSE, &info );
	info.fState = (g_bLookAtCusor ? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_LOOKATME, FALSE, &info );

	// ��ɍőO�ʂ̃`�F�b�N�}�[�N
	GetMenuItemInfo( hMenu, IDM_TOPMOST, FALSE, &info );
	info.fState = (g_bTopMost ? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_TOPMOST, FALSE, &info );

	// �������Z�̃`�F�b�N�}�[�N
	GetMenuItemInfo( hMenu, IDM_ENABLE_PHYSICS, FALSE, &info );
	info.fState = (g_bPhysics ? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_ENABLE_PHYSICS, FALSE, &info );

	// �|�[�Y�u�����h�̃`�F�b�N�}�[�N
	GetMenuItemInfo( hMenu, IDM_POSEBLEND, FALSE, &info );
	info.fState = (g_bBlendPose ? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_POSEBLEND, FALSE, &info );

	// �E�B���h�E�ړ��̃`�F�b�N�}�[�N
	GetMenuItemInfo( hMenu, IDM_MOVINGWINDOW, FALSE, &info );
	info.fState = (g_bFollowModel? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_MOVINGWINDOW, FALSE, &info );

	// FPS�\���̃`�F�b�N�}�[�N
	GetMenuItemInfo( hMenu, IDM_SHOW_FPS, FALSE, &info );
	info.fState = (g_bDispFPS ? MFS_CHECKED: MFS_UNCHECKED );
	SetMenuItemInfo( hMenu, IDM_SHOW_FPS, FALSE, &info );

	// FPS�����̃`�F�b�N�}�[�N
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

	// ���j���[���J�����W�����߂�
	//// ����̓^�X�N�o�[�E�N���b�N�ł͂��܂������Ȃ�����
	//mousepos.x = LOWORD(lParam);
	//mousepos.y = HIWORD(lParam);
	RECT rc;
	if (!GetWindowRect(hWnd, &rc)) return;
	if (!GetCursorPos(&mousepos)) return;
	mousepos.x -= rc.left;
	mousepos.y -= rc.top;
	ClientToScreen(hWnd, &mousepos);

	// ���j���[���J��
	TrackPopupMenu(hSubmenu, TPM_LEFTALIGN, mousepos.x, mousepos.y, 0, hWnd, NULL);

	DestroyMenu(hMenu);
}

//--------
// �J�[�\���ڐ����[�h���ɖڐ��x�N�g�����v�Z
//--------
void UpdateMousePosition( HWND hWnd )
{
	RECT rc;
	POINT pos;
	if (!GetWindowRect(hWnd, &rc)) return;
	if (!GetCursorPos(&pos)) return;

	Vector4 vecGm, vecRot, vecRotCon;
	vecGm.w = 0;
	vecGm.z = 20.0f;		// ���f���ɑ΂���J�[�\����z���W

	// ��ʂ̍��W���� MMD_FRUSTUM_NEAR �ʒu�ł̍��W��
	vecGm.x = ((float)(pos.x - rc.left) / (float)(rc.right - rc.left) - 0.5f) * 2.0f;
	vecGm.y = -((float)(pos.y - rc.top) / (float)(rc.bottom - rc.top) - 0.5f) * 2.0f;
	vecGm.y *= (float)(rc.bottom - rc.top) / (float)(rc.right - rc.left);	// �E�B���h�E�c����␳

	// �J�����p�[�X�̋t���f
	vecGm.x = (float)(vecGm.x * (-vecGm.z - MMD_CAMERA_Z) / MMD_FRUSTUM_NEAR);
	vecGm.y = (float)(vecGm.y * (-vecGm.z - MMD_CAMERA_Z) / MMD_FRUSTUM_NEAR);

	// �g�嗦�̋t���f
	vecGm.x /= g_fpScale;
	vecGm.y /= g_fpScale;
	vecGm.z /= g_fpScale;

	// ���i�̋t���f
	vecGm.x -= g_tagModelPosVec.x;
	vecGm.y -= g_tagModelPosVec.y;
	vecGm.z += g_tagModelPosVec.z;

	// ��]�̋t���f
	vecRot = g_tagModelRotQuat;
	vecRotCon.w = vecRot.w = -vecRot.w;					// �O���[�o�����W���烂�f�����W�ɂ��邽�߁A�t��]�Ƃ���
	vecRotCon.x = -vecRot.x;
	vecRotCon.y = -vecRot.y;
	vecRotCon.z = -vecRot.z;
	QuaternionMultiply(&vecGm, &vecRotCon, &vecGm);		// �N�H�[�^�j�I���ɂ���]
	QuaternionMultiply(&vecGm, &vecGm, &vecRot);		// �N�H�[�^�j�I���ɂ���]
	
	vecGm.y -= (float)MMD_MODEL_Y;						// �f�t�H���g�̍��W�����グ

	if (g_bFollowModel)									// �E�B���h�E�ړ����[�h�Ȃ�A�Z���^�[�ʒu��␳
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
// �őO�ʂɕ\����؂�ւ�
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
// ���f���̃Z���^�[�ɍ��킹�ăE�B���h�E���ړ�
//--------
void LocateWindow( HWND hWnd )
{
	Vector3 vec3ModelPos, vec3Pos;
	g_clPMDModel.getModelPosition( &vec3ModelPos );				// ���f�����W�擾
	vec3ModelPos.z = -vec3ModelPos.z;							// PMD�ƍ��W�n���Ⴄ
	Vector3Transform( &vec3Pos, &vec3ModelPos, g_mTransform );	// ���W�ϊ�

	// �E�B���h�E�ʒu�X�V
	int x = (int)(vec3Pos.x * 2.0f) + g_tagWindowOffset.x;		// glFrustum() ��-1����1�Ƃ��Ă��邽�߁A�~2.0����
	int y = (int)(-vec3Pos.y * 2.0f) + g_tagWindowOffset.y;

	//// �͂ݏo���`�F�b�N�i�}���`���j�^�p��MonitorFromWindow()���c�Ǝv���������܂������Ȃ������j
	//if ( !MonitorFromWindow( hWnd, MONITOR_DEFAULTTONULL ) )
	//{
	//	// �͂ݏo���Ă����狭���I�Ƀv���C�}�����j�^�ɔ[�߂�
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
// �h���b�O�A���h�h���b�v���ꂽ�t�@�C�����J��
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

	// �h���b�v���ꂽ�t�@�C�����𒲂ׂ�
	UINT nFiles = DragQueryFile(hDrop, 0xFFFFFFFF, NULL, NULL);

	// �Ή��t�@�C�������邩�ǂ������ׂ�
	//   ��������̃t�@�C�����������ꍇ�͍Ō�̕��̂ݗL��
	for (UINT i= 0; i < nFiles; i++) {
		// �t�@�C�����擾
		DragQueryFile(hDrop, i, wszFileName, MAX_PATH);

		// �g���q���擾
		GetFileExtension(wszExt, wszFileName);

		if (_wcsnicmp(wszExt, L".pmd", 4) == 0) {
			// PMD�t�@�C����������
			wcscpy_s(wszPMDFileName, MAX_PATH, wszFileName);
			bIsPMDFound = true;
		} else if (_wcsnicmp(wszExt, L".vmd", 4) == 0) {
			// VMD�t�@�C����������
			wcscpy_s(wszVMDFileName, wszFileName);
			bIsVMDFound = true;
		} else if (_wcsnicmp(wszExt, L".vpd", 4) == 0) {
			// VPD�t�@�C����������
			wcscpy_s(wszVPDFileName, wszFileName);
			bIsVPDFound = true;
		} else if ((_wcsnicmp(wszExt, L".mp3", 4) == 0)
			|| (_wcsnicmp(wszExt, L".wav", 4) == 0)
			|| (_wcsnicmp(wszExt, L".wma", 4) == 0)
			) {
			// ���y�t�@�C����������
			wcscpy_s(wszBGMFileName, MAX_PATH, wszFileName);
			bIsBGMFound = true;
		}
	}

	// PMD�t�@�C�����h���b�O����Ă���ΊJ��
	if (bIsPMDFound) {
		if (openNewModel(wszPMDFileName)) {
			ResetTime();
		}
	}

	// VPD�t�@�C�����h���b�O����Ă���ΊJ��
	if (bIsVPDFound) {
		if (openNewMotion(wszVPDFileName)) {
			ResetTime();
		}
	}

	// VMD�t�@�C�����h���b�O����Ă���ΊJ��
	if (bIsVMDFound) {
		if (openNewMotion(wszVMDFileName)) {
			ResetTime();
		}
	}

	// BGM�t�@�C�����h���b�O����Ă���ΊJ��
	if (bIsBGMFound) {
		if (OpenBGM(wszBGMFileName)) {
			g_clPMDModel.setLoop(FALSE);		// BGM�Đ�������ƃ��[�v��������

			//// �����߂��͍Đ���MM_MCINOTIFY�����m�����Ƃ��ɂ��邽�߁C�����ł��Ȃ�
			//g_clPMDModel.rewind();
			//ResetTime();
		}
	}

	// �h���b�v�̉��
	DragFinish(hDrop);
}

//--------
// �R�}���h���C����������
//--------
// <return>���f�����J���ꂽ�� TRUE ��Ԃ�</return>
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

	// �R�}���h���C���������
	lpArgv = CommandLineToArgvW(lpCmdLine, &nArgc);
	if (lpArgv == NULL) return FALSE;

	// �Ή��t�@�C�������邩�ǂ������ׂ�
	//   ��������̃t�@�C�����������ꍇ�͍Ō�̕��̂ݗL��
	for (int i = 1; i < nArgc; i++) {
		wszFileName = lpArgv[i];

		// �R�}���h���C���p�����[�^�̉���
		if (wszFileName[0] == L'/' || wszFileName[0] == '-') {
			switch (wszFileName[1]) {
				case 'F':
				case 'f':
					// "-F1"��FPS�\����ON�A"-F0"��OFF�i�f�t�H���g��OFF�j
					g_bDispFPS = (wszFileName[2] == '1');
					break;

				case 'T':
				case 't':
					// "-T1"�ōőO�ʂ�ON�A"-T0"��OFF�i�f�t�H���g��OFF�j
					g_bTopMost = (wszFileName[2] == '1');
					break;

				case 'B':
				case 'b':
					// "-B1"�ŕ������Z��ON�A"-B0"��OFF�i�f�t�H���g��OFF�j
					g_bPhysics = (wszFileName[2] == '1');
					break;

				case 'L':
				case 'l':
					// "-L1"�ŃJ�[�\���ڐ�ON�A"-L0"��OFF�i�f�t�H���g��OFF�j
					g_bLookAtCusor = (wszFileName[2] == '1');
					break;

				case 'W':
				case 'w':
					// "-W1"�ŃE�B���h�E�����ړ�ON�A"-W0"��OFF�i�f�t�H���g��ON�j
					g_bFollowModel = (wszFileName[2] == '1');
					break;
			}

			continue;	// '/'�܂���'-'�Ŏn�܂�ꍇ�̓t�@�C�����Ɖ��߂����Ȃ�
		}

		// �g���q���擾
		GetFileExtension(wszExt, wszFileName);

		if (_wcsnicmp(wszExt, L".pmd", 4) == 0) {
			// PMD�t�@�C����������
			wcscpy_s(wszPMDFileName, MAX_PATH, wszFileName);
			bIsPMDFound = true;
		} else if (_wcsnicmp(wszExt, L".vmd", 4) == 0) {
			// VMD�t�@�C����������
			wcscpy_s(wszVMDFileName, wszFileName);
			bIsVMDFound = true;
		} else if (_wcsnicmp(wszExt, L".vpd", 4) == 0) {
			// VPD�t�@�C����������
			wcscpy_s(wszVPDFileName, wszFileName);
			bIsVPDFound = true;
		} else if ((_wcsnicmp(wszExt, L".mp3", 4) == 0)
			|| (_wcsnicmp(wszExt, L".wav", 4) == 0)
			|| (_wcsnicmp(wszExt, L".wma", 4) == 0)
			) {
			// ���y�t�@�C����������
			wcscpy_s(wszBGMFileName, MAX_PATH, wszFileName);
			bIsBGMFound = true;
		}
	}
	// PMD�t�@�C�����h���b�O����Ă���ΊJ��
	if (bIsPMDFound) {
		if (openNewModel(wszPMDFileName)) {
			ResetTime();
		}
	}

	// VPD�t�@�C�����h���b�O����Ă���ΊJ��
	if (bIsVPDFound) {
		if (openNewMotion(wszVPDFileName)) {
			ResetTime();
		}
	}

	// VMD�t�@�C�����h���b�O����Ă���ΊJ��
	if (bIsVMDFound) {
		if (openNewMotion(wszVMDFileName)) {
			ResetTime();
		}
	}

	// BGM�t�@�C�����h���b�O����Ă���ΊJ��
	if (bIsBGMFound) {
		if (OpenBGM(wszBGMFileName)) {
			g_clPMDModel.setLoop(FALSE);		// BGM�Đ�������ƃ��[�v��������
		}
	}

	LocalFree(lpArgv);

	return bIsPMDFound;
}

//--------
// �L�����o�X�T�C�Y�I��
//--------
void ResizeWindow( HWND hWnd, int size )
{
	if (size == NULL) {
		// size == NULL ���ƃt���X�N���[���T�C�Y
		//  ���łɃE�B���h�E�ʒu�𒲐�
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
		// size == NULL ���ƃt���X�N���[���T�C�Y
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
// dwStartTime ����̎��Ԃ��~���b�ŕԂ�
//--------
DWORD GetElapsedTime( DWORD dwStartTime )
{
	DWORD	time = GetTickCount();
	DWORD	interval;

	if ( dwStartTime > time ) {
		// �{�v���O�����g�p���� GetTickCount() ��1�������ꍇ(�V�X�e���N�������49���o��)
		//   2���ȍ~�̓_���Ȃ̂ŁA����Ȃɒ����N�����Ȃ��łˁB
		interval = (DWORD)(0x100000000 + time - dwStartTime);
	} else {
		interval = time - dwStartTime;
	}

	return interval;
}

//--------
// ���[�V�����J�n���������Z�b�g
//--------
void ResetTime()
{
	g_dwStartTime = GetTickCount();		// �Đ��J�n�������Z�b�g
	g_fpLastDispFrame = 0.0f;			// �`��t���[���ԍ����Z�b�g
	g_fpLastCalcFrame = 0.0f;			// ���[�V�����X�V�t���[���ԍ����Z�b�g
}

//-------------------
// FPS�̌v��
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
// �I�����̏���
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

	// GDI+ �I��������
	Gdiplus::GdiplusShutdown( pGdiToken );
}

//--------
// �t�@�C���p�X���G�X�P�[�v���A�_�u���N�H�[�e�[�V�����ň͂�
//--------
void EscapeFilePath( LPWSTR wszDest, LPCWSTR wszSrc )
{
	int nLen = wcslen(wszSrc);
	int i, j;

	// �_�u���N�H�[�e�[�V���������Ă��Ȃ���΁A�t����
	if (wszSrc[0] != L'"') {
		wcscpy_s(wszDest, 2, L"\"");
		j = 1;		// wszDest �̃C���f�b�N�X�B�_�u���N�H�[�e�[�V�����̕���1����J�n
	} else {
		wcscpy_s(wszDest, 1, L"");
	}
	
	// �G�X�P�[�v���Ȃ���p�X�̓��e���R�s�[
	for (i = 0; i < nLen; i++) {
		if (wszSrc[i] == L'\\') {
			wszDest[j++] = L'\\';
		}
		wszDest[j++] = wszSrc[i];
	}

	// �Ō�Ƀ_�u���N�H�[�e�[�V���������Ă��Ȃ���΁A�t����
	if (wszSrc[i] != L'"') {
		wszDest[j++] = L'"';
	}

	// �I�[
	wszDest[j++] = '\0';
}

//--------
// BGM�̒�~�ƃN���[�Y
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
// BGM��Ǎ�
//--------
bool OpenBGM( LPWSTR wszFileName )
{
	WCHAR wszPath[394];
	WCHAR wszCommand[512];
	MCIERROR res;

	// �t�@�C���p�X�̃G�X�P�[�v
	EscapeFilePath(wszPath, wszFileName);

	// �i���łɊJ����Ă�����jBGM���N���[�Y
	CloseBGM();

	// BGM���J��
	wcscpy_s(wszCommand, L"open ");
	wcscat_s(wszCommand, 394, wszPath);
	wcscat_s(wszCommand, L" alias bgm");
	res = mciSendString(wszCommand, NULL, 0, NULL);
	if (res != 0) return FALSE;

	// �Đ�
	//res = mciSendString(L"play bgm notify", NULL, 0, g_hWnd);
	res = mciSendString(L"play bgm", NULL, 0, g_hWnd);
	if (res != 0) return FALSE;

	// �Đ������烂�[�V������擪�Ɋ����߂�
	g_clPMDModel.rewind();
	ResetTime();

	g_bBGM = TRUE;

	return TRUE;
}
