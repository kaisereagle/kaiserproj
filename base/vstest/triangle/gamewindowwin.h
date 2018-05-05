#ifndef __GAMEWINDOWWIN_H__
#define __GAMEWINDOWWIN_H__

class GameWindowWin {
	HWND hWnd;
	HDC dc;

public:
	GameWindowWin() : hWnd(), dc() {}
	~GameWindowWin() {}

    // �E�B���h�E�쐬
	bool create(HINSTANCE hInstance, const char* wndName, WNDPROC wndProc) {
		WNDCLASSEX wcex = {
			sizeof(WNDCLASSEX), CS_OWNDC | CS_HREDRAW | CS_VREDRAW, wndProc,
			0, 0, hInstance, NULL, NULL, (HBRUSH)(COLOR_WINDOW+1),
			NULL, wndName, NULL
		};
		if( !RegisterClassEx(&wcex) )
			return false;

		RECT R = { 0, 0, 640, 480 };
		AdjustWindowRect( &R, WS_OVERLAPPEDWINDOW, FALSE );
		hWnd = CreateWindow( wndName, wndName, WS_OVERLAPPEDWINDOW,
								CW_USEDEFAULT, 0, R.right - R.left, R.bottom - R.top,
								NULL, NULL, hInstance, NULL );
		if (hWnd == NULL)
			return false;

		dc = GetDC(hWnd);

		return true;
	}

	// �㏈��
	void terminate() {
		if (dc)
			ReleaseDC(hWnd, dc);
	}

	// �f�o�C�X�R���e�L�X�g���擾
	HDC getDC() {
		return dc;
	}

	// �E�B���h�E�n���h�����擾
	HWND getHWND() {
		return hWnd;
	}

	// �E�B���h�E�\��
	void show() {
		ShowWindow(hWnd, SW_SHOW);
	}
};

#endif
