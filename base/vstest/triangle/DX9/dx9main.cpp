#if 1

#pragma comment(lib, "d3d9.lib")

#include <windows.h>
#include <tchar.h>

#include "../application.h"
#include "../gamewindowwin.h"
#include "dx9renderer.h"

// �E�B���h�E�v���V�[�W��
LRESULT CALLBACK wndProc( HWND hWnd, UINT mes, WPARAM wParam, LPARAM lParam ){
	if( mes == WM_DESTROY || mes == WM_CLOSE ) { PostQuitMessage( 0 ); return 0; }
	return DefWindowProc( hWnd, mes, wParam, lParam );
}

int APIENTRY _tWinMain(	HINSTANCE hInstance, HINSTANCE,	LPTSTR,	int)
{
	// �Q�[���E�B���h�E������
	GameWindowWin window;
	if (!window.create(hInstance, "DirectX�e�X�g", wndProc))
		return -1;

	// OpenGL�����_��������
	DX9Renderer renderer(window.getHWND());
	if (!renderer.initialize())
		return -2;

	// �A�v���X�^�[�g
	Application app(window, renderer);
	return app.run();
}

#endif
