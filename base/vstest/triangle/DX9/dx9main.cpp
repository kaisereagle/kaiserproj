#if 1

#pragma comment(lib, "d3d9.lib")

#include <windows.h>
#include <tchar.h>

#include "../application.h"
#include "../gamewindowwin.h"
#include "dx9renderer.h"

// ウィンドウプロシージャ
LRESULT CALLBACK wndProc( HWND hWnd, UINT mes, WPARAM wParam, LPARAM lParam ){
	if( mes == WM_DESTROY || mes == WM_CLOSE ) { PostQuitMessage( 0 ); return 0; }
	return DefWindowProc( hWnd, mes, wParam, lParam );
}

int APIENTRY _tWinMain(	HINSTANCE hInstance, HINSTANCE,	LPTSTR,	int)
{
	// ゲームウィンドウ初期化
	GameWindowWin window;
	if (!window.create(hInstance, "DirectXテスト", wndProc))
		return -1;

	// OpenGLレンダラ初期化
	DX9Renderer renderer(window.getHWND());
	if (!renderer.initialize())
		return -2;

	// アプリスタート
	Application app(window, renderer);
	return app.run();
}

#endif
