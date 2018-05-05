#include "stdafx.h"

#include "zg_dx11.h"
#include <locale.h>

HWND g_hWnd = NULL;

HRESULT InitWindow(HINSTANCE hInstance, int nCmdShow );
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM );


int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow )
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	setlocale( LC_ALL, "Japanese" );

	if(FAILED( InitWindow(hInstance, nCmdShow))){
		return 0;
	}

	if(FAILED(InitDX11(g_hWnd))){
		ExitDX11();
		return 0;
	}

	// Main message loop
	MSG msg = {0};
	while( WM_QUIT != msg.message ){
		if( PeekMessage( &msg, NULL, 0, 0, PM_REMOVE ) ){
			TranslateMessage( &msg );
			DispatchMessage( &msg );
		}else{
			RenderDX11();
			Sleep(1);
		}
	}

	ExitDX11();

	return (int)msg.wParam;
}

HRESULT InitWindow( HINSTANCE hInstance, int nCmdShow )
{
	WNDCLASSEX wcex;
	wcex.cbSize = sizeof(WNDCLASSEX);
	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= NULL;
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= NULL;
	wcex.lpszClassName	= L"dx11WindowClass";
	wcex.hIconSm		=  NULL;

	if(!RegisterClassEx(&wcex)){
		return E_FAIL;
	}
	//ウインドウのクライアント領域（=DirectXの描画領域）を指定
	RECT rc = { 0, 0, 640, 480 };
	AdjustWindowRect( &rc, WS_OVERLAPPEDWINDOW, FALSE );

	g_hWnd = CreateWindow( L"dx11WindowClass", L"dx11Model6", WS_OVERLAPPEDWINDOW,
						CW_USEDEFAULT, CW_USEDEFAULT, rc.right - rc.left, rc.bottom - rc.top,
						NULL, NULL, hInstance, NULL);
	if (!g_hWnd){
		return E_FAIL;
	}

	ShowWindow(g_hWnd, nCmdShow);
	UpdateWindow(g_hWnd);

    return S_OK;
}

LRESULT CALLBACK WndProc( HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam )
{
    PAINTSTRUCT ps;
    HDC hdc;

    switch( message )
    {
        case WM_PAINT:
            hdc = BeginPaint( hWnd, &ps );
            EndPaint( hWnd, &ps );
            break;

        case WM_DESTROY:
            PostQuitMessage( 0 );
            break;

        default:
            return DefWindowProc( hWnd, message, wParam, lParam );
    }

    return 0;
}
