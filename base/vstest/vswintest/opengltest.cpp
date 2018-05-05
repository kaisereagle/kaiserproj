/*
OpenGL描画テスト

2001/ 4/10　宍戸　輝光
*/

#include <windows.h>
#include <GL/gl.h>
//
//LRESULT CALLBACK __WndProc(HWND, UINT, WPARAM, LPARAM);
//
//HINSTANCE hInst;
//HWND hwMain;
//HGLRC hRC;
//
//int WINAPI _WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
//	PSTR szCmdLine, int iCmdShow) {
//
//	MSG  msg;
//	WNDCLASS wndclass;
//
//	hInst = hInstance; /* プロセスのハンドルを保存 */
//
//	wndclass.style = CS_HREDRAW | CS_VREDRAW;
//	wndclass.lpfnWndProc = __WndProc;
//	wndclass.cbClsExtra = 0;
//	wndclass.cbWndExtra = 0;
//	wndclass.hInstance = hInstance;
//	wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
//	wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
//	wndclass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
//	wndclass.lpszMenuName = NULL;
//	wndclass.lpszClassName = "CWindow";
//
//	RegisterClass(&wndclass);
//
//	hwMain = CreateWindow("CWindow", "OpenGL表示テスト", WS_OVERLAPPEDWINDOW,
//		CW_USEDEFAULT, CW_USEDEFAULT, 320, 240,
//		NULL, NULL, hInstance, NULL);
//
//	ShowWindow(hwMain, iCmdShow); /* ウインドウを表示 */
//	UpdateWindow(hwMain);        /* 再描画 */
//
//	while (GetMessage(&msg, NULL, 0, 0)) { /* メッセージループ */
//
//		TranslateMessage(&msg);
//		DispatchMessage(&msg);
//
//	}
//
//	return msg.wParam;
//
//}
//
//LRESULT CALLBACK ___WndProc(HWND hwnd, UINT iMsg, WPARAM wParam, LPARAM lParam) {
//
//	static HDC WinDC;
//	int pixelFormat;
//
//	PIXELFORMATDESCRIPTOR pfd = {
//		sizeof(PIXELFORMATDESCRIPTOR),   // size of this pfd 
//		1,                     // version number 
//		PFD_DRAW_TO_WINDOW |   // support window 
//		PFD_SUPPORT_OPENGL |   // support OpenGL 
//		PFD_DOUBLEBUFFER,      // double buffered 
//		PFD_TYPE_RGBA,         // RGBA type 
//		24,                    // 24-bit color depth 
//		0, 0, 0, 0, 0, 0,      // color bits ignored 
//		0,                     // no alpha buffer 
//		0,                     // shift bit ignored 
//		0,                     // no accumulation buffer 
//		0, 0, 0, 0,            // accum bits ignored 
//		32,                    // 32-bit z-buffer 
//		0,                     // no stencil buffer 
//		0,                     // no auxiliary buffer 
//		PFD_MAIN_PLANE,        // main layer 
//		0,                     // reserved 
//		0, 0, 0                // layer masks ignored 
//	};
//
//	switch (iMsg) {
//
//	case WM_CREATE:
//
//		WinDC = GetDC(hwnd);
//
//		pixelFormat = ChoosePixelFormat(WinDC, &pfd);
//		SetPixelFormat(WinDC, pixelFormat, &pfd);
//
//		hRC = wglCreateContext(WinDC);
//		wglMakeCurrent(WinDC, hRC);
//
//		return 0;
//
//	case WM_PAINT:
//
//		glClearColor(0.0, 0.4, 0.0, 0.0);
//		glClear(GL_COLOR_BUFFER_BIT);
//
//		glRectd(-0.5, -0.5, 0.5, 0.5);
//
//		glFlush();
//		SwapBuffers(WinDC);
//
//		return 0;
//
//	case WM_DESTROY:
//
//		wglMakeCurrent(WinDC, 0);
//		wglDeleteContext(hRC);
//
//		PostQuitMessage(0);
//		return 0;
//
//	}
//
//	return DefWindowProc(hwnd, iMsg, wParam, lParam);
//
//}