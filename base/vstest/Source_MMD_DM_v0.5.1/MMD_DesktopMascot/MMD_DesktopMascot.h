#pragma once

#include "resource.h"

#include "MMD.h"
#include "OffscreenGL.h"
#include <math.h>
#include <mmsystem.h>


// このコード モジュールに含まれる関数の宣言を転送します:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);

void	PopupMenu(HWND hWnd, LPARAM lParam);
void	Render(HWND hWnd);
void	UpdateMotion(HWND hWnd);
void	UpdateWindowSize(HWND hWnd);
void	UpdateMousePosition(HWND hWnd);
void	Rotate(HWND hWnd, float x, float y, float z, float angle);
void	Translate(HWND hWnd, float x, float y, float z);
void	Scale(HWND hWnd, float scale);
void	ResetPosition(HWND hWnd);
void	ToggleTopMost(HWND hWnd);
void	ResizeWindow(HWND hWnd, int size);
void	OpenDraggedFiles(HWND hWnd, HDROP hDrop);
bool	OpenCommandLineFiles(LPTSTR lpCmdLine);
void	GetFileExtension(LPWSTR wszExt, LPCWSTR wszFileName);
void	EscapeFilePath(LPWSTR wszDest, LPCWSTR wszSrc);
void	Release();
bool	OpenBGM(LPWSTR wszFileName);
void	CloseBGM();
void	ResetTime();
DWORD	GetElapsedTime(DWORD dwStartTime);
void	calcFps( void );
void	ResetCursor( void );
void	UpdateCursor( HWND hWnd );
void	LocateWindow( HWND hWnd );
