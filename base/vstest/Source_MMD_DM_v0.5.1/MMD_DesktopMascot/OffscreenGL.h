// ---------------------------------------------------------------------------
//
// OpenGLでオフスクリーンに描画する
//
// Copyright (c) 2009  Ru--en
//
// 2009/07/15	Ver. 0.0.2	サイズ変更関連の修正
// 2009/07/01	Ver. 0.0.1	整理
// 2009/06/30	Ver. 0.0.0	何とか動作する段階
//
// ---------------------------------------------------------------------------

#pragma once

#include <windows.h>

class OffscreenGL
{
private:
	HBITMAP _hBmp;								// ビットマップのハンドル
	HDC		_hDC;								// デバイスコンテキストのハンドル
	HGLRC	_hGLRC;
	HGDIOBJ	_hOldBmp;
	void	*_pPixels;
	SIZE	_tagBmpSize;

	int  SetupPixelFormat( HDC hdc );			// ピクセルフォーマットを設定する
	void SetupBitmap(int nWidth, int nHeight);	// ビットマップを作成
	void InitBitmap(int nWidth, int nHeight);
	void DeleteBitmap();

public:
	OffscreenGL();								// コンストラクタ
	OffscreenGL(int nWidth, int nHeight);		// サイズ指定付きコンストラクタ
	~OffscreenGL();								// デストラクタ

	void SetCurrent();							// レンダリング コンテキストをカレントにする
	void BeginRender();							// 描画開始時に呼ぶ
	void EndRender();							// 描画終了時に呼ぶ
	void ResizeBitmap(int nWidth, int nHeight);	// ビットマップのサイズを変更
	HDC  GetDC();								// デバイスコンテキスト取得
	HBITMAP GetBitmap();						// ビットマップ取得
	SIZE GetSize();								// ビットマップサイズ取得

	void DebugDraw();							// テスト用に画像作成
};

/// <summary>
/// コンストラクタ
/// </summay>
OffscreenGL::OffscreenGL()
{
	_hDC = CreateCompatibleDC(::GetDC(NULL));
	InitBitmap(100, 100);			// デフォルトのサイズで作成
}
OffscreenGL::OffscreenGL(int nWidth, int nHeight)
{
	_hDC = CreateCompatibleDC(::GetDC(NULL));
	InitBitmap(nWidth, nHeight);
}


/// <summary>
/// デストラクタ
/// </summary>
OffscreenGL::~OffscreenGL()
{
	// レンダリング コンテキストをカレントからはずす。
	wglMakeCurrent (NULL, NULL) ; 

	// 確保したビットマップを削除
	DeleteBitmap();

	// デバイスコンテキストを解放
	ReleaseDC(NULL, _hDC);
}

/// <summary>
/// OpenGLの描画先をこのビットマップに設定
/// </summary>
inline void OffscreenGL::SetCurrent()
{
	BOOL flg = wglMakeCurrent( _hDC, _hGLRC );
}

/// <summary>
/// OpenGL描画前の処理
///
/// OpenGLの描画先をこのビットマップに設定
/// </summary>
inline void OffscreenGL::BeginRender()
{
	//BOOL flg = wglMakeCurrent( _hDC, _hGLRC );
}

/// <summary>
/// OpenGL描画後の処理
///
/// OpenGLの描画先をクリア
/// </summary>
inline void OffscreenGL::EndRender()
{
	//BOOL flg = wglMakeCurrent( _hDC, NULL);
}

inline void OffscreenGL::DebugDraw()
{
	byte *data = (byte*)_pPixels;

	if (!_pPixels) return;

	for (int y = 0; y < _tagBmpSize.cy; y++) {
		for (int x = 0; x < _tagBmpSize.cx; x++)  {
			int offset = (y * _tagBmpSize.cy + x) * 4;
			byte val = y % 0x100;
			for (int i = 0; i < 4; i++) {
				data[offset + i] = val;
			}
		}
	}
}

/// <summary>
/// ビットマップのハンドルを返す
/// </summary>
inline HBITMAP OffscreenGL::GetBitmap()
{
	return _hBmp;
}

/// <summary>
/// デバイスコンテキストのハンドルを返す
/// </summary>
inline HDC OffscreenGL::GetDC()
{
	return _hDC;
}

/// <summary>
/// ビットマップサイズを返す
/// </summary>
inline SIZE OffscreenGL::GetSize()
{
	return _tagBmpSize;
}

/// <summary>
/// ピクセル フォーマットを設定する
/// </summary>
int OffscreenGL::SetupPixelFormat( HDC hdc )
{
	int pixelformat;

	static PIXELFORMATDESCRIPTOR pfd = {
		sizeof (PIXELFORMATDESCRIPTOR),		// 構造体のサイズ
		1,									// OpenGL バージョン
		PFD_DRAW_TO_BITMAP | 				// ビットマップ
		PFD_SUPPORT_GDI |					// GDI
		//PFD_DOUBLEBUFFER |				// ダブルバッファ
		PFD_SUPPORT_OPENGL,					// OpenGL を使う
		PFD_TYPE_RGBA,						// ピクセルのカラーデータ
		32,									// 色のビット数
		0, 0, 0, 0, 0, 0, 0, 0,				// RGBAカラーバッファのビット
		0, 0, 0, 0, 0,						// アキュムレーションバッファのピクセル当りのビット数
		32,									// デプスバッファ    のピクセル当りのビット数
		0,									// ステンシルバッファのピクセル当りのビット数
		0,									// 補助バッファ      のピクセル当りのビット数
		PFD_MAIN_PLANE,						// レイヤータイプ
		0,									// 
		0,									// 
		0,									// 
		0									// 
	};
    
	if ( 0 == (pixelformat = ChoosePixelFormat (hdc, &pfd)) )
	{
		return 1;
	}

	if ( FALSE == SetPixelFormat(hdc, pixelformat, &pfd) )
	{
		return 2;
	}

	return 0;
}

/// <summary>
/// ビットマップを作成
/// </summary>
void OffscreenGL::SetupBitmap(int nWidth, int nHeight)
{
	_tagBmpSize.cx = (LONG)nWidth;
	_tagBmpSize.cy = (LONG)nHeight;

	BITMAPINFO bmi = {0};
	bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmi.bmiHeader.biWidth = _tagBmpSize.cx;
	bmi.bmiHeader.biHeight = _tagBmpSize.cy;
	bmi.bmiHeader.biPlanes = 1;
	bmi.bmiHeader.biBitCount = 32;

	_hBmp	 = CreateDIBSection(_hDC, &bmi, DIB_RGB_COLORS, (void **)&_pPixels, NULL, NULL);
}

/// <summary>
/// ビットマップを作成してOpenGLを使用
/// </summary>
void OffscreenGL::InitBitmap(int nWidth, int nHeight)
{
	// ビットマップを用意
	SetupBitmap(nWidth, nHeight);
	_hOldBmp = SelectObject(_hDC, _hBmp);

	// ピクセル フォーマットを設定する
	if( 0 != SetupPixelFormat( _hDC ) ) {
		_hGLRC = NULL;
		return;
	}

	// レンダリング コンテキストの作成 
	_hGLRC = wglCreateContext (_hDC); 
}

/// <summary>
/// サイズを変えてビットマップを再作成
/// </summary>
void OffscreenGL::ResizeBitmap(int nWidth, int nHeight)
{
	wglMakeCurrent( _hDC, NULL);

	SelectObject(_hDC, _hOldBmp);
	DeleteObject(_hBmp);
	SetupBitmap(nWidth, nHeight);
	SelectObject(_hDC, _hBmp);

	//wglMakeCurrent( _hDC, _hGLRC);
}

/// <summary>
/// ビットマップを削除
/// </summary>
void OffscreenGL::DeleteBitmap()
{
	if (_hGLRC) { 
		// レンダリング コンテキストの削除
		wglMakeCurrent(NULL, NULL);
		wglDeleteContext (_hGLRC);
		_hGLRC = NULL;
	}

	if (_hOldBmp) {
		// 前のビットマップに戻す
		SelectObject(_hDC, _hOldBmp);
		DeleteObject(_hBmp);
		_hOldBmp = NULL;
		_pPixels = NULL;
	}
}
