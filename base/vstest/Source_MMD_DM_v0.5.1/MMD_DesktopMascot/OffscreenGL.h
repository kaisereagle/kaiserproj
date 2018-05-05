// ---------------------------------------------------------------------------
//
// OpenGL�ŃI�t�X�N���[���ɕ`�悷��
//
// Copyright (c) 2009  Ru--en
//
// 2009/07/15	Ver. 0.0.2	�T�C�Y�ύX�֘A�̏C��
// 2009/07/01	Ver. 0.0.1	����
// 2009/06/30	Ver. 0.0.0	���Ƃ����삷��i�K
//
// ---------------------------------------------------------------------------

#pragma once

#include <windows.h>

class OffscreenGL
{
private:
	HBITMAP _hBmp;								// �r�b�g�}�b�v�̃n���h��
	HDC		_hDC;								// �f�o�C�X�R���e�L�X�g�̃n���h��
	HGLRC	_hGLRC;
	HGDIOBJ	_hOldBmp;
	void	*_pPixels;
	SIZE	_tagBmpSize;

	int  SetupPixelFormat( HDC hdc );			// �s�N�Z���t�H�[�}�b�g��ݒ肷��
	void SetupBitmap(int nWidth, int nHeight);	// �r�b�g�}�b�v���쐬
	void InitBitmap(int nWidth, int nHeight);
	void DeleteBitmap();

public:
	OffscreenGL();								// �R���X�g���N�^
	OffscreenGL(int nWidth, int nHeight);		// �T�C�Y�w��t���R���X�g���N�^
	~OffscreenGL();								// �f�X�g���N�^

	void SetCurrent();							// �����_�����O �R���e�L�X�g���J�����g�ɂ���
	void BeginRender();							// �`��J�n���ɌĂ�
	void EndRender();							// �`��I�����ɌĂ�
	void ResizeBitmap(int nWidth, int nHeight);	// �r�b�g�}�b�v�̃T�C�Y��ύX
	HDC  GetDC();								// �f�o�C�X�R���e�L�X�g�擾
	HBITMAP GetBitmap();						// �r�b�g�}�b�v�擾
	SIZE GetSize();								// �r�b�g�}�b�v�T�C�Y�擾

	void DebugDraw();							// �e�X�g�p�ɉ摜�쐬
};

/// <summary>
/// �R���X�g���N�^
/// </summay>
OffscreenGL::OffscreenGL()
{
	_hDC = CreateCompatibleDC(::GetDC(NULL));
	InitBitmap(100, 100);			// �f�t�H���g�̃T�C�Y�ō쐬
}
OffscreenGL::OffscreenGL(int nWidth, int nHeight)
{
	_hDC = CreateCompatibleDC(::GetDC(NULL));
	InitBitmap(nWidth, nHeight);
}


/// <summary>
/// �f�X�g���N�^
/// </summary>
OffscreenGL::~OffscreenGL()
{
	// �����_�����O �R���e�L�X�g���J�����g����͂����B
	wglMakeCurrent (NULL, NULL) ; 

	// �m�ۂ����r�b�g�}�b�v���폜
	DeleteBitmap();

	// �f�o�C�X�R���e�L�X�g�����
	ReleaseDC(NULL, _hDC);
}

/// <summary>
/// OpenGL�̕`�������̃r�b�g�}�b�v�ɐݒ�
/// </summary>
inline void OffscreenGL::SetCurrent()
{
	BOOL flg = wglMakeCurrent( _hDC, _hGLRC );
}

/// <summary>
/// OpenGL�`��O�̏���
///
/// OpenGL�̕`�������̃r�b�g�}�b�v�ɐݒ�
/// </summary>
inline void OffscreenGL::BeginRender()
{
	//BOOL flg = wglMakeCurrent( _hDC, _hGLRC );
}

/// <summary>
/// OpenGL�`���̏���
///
/// OpenGL�̕`�����N���A
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
/// �r�b�g�}�b�v�̃n���h����Ԃ�
/// </summary>
inline HBITMAP OffscreenGL::GetBitmap()
{
	return _hBmp;
}

/// <summary>
/// �f�o�C�X�R���e�L�X�g�̃n���h����Ԃ�
/// </summary>
inline HDC OffscreenGL::GetDC()
{
	return _hDC;
}

/// <summary>
/// �r�b�g�}�b�v�T�C�Y��Ԃ�
/// </summary>
inline SIZE OffscreenGL::GetSize()
{
	return _tagBmpSize;
}

/// <summary>
/// �s�N�Z�� �t�H�[�}�b�g��ݒ肷��
/// </summary>
int OffscreenGL::SetupPixelFormat( HDC hdc )
{
	int pixelformat;

	static PIXELFORMATDESCRIPTOR pfd = {
		sizeof (PIXELFORMATDESCRIPTOR),		// �\���̂̃T�C�Y
		1,									// OpenGL �o�[�W����
		PFD_DRAW_TO_BITMAP | 				// �r�b�g�}�b�v
		PFD_SUPPORT_GDI |					// GDI
		//PFD_DOUBLEBUFFER |				// �_�u���o�b�t�@
		PFD_SUPPORT_OPENGL,					// OpenGL ���g��
		PFD_TYPE_RGBA,						// �s�N�Z���̃J���[�f�[�^
		32,									// �F�̃r�b�g��
		0, 0, 0, 0, 0, 0, 0, 0,				// RGBA�J���[�o�b�t�@�̃r�b�g
		0, 0, 0, 0, 0,						// �A�L�������[�V�����o�b�t�@�̃s�N�Z������̃r�b�g��
		32,									// �f�v�X�o�b�t�@    �̃s�N�Z������̃r�b�g��
		0,									// �X�e���V���o�b�t�@�̃s�N�Z������̃r�b�g��
		0,									// �⏕�o�b�t�@      �̃s�N�Z������̃r�b�g��
		PFD_MAIN_PLANE,						// ���C���[�^�C�v
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
/// �r�b�g�}�b�v���쐬
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
/// �r�b�g�}�b�v���쐬����OpenGL���g�p
/// </summary>
void OffscreenGL::InitBitmap(int nWidth, int nHeight)
{
	// �r�b�g�}�b�v��p��
	SetupBitmap(nWidth, nHeight);
	_hOldBmp = SelectObject(_hDC, _hBmp);

	// �s�N�Z�� �t�H�[�}�b�g��ݒ肷��
	if( 0 != SetupPixelFormat( _hDC ) ) {
		_hGLRC = NULL;
		return;
	}

	// �����_�����O �R���e�L�X�g�̍쐬 
	_hGLRC = wglCreateContext (_hDC); 
}

/// <summary>
/// �T�C�Y��ς��ăr�b�g�}�b�v���č쐬
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
/// �r�b�g�}�b�v���폜
/// </summary>
void OffscreenGL::DeleteBitmap()
{
	if (_hGLRC) { 
		// �����_�����O �R���e�L�X�g�̍폜
		wglMakeCurrent(NULL, NULL);
		wglDeleteContext (_hGLRC);
		_hGLRC = NULL;
	}

	if (_hOldBmp) {
		// �O�̃r�b�g�}�b�v�ɖ߂�
		SelectObject(_hDC, _hOldBmp);
		DeleteObject(_hBmp);
		_hOldBmp = NULL;
		_pPixels = NULL;
	}
}
