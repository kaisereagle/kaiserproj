#ifndef __OPENGL_OPENGL_RENDERER_H__
#define __OPENGL_OPENGL_RENDERER_H__

#include "../renderer.h"
#include "../vertex.h"
#include "glew.h"

class OpenGLRenderer : public Renderer {
	HDC dc;
	HGLRC glRC;

public:
	OpenGLRenderer(HDC dc) : dc(dc) {}
	virtual ~OpenGLRenderer() {}

	// 初期化
	virtual bool initialize() {
		//  ピクセルフォーマット初期化
		PIXELFORMATDESCRIPTOR pfd =
		{
			sizeof(PIXELFORMATDESCRIPTOR),
			1,
			PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,		//Flags
			PFD_TYPE_RGBA,													//The kind of framebuffer. RGBA or palette.
			32,																//Colordepth of the framebuffer.
			0, 0, 0, 0, 0, 0,
			0,
			0,
			0,
			0, 0, 0, 0,
			24,																//Number of bits for the depthbuffer
			8,																//Number of bits for the stencilbuffer
			0,																//Number of Aux buffers in the framebuffer.
			PFD_MAIN_PLANE,
			0,
			0, 0, 0
		};
		int format = ChoosePixelFormat(dc, &pfd);
		if (format == 0)
			return false;	// 該当するピクセルフォーマットが無い

		// OpenGLが使うデバイスコンテキストに指定のピクセルフォーマットをセット
		if (!SetPixelFormat(dc, format, &pfd))
			return false;	// DCへフォーマットを設定するのに失敗

		// OpenGL contextを作成
		glRC = wglCreateContext(dc);

		// 作成されたコンテキストがカレント（現在使用中のコンテキスト）か？
		if (!wglMakeCurrent(dc, glRC))
			return false;	// 何か正しくないみたい…

		// Extention Functionsをロード
		glewExperimental = TRUE;
		GLenum err = glewInit();
		if (err != GLEW_OK)
			return false;	// 関数群ロードに失敗

		return true;
	}

	// 後処理
	virtual void terminate() {
		//  カレントコンテキストを無効にする
		wglMakeCurrent(NULL, NULL);

		// カレントコンテキストを削除
		wglDeleteContext(glRC);
	}

	// 描画開始
	virtual void begin() {
		wglMakeCurrent(dc, glRC);
		glClearColor(0.0f, 0.5f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
	}

	// 頂点バッファを描画
	virtual void render(Vertex *vtx, unsigned vtxNum, unsigned primNum) {
		glBegin(GL_TRIANGLE_STRIP);
		for (unsigned i = 0; i < vtxNum; i++) {
			const Vertex &v = vtx[i];
			glColor4ub(v.r, v.g, v.b, v.a);
			glVertex3f(v.x, v.y, v.z);
		}
		glEnd();
	}

	// スワップ
	virtual void swap() {
		glFlush();
		SwapBuffers(dc);
	}

	// 描画終わり
	virtual void end() {
		wglMakeCurrent(NULL, NULL);
	}
};

#endif
