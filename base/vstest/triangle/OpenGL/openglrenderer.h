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

	// ������
	virtual bool initialize() {
		//  �s�N�Z���t�H�[�}�b�g������
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
			return false;	// �Y������s�N�Z���t�H�[�}�b�g������

		// OpenGL���g���f�o�C�X�R���e�L�X�g�Ɏw��̃s�N�Z���t�H�[�}�b�g���Z�b�g
		if (!SetPixelFormat(dc, format, &pfd))
			return false;	// DC�փt�H�[�}�b�g��ݒ肷��̂Ɏ��s

		// OpenGL context���쐬
		glRC = wglCreateContext(dc);

		// �쐬���ꂽ�R���e�L�X�g���J�����g�i���ݎg�p���̃R���e�L�X�g�j���H
		if (!wglMakeCurrent(dc, glRC))
			return false;	// �����������Ȃ��݂����c

		// Extention Functions�����[�h
		glewExperimental = TRUE;
		GLenum err = glewInit();
		if (err != GLEW_OK)
			return false;	// �֐��Q���[�h�Ɏ��s

		return true;
	}

	// �㏈��
	virtual void terminate() {
		//  �J�����g�R���e�L�X�g�𖳌��ɂ���
		wglMakeCurrent(NULL, NULL);

		// �J�����g�R���e�L�X�g���폜
		wglDeleteContext(glRC);
	}

	// �`��J�n
	virtual void begin() {
		wglMakeCurrent(dc, glRC);
		glClearColor(0.0f, 0.5f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
	}

	// ���_�o�b�t�@��`��
	virtual void render(Vertex *vtx, unsigned vtxNum, unsigned primNum) {
		glBegin(GL_TRIANGLE_STRIP);
		for (unsigned i = 0; i < vtxNum; i++) {
			const Vertex &v = vtx[i];
			glColor4ub(v.r, v.g, v.b, v.a);
			glVertex3f(v.x, v.y, v.z);
		}
		glEnd();
	}

	// �X���b�v
	virtual void swap() {
		glFlush();
		SwapBuffers(dc);
	}

	// �`��I���
	virtual void end() {
		wglMakeCurrent(NULL, NULL);
	}
};

#endif
