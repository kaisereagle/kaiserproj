#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include "gamewindowwin.h"
#include "renderer.h"

class Application {
	GameWindowWin &window;
	Renderer &renderer;
	Vertex vtx[4];

protected:
	virtual bool initialize() {
		// ���\�[�X�쐬
		Vertex v[4] = {
			{-0.5f, -0.5f, 0.0f, 0xffe0e060},
			{-0.5f,  0.5f, 0.0f, 0xffe0e060},
			{ 0.5f, -0.5f, 0.0f, 0xffe0e060},
			{ 0.5f,  0.3f, 0.0f, 0xffe0e060},
		};
		memcpy(vtx, v, sizeof(Vertex) * 4);

		return true;
	}

	virtual void terminate() {
		renderer.terminate();
		window.terminate();
	}

public:
	Application(GameWindowWin &window, Renderer &renderer) : window(window), renderer(renderer) {}

	// �A�v���J�n
	int run() {
		if (!initialize())
			return -1;

		// �E�B���h�E�\��
		window.show();

		// ���b�Z�[�W ���[�v
		MSG msg;
		do{
			if( PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) ){
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			} else {
				renderer.begin();
				renderer.render(vtx, 4, 2);
				renderer.swap();
				renderer.end();
			}
		}while( msg.message != WM_QUIT );

		// �e��㏈��
		terminate();

		return 0;
	}
};

#endif
