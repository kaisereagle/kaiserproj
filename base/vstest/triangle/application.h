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
		// リソース作成
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

	// アプリ開始
	int run() {
		if (!initialize())
			return -1;

		// ウィンドウ表示
		window.show();

		// メッセージ ループ
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

		// 各種後処理
		terminate();

		return 0;
	}
};

#endif
