#ifndef __RENDERER_H__
#define __RENDERER_H__

#include "vertex.h"

class Renderer {
public:
	// 初期化
	virtual bool initialize() = 0;

	// 後処理
	virtual void terminate() = 0;

	// 描画開始
	virtual void begin() = 0;

	// スワップ
	virtual void swap() = 0;

	// 描画終わり
	virtual void end() = 0;

	// 頂点バッファを描画
	virtual void render(Vertex *vtx, unsigned vtxNum, unsigned primNum) = 0;
};

#endif
