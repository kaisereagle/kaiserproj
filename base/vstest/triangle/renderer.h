#ifndef __RENDERER_H__
#define __RENDERER_H__

#include "vertex.h"

class Renderer {
public:
	// ������
	virtual bool initialize() = 0;

	// �㏈��
	virtual void terminate() = 0;

	// �`��J�n
	virtual void begin() = 0;

	// �X���b�v
	virtual void swap() = 0;

	// �`��I���
	virtual void end() = 0;

	// ���_�o�b�t�@��`��
	virtual void render(Vertex *vtx, unsigned vtxNum, unsigned primNum) = 0;
};

#endif
