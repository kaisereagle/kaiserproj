#pragma once
#include "Camera.h"

/// �`��C���^�[�t�F�[�X
class Drawable3D {
public:
	virtual ~Drawable3D(){};
	virtual void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera) = 0;
};
