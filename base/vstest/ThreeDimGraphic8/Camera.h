#pragma once

/// ���b�V���̃r���[�ϊ��A�ˉe�ϊ����s�Ȃ��J����
class Camera {
private:
	D3DXMATRIX view, projection;
public:
	Camera(D3DXVECTOR3 eyePoint, D3DXVECTOR3 lookAtPoint);
	void GetMatrix(D3DXMATRIX* view, D3DXMATRIX* projection);
};
