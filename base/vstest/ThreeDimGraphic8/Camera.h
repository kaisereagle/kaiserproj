#pragma once

/// メッシュのビュー変換、射影変換を行なうカメラ
class Camera {
private:
	D3DXMATRIX view, projection;
public:
	Camera(D3DXVECTOR3 eyePoint, D3DXVECTOR3 lookAtPoint);
	void GetMatrix(D3DXMATRIX* view, D3DXMATRIX* projection);
};
