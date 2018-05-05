#pragma once
#include "Mesh.h"

// ���[���h��Ԃ̐�΍��W
class Coord {
private:
	struct ColorPoint {
		D3DXVECTOR3 position;
		unsigned long color;
	};
	const int numAxis, numNet;
	ColorPoint* axis;
	ColorPoint* net;
	LPDIRECT3DDEVICE9 pDevice;
public:
	Coord(LPDIRECT3DDEVICE9 pDevice);
	~Coord();
	void Draw(Camera* camera);
};