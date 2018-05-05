#include "StdAfx.h"
#include "MyApplication.h"

MyApplication::MyApplication(HWND hWnd) {
	directXFramework = new DirectXFramework(hWnd);
	LPDIRECT3DDEVICE9 pDevice = directXFramework->GetD3DDevice();
	skinMesh = new MmdSkinMesh(TEXT("tda_nude.pmd"), TEXT("love&joy���ʖ���ver.vmd"), pDevice);
	coord = new Coord(pDevice);
}

MyApplication::~MyApplication() {
	delete coord;
	delete skinMesh;
	delete directXFramework;
}

void MyApplication::Run() {
	// ���C�g
	D3DLIGHT9 light = {D3DLIGHT_DIRECTIONAL, {1, 1, 1, 0}, {1, 1, 1, 0}, {1, 1, 1, 0}};	// �F
	light.Direction = D3DXVECTOR3(-1, -1, 1);		// ����

	// �J����
	D3DXVECTOR3 eyePoint = D3DXVECTOR3(0.5f, 1.0f, -5);	// ���_
	D3DXVECTOR3 lookAtPoint = D3DXVECTOR3(0, 0.8f, 0);		// �����_
	Camera camera(eyePoint, lookAtPoint);

	// �`��
	D3DXVECTOR3 position(0, 0, 0);	// �ʒu
	D3DXMATRIX rotation;			// ��]
	static int time;
	float speed = 0;
	D3DXMatrixRotationY(&rotation, ++time*speed);
	skinMesh->AdvanceTime();
	directXFramework->BeginScene(240, 180, 180);// �V�[���J�n
	coord->Draw(&camera);
	skinMesh->Draw(&position, &rotation, &light, &camera);
	//skinMesh->DrawBoneObj(&position, &rotation, &light, &camera);
	directXFramework->EndScene();	// �V�[���I��
}