#include "StdAfx.h"
#include "MyApplication.h"

MyApplication::MyApplication(HWND hWnd) {
	directXFramework = new DirectXFramework(hWnd);
	LPDIRECT3DDEVICE9 pDevice = directXFramework->GetD3DDevice();
	skinMesh = new MmdSkinMesh(TEXT("tda_nude.pmd"), TEXT("love&joyお面無しver.vmd"), pDevice);
	coord = new Coord(pDevice);
}

MyApplication::~MyApplication() {
	delete coord;
	delete skinMesh;
	delete directXFramework;
}

void MyApplication::Run() {
	// ライト
	D3DLIGHT9 light = {D3DLIGHT_DIRECTIONAL, {1, 1, 1, 0}, {1, 1, 1, 0}, {1, 1, 1, 0}};	// 色
	light.Direction = D3DXVECTOR3(-1, -1, 1);		// 方向

	// カメラ
	D3DXVECTOR3 eyePoint = D3DXVECTOR3(0.5f, 1.0f, -5);	// 視点
	D3DXVECTOR3 lookAtPoint = D3DXVECTOR3(0, 0.8f, 0);		// 注視点
	Camera camera(eyePoint, lookAtPoint);

	// 描画
	D3DXVECTOR3 position(0, 0, 0);	// 位置
	D3DXMATRIX rotation;			// 回転
	static int time;
	float speed = 0;
	D3DXMatrixRotationY(&rotation, ++time*speed);
	skinMesh->AdvanceTime();
	directXFramework->BeginScene(240, 180, 180);// シーン開始
	coord->Draw(&camera);
	skinMesh->Draw(&position, &rotation, &light, &camera);
	//skinMesh->DrawBoneObj(&position, &rotation, &light, &camera);
	directXFramework->EndScene();	// シーン終了
}