#ifndef __DX9_DX9RENDERER_H__
#define __DX9_DX9RENDERER_H__

#include "../renderer.h"
#include "d3d9.h"

class DX9Renderer : public Renderer {
	HWND hWnd;
	LPDIRECT3D9 pD3D;
	LPDIRECT3DDEVICE9 pDev;

public:
	DX9Renderer(HWND hWnd) : hWnd(hWnd) {}

	// ������
	virtual bool initialize() {
		if(!(pD3D = Direct3DCreate9( D3D_SDK_VERSION )))
			return false;

		D3DPRESENT_PARAMETERS d3dpp = {640, 480, D3DFMT_A8R8G8B8, 0, D3DMULTISAMPLE_NONE, 0, D3DSWAPEFFECT_DISCARD, NULL, TRUE, TRUE, D3DFMT_D24S8, 0, D3DPRESENT_RATE_DEFAULT, D3DPRESENT_INTERVAL_ONE};

		if( FAILED( pD3D->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_HARDWARE_VERTEXPROCESSING, &d3dpp, &pDev)))
		if( FAILED( pD3D->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_SOFTWARE_VERTEXPROCESSING, &d3dpp, &pDev)))
		{
			pD3D->Release();
			return false;
		}

		return true;
	}

	// �`��J�n
	virtual void begin() {
		pDev->BeginScene();
		pDev->Clear(0, 0, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, D3DCOLOR_COLORVALUE(0.0f, 0.5f, 1.0f, 1.0f), 1.0f, 0);
	}

	// �X���b�v
	virtual void swap() {
		pDev->EndScene();
		pDev->Present(0, 0, 0, 0);
	}

	// �`��I���
	virtual void end() {
		// DirectX�ł͉������܂���
	}

	// �㏈��
	virtual void terminate() {
		if (pDev) pDev->Release();
		if (pD3D) pD3D->Release();
	}

	// ���_�o�b�t�@��`��
	virtual void render(Vertex *vtx, unsigned vtxNum, unsigned primNum) {
		pDev->SetRenderState(D3DRS_LIGHTING, FALSE);
		pDev->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
		pDev->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, primNum, vtx, sizeof(Vertex));
	}
};

#endif
