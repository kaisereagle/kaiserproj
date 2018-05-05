#pragma once
#include "Mesh.h"
#include "XFileAnimationMesh.h"

// 平面投影法による影
class Shadow : public Drawable3D {
protected:
	LPDIRECT3DDEVICE9	pDevice;
	LPD3DXMESH			pMesh;
	int					NumMaterial;		// メッシュのマテリアル数
	D3DMATERIAL9		shadow_material;	// 影のマテリアル
	D3DXPLANE			plane;				// 影の落ちる平面
	void StencileEnable();					// ステンシルバッファを有効にする
	void StencileDisable();					// ステンシルバッファを無効にする

public:
	// @param pDevice		D3Dデバイス
	// @param pMesh			メッシュ
	// @param NumMaterial	メッシュの材料数
	Shadow(LPDIRECT3DDEVICE9 pDevice, LPD3DXMESH pMesh, int NumMaterial);

	// @param plane		影の落ちる平面
	virtual void SetPlane(D3DXPLANE* plane);

	virtual void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);
};


// マテリアルとテクスチャをセットしない影用フレーム描画用関数群
class FrameRenderingForShadow : public FrameRendering {
private:
	void RenderMeshContainer(LPDIRECT3DDEVICE9, MYMESHCONTAINER* ,MYFRAME*) override;
};


// XFileアニメーション用 平面投影法による影 
class XFileAnimationShadow : public Shadow {
	FrameRenderingForShadow frameRenderingForShadow;
	LPD3DXFRAME pFrame;
public:
	// @param pDevice	D3Dデバイス
	// @param pFrame	フレーム
	XFileAnimationShadow(LPDIRECT3DDEVICE9 pDevice, LPD3DXFRAME pFrame);

	void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);
};


// シャドウボリュームクラス
class ShadowVolume : public Drawable3D {
private:
	LPDIRECT3DDEVICE9	pDevice;
	LPD3DXMESH			pMesh;
	unsigned long		numVertex;			// シャドウボリュームの頂点の数
	D3DXVECTOR3			vertex[1000000];		// シャドウボリュームの頂点の位置データ
	void CreateVolume(D3DXVECTOR3 direction);
	void RenderVolume(D3DXMATRIX world);
	void RenderVolumeToStencil(D3DXMATRIX world);
	void RenderShadowBoard();
	D3DXMATRIX GetWorldMatrix(D3DXVECTOR3* position, D3DXMATRIX* rotation);
	D3DXVECTOR3 GetLocalDirection(D3DXMATRIX* rotation, D3DXVECTOR3* direction);
public:
	ShadowVolume(LPDIRECT3DDEVICE9 pDevice, LPD3DXMESH pMesh);
	void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);
	void DrawVolume(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light);
};
