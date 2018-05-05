#pragma once
#include "d3dx9anim.h"
#include "Mesh.h"


/// フレーム
struct MYFRAME : public D3DXFRAME {
	D3DXMATRIX CombinedTransformationMatrix;
};


/// メッシュコンテナ
struct MYMESHCONTAINER : public D3DXMESHCONTAINER {
	LPDIRECT3DTEXTURE9*  ppTextures;
	unsigned long maxFaceInfl;			// 1面あたりのボーンの最大数
	unsigned long numBoneCombination;	// ボーンコンビネーションの数
	LPD3DXBUFFER pBoneBuffer;			// ボーン・テーブル
	D3DXMATRIX** ppBoneMatrix;			// 全てのボーンのワールド行列の先頭ポインター
	D3DXMATRIX* pBoneOffsetMatrices;
};


/// フレーム描画用関数群
class FrameRendering {
private:
	virtual void RenderMeshContainer(LPDIRECT3DDEVICE9 pDevice, MYMESHCONTAINER* pMeshContainer, MYFRAME* pFrame);	/// メッシュコンテナのレンダリング
public:
	virtual void UpdateMatrices(LPD3DXFRAME pFrame, LPD3DXMATRIX world);	/// フレームのワールド変換
	virtual void Draw(LPDIRECT3DDEVICE9 pDevice, LPD3DXFRAME pFrame);		/// フレームのレンダリング
};


/// 階層構造生成消去関数群
class MyHierarchy : public ID3DXAllocateHierarchy {
private:
	void CreateTexture(MYMESHCONTAINER* pMeshContainer, LPDIRECT3DDEVICE9 pDevice, DWORD NumMaterials);
	void SetDefaultMaterial(MYMESHCONTAINER* pMeshContainer);
public:
	STDMETHOD(CreateFrame)(THIS_ LPCSTR , LPD3DXFRAME *);
	STDMETHOD(CreateMeshContainer)(THIS_ LPCSTR, CONST D3DXMESHDATA*, CONST D3DXMATERIAL*, CONST D3DXEFFECTINSTANCE*, DWORD, CONST DWORD*, LPD3DXSKININFO, LPD3DXMESHCONTAINER*);
	STDMETHOD(DestroyFrame)(THIS_ LPD3DXFRAME);
	STDMETHOD(DestroyMeshContainer)(THIS_ LPD3DXMESHCONTAINER);
};


/// アニメーションメッシュ
class AnimationMesh : public Drawable3D {
protected:
	LPDIRECT3DDEVICE9			pDevice;			// Direct3Dデバイスオブジェクト
	LPD3DXFRAME					pFrameRoot;			// フレーム
	LPD3DXANIMATIONCONTROLLER	pAnimController;	// アニメーションコントローラ
	LPD3DXANIMATIONSET			pAnimSet[100];		// アニメーションセット
	HRESULT AllocateBoneMatrix(LPD3DXMESHCONTAINER);// メッシュに行列を割り振る
	HRESULT AllocateAllBoneMatrices(LPD3DXFRAME);	// 再帰コード：フレーム階層を展開して各メッシュに行列を割り振る
	void FreeAnim(LPD3DXFRAME pF);					// 再帰コード：全てのメッシュコンテナをリリース
	FrameRendering				frameOperator;		// フレーム描画用関数群
	MyHierarchy					myHierarchy;		// 階層構造生成消去関数群
	AnimationMesh(LPDIRECT3DDEVICE9 pDevice);		// オブジェクト生成禁止
public:
	virtual ~AnimationMesh();
	virtual void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);/// 描画
	virtual void AdvanceTime(double time);			/// 時間を進行
	virtual void ChangeMotion(int motion_number);	/// 動きを切り替える
	virtual LPD3DXFRAME GetFrame();					/// フレームを取得
};


/// Xファイルから読込んだアニメーションメッシュ
class XFileAnimationMesh : public AnimationMesh {
public:
	XFileAnimationMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDevice);
};
