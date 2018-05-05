#pragma once

#include "zg_utils.h"
#include "zg_ptrx11.h"
#include "zg_shdfx.h"

#include <directxmath.h>


using DirectX::XMMATRIX;
using DirectX::XMVECTOR;
using DirectX::XMFLOAT4;


namespace zix11{

//シェーダ定数バッファ

// 固定パラメータ
// プログラムでの固定値などほぼ更新されないパラメータ
struct SCBFix
{
	static const zgU32 CBSLOT = 0;//定数バッファスロット番号
	//なし
};

// シーンパラメータ
// 透視変換行列などのパラメータ
struct SCBScene
{
	static const zgU32 CBSLOT = 1;//定数バッファスロット番号
	XMMATRIX mtxProj;	// ViewProjection行列
	XMMATRIX mtxView;	// WorldView行列
};

// 光源パラメータ
struct SCBLight
{
	static const zgU32 CBSLOT = 2;//スロット番号

	// 平行光源
	static const zgU32 DIRLIGHT_NUM = 4;
	XMVECTOR vDirLight[DIRLIGHT_NUM];
	XMVECTOR colDirLight[DIRLIGHT_NUM];

	//半球ライティング
	XMVECTOR vHemiDir;
	XMVECTOR colHemiSky;
	XMVECTOR colHemiGrd;

	//点光源
	static const zgU32 PLIGHT_NUM = 4;
	XMVECTOR posPLight[PLIGHT_NUM];
	XMVECTOR colPLight[PLIGHT_NUM];
	XMVECTOR prmPLight[PLIGHT_NUM];
};
// モデルパラメータ
struct SCBModel
{
	static const zgU32 CBSLOT = 3;//スロット番号
	XMVECTOR colModel;	// モデル色　RGBA
};
// オブジェクトパラメータ
// 姿勢（ワールド）行列などのパラメータ
struct SCBObject
{
	static const zgU32 CBSLOT = 4;//スロット番号
	XMMATRIX mtxWorld;	//WorldLocal行列
};

// マテリアルパラメータ
struct SCBMaterial
{
	static const zgU32 CBSLOT = 5;//スロット番号
	XMVECTOR vDiffuse;
	XMVECTOR vSpecular;
};

// ローカルパラメータ
// シェーダ固有のパラメータ
struct SCBLocal
{
	static const zgU32 CBSLOT = 6;//スロット番号
	//シェーダ毎に定義
};


struct SCBBoneMatrix
{
	static const zgU32 CBSLOT = 7;//スロット番号
	XMVECTOR mtxBone[1];//float3x4
	// ボーン数で可変
	static zgU32 getSize(zgU32 bone_num)
	{
		return sizeof(XMVECTOR)*3*bone_num;
	}
};

//DirectX11レンダリング環境
struct RenContextX11
{
	IDevicePtr pDev;
	IDevCtxPtr pImCtx;
	ISChainPtr pSwapChain;
	IRTViewPtr pRTView;
	ITex2DPtr  pDepthS;
	IDSViewPtr pDSView;

	//シェーダ定数バッファ
	SCBScene scbScene;
	SCBLight scbLight;

	IBuffPtr pCBScene;
	IBuffPtr pCBLight;

	// リソースライブラリ
	ResLibrary<ISRViewPtr> libTexImage;//テクスチャ画像
	ResLibrary<SEffectPtr> libEffect;//シェーダエフェクト

	void Clear(){
		pCBScene.reset();
		pCBLight.reset();
		pDSView.reset();
		pDepthS.reset();
		pRTView.reset();
		pSwapChain.reset();
		pImCtx.reset();
		pDev.reset();
	}
};

}//zix11
