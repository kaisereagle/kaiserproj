#pragma once

// DirectX11モデルデータ

#include "zg_types.h"
#include "zg_utils.h"
#include "zg_model.h"

#include "zg_ptrx11.h"

#include <directxmath.h>
#include <vector>
#include <string>

namespace zix11{

class ShaderPass;
class ShaderTech;
class MeshX11;
class ModelObjectX11;
class ModelX11;
typedef SPtr<ShaderPass>::Type SPassPtr;
typedef SPtr<ShaderTech>::Type STechPtr;
typedef SPtr<MeshX11>::Type MeshX11Ptr;
typedef SPtr<ModelObjectX11>::Type ModelObjX11Ptr;
typedef SPtr<ModelX11>::Type ModelX11Ptr;

// モデル
class ModelX11
{
public:
	ModelX11() : numBoneMtx(0), sizeBoneMtx(0){}
	std::vector<ModelObjX11Ptr> apObj;
	IBuffPtr pCBModel;//SCBModel定数バッファ
	IBuffPtr pCBBoneMtx;//ボーン行列
	zgU32 numBoneMtx;
	zgU32 sizeBoneMtx;
};

// オブジェクト
class ModelObjectX11
{
public:
	std::vector<MeshX11Ptr> apMesh;
	IBuffPtr pCBObject;//SCBObject定数バッファ
};

// メッシュ
class MeshX11
{
public:
	MeshX11(){}
	~MeshX11(){}
	static const zgU32 TECH_MAXNUM = 8;
	zgU32 numTech;
	Array<STechPtr, TECH_MAXNUM>::Type apTech;
	IBuffPtr pCBMateiral;	//SCBMaterial定数バッファ
private:
};

//----------------------------------
// シェーダテクニック
class ShaderTech
{
public:
	static const zgU32 PASS_MAXNUM = 4;
	zgU32 numPass;
	Array<SPassPtr, PASS_MAXNUM>::Type apPass;	
};

//----------------------------------
// シェーダパス（描画の最小単位）
class ShaderPass
{
public:
	static const zgU32 VBUFF_MAXNUM = 8;
	static const zgU32 TEX_MAXNUM = 8;
	static const zgU32 CBUFF_MAXNUM = 8;//16;
	// 数が少ないので固定の配列
	
	//頂点データ
	zgU32 numVBuff;
	Array<UINT, VBUFF_MAXNUM>::Type aStride;
	Array<UINT, VBUFF_MAXNUM>::Type aOffset;
	Array<IBuffPtr, VBUFF_MAXNUM>::Type	apVBuff;	
	IBuffPtr	pIBuff;
	IILayoutPtr	pLayout;

	// 状態
	IRsStatePtr pRsState;
	IBdStatePtr pBdState;
	IDsStatePtr pDsState;

	// シェーダ
	IVShaderPtr	pVShader;
	IPShaderPtr	pPShader;

	//テクスチャ
	zgU32 numTex;
	Array<ISRViewPtr, TEX_MAXNUM>::Type	apSRView;	
	Array<ISpStatePtr, TEX_MAXNUM>::Type apSpState;

	//定数バッファ
	zgU32 numCBuff;
	Array<IBuffPtr, CBUFF_MAXNUM>::Type apCBuff;
	//とりあえず頂点とピクセルで定数共有

	zgU32 nCount;
	D3D_PRIMITIVE_TOPOLOGY eTopo;
	ShaderPass() : numVBuff(0), numTex(0), numCBuff(0), nCount(0)
					,eTopo(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST){}
};

//-------------------------------------------
// DX11レンダリングモデルの作成
ModelX11Ptr CreateModelX11(zgmdl::Model& model);






}//zix11 
