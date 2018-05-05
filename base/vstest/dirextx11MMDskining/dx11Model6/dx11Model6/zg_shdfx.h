#pragma once

#include "zg_types.h"
#include "zg_utils.h"
#include "zg_ptrx11.h"
#include "zg_shdfx_desc.h"

namespace zix11{
class ShaderEffect;
typedef SPtr<ShaderEffect>::Type SEffectPtr;

// シェーダエフェクト
class ShaderEffect
{
public:
	struct PASS{
		//DirectX11リソース（インターフェイス）
		IVShaderPtr pVShader;	// 頂点シェーダ
		IBlobPtr pVSCode;		// 頂点シェーダバイナリコード　レイアウト作成用
		IPShaderPtr pPShader;	// ピクセルシェーダ
		IBlobPtr pPSCode;		// ピクセルシェーダバイナリコード

		IRsStatePtr pRsState;	// ラスタライザステート
		IBdStatePtr pBdState;	// ブレンドステート
		IDsStatePtr pDsState;	// デプスステンシルステート
	};

	struct TECHNIQUE{
		TECHNIQUE(){}
		TECHNIQUE(const std::string& name) : strName(name){}
		std::string strName;
		std::vector<PASS> aPass;
	};

	std::vector<TECHNIQUE> aTech;
};

//--------------------------------------------
// zgfxファイルから構築
SEffectPtr CreateShaderEffectFromFile(const WCHAR* file);


}//zix11
