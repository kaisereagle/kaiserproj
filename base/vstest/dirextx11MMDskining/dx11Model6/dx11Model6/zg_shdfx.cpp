#include "stdafx.h"

#include "zg_shdfx.h"
#include "zg_renctx11.h"

#include <d3d11.h>
#include <d3dcompiler.h>

#include <fstream>

using namespace DirectX;
zix11::RenContextX11& GetRenContext();

namespace {

// シェーダのコンパイル
HRESULT CompileShaderFromFile( const WCHAR* fname, LPCSTR entry_point, LPCSTR shd_model, ID3DBlob** pp_blob_out )
{
	HRESULT hr = S_OK;

	DWORD shd_flags = D3DCOMPILE_ENABLE_STRICTNESS;
#if defined( DEBUG ) || defined( _DEBUG )
	shd_flags |= D3DCOMPILE_DEBUG;
#endif

	ID3DBlob* err_blob;
	hr = D3DCompileFromFile( fname, NULL, D3D_COMPILE_STANDARD_FILE_INCLUDE,
					entry_point, shd_model, 
					shd_flags, 0, pp_blob_out, &err_blob );
	if( FAILED(hr) ){
		if( err_blob != NULL ){
			OutputDebugStringA( (char*)err_blob->GetBufferPointer() );
		}
		if( err_blob ) err_blob->Release();
		return hr;
	}
	if( err_blob ){
		err_blob->Release();
	}
	return S_OK;
}

}
namespace zix11{

//--------------------------------------------
SEffectPtr CreateShaderEffectFromFile(const WCHAR* shader_file)
{
	RenContextX11& renctx = GetRenContext();

	ShaderFXDesc shdfx_desc;
	{
		std::ifstream input;
		input.open(shader_file, std::ios::binary);
		if(input.is_open()){
			size_t fsize = (size_t)input.seekg(0, std::ios::end).tellg();
			input.seekg(0, std::ios::beg);
			BinObject blob(fsize+1);
			if(blob.get()){
				char* b = static_cast<char*>(blob.get());
				input.read(b, blob.size());
				b[fsize] = 0;
				//ZGSFX解析
				if(!CreateShaderFxDesc(b, blob.size(), shdfx_desc)){
					return SEffectPtr();
				}
			}
		}
	}
	SEffectPtr seff(new ShaderEffect);
	if(!seff)return seff;

	HRESULT hr;
	for(auto& tech_desc : shdfx_desc.aTech){
		seff->aTech.push_back( ShaderEffect::TECHNIQUE(tech_desc.strName) );
		ShaderEffect::TECHNIQUE& tech = seff->aTech.back();
		for(auto& pass_desc : tech_desc.aPass){
			tech.aPass.push_back( ShaderEffect::PASS() );
			ShaderEffect::PASS& pass = tech.aPass.back();

			// 頂点シェーダコンパイル
			hr = CompileShaderFromFile( shader_file,
								pass_desc.shdVertex.strEntry.c_str(),
								pass_desc.shdVertex.strTarget.c_str(),
								GSET_DXPTR(pass.pVSCode) );
			if( FAILED( hr ) ){ return SEffectPtr(); }
			
			// 頂点シェーダ作成
			hr = renctx.pDev->CreateVertexShader(
								pass.pVSCode->GetBufferPointer(),
								pass.pVSCode->GetBufferSize(),
								NULL, GSET_DXPTR(pass.pVShader) );
			if( FAILED( hr ) ){	return SEffectPtr(); }
			// ピクセルシェーダのコンパイル
			hr = CompileShaderFromFile( shader_file,
								pass_desc.shdPixel.strEntry.c_str(),
								pass_desc.shdPixel.strTarget.c_str(),
								GSET_DXPTR(pass.pPSCode) );
			if( FAILED( hr ) ){ return SEffectPtr(); }
			
			// ピクセルシェーダ作成
			hr = renctx.pDev->CreatePixelShader(
								pass.pPSCode->GetBufferPointer(),
								pass.pPSCode->GetBufferSize(),
								NULL, GSET_DXPTR(pass.pPShader) );
			if( FAILED( hr ) ){ return SEffectPtr(); }

			//ステート
			// ラスタライザステート
			hr = renctx.pDev->CreateRasterizerState(&pass_desc.descRS, GSET_DXPTR(pass.pRsState));
			if( FAILED( hr ) ){ return SEffectPtr(); }

			// ブレンドステート
			hr = renctx.pDev->CreateBlendState(&pass_desc.descBD, GSET_DXPTR(pass.pBdState));
			if( FAILED( hr ) ){ return SEffectPtr(); }

			// デプスステンシルステート
			hr = renctx.pDev->CreateDepthStencilState(&pass_desc.descDS, GSET_DXPTR(pass.pDsState));
			if( FAILED( hr ) ){ return SEffectPtr(); }

		}
	}

	return seff;
}

}//zix11