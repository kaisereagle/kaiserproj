#include "stdafx.h"

#include "zg_dx11.h"
#include <d3d11.h>
#include <d3dcompiler.h>
#include <directxmath.h>
#include "DirectXTex/DirectXTex.h"

#include "zg_mqo.h"
#include "zg_mmd.h"
#include "zg_model.h"

#include "zg_renctx11.h"
#include "zg_ptrx11.h"
#include "zg_mdlx11.h"
#include "zg_shdfx.h"

#include <fstream>
#include <string>
#include <stdlib.h>
#include <locale>

#include "sample_data.h"

using namespace zix11;
using namespace DirectX;
namespace{

struct MATRIX43
{
	XMVECTOR r[3];
};

XMVECTOR ToXMVec(const ZGVECTOR3& v)
{
	return XMVectorSet(v.x,v.y,v.z, 0);
}
XMVECTOR ToXMVecW1(const ZGVECTOR3& v)
{
	return XMVectorSet(v.x,v.y,v.z, 1);
}
XMVECTOR ToXMVec(const ZGVECTOR4& v)
{
	return XMVectorSet(v.x,v.y,v.z,v.w);
}
XMVECTOR ToXMVec(const ZGQUAT& v)
{
	return XMVectorSet(v.x,v.y,v.z,v.w);
}
ZGVECTOR3 ToVec3(XMVECTOR v)
{
	return ZGVECTOR3(XMVectorGetX(v),XMVectorGetY(v),XMVectorGetZ(v));
}
ZGQUAT ToQuat(XMVECTOR v)
{
	return ZGQUAT(XMVectorGetX(v),XMVectorGetY(v),XMVectorGetZ(v),XMVectorGetW(v));
}

// スケール、回転、移動行列
XMMATRIX SRTMatrix(XMVECTOR scale, XMVECTOR quat_rot, XMVECTOR trans)
{
	return XMMatrixMultiply(
					XMMatrixMultiply(
						XMMatrixScalingFromVector( scale),
						XMMatrixRotationQuaternion( quat_rot) ),
					XMMatrixTranslationFromVector( trans ) );
}

// http://d.hatena.ne.jp/edvakf/20111016/1318716097
float Bezier(float x1, float x2, float y1, float y2, float x)
{
	x1 = x1/127.0f;
	y1 = y1/127.0f;
	x2 = x2/127.0f;
	y2 = y2/127.0f;
	float t = 0.5f, s = 0.5f;
	for (int i = 0; i < 15; i++) {
		float ft = (3*s*s *t *x1) + (3*s *t*t *x2) + (t*t*t) - x;
		if (ft == 0) break;
		if (ft > 0){
			t -= 1.0f / float(4 << i);
		}else{ // ft < 0
			t += 1.0f / float(4 << i);
		}
		s = 1 - t;
	}
	return (3*s*s *t *y1) + (3*s *t*t *y2) + (t*t*t);
}

//----------------------------------------
// とりあえずVMDアニメーション再生＆IK
struct MeshBone
{
	XMVECTOR vScale;
	XMVECTOR qRot;
	XMVECTOR vPos;
	zgU32 idxParent;
	char name[32];

	//IK回転制限
	XMVECTOR ik_min;
	XMVECTOR ik_max;
};
//----------
struct KeyFrame
{
	zgU32 time;
	ZGVECTOR3 pos;
	ZGQUAT rot;
	zgS8 Intp[64];
	bool operator<(const KeyFrame& o){ return time < o.time;}//ソート用
};
//----------
struct MotionData
{
	std::string strBoneName;
	std::vector<KeyFrame> Key;
};
//----------
struct Motion
{
	std::vector<MotionData> aData;
};
// とりあえずVMDモーション再生
//----------

//-------------
struct MeshModel
{
	ModelX11Ptr pModel;
	ZGVECTOR3 v3Pos;
	ZGVECTOR4 colModel;

	//ボーン情報
	zgU32 numBone;
	BinObject binBone;
	BinObject binMtxPose;//姿勢行列
	BinObject binMtxInitBone;	//初期ボーン行列
	BinObject binMtxSubBone;	//差分ボーン行列（シェーダへ）

	// とりあえずアニメーション＆IK
	std::vector<PMDIk> aIK;
	Motion Mot;

	//姿勢行列更新
	void UpdatePose()
	{
		MeshBone* bone = reinterpret_cast<MeshBone*>(binBone.get());
		XMMATRIX* mtx_pose = reinterpret_cast<XMMATRIX*>(binMtxPose.get());

		// 階層計算
		for(zgU32 ib=0;ib<numBone;++ib){
			mtx_pose[ib] = SRTMatrix(bone[ib].vScale, bone[ib].qRot, bone[ib].vPos);

			if( bone[ib].idxParent < numBone){
				mtx_pose[ib] = XMMatrixMultiply(mtx_pose[ib], mtx_pose[bone[ib].idxParent]);
			}
		}
	}
};

}



bool					g_InitDX11 = false;//2重初期化防止
D3D_DRIVER_TYPE			g_DriverType = D3D_DRIVER_TYPE_NULL;
D3D_FEATURE_LEVEL		g_FeatureLevel = D3D_FEATURE_LEVEL_11_0;

RenContextX11 g_RenCtx;
std::vector<SPtr<MeshModel>::Type> g_aModel;


RenContextX11& GetRenContext()
{
	return g_RenCtx;
}


//---------------------------------------------------------
HRESULT InitDX11(HWND hWnd)
{
	if(g_InitDX11)return S_FALSE;//すでに初期化
	
	HRESULT hr = S_OK;

	RECT rc;
	GetClientRect( hWnd, &rc );
	UINT width = rc.right - rc.left;
	UINT height = rc.bottom - rc.top;

	UINT cdev_flags = 0;
#ifdef _DEBUG
	cdev_flags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
	RenContextX11& renctx =  GetRenContext();

	// DirectX11&ハードウェア対応のみ
	D3D_FEATURE_LEVEL feature_level = D3D_FEATURE_LEVEL_11_0;
	g_DriverType = D3D_DRIVER_TYPE_HARDWARE;


	//デバイス作成
	hr = D3D11CreateDevice(	NULL, D3D_DRIVER_TYPE_HARDWARE,	NULL, cdev_flags,
							&feature_level, 1, D3D11_SDK_VERSION, GSET_DXPTR(renctx.pDev),
							&g_FeatureLevel, GSET_DXPTR(renctx.pImCtx) );
	if( FAILED( hr ) ){
		return hr;
	}
	
	//使用可能なMSAAを取得
	DXGI_SAMPLE_DESC smpl_desc;
	//for(int i=0; i <= D3D11_MAX_MULTISAMPLE_SAMPLE_COUNT; i++){
	for(int i=0; i <= 4; i++){
		UINT Quality;
		if(SUCCEEDED(renctx.pDev->CheckMultisampleQualityLevels(DXGI_FORMAT_R8G8B8A8_UNORM, i, &Quality))){
			if(0 < Quality){
				smpl_desc.Count = i;
				smpl_desc.Quality = Quality - 1;
			}
		}
	}
	DXPtr<IDXGIDevice>::Type dxgi_dev;
	hr = renctx.pDev->QueryInterface(__uuidof(IDXGIDevice), (void**)GSET_DXPTR(dxgi_dev));
	if( FAILED( hr ) ){
		return hr;
	}
	DXPtr<IDXGIAdapter>::Type dxgi_adapter;
	hr = dxgi_dev->GetAdapter(GSET_DXPTR(dxgi_adapter));
	if( FAILED( hr ) ){
		return hr;
	}
	DXPtr<IDXGIFactory>::Type dxgi_factory;
	hr = dxgi_adapter->GetParent(__uuidof(IDXGIFactory), (void**)GSET_DXPTR(dxgi_factory));
	if( FAILED( hr ) ){
		return hr;
	}

	// スワップチェイン作成
	DXGI_SWAP_CHAIN_DESC sd;
	ZeroMemory( &sd, sizeof( sd ) );
	sd.BufferCount = 1;
	sd.BufferDesc.Width = width;
	sd.BufferDesc.Height = height;
	sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	sd.BufferDesc.RefreshRate.Numerator = 60;
	sd.BufferDesc.RefreshRate.Denominator = 1;	//1/60 = 60fps
	sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	sd.OutputWindow = hWnd;
	sd.SampleDesc = smpl_desc;
	sd.Windowed = TRUE;
	hr = dxgi_factory->CreateSwapChain(renctx.pDev.get(), &sd, GSET_DXPTR(renctx.pSwapChain));
	if( FAILED( hr ) ){
		return hr;
	}


	// スワップチェインに用意されたバッファ（2Dテクスチャ）を取得
	ITex2DPtr back_buff;
	hr = renctx.pSwapChain->GetBuffer( 0, __uuidof( ID3D11Texture2D ), (LPVOID*)GSET_DXPTR(back_buff) );
	if( FAILED( hr ) ){
		return hr;
	}

	hr = renctx.pDev->CreateRenderTargetView( back_buff.get(), NULL, GSET_DXPTR(renctx.pRTView) );
	if( FAILED( hr ) ){
		return hr;
	}

	// デプスバッファ作成
	D3D11_TEXTURE2D_DESC decs_depth;
	ZeroMemory( &decs_depth, sizeof(decs_depth) );
	decs_depth.Width = width;
	decs_depth.Height = height;
	decs_depth.MipLevels = 1;
	decs_depth.ArraySize = 1;
	decs_depth.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	decs_depth.SampleDesc = smpl_desc;
	decs_depth.Usage = D3D11_USAGE_DEFAULT;
	decs_depth.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	decs_depth.CPUAccessFlags = 0;
	decs_depth.MiscFlags = 0;
	hr = renctx.pDev->CreateTexture2D( &decs_depth, NULL, GSET_DXPTR(renctx.pDepthS) );
	if( FAILED( hr ) ){
		return hr;
	}

	// DepthStencilView作成
	D3D11_DEPTH_STENCIL_VIEW_DESC descDSV;
	ZeroMemory( &descDSV, sizeof(descDSV) );
	descDSV.Format = decs_depth.Format;
	descDSV.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2DMS;
	
	
	hr = renctx.pDev->CreateDepthStencilView( renctx.pDepthS.get(), &descDSV, GSET_DXPTR(renctx.pDSView) );
	if( FAILED( hr ) ){
		return hr;
	}
	
	// ターゲットビュー
	ID3D11RenderTargetView* const rtvs[] = {renctx.pRTView.get(),};
	renctx.pImCtx->OMSetRenderTargets( 1, rtvs, renctx.pDSView.get() );

	// viewport
	D3D11_VIEWPORT vp;
	vp.Width = (FLOAT)width;
	vp.Height = (FLOAT)height;
	vp.MinDepth = 0.0f;
	vp.MaxDepth = 1.0f;
	vp.TopLeftX = 0;
	vp.TopLeftY = 0;
	renctx.pImCtx->RSSetViewports( 1, &vp );

	D3D11_BUFFER_DESC bd;
	ZeroMemory( &bd, sizeof(bd) );
	// 定数バッファ作成
	bd.Usage = D3D11_USAGE_DEFAULT;
	bd.CPUAccessFlags = 0;
	bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	
	// 定数バッファ　シーンパラメータ
	bd.ByteWidth = sizeof(SCBScene);
	hr = renctx.pDev->CreateBuffer( &bd, NULL, GSET_DXPTR(renctx.pCBScene) );
	if( FAILED( hr ) ){
		return hr;
	}

	// 定数バッファ　光源パラメータ
	bd.ByteWidth = sizeof(SCBLight);
	hr = renctx.pDev->CreateBuffer( &bd, NULL, GSET_DXPTR(renctx.pCBLight) );
	if( FAILED( hr ) ){
		return hr;
	}

	// 定数バッファ 変換行列（プロジェクション、ビュー）の更新
	SCBScene& scbscn =  renctx.scbScene;
    XMVECTOR Eye = XMVectorSet( 0.0f, 12.0f, -35.0f, 0.0f );
    XMVECTOR At = XMVectorSet( 0.0f, 12.0f, 0.0f, 0.0f );
    XMVECTOR Up = XMVectorSet( 0.0f, 1.0f, 0.0f, 0.0f );
	scbscn.mtxView = XMMatrixTranspose(XMMatrixLookAtLH( Eye, At, Up ));
	scbscn.mtxProj = XMMatrixTranspose(XMMatrixPerspectiveFovLH( XM_PIDIV4, width / (FLOAT)height, 0.1f, 1000.0f ));
	renctx.pImCtx->UpdateSubresource( renctx.pCBScene.get(), 0, NULL, &scbscn, 0, 0 );

	// 光源パラメータの更新
	SCBLight& scblight = renctx.scbLight;
	ZeroMemory( &scblight, sizeof(scblight) );
	scblight.vDirLight[0] = XMVector3Normalize( XMVectorSet(1,1,-3, 0) );
	scblight.colDirLight[0] = XMVectorSet(0.7f,0.7f,0.7f,0);
	scblight.vHemiDir = XMVectorSet(0,1,0,0);
	scblight.colHemiSky = XMVectorSet(0.2f,0.2f,0.3f,0.0f);
	scblight.colHemiGrd = XMVectorSet(0.3f,0.3f,0.2f,0.0f);
	renctx.pImCtx->UpdateSubresource( renctx.pCBLight.get(), 0, NULL, &scblight, 0, 0 );

	for(int i=0;i<2;++i){
		g_aModel.push_back(SPtr<MeshModel>::Type(new MeshModel));
	}

	{//地面
		MQOModel mqo;
		CreateMQOFromFile(L"data\\plane.mqo", mqo);
		zgmdl::ModelPtr zgmodel_mqo = CreateModel(mqo);
		g_aModel[0]->pModel = CreateModelX11(*zgmodel_mqo);
		g_aModel[0]->v3Pos = ZGVECTOR3(0,-2,0);
		g_aModel[0]->colModel = ZGVECTOR4(1,1,1,1);
	}

	{//PMDモデル
		PMDModel pmd;
		VMDMotion vmd;
		bool load_pmd = CreatePMD(PMD_MODEL_FILE, pmd);
		bool load_vmd = CreateVMD(VMD_MOTION_FILE, vmd);
		if(!load_pmd || !load_vmd){
			MessageBox(NULL,  L"PMD、VMDファイルが見つかりません。\nsample_data.hにファイルパスを設定してください。",L"",MB_OK);
		}

		zgmdl::ModelPtr zgmodel_pmd = CreateModel(pmd);
		g_aModel[1]->pModel = CreateModelX11(*zgmodel_pmd);
		g_aModel[1]->v3Pos = ZGVECTOR3(0,0,0);
		g_aModel[1]->colModel = ZGVECTOR4(1,1,1,1);
		g_aModel[1]->aIK = pmd.aIK;
	
		//仮のアニメーション処理　VMD再生とIK処理
		MeshModel& mmodel = *g_aModel[1];
		zgU32 bone_num = zgmodel_pmd->BoneNum();
		mmodel.numBone = bone_num;
		
		//行列用メモリ確保　16byte alignにしないとベクトル計算でエラー
		mmodel.binBone.Alloc(sizeof(MeshBone)*bone_num,16);
		mmodel.binMtxPose.Alloc(sizeof(XMMATRIX)*bone_num,16);
		mmodel.binMtxInitBone.Alloc(sizeof(XMMATRIX)*bone_num,16);
		mmodel.binMtxSubBone.Alloc(sizeof(XMVECTOR)*3*bone_num,16);//float3x4

		MeshBone* bone = reinterpret_cast<MeshBone*>(mmodel.binBone.get());
		XMMATRIX* mtx_pose = reinterpret_cast<XMMATRIX*>(mmodel.binMtxPose.get());
		XMMATRIX* mtx_init = reinterpret_cast<XMMATRIX*>(mmodel.binMtxInitBone.get());
		MATRIX43* mtx_sub = reinterpret_cast<MATRIX43*>(mmodel.binMtxSubBone.get());

		for(zgU32 ib=0;ib<bone_num;++ib){
			const zgmdl::Bone* mdl_bone = zgmodel_pmd->getBone(ib);
			const zgmdl::Hierarchy* mdl_hier = zgmodel_pmd->getHier(ib);
			bone[ib].idxParent = mdl_hier->idxParent;
			bone[ib].qRot = ToXMVec(mdl_bone->qRot);
			bone[ib].vPos = ToXMVec(mdl_bone->vPos);
			bone[ib].vScale = ToXMVec(mdl_bone->vScale);
			strcpy_s(bone[ib].name, mdl_bone->strName.c_str());
			bone[ib].ik_min = XMVectorSet(-XM_PI,-XM_PI,-XM_PI,0);
			bone[ib].ik_max = XMVectorSet(XM_PI,XM_PI,XM_PI,0);

			//名前に"ひざ"があったら回転制限
			if(std::string::npos != mdl_bone->strName.find("ひざ")){
				bone[ib].ik_min = XMVectorSet(-XM_PI,0,0,0);
				bone[ib].ik_max = XMVectorSet(0,0,0,0);
			}

			//ワールド行列計算
			mtx_pose[ib] = SRTMatrix( bone[ib].vScale, bone[ib].qRot, bone[ib].vPos);
			if( bone[ib].idxParent < bone_num){
				mtx_pose[ib] = XMMatrixMultiply(mtx_pose[ib], mtx_pose[bone[ib].idxParent]);
			}

			mtx_init[ib] = mtx_pose[ib];
			XMMATRIX iden = XMMatrixIdentity();
			mtx_sub[ib].r[0] = iden.r[0];
			mtx_sub[ib].r[1] = iden.r[1];
			mtx_sub[ib].r[2] = iden.r[2];
		}

		// とりあえずモーションデータ作成
		// ボーンごとに仕分け、キー値でソート
		Motion& mot = g_aModel[1]->Mot;
		for( auto& key : vmd.aKeyFrame){
			zgU32 midx = 0;
			for( auto& m : mot.aData){
				if(m.strBoneName == key.BoneName){
					break;
				}
				++midx;
			}
			if( midx >= mot.aData.size()){
				mot.aData.push_back(MotionData());
			}
			MotionData& data = mot.aData[midx];
			data.strBoneName = key.BoneName;

			KeyFrame frame;
			frame.pos = key.Location;
			frame.rot = key.Rotatation;
			frame.time = key.FlameNo;
			memcpy(frame.Intp, key.Interpolation, sizeof(frame.Intp));

			//相対値から絶対値に変更
			zgU32 bidx = 0;
			for(bidx=0;bidx<bone_num;++bidx){
				if(data.strBoneName == bone[bidx].name){
					frame.pos.v[0] += XMVectorGetX(bone[bidx].vPos);
					frame.pos.v[1] += XMVectorGetY(bone[bidx].vPos);
					frame.pos.v[2] += XMVectorGetZ(bone[bidx].vPos);
					break;
				}
			}
			//ものすごく効率悪いけど気にしない

			data.Key.push_back(frame);

		}
		// ソート　データの並びがでたらめなので
		for( auto& m : mot.aData){
			std::sort(m.Key.begin(),m.Key.end());
		}
	}

	g_InitDX11 = true;
	return S_OK;
}


//---------------------------------------------------------
void ExitDX11()
{
	//if(!g_InitDX11)return;//初期化してない
	//ここだと初期化途中で失敗した場合リソースリーク発生
	RenContextX11& renctx = GetRenContext();
	if( renctx.pImCtx ) renctx.pImCtx->ClearState();
	
	g_aModel.clear();
	
	g_RenCtx.Clear();

	if(!g_InitDX11)return;//初期化してない
	g_InitDX11 = false;
}

namespace{
struct REN_PASS
{
	REN_PASS() : iPass(0){}
	REN_PASS(zgU32 ip, const SPassPtr& pass) : iPass(ip), pPass(pass){}

	bool operator<(const REN_PASS& rp) const {return iPass<rp.iPass;}

	zgU32 iPass;
	SPassPtr pPass;
};


}

//---------------------------------------------------------
void RenderDX11()
{
 	RenContextX11& renctx = GetRenContext();

	//回転
	static float g_fRotY = 0.0f;
	static DWORD dwTimeStart = 0;
	DWORD dwTimeCur = GetTickCount();
	if( dwTimeStart == 0 ) dwTimeStart = dwTimeCur;
	g_fRotY = 0;//( dwTimeCur - dwTimeStart ) / 1000.0f;

	float key_time = float( dwTimeCur - dwTimeStart ) / 33.0f;

	//499;//
	std::vector<REN_PASS> pass_list;
	XMMATRIX mmmm = XMMatrixIdentity();
	zgU32 mdl_cnt = 0;
	for(auto& mdl : g_aModel){
		if(!mdl)continue;
		MeshModel& model = *mdl;

		XMMATRIX mtx = XMMatrixTranslation(model.v3Pos.x, model.v3Pos.y, model.v3Pos.z);
		if(mdl_cnt == 0){
			mtx = XMMatrixTranspose(mtx);
		}else{
			mtx = XMMatrixMultiplyTranspose(XMMatrixRotationY(g_fRotY), mtx);
		}

		if(!model.pModel)continue;
		ModelX11& mdx11 = *model.pModel;

		//ボーン行列更新
		if( mdx11.numBoneMtx > 0 && model.numBone==mdx11.numBoneMtx ){
			MeshBone* bone = reinterpret_cast<MeshBone*>(model.binBone.get());
			XMMATRIX* mtx_pose = reinterpret_cast<XMMATRIX*>(model.binMtxPose.get());

			// とりあえずアニメーション
			Motion& mot = model.Mot;
			for(auto& m : mot.aData){
				if(m.Key.size() == 0)continue;

				//キー検索
				zgU32 idx = 0;
				for(auto& k : m.Key){
					if(k.time >= key_time){
						break;
					}
					++idx;
				}
				if(idx >= m.Key.size())idx = m.Key.size()-1;

				zgU32 prev = idx;
				if(idx>0)prev = idx-1;
				KeyFrame& key0 = m.Key[prev];
				KeyFrame& key1 = m.Key[idx];

				float t = 1.0f;
				if(idx!=prev){
					t = float(key_time - key0.time )/float(key1.time - key0.time);
				}
				if(t < 0.0f)t = 0.0f;
				if(t > 1.0f)t = 1.0f;
				
				for(zgU32 ib=0;ib<model.numBone;++ib){
					if(m.strBoneName == bone[ib].name){
						XMVECTOR pos0 = ToXMVec(key0.pos);
						XMVECTOR pos1 = ToXMVec(key1.pos);
						XMVECTOR rot0 = ToXMVec(key0.rot);
						XMVECTOR rot1 = ToXMVec(key1.rot);
						// key0.Intp[] = 
						// X_x1, Y_x1, Z_x1, R_x1, X_y1,Y_y1,Z_y1,R_y1,
						// X_x2,Y_x2,Z_x2,R_x2,X_y2,Y_y2,Z_y2,R_y2,
						float tx = Bezier(key0.Intp[0],key0.Intp[8], key0.Intp[4], key0.Intp[12], t);
						float ty = Bezier(key0.Intp[1],key0.Intp[9], key0.Intp[5], key0.Intp[13], t);
						float tz = Bezier(key0.Intp[2],key0.Intp[10], key0.Intp[6], key0.Intp[14], t);
						float tr = Bezier(key0.Intp[3],key0.Intp[11], key0.Intp[7], key0.Intp[15], t);

						bone[ib].vPos = XMVectorLerpV(pos0, pos1, XMVectorSet(tx,ty,tz,0.0f));
						bone[ib].qRot = XMQuaternionSlerp(rot0, rot1, tr);
						break;
					}
				}
			}

			// アニメーションでローカル行列が変更されたので
			// ワールド行列を再計算
			model.UpdatePose();

			// IK計算
			XMVECTOR eps0 = {0.0000001f,0.0000001f,0.0000001f,0.0000001f};
			for(auto& ik : model.aIK){

				for(zgU32 ite=0;ite<ik.iterations;++ite){
					zgU32 tg_idx = ik.target_bone_index;
					zgU32 ik_idx = ik.bone_index;
					XMVECTOR target_pos = mtx_pose[tg_idx].r[3];
					for(zgU32 chn=0;chn<ik.chain_length;++chn){
						zgU32 pa_idx = ik.child_bone_index[chn];//
						
						//親のローカル空間に変換
						XMMATRIX inv_mtx = XMMatrixInverse(nullptr, mtx_pose[pa_idx]);
						XMVECTOR tg_pos = XMVector4Transform(target_pos, inv_mtx);
						XMVECTOR ik_pos = XMVector4Transform( mtx_pose[ik_idx].r[3], inv_mtx);

						//目標方向
						XMVECTOR ik_dir = XMVector3Normalize(ik_pos);
						XMVECTOR tg_dir = XMVector3Normalize(tg_pos);
						if( XMVector3NearEqual(tg_dir, ik_dir, eps0) ){
							continue;
						}

						// 回転軸と角度 
						XMVECTOR rot_axis = XMVector3Cross(tg_dir, ik_dir);
						if(XMVector3NearEqual(rot_axis,XMVectorSet(0,0,0,0),eps0)){
							continue;
						}
						rot_axis = XMVector3Normalize(rot_axis);
						float ang = XMVectorGetX( XMVectorACos(XMVector3Dot(tg_dir, ik_dir)) );

						//tg_dirをik_dirに一致させるための回転
						XMVECTOR rot = XMQuaternionRotationAxis(rot_axis,ang*ik.control_weight);

						bone[pa_idx].qRot = XMQuaternionMultiply( rot ,bone[pa_idx].qRot );	

						// 回転制限
						XMMATRIX mtx = XMMatrixRotationQuaternion(bone[pa_idx].qRot);

					/*	//ZXY X=-90～+90°X軸＝膝回転　膝が曲がらないので使えない
						float rx = -asinf(XMVectorGetY(mtx.r[2]));
						float ry = atan2f(XMVectorGetX(mtx.r[2]),XMVectorGetZ(mtx.r[2]));
						float rz = atan2f(XMVectorGetY(mtx.r[0]),XMVectorGetY(mtx.r[1]));
						XMVECTOR rot_xyz = {rx,ry,rz,0};
						rot_xyz = XMVectorMax(rot_xyz, bone[pa_idx].ik_min);
						rot_xyz = XMVectorMin(rot_xyz, bone[pa_idx].ik_max);
						bone[pa_idx].qRot = XMQuaternionRotationRollPitchYawFromVector(rot_xyz);
						*/

						//ZYX　Y=-90～90°Y軸＝ねじり方向
						//通常X軸がボーンの方向（X軸回転＝ねじり）だけど、PMDの足ボーンはY軸
						float rx = -atan2f(XMVectorGetY(mtx.r[2]),XMVectorGetZ(mtx.r[2]));
						float ry = asinf(XMVectorGetX(mtx.r[2]));
						float rz = -atan2f(XMVectorGetX(mtx.r[1]),XMVectorGetX(mtx.r[0]));
						XMVECTOR rot_xyz = {rx,ry,rz,0};
						rot_xyz = XMVectorMax(rot_xyz, bone[pa_idx].ik_min);
						rot_xyz = XMVectorMin(rot_xyz, bone[pa_idx].ik_max);
						mtx = XMMatrixRotationZ( XMVectorGetZ(rot_xyz) );
						mtx = XMMatrixMultiply(mtx, XMMatrixRotationY( XMVectorGetY(rot_xyz) ));
						mtx = XMMatrixMultiply(mtx, XMMatrixRotationX( XMVectorGetX(rot_xyz) ));
						bone[pa_idx].qRot = XMQuaternionRotationMatrix(mtx);
						
						//ワールド行列更新
						mtx_pose[pa_idx] = SRTMatrix( bone[pa_idx].vScale, bone[pa_idx].qRot, bone[pa_idx].vPos);
						if( bone[pa_idx].idxParent < mdx11.numBoneMtx){
							mtx_pose[pa_idx] = XMMatrixMultiply(mtx_pose[pa_idx], mtx_pose[bone[pa_idx].idxParent]);
						}
						mtx_pose[tg_idx] = SRTMatrix( bone[tg_idx].vScale, bone[tg_idx].qRot, bone[tg_idx].vPos);
						if( bone[tg_idx].idxParent < mdx11.numBoneMtx){
							mtx_pose[tg_idx] = XMMatrixMultiply(mtx_pose[tg_idx], mtx_pose[bone[tg_idx].idxParent]);
						}
						
						//目標位置更新
						target_pos = mtx_pose[tg_idx].r[3];
					}
				}

				//IKの計算結果を子階層に反映
				model.UpdatePose();

			}
		}

		// ボーン差分行列の作成、定数バッファに転送
		if( mdx11.numBoneMtx > 0 && mdx11.pCBBoneMtx ){
			XMMATRIX* mtx_pose = reinterpret_cast<XMMATRIX*>(model.binMtxPose.get());
			XMMATRIX* mtx_init = reinterpret_cast<XMMATRIX*>(model.binMtxInitBone.get());
			MATRIX43* mtx_sub = reinterpret_cast<MATRIX43*>(model.binMtxSubBone.get());
			for(zgU32 ib=0;ib<mdx11.numBoneMtx;++ib){
				XMMATRIX mtx = XMMatrixMultiplyTranspose(
									XMMatrixInverse(nullptr,mtx_init[ib]),mtx_pose[ib]);
				mtx_sub[ib].r[0] = mtx.r[0];
				mtx_sub[ib].r[1] = mtx.r[1];
				mtx_sub[ib].r[2] = mtx.r[2];
			}
			if( model.binMtxSubBone.size() <= mdx11.sizeBoneMtx){
				renctx.pImCtx->UpdateSubresource( mdx11.pCBBoneMtx.get(), 0, NULL, model.binMtxSubBone.get(), 0, 0 );
			}
		}

		// モデルパラメータ更新
		SCBModel sbc_model;
		sbc_model.colModel = XMVectorSet(model.colModel.x,model.colModel.y,model.colModel.z,model.colModel.w);
		renctx.pImCtx->UpdateSubresource( mdx11.pCBModel.get(), 0, NULL, &sbc_model, 0, 0 );

		for(auto& obj : mdx11.apObj){
			if(!obj)continue;
			
			// 姿勢行列を更新
			SCBObject cbobj;
			cbobj.mtxWorld = mtx;
			renctx.pImCtx->UpdateSubresource( obj->pCBObject.get(), 0, NULL, &cbobj, 0, 0 );
			
			for(auto &mesh : obj->apMesh){
				if(!mesh)continue;
				if(mesh->numTech < 1)break;
				if(!mesh->apTech[0])continue;
				ShaderTech& tech = *mesh->apTech[0];
			
				for(zgU32 ipass=0;ipass<tech.numPass;++ipass){
					if(!tech.apPass[ipass])break;

					// パスリストに追加
					pass_list.push_back(REN_PASS((ipass|(64-mdl_cnt)<<8),tech.apPass[ipass]));
					// モデルとパスでソート
					// (ipass|mdl_cnt<<8)この値でソート
				}
			}
		}
		++mdl_cnt;
	}

	// パスでソート
	// ShaderPassが独立した処理で順番に依存しないのでソートができる
	std::sort(pass_list.begin(),pass_list.end());


	// 指定色で画面クリア
	//float ClearColor[4] = { 0.0f, 0.125f, 0.3f, 1.0f }; //red,green,blue,alpha
	float ClearColor[4] = { 0.15f, 0.35f, 0.7f, 1.0f }; //red,green,blue,alpha
	renctx.pImCtx->ClearRenderTargetView( renctx.pRTView.get(), ClearColor );

	//デプスバッファクリア
	renctx.pImCtx->ClearDepthStencilView( renctx.pDSView.get(), D3D11_CLEAR_DEPTH, 1.0f, 0 );

	//パスリストに追加されたShaderPassを描画
	for(auto& p : pass_list){
		ShaderPass& pass = *p.pPass;
		
		// 入力レイアウト
		renctx.pImCtx->IASetInputLayout( pass.pLayout.get() );

		// インデックスバッファ
		renctx.pImCtx->IASetIndexBuffer( pass.pIBuff.get(), DXGI_FORMAT_R16_UINT, 0 );

		// 頂点バッファ
		ID3D11Buffer* vbs[ShaderPass::VBUFF_MAXNUM];
		for(zgU32 vi=0;vi<pass.numVBuff;++vi){
			vbs[vi] = pass.apVBuff[vi].get();
		};
		renctx.pImCtx->IASetVertexBuffers( 0, pass.numVBuff, vbs,
										pass.aStride.data(), pass.aOffset.data() );

		// ステート
		FLOAT bdfactor[4]={0,0,0,0};
		renctx.pImCtx->RSSetState( pass.pRsState.get() );
		renctx.pImCtx->OMSetBlendState( pass.pBdState.get(), bdfactor, 0xffffffff );
		renctx.pImCtx->OMSetDepthStencilState( pass.pDsState.get(), 0 );
	
		// シェーダ
		{
			ID3D11Buffer* cb[ShaderPass::CBUFF_MAXNUM];
			for(zgU32 ci=0;ci<pass.numCBuff;++ci){
				cb[ci] = pass.apCBuff[ci].get();
			}
			renctx.pImCtx->VSSetConstantBuffers( 0, pass.numCBuff, cb );
			renctx.pImCtx->VSSetShader( pass.pVShader.get(), NULL, 0 );

			ID3D11ShaderResourceView* srv[ShaderPass::TEX_MAXNUM];
			ID3D11SamplerState* ss[ShaderPass::TEX_MAXNUM];
			for(zgU32 ti=0;ti<pass.numTex;++ti){
				srv[ti] = pass.apSRView[ti].get();
				ss[ti] = pass.apSpState[ti].get();
			}
			renctx.pImCtx->PSSetShaderResources( 0, pass.numTex, srv );
			renctx.pImCtx->PSSetSamplers( 0, pass.numTex, ss );

			renctx.pImCtx->PSSetShader( pass.pPShader.get(), NULL, 0 );
			renctx.pImCtx->PSSetConstantBuffers( 0, pass.numCBuff, cb );
		}

		// プリミティブ形状
		renctx.pImCtx->IASetPrimitiveTopology( pass.eTopo ); 
		//ポリゴン描画
		renctx.pImCtx->DrawIndexed( pass.nCount, 0, 0 );
	}

	//結果をウインドウに反映
    renctx.pSwapChain->Present( 0, 0 );
}
