#include "stdafx.h"

#include "zg_mdlx11.h"
#include "zg_renctx11.h"
#include "zg_shdfx.h"

#include <d3d11.h>
#include <d3dcompiler.h>
#include <directxmath.h>
#include "DirectXTex/DirectXTex.h"

#include <stdlib.h>
#include <locale>
#include <fstream>

#include "sample_data.h"

using namespace DirectX;

zix11::RenContextX11& GetRenContext();
namespace zix11{

namespace{

//���_�f�[�^�\����
struct VertexData
{
	XMFLOAT3 pos;
	XMFLOAT3 nor;
	XMFLOAT2 tex;
};
struct BoneWeight
{
	zgU16 idx[4];
	zgU8 weight[4];//0~100%
};

//-----------------------
// string->wstring�ϊ�
bool to_wstring(const std::string& str, std::wstring& wstr)
{
	 size_t size;
	 if( 0!= mbstowcs_s(&size, NULL, 0, str.c_str(), 0) ){
		 return false;
	 }
	 std::vector<wchar_t> wch(size);
	 if( 0!= mbstowcs_s(&size, wch.data(), wch.size(), str.c_str(), str.length()) ){
		return false;
	 }
	 wstr = wch.data();
	 return true;
}

// ���̓��C�A�E�g�`�F�b�N
// ���_�V�F�[�_���v�����Ă�����̓p�����[�^����`����Ă��邩
// shader_code,code_size �V�F�[�_���R���p�C�������o�C�g�R�[�h
bool CheckInputLayout(const void* shader_code, size_t code_size,
					  const D3D11_INPUT_ELEMENT_DESC* layout, size_t layout_num)
{
	DXPtr<ID3D11ShaderReflection>::Type vs_ref; 
	HRESULT hr = D3DReflect( shader_code,code_size,
				IID_ID3D11ShaderReflection, (void**)GSET_DXPTR(vs_ref));
	// dxguid.lib�̃����N���K�v
	if( FAILED( hr ) )return false;

	// �V�F�[�_���擾
	D3D11_SHADER_DESC shd_desc;
	hr = vs_ref->GetDesc(&shd_desc);
	if( FAILED( hr ) ){
		return false;
	}

	ID3D11ShaderReflectionConstantBuffer* cbuff = vs_ref->GetConstantBufferByName("CB7");
	if(cbuff){
		D3D11_SHADER_BUFFER_DESC cb_desc;
		hr = cbuff->GetDesc(&cb_desc);
		if( SUCCEEDED( hr ) ){
			for(UINT v=0;v<cb_desc.Variables;++v){
				D3D11_SHADER_VARIABLE_DESC vdesc;
				ID3D11ShaderReflectionVariable* var = cbuff->GetVariableByIndex(v);
				if( SUCCEEDED( var->GetDesc(&vdesc) )){
					var = var;
				}
			}
		}

	}

	for(UINT ip=0;ip<shd_desc.InputParameters;++ip){
		D3D11_SIGNATURE_PARAMETER_DESC desc;
		hr = vs_ref->GetInputParameterDesc(ip, &desc);
		if( FAILED( hr ) ){
			return false;
		}
		
		//���̓��C�A�E�g�ɕK�v�ȃZ�}���e�B�N�����Ȃ���Η��p�s��
		UINT il=0;
		for(il=0;il<layout_num;++il){
			if( strcmp(layout[il].SemanticName, desc.SemanticName) == 0){
				break;
			}
		}
		if( il==layout_num){
			return false;//������Ȃ�
		}
	}

	return true;
}

}



// DX11�����_�����O���f���̍쐬
ModelX11Ptr CreateModelX11(zgmdl::Model& model)
{
	HRESULT hr;
	RenContextX11& renctx = GetRenContext();
	
	// ���\�[�X���C�u����
	ResLibrary<ISRViewPtr>& texlib = renctx.libTexImage;//�e�N�X�`���摜
	ResLibrary<SEffectPtr>& efflib = renctx.libEffect;//�V�F�[�_�G�t�F�N�g



	// SamplerState�쐬
    D3D11_SAMPLER_DESC sampDesc;
    ZeroMemory( &sampDesc, sizeof(sampDesc) );
    sampDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
    sampDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
    sampDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
    sampDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
    sampDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
    sampDesc.MinLOD = 0;
    sampDesc.MaxLOD = D3D11_FLOAT32_MAX;
	ISpStatePtr spstate;
    hr = renctx.pDev->CreateSamplerState( &sampDesc, GSET_DXPTR(spstate) );
	if( FAILED( hr ) ){
		return ModelX11Ptr();
	}

	ModelX11Ptr mdx11(new ModelX11);
	if(!mdx11)return mdx11;

	const WCHAR* shader_file = BASIC_SHADER_FX_FILE;
	const char* shader_name = "shader_basic";
	if(model.BoneNum() > 0){
		//�{�[������
		shader_name = "shader_blend";
		shader_file = BLEND_SHADER_FX_FILE;//�{�[���ό`�����_�u�����h
	}

	SEffectPtr seffect;
	if( !efflib.getRes(shader_name, seffect) ){
		//�V�K�쐬
		seffect = CreateShaderEffectFromFile(shader_file);
		if(!seffect){
			MessageBox( NULL, L"�V�F�[�_�G�t�F�N�g�쐬���s", L"Error", MB_OK );
			return ModelX11Ptr();
		}

		// �ǉ�
		efflib.AddRes(shader_name, seffect);
	}

	// �萔�o�b�t�@�쐬�p��`
	D3D11_SUBRESOURCE_DATA cb_inidat;
	D3D11_BUFFER_DESC cb_bd;
	ZeroMemory( &cb_inidat, sizeof(cb_inidat) );
	ZeroMemory( &cb_bd, sizeof(cb_bd) );
	// �萔�o�b�t�@�쐬
	cb_bd.Usage = D3D11_USAGE_DEFAULT;
	cb_bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cb_bd.CPUAccessFlags = 0;

	SCBModel scb_model;
	scb_model.colModel = XMVectorSet(1,1,1,1);
	cb_bd.ByteWidth = sizeof(SCBModel);
	cb_inidat.pSysMem = &scb_model;
	hr = renctx.pDev->CreateBuffer( &cb_bd, &cb_inidat, GSET_DXPTR(mdx11->pCBModel) );
	if( FAILED( hr ) ){ return ModelX11Ptr(); }

	// �{�[���s��
	if( model.BoneNum() > 0 ){
		cb_bd.ByteWidth = SCBBoneMatrix::getSize(model.BoneNum());
		hr = renctx.pDev->CreateBuffer( &cb_bd, NULL, GSET_DXPTR(mdx11->pCBBoneMtx) );
		if( FAILED( hr ) ){ return ModelX11Ptr(); }
		mdx11->numBoneMtx = model.BoneNum();
		mdx11->sizeBoneMtx = cb_bd.ByteWidth;
	}

	for(zgU32 o=0;o< model.ObjNum();++o){
		zgmdl::ObjectPtr obj = model.getObj(o);
		if(!obj)return ModelX11Ptr();

		ModelObjX11Ptr mox11(new ModelObjectX11);
		if(!mox11)return ModelX11Ptr();
		mdx11->apObj.push_back(mox11);
		
		//�I�u�W�F�N�g�p�����[�^�萔�o�b�t�@
		SCBObject scb_object;
		scb_object.mtxWorld = XMMatrixIdentity();//�{���̓��f���f�[�^����擾�A�f�[�^���Ȃ��̂ŒP�ʍs��
		cb_bd.ByteWidth = sizeof(scb_object);
		cb_inidat.pSysMem = &scb_object;
		hr = renctx.pDev->CreateBuffer( &cb_bd, &cb_inidat, GSET_DXPTR(mox11->pCBObject) );
		if( FAILED( hr ) ){ return ModelX11Ptr(); }

		for(zgU32 im=0;im<obj->MeshNum();++im){
			zgmdl::MeshPtr mesh = obj->getMesh(im);
			if(!mesh)continue;
			zgmdl::MaterialPtr mat = model.getMat(mesh->getMatIdx());

			//���b�V���쐬
			MeshX11Ptr mesh11(new MeshX11);
			if(!mesh11)return ModelX11Ptr();

			//�}�e���A���p�����[�^
			SCBMaterial scb_mat;
			if(mat){
				scb_mat.vDiffuse = XMVectorSet(mat->v4Diff.x,mat->v4Diff.y,mat->v4Diff.z,mat->v4Diff.w);
			}else{
				scb_mat.vDiffuse = XMVectorSet(1,1,1,1);
			}

			//�}�e���A���p�����[�^�萔�o�b�t�@�@
			cb_inidat.pSysMem = &scb_mat;
			cb_bd.ByteWidth = sizeof(scb_mat);
			hr = renctx.pDev->CreateBuffer( &cb_bd, &cb_inidat, GSET_DXPTR(mesh11->pCBMateiral) );
			if( FAILED( hr ) ){ return ModelX11Ptr(); }

			IBuffPtr vertbuff;
			IBuffPtr bwgtbuff;
			IBuffPtr idxbuff;
			zgU32 stride = 0;
			zgU32 bwgt_stride = 0;
			size_t count = 0;

			// ���̓��C�A�E�g��`
			D3D11_INPUT_ELEMENT_DESC layout[] = {
				{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
				{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
				{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			};
			D3D11_INPUT_ELEMENT_DESC layout_bwgt[] = {
				{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
				{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
				{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 },
				{ "BONEINDEX", 0, DXGI_FORMAT_R16G16B16A16_UINT, 1, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
				{ "BONEWEIGHT", 0, DXGI_FORMAT_R8G8B8A8_UINT, 1, 8, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			};
			
			{//�W�I���g���쐬
				;
				// ���_�o�b�t�@
				zgU32 pos_slot, nor_slot, tc0_slot;
				if( !mesh->getSlot(zgmdl::Vertex::POSITION, pos_slot)
					|| !mesh->getSlot(zgmdl::Vertex::NORMAL, nor_slot)
					|| !mesh->getSlot(zgmdl::Vertex::TEXCRD0, tc0_slot)
				){
					continue;
				}

				zgmdl::Vertex& pos = mesh->getVertex(pos_slot);
				zgmdl::Vertex& nor = mesh->getVertex(nor_slot);
				zgmdl::Vertex& tc0 = mesh->getVertex(tc0_slot);
				zgmdl::Vertex& idx = mesh->getIndex();
				if( pos.getNum() != tc0.getNum())continue;

				std::vector<VertexData> vertex;
				vertex.reserve(pos.getNum());

				for(unsigned vi=0;vi<pos.getNum();++vi){
					ZGVECTOR3 v;
					ZGVECTOR3 n;
					ZGVECTOR2 uv;
					pos.getData(vi, v);
					nor.getData(vi, n);
					tc0.getData(vi, uv);
					VertexData vd = { XMFLOAT3(v.x, v.y, v.z), XMFLOAT3(n.x, n.y, n.z),XMFLOAT2(uv.x, uv.y) };
					vertex.push_back(vd);
				}

				//�{�[���E�G�C�g
				zgU32 bidx_slot, bwgt_slot;
				std::vector<BoneWeight> boneweight;
				if( mesh->getSlot(zgmdl::Vertex::BONEINDEX, bidx_slot)
					&& mesh->getSlot(zgmdl::Vertex::BONEWEIGHT, bwgt_slot)
				){
					zgmdl::Vertex& bidx = mesh->getVertex(bidx_slot);
					zgmdl::Vertex& bwgt = mesh->getVertex(bwgt_slot);
					
					vertex.reserve(bidx.getNum());

					for(unsigned vi=0;vi<bidx.getNum();++vi){
						BoneWeight bw;
						bidx.getData(vi, bw.idx);
						bwgt.getData(vi, bw.weight);
						boneweight.push_back(bw);
					}
				}

				//���_�o�b�t�@�쐬
				size_t vnum = vertex.size();
				stride = sizeof( VertexData );
				D3D11_BUFFER_DESC bd;
				ZeroMemory( &bd, sizeof(bd) );
				bd.Usage = D3D11_USAGE_DEFAULT;
				bd.ByteWidth = sizeof( VertexData ) * vnum;
				bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
				bd.CPUAccessFlags = 0;

				D3D11_SUBRESOURCE_DATA InitData;
				ZeroMemory( &InitData, sizeof(InitData) );
				InitData.pSysMem = vertex.data();
				hr = renctx.pDev->CreateBuffer( &bd, &InitData, GSET_DXPTR(vertbuff) );
				if( FAILED( hr ) ){
					return ModelX11Ptr();
				}

				//�{�[���E�G�C�g
				if(boneweight.size() == vnum){
					bwgt_stride = sizeof( BoneWeight );
					bd.ByteWidth = sizeof( BoneWeight ) * vnum;
					InitData.pSysMem = boneweight.data();
					hr = renctx.pDev->CreateBuffer( &bd, &InitData, GSET_DXPTR(bwgtbuff) );
					if( FAILED( hr ) ){
						return ModelX11Ptr();
					}
				}

				// �C���f�b�N�X�o�b�t�@
				std::vector<zgU16> index;
				index.reserve(idx.getNum());
				for(zgU32 ii=0;ii<idx.getNum();++ii){
					zgU32 i=0;
					idx.getData(ii,i);
					index.push_back(static_cast<zgU16>(i));
				}

				count = index.size();
				bd.Usage = D3D11_USAGE_DEFAULT;
				bd.ByteWidth = sizeof( zgU16 ) * count;
				bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
				bd.CPUAccessFlags = 0;
				InitData.pSysMem = index.data();
				hr = renctx.pDev->CreateBuffer( &bd, &InitData, GSET_DXPTR(idxbuff) );
				if( FAILED( hr ) ){
					return ModelX11Ptr();
				}
			}//�W�I���g���쐬

			// �e�N�X�`���摜
			ISRViewPtr srvtex;
			std::string texname = "notex.png";
			if(mat && mat->strTex!="")texname = mat->strTex;
			
			if( !texlib.getRes(texname, srvtex) ){
				//�V�K�쐬
				std::wstring wstr;
				to_wstring(texname, wstr);
				wstr = std::wstring(TEXTURE_FOLDER_PATH) + wstr;

				TexMetadata metadata;
				DirectX::ScratchImage image;
				if( std::wstring::npos != wstr.rfind(L".tga") ){
					hr = LoadFromTGAFile(wstr.c_str(), &metadata, image);
				}else{
					hr = LoadFromWICFile(wstr.c_str(), 0, &metadata, image);
				}
				if( FAILED(hr) ){
					hr = LoadFromWICFile(L"data\\notex.png", 0, &metadata, image);						
				}
				if( SUCCEEDED( hr ) ){
					CreateShaderResourceView(renctx.pDev.get(),
								image.GetImages(), image.GetImageCount(),
								metadata, GSET_DXPTR(srvtex));
				}
					
				// �ǉ�
				texlib.AddRes(texname, srvtex);
			}

			mesh11->numTech = 0;
			for(auto& se_tech : seffect->aTech){
				// �e�N�j�b�N
				STechPtr tech(new ShaderTech);
				if(!tech)return ModelX11Ptr();
				mesh11->apTech[mesh11->numTech++] = tech;

				tech->numPass = 0;
				for(auto& se_pass : se_tech.aPass){
					// �p�X
					SPassPtr pass(new ShaderPass);
					if(!pass)return ModelX11Ptr();
					if(!se_pass.pVShader || !se_pass.pPShader)break;		
			
					//��������DX11���\�[�X���V�F�[�_�p�X��

					// ���_�o�b�t�@
					pass->numVBuff = 1;
					pass->apVBuff[0] = vertbuff;
					pass->aStride[0] = stride;
					pass->aOffset[0] = 0;
					if(bwgtbuff){
						pass->numVBuff = 2;
						pass->apVBuff[1] = bwgtbuff;
						pass->aStride[1] = bwgt_stride;
						pass->aOffset[1] = 0;	
					}
					pass->pIBuff = idxbuff;

					// ���̓��C�A�E�g�쐬
					UINT elem_num = ARRAYSIZE( layout );
					const D3D11_INPUT_ELEMENT_DESC* input_layout = layout;
					if(bwgtbuff){
						elem_num = ARRAYSIZE( layout_bwgt );
						input_layout = layout_bwgt;
					}

					// ���̓p�����[�^�̃`�F�b�N
					// CreateInputLayout�Ń`�F�b�N�����
					// first chance exception�ƃG���[���O���o�͂����̂Ŏ����Ń`�F�b�N
					if( !CheckInputLayout(
								se_pass.pVSCode->GetBufferPointer(),
								se_pass.pVSCode->GetBufferSize(),
								input_layout, elem_num)
					){
						//���̃V�F�[�_�͎g���Ȃ�
						break;
					}


					hr = renctx.pDev->CreateInputLayout( input_layout, elem_num,
										se_pass.pVSCode->GetBufferPointer(),
										se_pass.pVSCode->GetBufferSize(), GSET_DXPTR(pass->pLayout) );
					if( FAILED( hr ) ){ return ModelX11Ptr(); }
					

					// �X�e�[�g
					pass->pRsState = se_pass.pRsState;
					pass->pBdState = se_pass.pBdState;
					pass->pDsState = se_pass.pDsState;

					// �V�F�[�_
					pass->pVShader = se_pass.pVShader;
					pass->pPShader = se_pass.pPShader;
					
					// �e�N�X�`��
					pass->numTex = 1;
					pass->apSRView[0] = srvtex;
					pass->apSpState[0] = spstate;
					
					// �萔�o�b�t�@
					pass->numCBuff = pass->apCBuff.size();
					pass->apCBuff[SCBScene::CBSLOT] = renctx.pCBScene;
					pass->apCBuff[SCBLight::CBSLOT] = renctx.pCBLight;
					pass->apCBuff[SCBModel::CBSLOT] = mdx11->pCBModel;
					pass->apCBuff[SCBBoneMatrix::CBSLOT] = mdx11->pCBBoneMtx;
					pass->apCBuff[SCBObject::CBSLOT] = mox11->pCBObject;
					pass->apCBuff[SCBMaterial::CBSLOT] = mesh11->pCBMateiral;
					pass->nCount = count;
				
					tech->apPass[tech->numPass++] = pass;
				}
			}
			mox11->apMesh.push_back(mesh11);
		}
	}
	return mdx11;
}

}// zix11
