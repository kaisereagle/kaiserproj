#pragma once

// DirectX11���f���f�[�^

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

// ���f��
class ModelX11
{
public:
	ModelX11() : numBoneMtx(0), sizeBoneMtx(0){}
	std::vector<ModelObjX11Ptr> apObj;
	IBuffPtr pCBModel;//SCBModel�萔�o�b�t�@
	IBuffPtr pCBBoneMtx;//�{�[���s��
	zgU32 numBoneMtx;
	zgU32 sizeBoneMtx;
};

// �I�u�W�F�N�g
class ModelObjectX11
{
public:
	std::vector<MeshX11Ptr> apMesh;
	IBuffPtr pCBObject;//SCBObject�萔�o�b�t�@
};

// ���b�V��
class MeshX11
{
public:
	MeshX11(){}
	~MeshX11(){}
	static const zgU32 TECH_MAXNUM = 8;
	zgU32 numTech;
	Array<STechPtr, TECH_MAXNUM>::Type apTech;
	IBuffPtr pCBMateiral;	//SCBMaterial�萔�o�b�t�@
private:
};

//----------------------------------
// �V�F�[�_�e�N�j�b�N
class ShaderTech
{
public:
	static const zgU32 PASS_MAXNUM = 4;
	zgU32 numPass;
	Array<SPassPtr, PASS_MAXNUM>::Type apPass;	
};

//----------------------------------
// �V�F�[�_�p�X�i�`��̍ŏ��P�ʁj
class ShaderPass
{
public:
	static const zgU32 VBUFF_MAXNUM = 8;
	static const zgU32 TEX_MAXNUM = 8;
	static const zgU32 CBUFF_MAXNUM = 8;//16;
	// �������Ȃ��̂ŌŒ�̔z��
	
	//���_�f�[�^
	zgU32 numVBuff;
	Array<UINT, VBUFF_MAXNUM>::Type aStride;
	Array<UINT, VBUFF_MAXNUM>::Type aOffset;
	Array<IBuffPtr, VBUFF_MAXNUM>::Type	apVBuff;	
	IBuffPtr	pIBuff;
	IILayoutPtr	pLayout;

	// ���
	IRsStatePtr pRsState;
	IBdStatePtr pBdState;
	IDsStatePtr pDsState;

	// �V�F�[�_
	IVShaderPtr	pVShader;
	IPShaderPtr	pPShader;

	//�e�N�X�`��
	zgU32 numTex;
	Array<ISRViewPtr, TEX_MAXNUM>::Type	apSRView;	
	Array<ISpStatePtr, TEX_MAXNUM>::Type apSpState;

	//�萔�o�b�t�@
	zgU32 numCBuff;
	Array<IBuffPtr, CBUFF_MAXNUM>::Type apCBuff;
	//�Ƃ肠�������_�ƃs�N�Z���Œ萔���L

	zgU32 nCount;
	D3D_PRIMITIVE_TOPOLOGY eTopo;
	ShaderPass() : numVBuff(0), numTex(0), numCBuff(0), nCount(0)
					,eTopo(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST){}
};

//-------------------------------------------
// DX11�����_�����O���f���̍쐬
ModelX11Ptr CreateModelX11(zgmdl::Model& model);






}//zix11 
