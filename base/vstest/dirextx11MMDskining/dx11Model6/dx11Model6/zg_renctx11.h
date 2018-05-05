#pragma once

#include "zg_utils.h"
#include "zg_ptrx11.h"
#include "zg_shdfx.h"

#include <directxmath.h>


using DirectX::XMMATRIX;
using DirectX::XMVECTOR;
using DirectX::XMFLOAT4;


namespace zix11{

//�V�F�[�_�萔�o�b�t�@

// �Œ�p�����[�^
// �v���O�����ł̌Œ�l�ȂǂقڍX�V����Ȃ��p�����[�^
struct SCBFix
{
	static const zgU32 CBSLOT = 0;//�萔�o�b�t�@�X���b�g�ԍ�
	//�Ȃ�
};

// �V�[���p�����[�^
// �����ϊ��s��Ȃǂ̃p�����[�^
struct SCBScene
{
	static const zgU32 CBSLOT = 1;//�萔�o�b�t�@�X���b�g�ԍ�
	XMMATRIX mtxProj;	// ViewProjection�s��
	XMMATRIX mtxView;	// WorldView�s��
};

// �����p�����[�^
struct SCBLight
{
	static const zgU32 CBSLOT = 2;//�X���b�g�ԍ�

	// ���s����
	static const zgU32 DIRLIGHT_NUM = 4;
	XMVECTOR vDirLight[DIRLIGHT_NUM];
	XMVECTOR colDirLight[DIRLIGHT_NUM];

	//�������C�e�B���O
	XMVECTOR vHemiDir;
	XMVECTOR colHemiSky;
	XMVECTOR colHemiGrd;

	//�_����
	static const zgU32 PLIGHT_NUM = 4;
	XMVECTOR posPLight[PLIGHT_NUM];
	XMVECTOR colPLight[PLIGHT_NUM];
	XMVECTOR prmPLight[PLIGHT_NUM];
};
// ���f���p�����[�^
struct SCBModel
{
	static const zgU32 CBSLOT = 3;//�X���b�g�ԍ�
	XMVECTOR colModel;	// ���f���F�@RGBA
};
// �I�u�W�F�N�g�p�����[�^
// �p���i���[���h�j�s��Ȃǂ̃p�����[�^
struct SCBObject
{
	static const zgU32 CBSLOT = 4;//�X���b�g�ԍ�
	XMMATRIX mtxWorld;	//WorldLocal�s��
};

// �}�e���A���p�����[�^
struct SCBMaterial
{
	static const zgU32 CBSLOT = 5;//�X���b�g�ԍ�
	XMVECTOR vDiffuse;
	XMVECTOR vSpecular;
};

// ���[�J���p�����[�^
// �V�F�[�_�ŗL�̃p�����[�^
struct SCBLocal
{
	static const zgU32 CBSLOT = 6;//�X���b�g�ԍ�
	//�V�F�[�_���ɒ�`
};


struct SCBBoneMatrix
{
	static const zgU32 CBSLOT = 7;//�X���b�g�ԍ�
	XMVECTOR mtxBone[1];//float3x4
	// �{�[�����ŉ�
	static zgU32 getSize(zgU32 bone_num)
	{
		return sizeof(XMVECTOR)*3*bone_num;
	}
};

//DirectX11�����_�����O��
struct RenContextX11
{
	IDevicePtr pDev;
	IDevCtxPtr pImCtx;
	ISChainPtr pSwapChain;
	IRTViewPtr pRTView;
	ITex2DPtr  pDepthS;
	IDSViewPtr pDSView;

	//�V�F�[�_�萔�o�b�t�@
	SCBScene scbScene;
	SCBLight scbLight;

	IBuffPtr pCBScene;
	IBuffPtr pCBLight;

	// ���\�[�X���C�u����
	ResLibrary<ISRViewPtr> libTexImage;//�e�N�X�`���摜
	ResLibrary<SEffectPtr> libEffect;//�V�F�[�_�G�t�F�N�g

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
