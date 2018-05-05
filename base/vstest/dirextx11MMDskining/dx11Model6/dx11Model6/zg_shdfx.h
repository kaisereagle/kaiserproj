#pragma once

#include "zg_types.h"
#include "zg_utils.h"
#include "zg_ptrx11.h"
#include "zg_shdfx_desc.h"

namespace zix11{
class ShaderEffect;
typedef SPtr<ShaderEffect>::Type SEffectPtr;

// �V�F�[�_�G�t�F�N�g
class ShaderEffect
{
public:
	struct PASS{
		//DirectX11���\�[�X�i�C���^�[�t�F�C�X�j
		IVShaderPtr pVShader;	// ���_�V�F�[�_
		IBlobPtr pVSCode;		// ���_�V�F�[�_�o�C�i���R�[�h�@���C�A�E�g�쐬�p
		IPShaderPtr pPShader;	// �s�N�Z���V�F�[�_
		IBlobPtr pPSCode;		// �s�N�Z���V�F�[�_�o�C�i���R�[�h

		IRsStatePtr pRsState;	// ���X�^���C�U�X�e�[�g
		IBdStatePtr pBdState;	// �u�����h�X�e�[�g
		IDsStatePtr pDsState;	// �f�v�X�X�e���V���X�e�[�g
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
// zgfx�t�@�C������\�z
SEffectPtr CreateShaderEffectFromFile(const WCHAR* file);


}//zix11
