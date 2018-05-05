#pragma once
#include "d3dx9anim.h"
#include "Mesh.h"


/// �t���[��
struct MYFRAME : public D3DXFRAME {
	D3DXMATRIX CombinedTransformationMatrix;
};


/// ���b�V���R���e�i
struct MYMESHCONTAINER : public D3DXMESHCONTAINER {
	LPDIRECT3DTEXTURE9*  ppTextures;
	unsigned long maxFaceInfl;			// 1�ʂ�����̃{�[���̍ő吔
	unsigned long numBoneCombination;	// �{�[���R���r�l�[�V�����̐�
	LPD3DXBUFFER pBoneBuffer;			// �{�[���E�e�[�u��
	D3DXMATRIX** ppBoneMatrix;			// �S�Ẵ{�[���̃��[���h�s��̐擪�|�C���^�[
	D3DXMATRIX* pBoneOffsetMatrices;
};


/// �t���[���`��p�֐��Q
class FrameRendering {
private:
	virtual void RenderMeshContainer(LPDIRECT3DDEVICE9 pDevice, MYMESHCONTAINER* pMeshContainer, MYFRAME* pFrame);	/// ���b�V���R���e�i�̃����_�����O
public:
	virtual void UpdateMatrices(LPD3DXFRAME pFrame, LPD3DXMATRIX world);	/// �t���[���̃��[���h�ϊ�
	virtual void Draw(LPDIRECT3DDEVICE9 pDevice, LPD3DXFRAME pFrame);		/// �t���[���̃����_�����O
};


/// �K�w�\�����������֐��Q
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


/// �A�j���[�V�������b�V��
class AnimationMesh : public Drawable3D {
protected:
	LPDIRECT3DDEVICE9			pDevice;			// Direct3D�f�o�C�X�I�u�W�F�N�g
	LPD3DXFRAME					pFrameRoot;			// �t���[��
	LPD3DXANIMATIONCONTROLLER	pAnimController;	// �A�j���[�V�����R���g���[��
	LPD3DXANIMATIONSET			pAnimSet[100];		// �A�j���[�V�����Z�b�g
	HRESULT AllocateBoneMatrix(LPD3DXMESHCONTAINER);// ���b�V���ɍs�������U��
	HRESULT AllocateAllBoneMatrices(LPD3DXFRAME);	// �ċA�R�[�h�F�t���[���K�w��W�J���Ċe���b�V���ɍs�������U��
	void FreeAnim(LPD3DXFRAME pF);					// �ċA�R�[�h�F�S�Ẵ��b�V���R���e�i�������[�X
	FrameRendering				frameOperator;		// �t���[���`��p�֐��Q
	MyHierarchy					myHierarchy;		// �K�w�\�����������֐��Q
	AnimationMesh(LPDIRECT3DDEVICE9 pDevice);		// �I�u�W�F�N�g�����֎~
public:
	virtual ~AnimationMesh();
	virtual void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);/// �`��
	virtual void AdvanceTime(double time);			/// ���Ԃ�i�s
	virtual void ChangeMotion(int motion_number);	/// ������؂�ւ���
	virtual LPD3DXFRAME GetFrame();					/// �t���[�����擾
};


/// X�t�@�C������Ǎ��񂾃A�j���[�V�������b�V��
class XFileAnimationMesh : public AnimationMesh {
public:
	XFileAnimationMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDevice);
};
