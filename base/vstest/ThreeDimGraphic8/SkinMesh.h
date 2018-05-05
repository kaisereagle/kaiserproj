#pragma once
#include "Interface.h"
#include "Camera.h"
#include "VmdMotionController.h"

/// �X�L�����b�V��
class SkinMesh : public Drawable3D {
protected:
	/// ��{ ///
	SkinMesh(LPDIRECT3DDEVICE9 pDevice);// �I�u�W�F�N�g�����֎~
	LPDIRECT3DDEVICE9 pDevice;			// Direct3D�f�o�C�X�I�u�W�F�N�g
	/// ���b�V�� ///
	LPD3DXMESH pMesh;					// ���b�V��
	vector<D3DMATERIAL9> materials;		// �}�e���A���z��
	vector<LPDIRECT3DTEXTURE9> textures;// �e�N�X�`���z��
	vector<D3DXBONECOMBINATION> boneCombination;// �{�[���R���r�l�[�V�����z��
	virtual LPD3DXSKININFO CreateSkinInfo(const vector<BlendVertex>, const unsigned int, const D3DVERTEXELEMENT9*);	// ���b�V�������̂��߂̃X�L�������쐬
	virtual void DivideMesh(unsigned int numFace, LPD3DXMESH pOrgMesh, LPD3DXSKININFO pSkinInfo);					// �u�����h�s��p���b�g�̃T�C�Y���z���Ȃ��悤�Ƀ��b�V���𕪊�
	/// �{�[�� ///
	vector<Bone> bones;					// �{�[���\����
	vector<ID3DXMesh*> boneObj;			// �\���p�{�[��
	virtual void CreateBoneObj();													// �\���p�{�[�����쐬
	virtual vector<D3DXMATRIX> GetWorlds(const D3DXMATRIX* world);					// �{�[���s�񂩂�ϊ��������[���h�ϊ��s��̔z����擾
	/// �V�F�[�_ ///
	D3DXHANDLE hTech, hWorld, hView, hProj, hAmbient, hDiffuse, hLightDir, hTexture;// �n���h��
	LPD3DXEFFECT pFX;		// �V�F�[�_
	void CreateShader();	// �V�F�[�_�̍쐬
	/// �{�[���s��̍X�V
	virtual void UpdateBoneMatrix() = 0;	// �p����Ɏ���
public:
	virtual ~SkinMesh();
	virtual void AdvanceTime() = 0;						// ���Ԃ������߂�
	virtual void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);		// �`��
	virtual void DrawBoneObj(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);// �{�[����\��
};


/// MMD�X�L�����b�V��
class MmdSkinMesh : public SkinMesh {
private:
	VmdMotionController* vmdMotionController;
	vector<MmdStruct::PmdIkData> pmdIkData;
	void LoadPmdfile(const LPCTSTR&, vector<MmdStruct::PmdVertex>&, vector<unsigned short>&, vector<MmdStruct::PmdMaterial>&, vector<MmdStruct::PmdBone>&);
	void CreateMesh(vector<MmdStruct::PmdVertex>, vector<unsigned short>, vector<MmdStruct::PmdMaterial>, const unsigned int numBone);
	void CreateBoneMatrix(vector<MmdStruct::PmdBone> pmdBones);
	void CopyMaterial(D3DMATERIAL9&, const MmdStruct::PmdMaterial&);
	void UpdateBoneMatrix() override;
public:
	MmdSkinMesh(LPCTSTR pmdFilename, LPCTSTR vmdFilename, LPDIRECT3DDEVICE9 pDevice);
	~MmdSkinMesh();
	void AdvanceTime();
};

