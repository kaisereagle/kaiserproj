#pragma once
#include "Mesh.h"
#include "XFileAnimationMesh.h"

// ���ʓ��e�@�ɂ��e
class Shadow : public Drawable3D {
protected:
	LPDIRECT3DDEVICE9	pDevice;
	LPD3DXMESH			pMesh;
	int					NumMaterial;		// ���b�V���̃}�e���A����
	D3DMATERIAL9		shadow_material;	// �e�̃}�e���A��
	D3DXPLANE			plane;				// �e�̗����镽��
	void StencileEnable();					// �X�e���V���o�b�t�@��L���ɂ���
	void StencileDisable();					// �X�e���V���o�b�t�@�𖳌��ɂ���

public:
	// @param pDevice		D3D�f�o�C�X
	// @param pMesh			���b�V��
	// @param NumMaterial	���b�V���̍ޗ���
	Shadow(LPDIRECT3DDEVICE9 pDevice, LPD3DXMESH pMesh, int NumMaterial);

	// @param plane		�e�̗����镽��
	virtual void SetPlane(D3DXPLANE* plane);

	virtual void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);
};


// �}�e���A���ƃe�N�X�`�����Z�b�g���Ȃ��e�p�t���[���`��p�֐��Q
class FrameRenderingForShadow : public FrameRendering {
private:
	void RenderMeshContainer(LPDIRECT3DDEVICE9, MYMESHCONTAINER* ,MYFRAME*) override;
};


// XFile�A�j���[�V�����p ���ʓ��e�@�ɂ��e 
class XFileAnimationShadow : public Shadow {
	FrameRenderingForShadow frameRenderingForShadow;
	LPD3DXFRAME pFrame;
public:
	// @param pDevice	D3D�f�o�C�X
	// @param pFrame	�t���[��
	XFileAnimationShadow(LPDIRECT3DDEVICE9 pDevice, LPD3DXFRAME pFrame);

	void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);
};


// �V���h�E�{�����[���N���X
class ShadowVolume : public Drawable3D {
private:
	LPDIRECT3DDEVICE9	pDevice;
	LPD3DXMESH			pMesh;
	unsigned long		numVertex;			// �V���h�E�{�����[���̒��_�̐�
	D3DXVECTOR3			vertex[1000000];		// �V���h�E�{�����[���̒��_�̈ʒu�f�[�^
	void CreateVolume(D3DXVECTOR3 direction);
	void RenderVolume(D3DXMATRIX world);
	void RenderVolumeToStencil(D3DXMATRIX world);
	void RenderShadowBoard();
	D3DXMATRIX GetWorldMatrix(D3DXVECTOR3* position, D3DXMATRIX* rotation);
	D3DXVECTOR3 GetLocalDirection(D3DXMATRIX* rotation, D3DXVECTOR3* direction);
public:
	ShadowVolume(LPDIRECT3DDEVICE9 pDevice, LPD3DXMESH pMesh);
	void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);
	void DrawVolume(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light);
};
