#pragma once
#include "Interface.h"
#include "MmdStruct.h"
#include "Camera.h"

/// ���b�V���̒��_�f�[�^
/// ���_�t�H�[�}�b�g D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_TEX1
struct Vertex {
	D3DXVECTOR3 position;	// ���_�ʒu
	D3DXVECTOR3 normal;		// �@���x�N�g��
	D3DXVECTOR2 texture;	// �e�N�X�`�����W
};

/// ���b�V���̃|���S���f�[�^
struct Face {
	unsigned short indices[3];		// 3���_�̃C���f�b�N�X
	unsigned long material_number;	// �ޗ��ԍ�
};

// ���b�V���f�[�^�ꎞ�ۑ��p�\���́B�Ǝ��`���f�[�^�͈�x���̍\���̂Ɋi�[���AMesh::SetMesh()��Mesh::pMesh�ɃZ�b�g����B
struct MeshData {
	vector<Vertex> vertices;
	vector<Face> faces;
	vector<D3DMATERIAL9> material;
	vector<string> texture_filename;
};

/// ���b�V���̃x�[�X
class Mesh : public Drawable3D {
protected:
	LPDIRECT3DDEVICE9 pDevice;			// Direct3D�f�o�C�X�I�u�W�F�N�g
	LPD3DXMESH pMesh;					// ���b�V��
	D3DMATERIAL9* pMeshMaterials;		// �}�e���A���z��
	LPDIRECT3DTEXTURE9*	pMeshTextures;	// �e�N�X�`���z��
	unsigned long numMaterial;				// �}�e���A���E�e�N�X�`���z��̑傫��
	void AddNormalVector(MeshData& meshData);// MeshData�ɖ@���x�N�g����ǉ�
	void SetMesh(MeshData meshData);	// MeshData��pMesh�ɃZ�b�g
	Mesh(LPDIRECT3DDEVICE9 pDevice);	// �I�u�W�F�N�g�����֎~
public:
	virtual ~Mesh();
	virtual void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);
	virtual LPD3DXMESH GetMesh();
	virtual int GetNumMaterial();
};

/// X�t�@�C������Ǎ��񂾃��b�V��
class XFileMesh : public Mesh {
public:
	XFileMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDevice);
};

/// ���^�Z�R�C�A�t�@�C������Ǎ��񂾃��b�V��
class MqoMesh : public Mesh {
private:
	template <typename T> vector<T> MaterialPickOut(string str, char* name, int n); // �}�e���A���s�̕����񂩂�w�肵�����O�̒l�̔z����擾
	void LoadMaterial(ifstream& ifs, MeshData& meshData);
	void LoadObject(ifstream& ifs, MeshData& meshData);
	MeshData GetMeshDataFromMQO(LPCTSTR filename); // ���^�Z�R�C�A�t�@�C�����烁�b�V���f�[�^��Ǎ���
public:
	MqoMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDevice);
};

/// PMD�t�@�C������Ǎ��񂾃��b�V��
class PmdMesh : public Mesh {
	void CopyMaterial(D3DMATERIAL9& material, MmdStruct::PmdMaterial& pmdMaterial);	// PmdMaterial����D3DMATERIAL9�Ƀf�[�^���R�s�[
public:
	PmdMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDevice);
};