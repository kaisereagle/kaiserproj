#pragma once
#include "Interface.h"
#include "MmdStruct.h"
#include "Camera.h"

/// メッシュの頂点データ
/// 頂点フォーマット D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_TEX1
struct Vertex {
	D3DXVECTOR3 position;	// 頂点位置
	D3DXVECTOR3 normal;		// 法線ベクトル
	D3DXVECTOR2 texture;	// テクスチャ座標
};

/// メッシュのポリゴンデータ
struct Face {
	unsigned short indices[3];		// 3頂点のインデックス
	unsigned long material_number;	// 材料番号
};

// メッシュデータ一時保存用構造体。独自形式データは一度この構造体に格納し、Mesh::SetMesh()でMesh::pMeshにセットする。
struct MeshData {
	vector<Vertex> vertices;
	vector<Face> faces;
	vector<D3DMATERIAL9> material;
	vector<string> texture_filename;
};

/// メッシュのベース
class Mesh : public Drawable3D {
protected:
	LPDIRECT3DDEVICE9 pDevice;			// Direct3Dデバイスオブジェクト
	LPD3DXMESH pMesh;					// メッシュ
	D3DMATERIAL9* pMeshMaterials;		// マテリアル配列
	LPDIRECT3DTEXTURE9*	pMeshTextures;	// テクスチャ配列
	unsigned long numMaterial;				// マテリアル・テクスチャ配列の大きさ
	void AddNormalVector(MeshData& meshData);// MeshDataに法線ベクトルを追加
	void SetMesh(MeshData meshData);	// MeshDataをpMeshにセット
	Mesh(LPDIRECT3DDEVICE9 pDevice);	// オブジェクト生成禁止
public:
	virtual ~Mesh();
	virtual void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);
	virtual LPD3DXMESH GetMesh();
	virtual int GetNumMaterial();
};

/// Xファイルから読込んだメッシュ
class XFileMesh : public Mesh {
public:
	XFileMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDevice);
};

/// メタセコイアファイルから読込んだメッシュ
class MqoMesh : public Mesh {
private:
	template <typename T> vector<T> MaterialPickOut(string str, char* name, int n); // マテリアル行の文字列から指定した名前の値の配列を取得
	void LoadMaterial(ifstream& ifs, MeshData& meshData);
	void LoadObject(ifstream& ifs, MeshData& meshData);
	MeshData GetMeshDataFromMQO(LPCTSTR filename); // メタセコイアファイルからメッシュデータを読込む
public:
	MqoMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDevice);
};

/// PMDファイルから読込んだメッシュ
class PmdMesh : public Mesh {
	void CopyMaterial(D3DMATERIAL9& material, MmdStruct::PmdMaterial& pmdMaterial);	// PmdMaterialからD3DMATERIAL9にデータをコピー
public:
	PmdMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDevice);
};