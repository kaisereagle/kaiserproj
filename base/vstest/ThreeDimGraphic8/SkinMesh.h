#pragma once
#include "Interface.h"
#include "Camera.h"
#include "VmdMotionController.h"

/// スキンメッシュ
class SkinMesh : public Drawable3D {
protected:
	/// 基本 ///
	SkinMesh(LPDIRECT3DDEVICE9 pDevice);// オブジェクト生成禁止
	LPDIRECT3DDEVICE9 pDevice;			// Direct3Dデバイスオブジェクト
	/// メッシュ ///
	LPD3DXMESH pMesh;					// メッシュ
	vector<D3DMATERIAL9> materials;		// マテリアル配列
	vector<LPDIRECT3DTEXTURE9> textures;// テクスチャ配列
	vector<D3DXBONECOMBINATION> boneCombination;// ボーンコンビネーション配列
	virtual LPD3DXSKININFO CreateSkinInfo(const vector<BlendVertex>, const unsigned int, const D3DVERTEXELEMENT9*);	// メッシュ分割のためのスキン情報を作成
	virtual void DivideMesh(unsigned int numFace, LPD3DXMESH pOrgMesh, LPD3DXSKININFO pSkinInfo);					// ブレンド行列パレットのサイズを越えないようにメッシュを分割
	/// ボーン ///
	vector<Bone> bones;					// ボーン構造体
	vector<ID3DXMesh*> boneObj;			// 表示用ボーン
	virtual void CreateBoneObj();													// 表示用ボーンを作成
	virtual vector<D3DXMATRIX> GetWorlds(const D3DXMATRIX* world);					// ボーン行列から変換したワールド変換行列の配列を取得
	/// シェーダ ///
	D3DXHANDLE hTech, hWorld, hView, hProj, hAmbient, hDiffuse, hLightDir, hTexture;// ハンドル
	LPD3DXEFFECT pFX;		// シェーダ
	void CreateShader();	// シェーダの作成
	/// ボーン行列の更新
	virtual void UpdateBoneMatrix() = 0;	// 継承先に実装
public:
	virtual ~SkinMesh();
	virtual void AdvanceTime() = 0;						// 時間をすすめる
	virtual void Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);		// 描画
	virtual void DrawBoneObj(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera);// ボーンを表示
};


/// MMDスキンメッシュ
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

