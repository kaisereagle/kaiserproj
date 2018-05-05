#pragma once

// 中間モデルデータ（工事中）

#include "zg_types.h"
#include "zg_utils.h"
#include <vector>
#include <string>

struct MQOModel;

// 文字列
typedef std::string String;


namespace zgmdl
{
class Material;
class Mesh;
class Object;
class Model;
typedef SPtr<Material>::Type MaterialPtr;
typedef SPtr<Mesh>::Type MeshPtr;
typedef SPtr<Object>::Type ObjectPtr;
typedef SPtr<Model>::Type ModelPtr;

//----------------------------------------
// マテリアル　とりあえず単純な構造
//　テクスチャ1枚のみ
class Material
{
public:
	Material() : v4Diff(1,1,1,1){}
	String strName;
	String strTex;
	//シェーダ、エフェクトなど追加予定
	ZGVECTOR4 v4Diff;
};

//----------------------------------------
// 頂点データ
class Vertex : boost::noncopyable
{
public:
	enum FORMAT
	{
		FMT_NULL = 0,
		FMT_F32 = 1,	//32bit float
		FMT_U32,		//32bit unsigned int
		FMT_U16,		//16bit unsigned int
		FMT_U8,			//8bit unsigned int
	};
	enum VATTR{//頂点属性
		VATTR_NULL = 0,
		POSITION = 1,
		NORMAL,
		TEXCRD0,
		TEXCRD1,
		TEXCRD2,
		TEXCRD3,
		BONEINDEX,
		BONEWEIGHT,

		INDEX,
	};


	Vertex();
	~Vertex();

	//----------------------------------------
	bool Create(VATTR attr, zgU32 num, FORMAT format, zgU32 elem_num);

	//----------------------------------------
	zgU32 getNum() const { return numData;}

	VATTR getAttr() const { return eAttr;}
	
	//とりあえず安全重視、直接アクセスなし

	template <typename T>
	bool getData(zgU32 idx, T& data) const{
		//T=PODのチェックが必要	
		const void* p = rawget(idx, sizeof(data));
		if(!p)return false;
		memcpy(&data, p, sizeof(data));
		return true;
	}

	template <typename T>
	bool setData(zgU32 idx, const T& data){
		//T=PODのチェックが必要		
		void* p = rawget(idx, sizeof(data));
		if(!p)return false;
		memcpy(p, &data, sizeof(data));
		return true;
	}

private:
	void* rawget(zgU32 idx, zgU32 size);
	const void* rawget(zgU32 idx, zgU32 size) const;
private:
	FORMAT eFormat;
	VATTR eAttr;
	zgU32 sizeStride;
	zgU32 sizeData;
	zgU32 numData;
	void* pData;
};

//----------------------------------------
// メッシュ（描画処理単位）
class Mesh
{
public:
	Mesh() : idxMat(0){}

	enum{
		VERTEX_NUM = 8,
	};

	//--------------------------------------------
	const Vertex& getVertex(zgU32 slot) const { return aVertex[slot];}
	Vertex& getVertex(zgU32 slot) { return aVertex[slot];}
	
	//-------------------------------------
	// 指定した属性のスロット番号を取得
	bool getSlot(Vertex::VATTR attr, zgU32& slot) const;

	//--------------------------------------------
	void setMatIdx(zgU32 idx){ idxMat = idx;}
	zgU32 getMatIdx() const { return idxMat;}

	//--------------------------------------------
	const Vertex& getIndex() const { return Index;}
	Vertex& getIndex() { return Index;}

private:
	zgU32 idxMat;
	Array<Vertex, VERTEX_NUM>::Type aVertex;
	Vertex Index;
};

//-----------------------------------------
class Object
{
public:
	zgU32 MeshNum() const;
	MeshPtr getMesh(zgU32 idx); 

public://作成
	void SetMeshNum(zgU32 num);
	bool setMesh(zgU32 idx, const MeshPtr& mesh); 

private:
	std::vector<MeshPtr> apMesh;	
};

//---------------------------------------
// ボーン
class Bone
{
public:
	Bone() : vScale(1,1,1), qRot(0,0,0,1), vPos(1,1,1){}
	String strName; 
	ZGVECTOR3 vScale;	//スケールxyz
	ZGQUAT qRot;		//回転（クォータニオン）
	ZGVECTOR3 vPos;		//位置
	ZGMATRIX44 mtxPose;	//
private:
};

//------------------------------------
// 階層構造
class Hierarchy
{
public:
	Hierarchy() : idxSelf(zgU32(-1)), idxParent(zgU32(-1)){}
	zgU32 idxSelf;		// ボーンidx
	zgU32 idxParent;	// 親階層idx
};

//----------------------------------------
class Model
{
public:
	zgU32 MatNum() const;
	zgU32 ObjNum() const;
	zgU32 BoneNum() const;
	MaterialPtr getMat(zgU32 idx);
	ObjectPtr getObj(zgU32 idx);

	Bone* getBone(zgU32 idx);
	const Bone* getBone(zgU32 idx) const;
	Hierarchy* getHier(zgU32 idx);
	const Hierarchy* getHier(zgU32 idx) const;

public://作成
	void SetMatNum(zgU32 num);
	void SetObjNum(zgU32 num);
	void SetBoneNum(zgU32 num);
	bool setMat(zgU32 idx, const MaterialPtr& mat);
	bool setObj(zgU32 idx, const ObjectPtr& mat);

private:
	std::vector<MaterialPtr> apMat;	// マテリアル配列
	std::vector<ObjectPtr> apObj;	// オブジェクト配列
	std::vector<Bone> aBone;		// ボーン配列
	std::vector<Hierarchy> aHier;	// 階層構造
};


}//zgmdl

struct MQOModel;
//----------------------------------------------
zgmdl::ModelPtr CreateModel(const MQOModel& mqo);

struct PMDModel;
//-------------------------------------------
zgmdl::ModelPtr CreateModel(const PMDModel& pmd);
