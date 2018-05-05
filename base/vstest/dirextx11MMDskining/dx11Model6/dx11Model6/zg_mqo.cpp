#include "stdafx.h"

#include "zg_mqo.h"
#include "zg_utils.h"
#include "zg_model.h"

#include <directxmath.h>
#include <map>
#include <vector>
#include <fstream>

using namespace DirectX;

namespace{

XMVECTOR ToXMVec(const ZGVECTOR3& v)
{
	return XMVectorSet(v.x,v.y,v.z, 0);
}
ZGVECTOR3 ToVec3(XMVECTOR v)
{
	return ZGVECTOR3(XMVectorGetX(v),XMVectorGetY(v),XMVectorGetZ(v));
}

//三角形ポリゴンの法線を求める
XMVECTOR TriangleNormal(XMVECTOR p0, XMVECTOR p1, XMVECTOR p2)
{
	XMVECTOR v10 = XMVectorSubtract(p1, p0);
	XMVECTOR v20 = XMVectorSubtract(p2, p0);
	XMVECTOR nor = XMVector3Cross(v10, v20);
	return XMVector3Normalize(nor);
}

// 三角形ポリゴンの角度を求める
// ベクトルp1-p0とp2-p0の角度
float TriangleAngle(XMVECTOR p0, XMVECTOR p1, XMVECTOR p2)
{
	XMVECTOR v10 = XMVector3Normalize( XMVectorSubtract(p1,p0) );
	XMVECTOR v20 = XMVector3Normalize( XMVectorSubtract(p2,p0) );
	// 内積 v10・v20 = |v10||v20|cosθ
	// 正規化しているので|v10| = |v20| = 1
	XMVECTOR cs = XMVector3Dot(v10,v20);//cosθ
	return XMScalarACos( XMVectorGetX(cs) );//θ：2つのベクトルの角度
}

// 三角形の面積を求める
float TriangleArea(XMVECTOR p0, XMVECTOR p1, XMVECTOR p2)
{
	XMVECTOR v10 = XMVectorSubtract(p1, p0);
	XMVECTOR v20 = XMVectorSubtract(p2, p0);
	XMVECTOR cross = XMVector3Cross(v10, v20);
	// 外積の長さ＝v10とv20で作られる平行四辺形の面積　
	// 半分にすると三角形の面積
	return XMVectorGetX( XMVector3Length(cross) )*0.5f;
}


//----------------------------------------
//指定したフェイスの法線を求める
XMVECTOR FaceNormal(const MQOObject& obj, zgU32 face_idx)
{
	XMVECTOR nor = XMVectorSet(0,1,0,0);
	if( face_idx >= obj.aFace.size() )return nor;
	const MQOFace& face = obj.aFace[face_idx];
	XMVECTOR v0 = ToXMVec( obj.aVertex[ face.aVidx[0] ] );
	XMVECTOR v1 = ToXMVec( obj.aVertex[ face.aVidx[1] ] );
	XMVECTOR v2 = ToXMVec( obj.aVertex[ face.aVidx[2] ] );
	return TriangleNormal(v0,v1,v2);
}

//----------------------------------------
// 指定した頂点を角にしたポリゴンの角度(rad)
float FacePolyAngle(const MQOObject& obj, zgU32 face_idx, zgU32 vidx)
{
	if( face_idx >= obj.aFace.size() )return 0.0f;
	const MQOFace& face = obj.aFace[face_idx];
	
	for(zgU32 p=0;p<3;++p){
		if(vidx == face.aVidx[p]){
			XMVECTOR v0 = ToXMVec( obj.aVertex[ face.aVidx[p] ] );
			XMVECTOR v1 = ToXMVec( obj.aVertex[ face.aVidx[(p+1)%3] ] );
			XMVECTOR v2 = ToXMVec( obj.aVertex[ face.aVidx[(p+2)%3] ] );
			return TriangleAngle(v0,v1,v2);
		}
	}
	return 0.0f;//こないはずだけど
}
//----------------------------------------
//指定したフェイスの面積を求める
float FacePolyArea(const MQOObject& obj, zgU32 face_idx)
{
	if( face_idx >= obj.aFace.size() )return 0.0f;
	const MQOFace& face = obj.aFace[face_idx];
	XMVECTOR v0 = ToXMVec( obj.aVertex[ face.aVidx[0] ] );
	XMVECTOR v1 = ToXMVec( obj.aVertex[ face.aVidx[1] ] );
	XMVECTOR v2 = ToXMVec( obj.aVertex[ face.aVidx[2] ] );
	return TriangleArea(v0,v1,v2);
}

}//ns

//----------------------------------------
// 法線を計算で求める
bool ComputeNormal(MQOModel& mqo)
{
	zgU32 obj_num = mqo.aObject.size();
	for(zgU32 o=0;o<obj_num;++o){
		MQOObject& obj = mqo.aObject[o];
		//法線スムージング角度
		float smooth = cosf(obj.radFacet);

		//頂点が構成するポリゴン（フェイス）のリストアップ
		std::map<zgU32, std::vector<zgU32>> facemap;//map<頂点idx, フェイスidx[]>
		std::map<zgU32, std::vector<zgU32>>::iterator it;
		for(zgU32 f=0;f<obj.aFace.size();++f){
			const MQOFace& face = obj.aFace[f];
			XMVECTOR face_nor = FaceNormal(obj, f);
			for(zgU32 p=0;p<3;++p){
				zgU32 vidx = face.aVidx[p];
				it = facemap.find(vidx);
				if(it == facemap.end()){
					std::vector<zgU32> flist;
					flist.push_back(f);
					facemap.insert( std::pair<zgU32, std::vector<zgU32>>(vidx,flist) );
				}else{
					std::vector<zgU32>& flist = it->second;
					zgU32 fl=0;
					for(fl=0;fl<flist.size();++fl){
						if(flist[fl] == f)break;//すでに登録
					}
					if(fl == flist.size()){
						 flist.push_back(f);
					}
				}
			}
		}
		//メモリ消費が気になるなら、法線の計算ごとに頂点が構成するポリゴンを検索


		for(zgU32 f=0;f<obj.aFace.size();++f){
			MQOFace& face = obj.aFace[f];
			XMVECTOR face_nor = FaceNormal(obj, f);
			float face_area = FacePolyArea(obj, f);//面積
			if(face_area < 0.000001f)continue;//面積0のポリゴン

			face.aNor[0] = ToVec3(face_nor);
			face.aNor[1] = face.aNor[0];
			face.aNor[2] = face.aNor[0];
			for(zgU32 p=0;p<3;++p){
				zgU32 vidx = face.aVidx[p];
				it = facemap.find(vidx);
				if(it == facemap.end())continue;//ないはずだけど
				std::vector<zgU32>& flist = it->second;

				XMVECTOR nor = {0,0,0,0};
				float ang0 = FacePolyAngle(obj, f, vidx);
				//face_area>0ならang0>0

				//頂点vidxを含むポリゴンの法線を平均化
				for(auto fl : flist){//
					//ポリゴンの角度と面積を重みにして平均
					float ang = FacePolyAngle(obj, fl, vidx);
					float area = FacePolyArea(obj, fl);
					XMVECTOR n = FaceNormal(obj, fl);

					//一定の角度以上はスムージングしない
					if(XMVectorGetX(XMVector3Dot(n,face_nor)) > smooth){
						n = XMVectorScale(n, ang*area/ang0/face_area);
						//n = XMVectorScale(n, ang/ang0);
						nor = XMVectorAdd(nor, n);
					}

					//メモ　
					//XMVectorGreaterなどを使ったほうが効率がいいかも
					//ベクトル型とfloatの変換は効率が悪いはず
					//メタセコイアと微妙にスムージング判定が違うみたい
				}
				//平均化（スムージング）された法線
				nor = XMVector3Normalize(nor);
				face.aNor[p] = ToVec3(nor);
			}
		}
	} 
	return true;
}

//--------------------------------------------
bool CreateMQOFromFile(const WCHAR* file, MQOModel& mqo)
{
	std::ifstream input;
	input.open(file, std::ios::binary);
	if( !input.is_open() )return false;
	size_t fsize = (size_t)input.seekg(0, std::ios::end).tellg();
	input.seekg(0, std::ios::beg);
	BinObject blob(fsize);
	if( !blob.get() )return false;
	char* b = static_cast<char*>(blob.get());
	input.read(b, blob.size());
	return CreateMQO(b, blob.size(), mqo);
}


namespace{
using zgmdl::ModelPtr;
using zgmdl::Model;
using zgmdl::MaterialPtr;
using zgmdl::Material;
using zgmdl::ObjectPtr;
using zgmdl::Object;
using zgmdl::Vertex;
using zgmdl::MeshPtr;
using zgmdl::Mesh;
	

//------------
	struct MQOMeshInfo
{
	zgU32 idxMat;
	zgU32 numPoly3;//三角形ポリゴン数
};

//------------
struct MQOObjInfo
{
	zgU32 numMesh;	// 使用マテリアル数
	std::vector<MQOMeshInfo> aMesh;
};

//----------------------------------------------
// オブジェクトのポリゴン数、使用マテリアル数などを計算
bool GetMQOObjInfo(const MQOObject& obj, MQOObjInfo& info)
{
	//map<マテリアルidx,　ポリゴン数>
	std::map<zgU32,zgU32> matmap;
	std::map<zgU32,zgU32>::iterator it;

	zgU32 fnum = obj.aFace.size();
	for(zgU32 f=0;f<fnum;++f){
		const MQOFace& face = obj.aFace[f];
		zgU32 midx = face.idxMat;
		it = matmap.find(midx);
		if(it == matmap.end()){
			//新しいマテリアル
			matmap.insert( std::pair<zgU32,zgU32>(midx, 1) );
		}else{
			//マテリアルごとにポリゴン数計算
			(it->second) += 1;
		}
	}

	//MQOMeshInfoの作成
	zgU32 use_num = matmap.size();
	info.numMesh = use_num;
	info.aMesh.resize(use_num);
	it = matmap.begin();
	for(zgU32 m=0;m<use_num;++it,++m){
		MQOMeshInfo& mesh = info.aMesh[m];
		mesh.idxMat = it->first;
		mesh.numPoly3 = it->second;
	}

	return true;
}

//------------------------------------------------------
MeshPtr Create(const MQOObject& obj, const MQOMeshInfo& minfo)
{
	MeshPtr mesh(new Mesh);
	if(!mesh)return mesh;
	mesh->setMatIdx(minfo.idxMat);

	// 頂点データ作成
	Vertex& pos = mesh->getVertex(0);//Mesh::POSITION);
	Vertex& nor = mesh->getVertex(1);//Mesh::NORMAL);
	Vertex& tc0 = mesh->getVertex(2);//Mesh::TEXCRD0);
	Vertex& idx = mesh->getIndex();

	pos.Create(Vertex::POSITION, minfo.numPoly3*3, Vertex::FMT_F32, 3);
	nor.Create(Vertex::NORMAL, minfo.numPoly3*3, Vertex::FMT_F32, 3);
	tc0.Create(Vertex::TEXCRD0, minfo.numPoly3*3, Vertex::FMT_F32, 2);
	idx.Create(Vertex::INDEX, minfo.numPoly3*3, Vertex::FMT_U32, 1);

	// faceから3角形ポリゴン生成
	zgU32 vcnt = 0;
	for(zgU32 f=0;f<obj.aFace.size();++f){
		const MQOFace& face = obj.aFace[f];
		if(face.idxMat != minfo.idxMat)continue;//違うマテリアル
		// 三角形に変換して追加
		zgU32 t0 = 0;	
		zgU32 t1 = 1;	// ポリゴンの方向に注意
		zgU32 t2 = 2;	
		zgU32 i0 = face.aVidx[t0];
		zgU32 i1 = face.aVidx[t1];
		zgU32 i2 = face.aVidx[t2];

		pos.setData(vcnt + 0, obj.aVertex[i0]);
		pos.setData(vcnt + 1, obj.aVertex[i1]);
		pos.setData(vcnt + 2, obj.aVertex[i2]);

		nor.setData(vcnt + 0, face.aNor[t0]);
		nor.setData(vcnt + 1, face.aNor[t1]);
		nor.setData(vcnt + 2, face.aNor[t2]);
			
		tc0.setData(vcnt + 0, face.aUV[t0]);
		tc0.setData(vcnt + 1, face.aUV[t1]);
		tc0.setData(vcnt + 2, face.aUV[t2]);

		idx.setData(vcnt + 0, vcnt + 0);//単純にインデックス作成
		idx.setData(vcnt + 1, vcnt + 1);//頂点の共有なし
		idx.setData(vcnt + 2, vcnt + 2);

		vcnt += 3;
	}

	return mesh;
}

}//ns

//-------------------------------------------
ModelPtr CreateModel(const MQOModel& mqo)
{
	ModelPtr model(new Model);
	if(!model)return ModelPtr();

	//マテリアル
	zgU32 mat_num = mqo.aMaterial.size();
	model->SetMatNum(mat_num);
	for(zgU32 m=0;m<mat_num;++m){
		MaterialPtr mat(new Material);
		if(!mat)break;
		mat->strName = mqo.aMaterial[m].strName;
		mat->strTex = mqo.aMaterial[m].strTexName;
		mat->v4Diff = mqo.aMaterial[m].v4Col;
		model->setMat(m, mat);
	}

	//オブジェクト
	zgU32 obj_num = mqo.aObject.size();
	model->SetObjNum(obj_num);
	for(zgU32 o=0;o<obj_num;++o){
		MQOObjInfo objinfo;
		if( !GetMQOObjInfo(mqo.aObject[o], objinfo) ){
			continue;//不正なデータ
		}
		ObjectPtr obj(new Object);
		if(!obj)return false;

		//メッシュ
		obj->SetMeshNum(objinfo.numMesh);
		for(zgU32 m=0;m<objinfo.numMesh;++m){
			MeshPtr mesh = Create(mqo.aObject[o], objinfo.aMesh[m]);
			obj->setMesh(m, mesh);

		}
		 
		model->setObj(o, obj);
	} 

	return model;
}