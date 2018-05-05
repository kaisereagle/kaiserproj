#include "stdafx.h"
#include "zg_mmd.h"

#include "zg_utils.h"
#include "zg_model.h"

#include <directxmath.h>
#include <fstream>
#include <boost/format.hpp>
using namespace DirectX;

namespace{

bool get_byte(const void* &ptr, const void* end, void* buff, size_t n)
{
	const zgU8* last = (zgU8*)ptr + n;
	if(last > end)return false;//範囲オーバー

	memcpy(buff, ptr, n);
	ptr = last;
	return true;
}

template <typename TYPE>
bool get_val(const void* &ptr, const void* end, TYPE& v)
{
	return get_byte(ptr,end, &v, sizeof(TYPE));
}

XMVECTOR ToXMVec(const ZGVECTOR3& v)
{
	return XMVectorSet(v.x,v.y,v.z, 0);
}
XMVECTOR ToXMVec(const ZGVECTOR4& v)
{
	return XMVectorSet(v.x,v.y,v.z,v.w);
}
XMVECTOR ToXMVecW1(const ZGVECTOR3& v)
{
	return XMVectorSet(v.x,v.y,v.z, 1);
}
void ToXMMat(XMMATRIX& xmm, const ZGMATRIX44& m44)
{
	xmm.r[0] = ToXMVec(m44.m[0]);
	xmm.r[1] = ToXMVec(m44.m[1]);
	xmm.r[2] = ToXMVec(m44.m[2]);
	xmm.r[3] = ToXMVec(m44.m[3]);
}

ZGVECTOR3 ToVec3(XMVECTOR v)
{
	return ZGVECTOR3(XMVectorGetX(v),XMVectorGetY(v),XMVectorGetZ(v));
}
ZGVECTOR4 ToVec4(XMVECTOR v)
{
	return ZGVECTOR4(XMVectorGetX(v),XMVectorGetY(v),XMVectorGetZ(v),XMVectorGetW(v));
}
ZGQUAT ToQuat(XMVECTOR v)
{
	return ZGQUAT(XMVectorGetX(v),XMVectorGetY(v),XMVectorGetZ(v),XMVectorGetW(v));
}

void ToMatrix44(ZGMATRIX44& m44, const XMMATRIX& xmm)
{
	m44.m[0] = ToVec4(xmm.r[0]);
	m44.m[1] = ToVec4(xmm.r[1]);
	m44.m[2] = ToVec4(xmm.r[2]);
	m44.m[3] = ToVec4(xmm.r[3]);
}

}//ns

//--------------------------------------------------------
bool CreatePMD(const WCHAR* file, PMDModel& pmd)
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

	const void* ptr = b;
	const void* end = b + blob.size();

	get_val(ptr,end, pmd.Header.magic);
	get_val(ptr,end, pmd.Header.version);
	get_val(ptr,end, pmd.Header.model_name);
	get_val(ptr,end, pmd.Header.comment);

	zgU32 vertex_num = 0;
	get_val(ptr,end, vertex_num);
	pmd.aVertex.resize(vertex_num);

	for(zgU32 vi=0;vi<vertex_num;++vi){
		PMDVertex& vertex = pmd.aVertex[vi];
		get_val(ptr,end, vertex.pos);
		get_val(ptr,end, vertex.normal);
		get_val(ptr,end, vertex.uv);
		get_val(ptr,end, vertex.bone_idx);
		get_val(ptr,end, vertex.bone_weight);
		get_val(ptr,end, vertex.edge_flag);
		//paddingがあるのでget_val(ptr,end, vertex);はだめ
	}

	zgU32 face_vidx_num = 0;
	get_val(ptr,end, face_vidx_num);
	pmd.aFaceVidx.resize(face_vidx_num);

	for(zgU32 fi=0;fi<face_vidx_num;++fi){
		zgU16& face_vidx = pmd.aFaceVidx[fi];
		get_val(ptr,end, face_vidx);
	}

	zgU32 material_num = 0;
	get_val(ptr,end, material_num);
	pmd.aMaterial.resize(material_num);

	for(zgU32 mi=0;mi<material_num;++mi){
		PMDMaterial& material = pmd.aMaterial[mi];
		get_val(ptr,end, material.diffuse);
		get_val(ptr,end, material.specularity);
		get_val(ptr,end, material.specular);
		get_val(ptr,end, material.ambient);
		get_val(ptr,end, material.toon_index);
		get_val(ptr,end, material.edge_flag);
		get_val(ptr,end, material.face_vert_count);
		get_val(ptr,end, material.texfile_name);
	}

	zgU16 bone_num = 0;
	get_val(ptr,end, bone_num);
	pmd.aBone.resize(bone_num);

	for(zgU32 bi=0;bi<bone_num;++bi){
		PMDBone& bone = pmd.aBone[bi];
		get_val(ptr,end, bone.bone_name);
		get_val(ptr,end, bone.parent_bidx);
		get_val(ptr,end, bone.tail_pos_bidx);
		get_val(ptr,end, bone.bone_type);
		get_val(ptr,end, bone.ik_parent_bidx);
		get_val(ptr,end, bone.bone_head_pos);
	}

	zgU16 ik_num = 0;
	get_val(ptr, end, ik_num);
	pmd.aIK.resize(ik_num);
	for(auto& ik : pmd.aIK){
		get_val(ptr,end, ik.bone_index);
		get_val(ptr,end, ik.target_bone_index);
		get_val(ptr,end, ik.chain_length);
		get_val(ptr,end, ik.iterations);
		get_val(ptr,end, ik.control_weight);
		ik.child_bone_index.resize(ik.chain_length);
		for(auto& cb : ik.child_bone_index){
			get_val(ptr, end, cb);
		}
	}


	return true;
}

//---------------------------------------------
bool CreateVMD(const WCHAR* file, VMDMotion& vmd)
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

	const void* ptr = b;
	const void* end = b + blob.size();

	get_val(ptr,end, vmd.Header.Header);
	get_val(ptr,end, vmd.Header.ModelName);

	zgU32 key_num = 0;
	get_val(ptr,end, key_num);
	vmd.aKeyFrame.resize(key_num);
	for(zgU32 ik=0;ik<key_num;++ik){
		VMDKeyFlame& key = vmd.aKeyFrame[ik];
		get_val(ptr,end, key.BoneName);
		get_val(ptr,end, key.FlameNo);
		get_val(ptr,end, key.Location);
		get_val(ptr,end, key.Rotatation);
		get_val(ptr,end, key.Interpolation);
	}

	return true;
}


//-------------------------------------------
zgmdl::ModelPtr CreateModel(const PMDModel& pmd)
{
	using zgmdl::ModelPtr;
	using zgmdl::Model;
	using zgmdl::MaterialPtr;
	using zgmdl::Material;
	using zgmdl::ObjectPtr;
	using zgmdl::Object;
	using zgmdl::Vertex;
	using zgmdl::MeshPtr;
	using zgmdl::Mesh;
	using zgmdl::Bone;
	using zgmdl::Hierarchy;
	
	ModelPtr model(new Model);
	if(!model)return ModelPtr();
	
	//マテリアル
	zgU32 mat_num = pmd.aMaterial.size();
	model->SetMatNum(mat_num);
	for(zgU32 m=0;m<mat_num;++m){
		MaterialPtr mat(new Material);
		if(!mat)break;
		mat->strName = (boost::format("mat%1%") % m).str();

		//テクスチャファイル
		//*スフィアマップの分離 texfile_nameが0で終わらないようなので面倒
		auto& texfile_name = pmd.aMaterial[m].texfile_name;
		char texfile[sizeof(texfile_name)+1];
		strncpy_s(texfile, texfile_name, sizeof(texfile_name));
		texfile[sizeof(texfile_name)] = 0;
		char* str = strchr(texfile,'*');
		if(str)*str = 0;

		mat->strTex = texfile;
		mat->v4Diff = pmd.aMaterial[m].diffuse;

		model->setMat(m, mat);
	}

	model->SetObjNum(1);
	ObjectPtr obj(new Object);
	if(!obj)return ModelPtr();
	
	//メッシュ
	obj->SetMeshNum(mat_num);
	zgU32 vidx_start = 0;
	for(zgU32 m=0;m<mat_num;++m){
		MeshPtr mesh(new Mesh);
		if(!mesh)return ModelPtr();
		
		mesh->setMatIdx(m);

		zgU32 face_vidx_num = pmd.aMaterial[m].face_vert_count;
		Vertex& pos = mesh->getVertex(0);
		Vertex& nor = mesh->getVertex(1);
		Vertex& tc0 = mesh->getVertex(2);
		Vertex& boneidx = mesh->getVertex(3);
		Vertex& bonewgt = mesh->getVertex(4);
		Vertex& idx = mesh->getIndex();

		pos.Create(Vertex::POSITION, face_vidx_num, Vertex::FMT_F32, 3);
		nor.Create(Vertex::NORMAL, face_vidx_num, Vertex::FMT_F32, 3);
		tc0.Create(Vertex::TEXCRD0, face_vidx_num, Vertex::FMT_F32, 2);
		boneidx.Create(Vertex::BONEINDEX, face_vidx_num, Vertex::FMT_U16, 4);
		bonewgt.Create(Vertex::BONEWEIGHT, face_vidx_num, Vertex::FMT_U8, 4);
		idx.Create(Vertex::INDEX, face_vidx_num, Vertex::FMT_U32, 1);

		for(zgU32 v=0;v<face_vidx_num;++v){
			zgU32 vidx = pmd.aFaceVidx[vidx_start + v]; 
			auto& vtx = pmd.aVertex[vidx];
			pos.setData(v, vtx.pos);
			nor.setData(v, vtx.normal);
			tc0.setData(v, vtx.uv);

			// ボーンウエイト
			zgU16 bidx[4] = {vtx.bone_idx[0], vtx.bone_idx[1], 0, 0};
			zgU8 bwgt[4] = {vtx.bone_weight, 100-vtx.bone_weight, 0, 0};
			boneidx.setData(v, bidx);
			bonewgt.setData(v, bwgt);

			// 共有頂点をすべて分離
			// 中間モデルで再度頂点の共有（マージ）予定
			idx.setData(v, v);
		}

		vidx_start += face_vidx_num;

		obj->setMesh(m, mesh);
	}

	model->setObj(0, obj);

	//ボーン
	zgU32 bone_num = pmd.aBone.size();
	model->SetBoneNum(bone_num);
	for(zgU32 ib=0;ib<bone_num;++ib){
		Bone* bone = model->getBone(ib);
		Hierarchy* hier = model->getHier(ib);
		if(!bone || !hier)break;
		bone->strName = pmd.aBone[ib].bone_name;
		hier->idxParent = pmd.aBone[ib].parent_bidx;


		if(hier->idxParent >= bone_num) hier->idxParent = zgU32(-1);
		hier->idxSelf = ib;

		// とりあえず回転を無視（単位行列固定）した階層構造
		// ボーンの位置に合わせた姿勢にするとモーションがうまく再生できない
		// それっぽく動いているので、とりあえず後回しに
		// MMDでボーンの方向を見てみると
		// 足のボーンは-Y軸方向
		// 腕のボーンは+X軸方向
		// どういう基準でボーンの方向を決定しているか不明
		// たぶんボーンのhead-tail方向に一番近い軸と思うけど

		const PMDBone& pmd_bone = pmd.aBone[ib];


		XMVECTOR head_pos = ToXMVec(pmd_bone.bone_head_pos);
		XMVECTOR parent_pos =  {0,0,0,1};
		if( hier->idxParent < bone_num ){
			parent_pos = ToXMVec( pmd.aBone[ hier->idxParent ].bone_head_pos );
		}

		XMVECTOR tail_pos = {0,0,0,1};
		if( pmd_bone.tail_pos_bidx < bone_num ){
			tail_pos = ToXMVec(pmd.aBone[ pmd_bone.tail_pos_bidx ].bone_head_pos);
		}
		XMVECTOR local_pos = XMVectorSubtract(head_pos, parent_pos);

		bone->vPos = ToVec3(local_pos);
		bone->vScale = ZGVECTOR3(1.0f, 1.0f, 1.0f);
		bone->qRot = ToQuat(XMQuaternionIdentity());
		
	}


	return model;
}
