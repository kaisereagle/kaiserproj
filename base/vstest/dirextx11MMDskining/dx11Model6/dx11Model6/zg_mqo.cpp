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

//�O�p�`�|���S���̖@�������߂�
XMVECTOR TriangleNormal(XMVECTOR p0, XMVECTOR p1, XMVECTOR p2)
{
	XMVECTOR v10 = XMVectorSubtract(p1, p0);
	XMVECTOR v20 = XMVectorSubtract(p2, p0);
	XMVECTOR nor = XMVector3Cross(v10, v20);
	return XMVector3Normalize(nor);
}

// �O�p�`�|���S���̊p�x�����߂�
// �x�N�g��p1-p0��p2-p0�̊p�x
float TriangleAngle(XMVECTOR p0, XMVECTOR p1, XMVECTOR p2)
{
	XMVECTOR v10 = XMVector3Normalize( XMVectorSubtract(p1,p0) );
	XMVECTOR v20 = XMVector3Normalize( XMVectorSubtract(p2,p0) );
	// ���� v10�Ev20 = |v10||v20|cos��
	// ���K�����Ă���̂�|v10| = |v20| = 1
	XMVECTOR cs = XMVector3Dot(v10,v20);//cos��
	return XMScalarACos( XMVectorGetX(cs) );//�ƁF2�̃x�N�g���̊p�x
}

// �O�p�`�̖ʐς����߂�
float TriangleArea(XMVECTOR p0, XMVECTOR p1, XMVECTOR p2)
{
	XMVECTOR v10 = XMVectorSubtract(p1, p0);
	XMVECTOR v20 = XMVectorSubtract(p2, p0);
	XMVECTOR cross = XMVector3Cross(v10, v20);
	// �O�ς̒�����v10��v20�ō���镽�s�l�ӌ`�̖ʐρ@
	// �����ɂ���ƎO�p�`�̖ʐ�
	return XMVectorGetX( XMVector3Length(cross) )*0.5f;
}


//----------------------------------------
//�w�肵���t�F�C�X�̖@�������߂�
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
// �w�肵�����_���p�ɂ����|���S���̊p�x(rad)
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
	return 0.0f;//���Ȃ��͂�������
}
//----------------------------------------
//�w�肵���t�F�C�X�̖ʐς����߂�
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
// �@�����v�Z�ŋ��߂�
bool ComputeNormal(MQOModel& mqo)
{
	zgU32 obj_num = mqo.aObject.size();
	for(zgU32 o=0;o<obj_num;++o){
		MQOObject& obj = mqo.aObject[o];
		//�@���X���[�W���O�p�x
		float smooth = cosf(obj.radFacet);

		//���_���\������|���S���i�t�F�C�X�j�̃��X�g�A�b�v
		std::map<zgU32, std::vector<zgU32>> facemap;//map<���_idx, �t�F�C�Xidx[]>
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
						if(flist[fl] == f)break;//���łɓo�^
					}
					if(fl == flist.size()){
						 flist.push_back(f);
					}
				}
			}
		}
		//����������C�ɂȂ�Ȃ�A�@���̌v�Z���Ƃɒ��_���\������|���S��������


		for(zgU32 f=0;f<obj.aFace.size();++f){
			MQOFace& face = obj.aFace[f];
			XMVECTOR face_nor = FaceNormal(obj, f);
			float face_area = FacePolyArea(obj, f);//�ʐ�
			if(face_area < 0.000001f)continue;//�ʐ�0�̃|���S��

			face.aNor[0] = ToVec3(face_nor);
			face.aNor[1] = face.aNor[0];
			face.aNor[2] = face.aNor[0];
			for(zgU32 p=0;p<3;++p){
				zgU32 vidx = face.aVidx[p];
				it = facemap.find(vidx);
				if(it == facemap.end())continue;//�Ȃ��͂�������
				std::vector<zgU32>& flist = it->second;

				XMVECTOR nor = {0,0,0,0};
				float ang0 = FacePolyAngle(obj, f, vidx);
				//face_area>0�Ȃ�ang0>0

				//���_vidx���܂ރ|���S���̖@���𕽋ω�
				for(auto fl : flist){//
					//�|���S���̊p�x�Ɩʐς��d�݂ɂ��ĕ���
					float ang = FacePolyAngle(obj, fl, vidx);
					float area = FacePolyArea(obj, fl);
					XMVECTOR n = FaceNormal(obj, fl);

					//���̊p�x�ȏ�̓X���[�W���O���Ȃ�
					if(XMVectorGetX(XMVector3Dot(n,face_nor)) > smooth){
						n = XMVectorScale(n, ang*area/ang0/face_area);
						//n = XMVectorScale(n, ang/ang0);
						nor = XMVectorAdd(nor, n);
					}

					//�����@
					//XMVectorGreater�Ȃǂ��g�����ق�����������������
					//�x�N�g���^��float�̕ϊ��͌����������͂�
					//���^�Z�R�C�A�Ɣ����ɃX���[�W���O���肪�Ⴄ�݂���
				}
				//���ω��i�X���[�W���O�j���ꂽ�@��
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
	zgU32 numPoly3;//�O�p�`�|���S����
};

//------------
struct MQOObjInfo
{
	zgU32 numMesh;	// �g�p�}�e���A����
	std::vector<MQOMeshInfo> aMesh;
};

//----------------------------------------------
// �I�u�W�F�N�g�̃|���S�����A�g�p�}�e���A�����Ȃǂ��v�Z
bool GetMQOObjInfo(const MQOObject& obj, MQOObjInfo& info)
{
	//map<�}�e���A��idx,�@�|���S����>
	std::map<zgU32,zgU32> matmap;
	std::map<zgU32,zgU32>::iterator it;

	zgU32 fnum = obj.aFace.size();
	for(zgU32 f=0;f<fnum;++f){
		const MQOFace& face = obj.aFace[f];
		zgU32 midx = face.idxMat;
		it = matmap.find(midx);
		if(it == matmap.end()){
			//�V�����}�e���A��
			matmap.insert( std::pair<zgU32,zgU32>(midx, 1) );
		}else{
			//�}�e���A�����ƂɃ|���S�����v�Z
			(it->second) += 1;
		}
	}

	//MQOMeshInfo�̍쐬
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

	// ���_�f�[�^�쐬
	Vertex& pos = mesh->getVertex(0);//Mesh::POSITION);
	Vertex& nor = mesh->getVertex(1);//Mesh::NORMAL);
	Vertex& tc0 = mesh->getVertex(2);//Mesh::TEXCRD0);
	Vertex& idx = mesh->getIndex();

	pos.Create(Vertex::POSITION, minfo.numPoly3*3, Vertex::FMT_F32, 3);
	nor.Create(Vertex::NORMAL, minfo.numPoly3*3, Vertex::FMT_F32, 3);
	tc0.Create(Vertex::TEXCRD0, minfo.numPoly3*3, Vertex::FMT_F32, 2);
	idx.Create(Vertex::INDEX, minfo.numPoly3*3, Vertex::FMT_U32, 1);

	// face����3�p�`�|���S������
	zgU32 vcnt = 0;
	for(zgU32 f=0;f<obj.aFace.size();++f){
		const MQOFace& face = obj.aFace[f];
		if(face.idxMat != minfo.idxMat)continue;//�Ⴄ�}�e���A��
		// �O�p�`�ɕϊ����Ēǉ�
		zgU32 t0 = 0;	
		zgU32 t1 = 1;	// �|���S���̕����ɒ���
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

		idx.setData(vcnt + 0, vcnt + 0);//�P���ɃC���f�b�N�X�쐬
		idx.setData(vcnt + 1, vcnt + 1);//���_�̋��L�Ȃ�
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

	//�}�e���A��
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

	//�I�u�W�F�N�g
	zgU32 obj_num = mqo.aObject.size();
	model->SetObjNum(obj_num);
	for(zgU32 o=0;o<obj_num;++o){
		MQOObjInfo objinfo;
		if( !GetMQOObjInfo(mqo.aObject[o], objinfo) ){
			continue;//�s���ȃf�[�^
		}
		ObjectPtr obj(new Object);
		if(!obj)return false;

		//���b�V��
		obj->SetMeshNum(objinfo.numMesh);
		for(zgU32 m=0;m<objinfo.numMesh;++m){
			MeshPtr mesh = Create(mqo.aObject[o], objinfo.aMesh[m]);
			obj->setMesh(m, mesh);

		}
		 
		model->setObj(o, obj);
	} 

	return model;
}