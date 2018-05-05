#include "stdafx.h"

#include "zg_model.h"
#include "zg_mqo.h"

#include <map>

namespace zgmdl
{

//-------------------------------------
Vertex::Vertex()
	: eFormat(FMT_NULL), eAttr(VATTR_NULL), sizeStride(0), sizeData(0), numData(0), pData(nullptr)
{}

Vertex::~Vertex()
{
	if(pData){
		::operator delete( pData );
	}
}
//-------------------------------------
bool Vertex::Create(VATTR attr, zgU32 num, FORMAT format, zgU32 elem_num)
{
	zgU32 elem_size = 0;
	switch(format){
	case FMT_F32:
	case FMT_U32:
		elem_size = 4;
		break;
	case FMT_U16:
		elem_size = 2;
		break;
	case FMT_U8:
		elem_size = 2;
		break;
	}
	if(elem_size == 0)return false;

	size_t s = num*elem_size*elem_num;
	
	pData = ::operator new(s);
	if(!pData)return false;
	numData = num;
	sizeData = s;
	sizeStride = elem_size*elem_num;
	eFormat = format;
	eAttr = attr;
	return true;
}
void* Vertex::rawget(zgU32 idx, zgU32 size)
{
	if(idx*size >= sizeData)return nullptr;
	return reinterpret_cast<zgU8*>(pData) + idx*size;
}
const void* Vertex::rawget(zgU32 idx, zgU32 size) const
{
	if(idx*size >= sizeData)return nullptr;
	return reinterpret_cast<zgU8*>(pData) + idx*size;
}

//-------------------------------------
bool Mesh::getSlot(Vertex::VATTR attr, zgU32& slot) const
{
	zgU32 s = 0;
	for(auto& vtx : aVertex){
		if(vtx.getAttr() == attr){
			slot = s;
			return true;
		}
		++s;
	}
	return false;//‚È‚¢
}




//-------------------------------------
zgU32 Object::MeshNum() const
{
	return apMesh.size();
}
//-------------------------------------
void Object::SetMeshNum(zgU32 num)
{
	apMesh.resize(num);
}
//-------------------------------------
bool Object::setMesh(zgU32 idx, const MeshPtr& mesh)
{
	if(idx >= MeshNum())return false;
	apMesh[idx] = mesh;
	return true;
}
//-------------------------------------
MeshPtr Object::getMesh(zgU32 idx)
{
	if(idx >= MeshNum())return MeshPtr();
	return apMesh[idx];
}


//-------------------------------------
zgU32 Model::MatNum() const
{
	return apMat.size();
}
zgU32 Model::ObjNum() const
{
	return apObj.size();
}
zgU32 Model::BoneNum() const
{
	return aBone.size();
}

//-------------------------------------
void Model::SetMatNum(zgU32 num)
{
	apMat.resize(num);
}
//-------------------------------------
bool Model::setMat(zgU32 idx, const MaterialPtr& mat)
{
	if(idx >= MatNum())return false;
	apMat[idx] = mat;
	return true;
}
//-------------------------------------
MaterialPtr Model::getMat(zgU32 idx)
{
	if(idx >= MatNum())return MaterialPtr();
	return apMat[idx];

}

//-------------------------------------
void Model::SetObjNum(zgU32 num)
{
	apObj.resize(num);
}
//-------------------------------------
bool Model::setObj(zgU32 idx, const ObjectPtr& obj)
{
	if(idx >= ObjNum())return false;
	apObj[idx] = obj;
	return true;
}
//-------------------------------------
ObjectPtr Model::getObj(zgU32 idx)
{
	if(idx >= ObjNum())return ObjectPtr();
	return apObj[idx];
}
//-------------------------------------
void Model::SetBoneNum(zgU32 num)
{
	aBone.resize(num);
	aHier.resize(num);
}
Bone* Model::getBone(zgU32 idx)
{
	if(idx >= aBone.size())return nullptr;
	return &aBone[idx];
}
const Bone* Model::getBone(zgU32 idx) const
{
	if(idx >= aBone.size())return nullptr;
	return &aBone[idx];
}
Hierarchy* Model::getHier(zgU32 idx)
{
	if(idx >= aHier.size())return nullptr;
	return &aHier[idx];
}
const Hierarchy* Model::getHier(zgU32 idx) const
{
	if(idx >= aHier.size())return nullptr;
	return &aHier[idx];
}

}//zgmdl
