#pragma once

#include <string>
#include <vector>
#include "zg_types.h"

//-----------------------------
struct MQOMaterial
{
	std::string strName;
	std::string strTexName;
	ZGVECTOR4 v4Col;
};

//-----------------------------
struct MQOFace
{
	// 三角形ポリゴンのみ 
	zgU32 idxMat;
	zgU32 aVidx[3];
	ZGVECTOR2 aUV[3];

	// 頂点から生成
	ZGVECTOR3 aNor[3];
};

//-----------------------------
struct MQOObject
{
	MQOObject() : radFacet(1.03847f){}//59.5°
	std::string strName;
	std::vector<ZGVECTOR3> aVertex;
	std::vector<MQOFace> aFace;

	zgF32 radFacet;//法線スムージング角度（rad)
};

//----------------------------------
struct MQOModel
{
	std::vector<MQOMaterial> aMaterial;
	std::vector<MQOObject> aObject;
};

//----------------------------------
// mqoファイルからMQOModel作成
bool CreateMQO(const char* data, size_t size, MQOModel& mqo);

//--------------------------------------------
bool CreateMQOFromFile(const WCHAR* file, MQOModel& mqo);

// 法線を計算で求める
bool ComputeNormal(MQOModel& mqo);



