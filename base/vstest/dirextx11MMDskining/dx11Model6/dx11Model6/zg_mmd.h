#pragma once
#include "zg_types.h"

#include <vector>

//--------------------------------
struct PMDHeader
{
	char magic[3]; // "Pmd"
	float version; //
	char model_name[20];
	char comment[256];
};
//--------------------------------
struct PMDVertex
{
	ZGVECTOR3 pos;		// 位置 
	ZGVECTOR3 normal;	// 法線ベクトル
	ZGVECTOR2 uv;		// UV座標
	zgU16 bone_idx[2];	// ボーン番号
	zgU8 bone_weight;	// ボーン1ウエイト min:0 max:100  ボーン2=100 - bone_weight
	zgU8 edge_flag;		// 0:通常、1:エッジ無効
};
//--------------------------------
struct PMDMaterial
{
	ZGVECTOR4 diffuse;
	float specularity;
	ZGVECTOR3 specular;	// 光沢色
	ZGVECTOR3 ambient;	// 環境色(ambient)
	zgU8 toon_index;	// toon??.bmp // 0.bmp:0xFF, 1(01).bmp:0x00 ・・・ 10.bmp:0x09
	zgU8 edge_flag;		// 輪郭、影
	zgU32 face_vert_count; // 面頂点数 // インデックスに変換する場合は、材質0から順に加算
	char texfile_name[20]; // テクスチャファイル名またはスフィアファイル名 // 20バイトぎりぎりまで使える(終端の0x00は無くても動く)
};
//--------------------------------
struct PMDBone
{
	char bone_name[20];	// ボーン名
	zgU16 parent_bidx;	// 親ボーン番号(ない場合は0xFFFF)
	WORD tail_pos_bidx; // tail位置のボーン番号(チェーン末端の場合は0xFFFF) // 親：子は1：多なので、主に位置決め用
	zgU8 bone_type;		// ボーンの種類
	WORD ik_parent_bidx;// IKボーン番号(影響IKボーン。ない場合は0)
	ZGVECTOR3 bone_head_pos; // x, y, z // ボーンのヘッドの位置
};
//--------------------------------
struct PMDIk
{
	zgU16 bone_index; // IKボーン番号
	zgU16 target_bone_index; // IKターゲットボーン番号 // IKボーンが最初に接続するボーン
	zgU8 chain_length; // IKチェーンの長さ(子の数)
	zgU16 iterations; // 再帰演算回数 // IK値1
	zgF32 control_weight; // IKの影響度 // IK値2
	std::vector<zgU16> child_bone_index;//[ik_chain_length] IK影響下のボーン番号
};

//------------------------------------
struct PMDModel
{ 
	PMDHeader Header;
	std::vector<PMDVertex> aVertex;
	std::vector<zgU16> aFaceVidx;
	std::vector<PMDMaterial> aMaterial;
	std::vector<PMDBone> aBone;
	std::vector<PMDIk> aIK;
};


//--------------------------------------
struct VMDHeader
{
	char Header[30]; // "Vocaloid Motion Data 0002"
	char ModelName[20];
};

//--------------------------------------
struct VMDKeyFlame
{
	char BoneName[15];
	zgU32 FlameNo;
	ZGVECTOR3 Location;
	ZGQUAT Rotatation; // Quaternion
	zgS8 Interpolation[64]; // [4][4][4]
};

//------------------------------------
struct VMDMotion
{
	VMDHeader Header;
	std::vector<VMDKeyFlame> aKeyFrame;
};


//---------------------------------------------
bool CreatePMD(const WCHAR* file, PMDModel& pmd);

//---------------------------------------------
bool CreateVMD(const WCHAR* file, VMDMotion& vmd);



