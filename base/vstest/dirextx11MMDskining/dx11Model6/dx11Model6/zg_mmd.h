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
	ZGVECTOR3 pos;		// �ʒu 
	ZGVECTOR3 normal;	// �@���x�N�g��
	ZGVECTOR2 uv;		// UV���W
	zgU16 bone_idx[2];	// �{�[���ԍ�
	zgU8 bone_weight;	// �{�[��1�E�G�C�g min:0 max:100  �{�[��2=100 - bone_weight
	zgU8 edge_flag;		// 0:�ʏ�A1:�G�b�W����
};
//--------------------------------
struct PMDMaterial
{
	ZGVECTOR4 diffuse;
	float specularity;
	ZGVECTOR3 specular;	// ����F
	ZGVECTOR3 ambient;	// ���F(ambient)
	zgU8 toon_index;	// toon??.bmp // 0.bmp:0xFF, 1(01).bmp:0x00 �E�E�E 10.bmp:0x09
	zgU8 edge_flag;		// �֊s�A�e
	zgU32 face_vert_count; // �ʒ��_�� // �C���f�b�N�X�ɕϊ�����ꍇ�́A�ގ�0���珇�ɉ��Z
	char texfile_name[20]; // �e�N�X�`���t�@�C�����܂��̓X�t�B�A�t�@�C���� // 20�o�C�g���肬��܂Ŏg����(�I�[��0x00�͖����Ă�����)
};
//--------------------------------
struct PMDBone
{
	char bone_name[20];	// �{�[����
	zgU16 parent_bidx;	// �e�{�[���ԍ�(�Ȃ��ꍇ��0xFFFF)
	WORD tail_pos_bidx; // tail�ʒu�̃{�[���ԍ�(�`�F�[�����[�̏ꍇ��0xFFFF) // �e�F�q��1�F���Ȃ̂ŁA��Ɉʒu���ߗp
	zgU8 bone_type;		// �{�[���̎��
	WORD ik_parent_bidx;// IK�{�[���ԍ�(�e��IK�{�[���B�Ȃ��ꍇ��0)
	ZGVECTOR3 bone_head_pos; // x, y, z // �{�[���̃w�b�h�̈ʒu
};
//--------------------------------
struct PMDIk
{
	zgU16 bone_index; // IK�{�[���ԍ�
	zgU16 target_bone_index; // IK�^�[�Q�b�g�{�[���ԍ� // IK�{�[�����ŏ��ɐڑ�����{�[��
	zgU8 chain_length; // IK�`�F�[���̒���(�q�̐�)
	zgU16 iterations; // �ċA���Z�� // IK�l1
	zgF32 control_weight; // IK�̉e���x // IK�l2
	std::vector<zgU16> child_bone_index;//[ik_chain_length] IK�e�����̃{�[���ԍ�
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



