//*******************************
// VPD�|�[�Y�p�e��\���̒�`
//*******************************

#ifndef	_VPDTYPES_H_
#define	_VPDTYPES_H_

#include	"VecMatQuat.h"

#pragma pack( push, 1 )

// �t�@�C���w�b�_
struct VPD_Header
{
	char	szHeader[30];			// "Vocaloid Pose Data file"
	char	szBaseFile[30];			// �e�t�@�C����
	unsigned int unNumBones;		// �{�[����
};

// �{�[���f�[�^
struct VPD_Bone
{
	char	szBoneName[15];			// �{�[����

	Vector3	vec3Position;			// �ʒu
	Vector4	vec4Rotation;			// ��](�N�H�[�^�j�I��)
};

#pragma pack( pop )

#endif	// _VPDTYPES_H_
