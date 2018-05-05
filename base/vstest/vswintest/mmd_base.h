#pragma once
#include <stdint.h>
typedef unsigned char byte;
typedef union byte2
{
	unsigned char bit[2];
	unsigned char up[1];
	unsigned char low[1];
} byte2;
typedef union byte4
{
	unsigned char bit[4];
} byte4;

// 3�v�f�x�N�g��
typedef struct vec3
{
	float	x, y, z;
} vec3;

// 4�v�f�x�N�g��(�N�H�[�^�j�I�����p)
typedef struct vec4
{
	float	x, y, z, w;
} vec4;

// �e�N�X�`��UV
typedef struct tex
{
	float	u, v;
} tex;

// �A���t�@�����J���[
typedef struct color3
{
	float	r, g, b;
} color;

// �A���t�@����J���[
typedef struct color4
{
	float	r, g, b, a;
} color4;

// �s��
typedef float	matrix[4][4];



typedef struct mmd_head
{
	char magic[3];
	float ver;
	char name[20];
	char comment[256];
} mmd_head;

typedef struct mmd_vtx
{
	vec3 pos;
	vec3 nomal;
	tex uv;
	unsigned char bone[2];
	byte2 bone[2];
	unsigned char weight;
	unsigned char edge_flg;
}mmd_vtx;

typedef struct mat
{
	color4 diffuse;//�f�t���[�Y�F�����F
	float alpha;//�����F�̕s�����x
	color3	specular,//�X�y�L�����[
			ambient;//�A���r�G���g
	byte	toon_index,//unknown
		edge_flag; // �֊s�A�e �g��Ȃ�
	int32_t num;//���_��
	char texture_file_name[20];
};

typedef struct bone
{
	char	name[20];			// �{�[���� (0x00 �I�[�C�]���� 0xFD)
	uint16_t	nParentNo;			// �e�{�[���ԍ� (�Ȃ���� -1)
	uint16_t	nChildNo;			// �q�{�[���ԍ�

	unsigned char	kind;		// �{�[���̎��
	uint16_t	ik_target;	// IK���̃^�[�Q�b�g�{�[��

	vec3		pos;	// ���f�����_����̈ʒu
} bone;
/*
kind
0x00�@�ʏ�{�[��(��]�̂�)
0x01�@�ʏ�{�[��(�ړ��\)
0x02�@IK�{�[��(�ړ��\)
0x03�@�����Ȃ�(�I����)
0x04�@IK�{�[��(��]�̂�)
0x05�@��]�Ǐ](��]�̂�)
0x06�@IK�ڑ���
0x07�@�����Ȃ�(�I��s��)
0x08�@�Ђ˂�
0x09�@��]�^��
*/
typedef struct ik
{
	uint16_t			target;	// IK�^�[�Q�b�g�{�[���ԍ�
	uint16_t			index;		// IK��[�{�[���ԍ�

	byte	num;	// IK���\������{�[���̐�

	uint16_t	iterations;
	float			control_weight;

	unsigned short	ik_data[1];// IK���\������{�[���̔z��
}ik;

typedef struct face_vtx
{
	unsigned int	index;
	vec3 pos;
} face_vtx;

// �\��f�[�^
typedef struct face
{
	char			name[20];		// �\� (0x00 �I�[�C�]���� 0xFD)

	uint32_t	num;	// �\��_��
	unsigned char	type;			// ����

	face_vtx		data[1];	// �\��_�f�[�^
} face;

// ���̃f�[�^
typedef struct rigid_body
{
	char			szName[20];		// ���̖�

	unsigned short	unBoneIndex;	// �֘A�{�[���ԍ�
	unsigned char	cbColGroupIndex;// �Փ˃O���[�v
	unsigned short	unColGroupMask;	// �Փ˃O���[�v�}�X�N

	unsigned char	cbShapeType;	// �`��  0:�� 1:�� 2:�J�v�Z��

	float			fWidth;			// ���a(��)
	float			fHeight;		// ����
	float			fDepth;			// ���s

	vec3			vec3Position;	// �ʒu(�{�[������)
	vec3			vec3Rotation;	// ��](radian)

	float			fMass;			// ����
	float			fLinearDamping;	// �ړ���
	float			fAngularDamping;// ��]��
	float			fRestitution;	// ������
	float			fFriction;		// ���C��

	unsigned char	cbRigidBodyType;// �^�C�v 0:Bone�Ǐ] 1:�������Z 2:�������Z(Bone�ʒu����)
} rigid_body;

// �R���X�g���C���g(�W���C���g)�f�[�^
typedef struct constraint
{
	char			szName[20];		// �R���X�g���C���g��

	uint32_t	ulRigidA;		// ����A
	uint32_t	ulRigidB;		// ����B

	vec3			vec3Position;	// �ʒu(���f�����_���S)
	vec3			vec3Rotation;	// ��](radian)

	vec3			vec3PosLimitL;	// �ړ�����1
	vec3			vec3PosLimitU;	// �ړ�����2

	vec3			vec3RotLimitL;	// ��]����1
	vec3			vec3RotLimitU;	// ��]����2

	vec3			vec3SpringPos;	// �΂ˈړ�
	vec3			vec3SpringRot;	// �΂ˉ�]
} constraint;