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

// 3要素ベクトル
typedef struct vec3
{
	float	x, y, z;
} vec3;

// 4要素ベクトル(クォータニオン兼用)
typedef struct vec4
{
	float	x, y, z, w;
} vec4;

// テクスチャUV
typedef struct tex
{
	float	u, v;
} tex;

// アルファ無しカラー
typedef struct color3
{
	float	r, g, b;
} color;

// アルファありカラー
typedef struct color4
{
	float	r, g, b, a;
} color4;

// 行列
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
	color4 diffuse;//デフューズ：減衰色
	float alpha;//減衰色の不透明度
	color3	specular,//スペキュラー
			ambient;//アンビエント
	byte	toon_index,//unknown
		edge_flag; // 輪郭、影 使わない
	int32_t num;//頂点数
	char texture_file_name[20];
};

typedef struct bone
{
	char	name[20];			// ボーン名 (0x00 終端，余白は 0xFD)
	uint16_t	nParentNo;			// 親ボーン番号 (なければ -1)
	uint16_t	nChildNo;			// 子ボーン番号

	unsigned char	kind;		// ボーンの種類
	uint16_t	ik_target;	// IK時のターゲットボーン

	vec3		pos;	// モデル原点からの位置
} bone;
/*
kind
0x00　通常ボーン(回転のみ)
0x01　通常ボーン(移動可能)
0x02　IKボーン(移動可能)
0x03　見えない(選択可)
0x04　IKボーン(回転のみ)
0x05　回転追従(回転のみ)
0x06　IK接続先
0x07　見えない(選択不可)
0x08　ひねり
0x09　回転運動
*/
typedef struct ik
{
	uint16_t			target;	// IKターゲットボーン番号
	uint16_t			index;		// IK先端ボーン番号

	byte	num;	// IKを構成するボーンの数

	uint16_t	iterations;
	float			control_weight;

	unsigned short	ik_data[1];// IKを構成するボーンの配列
}ik;

typedef struct face_vtx
{
	unsigned int	index;
	vec3 pos;
} face_vtx;

// 表情データ
typedef struct face
{
	char			name[20];		// 表情名 (0x00 終端，余白は 0xFD)

	uint32_t	num;	// 表情頂点数
	unsigned char	type;			// 分類

	face_vtx		data[1];	// 表情頂点データ
} face;

// 剛体データ
typedef struct rigid_body
{
	char			szName[20];		// 剛体名

	unsigned short	unBoneIndex;	// 関連ボーン番号
	unsigned char	cbColGroupIndex;// 衝突グループ
	unsigned short	unColGroupMask;	// 衝突グループマスク

	unsigned char	cbShapeType;	// 形状  0:球 1:箱 2:カプセル

	float			fWidth;			// 半径(幅)
	float			fHeight;		// 高さ
	float			fDepth;			// 奥行

	vec3			vec3Position;	// 位置(ボーン相対)
	vec3			vec3Rotation;	// 回転(radian)

	float			fMass;			// 質量
	float			fLinearDamping;	// 移動減
	float			fAngularDamping;// 回転減
	float			fRestitution;	// 反発力
	float			fFriction;		// 摩擦力

	unsigned char	cbRigidBodyType;// タイプ 0:Bone追従 1:物理演算 2:物理演算(Bone位置合せ)
} rigid_body;

// コンストレイント(ジョイント)データ
typedef struct constraint
{
	char			szName[20];		// コンストレイント名

	uint32_t	ulRigidA;		// 剛体A
	uint32_t	ulRigidB;		// 剛体B

	vec3			vec3Position;	// 位置(モデル原点中心)
	vec3			vec3Rotation;	// 回転(radian)

	vec3			vec3PosLimitL;	// 移動制限1
	vec3			vec3PosLimitU;	// 移動制限2

	vec3			vec3RotLimitL;	// 回転制限1
	vec3			vec3RotLimitU;	// 回転制限2

	vec3			vec3SpringPos;	// ばね移動
	vec3			vec3SpringRot;	// ばね回転
} constraint;