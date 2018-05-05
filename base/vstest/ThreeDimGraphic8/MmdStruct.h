#pragma once


/// MMD専用構造体定義
namespace MmdStruct {
#pragma pack(push,1)	//アラインメント制御をオフ

	/// PMD構造体定義
	struct PmdHeader {
		unsigned char magic[3];
		float version;
		unsigned char model_name[20];
		unsigned char comment[256];
	};
	struct PmdVertex{
		float pos[3];
		float normal_vec[3];
		float uv[2];
		unsigned short bone_num[2];
		unsigned char bone_weight;
		unsigned char edge_flag;
	};
	struct PmdMaterial{
		float diffuse_color[3];
		float alpha;
		float specularity;
		float specular_color[3];
		float mirror_color[3];
		unsigned char toon_index;
		unsigned char edge_flag;
		unsigned long face_vert_count;	// この材料の面頂点数 → 材料番号iのポリゴン番号： pmdMaterial[i - 1].face_vert_count/3 ～ pmdMaterial[i].face_vert_count/3 - 1
		char texture_file_name[20];
	};
	struct PmdBone {
		char bone_name[20];
		unsigned short parent_bone_index;	// ない場合は0xFFFF
		unsigned short tail_pos_bone_index;	// ない場合は0xFFFF
		unsigned char bone_type;			// 0:回転 1:回転と移動 2:IK 3:不明 4:IK影響下 5:回転影響下 6:IK接続先 7:非表示 8:捻り 9:回転運動 (8, 9はMMD4.0以降)
		unsigned short ik_parent_bone_index;// ない場合は0
		float bone_head_pos[3];
	};
	struct PmdIkDataWithoutArray {
		unsigned short ik_bone_index;			// IKボーン番号
		unsigned short ik_target_bone_index;	// IKボーンの位置にこのボーンを一致させるようにIK処理が行われる
		unsigned char ik_chain_length;			// IKチェーンの長さ
		unsigned short iterations;				// 再帰演算回数
		float control_weight;					// ボーンの単位制限角 1.0 → 4.0[rad]。また「ひざ」を含むボーン名をもつボーンはX軸方向にしか動かない制限がある。
		// unsigned short ik_child_bone_index[ik_chain_length]; // IK影響下のボーン番号 (配列の大きさが変化)
	};
	struct PmdIkData : public PmdIkDataWithoutArray {
		vector<unsigned short> ik_child_bone_index;
	};


	/// VMD構造体定義
	struct VmdHeader {
		char vmdHeader[30];
		char vmdModelName[20];
	};

	struct VmdMotion {
		char boneName[15];
		unsigned long frameNo;
		float location[3];	// 移動量
		float rotation[4];	// モデルローカル座標系
		unsigned char interpolation[64];
	};

	/// スケール変換定数
	const float scale = 0.1f;

#pragma pack(pop)
}