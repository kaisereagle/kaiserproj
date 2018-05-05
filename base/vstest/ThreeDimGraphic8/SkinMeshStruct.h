#pragma once
///// スキンメッシュアニメーション用構造体 /////

///// 名称定義 ・・・ 本ヘッダ中では以下のようにボーンの座標系の名称を定義する	/////
///// ワールド座標系： 絶対座標系												/////
///// モデルローカル座標系：ワールド座標変換を含まない3Dモデルのローカル座標系	/////
///// ボーンローカル座標系：自身の親ボーンを基準にしたローカル座標系			/////

// ブレンド重み付メッシュの頂点データ
struct BlendVertex {
	D3DXVECTOR3 position;
	float weight[3];
	unsigned char matrixIndex[4];
	D3DXVECTOR3 normal;
	D3DXVECTOR2 texture;
};

// ボーン
struct Bone {
	int id;					// ボーンID（通し番号）
	string name;			// ボーン名
	unsigned char type;		// ボーンタイプ (MMDの場合 0:回転 1:回転と移動 2:IK 3:不明 4:IK影響下 5:回転影響下 6:IK接続先 7:非表示 8:捻り 9:回転運動 )
	Bone* parent;			// 親ボーン
	Bone* firstChild;		// 第1子ボーン
	Bone* sibling;			// 次の兄弟ボーン
	D3DXMATRIX initMat;		// 初期姿勢行列(ボーンローカル座標系)
	D3DXMATRIX boneMat;		// ボーン行列(ボーンローカル座標系)
	D3DXMATRIX offsetMat;	// ボーンオフセット行列(モデルローカル座標系)
	Bone() : type(), id(), parent(), firstChild(), sibling() {	// コンストラクタ
		D3DXMatrixIdentity(&initMat);
		D3DXMatrixIdentity(&offsetMat);
		D3DXMatrixIdentity(&boneMat);
	};
	D3DXMATRIX GetModelLocalBoneMat() {				// モデルローカル座標系でのボーン行列を取得
		if (parent) return boneMat*parent->GetModelLocalBoneMat();
		else return boneMat;
	}
	D3DXVECTOR3 Bone::GetModelLocalPosition() {		// モデルローカル座標系でのボーン位置を取得
		D3DXVECTOR3 v(0, 0, 0);
		return *D3DXVec3TransformCoord(&v, &v, &GetModelLocalBoneMat());
	}
};

// キーフレームアニメーションデータ
struct KeyFrame {
	string boneName;		// ボーン名
	unsigned long frameNo;	// フレーム番号
	D3DXVECTOR3 position;	// 位置
	D3DXQUATERNION rotation;// 回転
	D3DXVECTOR2 interpolation_x[2];	// position.xのベジェ補完パラメータ
	D3DXVECTOR2 interpolation_y[2];	// position.yのベジェ補完パラメータ
	D3DXVECTOR2 interpolation_z[2];	// position.zのベジェ補完パラメータ
	D3DXVECTOR2 interpolation_r[2];	// rotationのベジェ補完パラメータ
	bool operator < (const KeyFrame &k) const {	// フレーム番号でソートするための比較演算子
		return frameNo < k.frameNo;
	}
};