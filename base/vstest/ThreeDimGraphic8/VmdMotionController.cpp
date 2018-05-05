#include "StdAfx.h"
#include "VmdMotionController.h"

VmdMotionController::VmdMotionController(LPCTSTR filename, vector<Bone>* b, vector<MmdStruct::PmdIkData>* p) : bones(b), pmdIkData(p), time() {
	const int frame_rate = 60;			// 本プログラムのフレームレート
	const int mmd_frame_rate = 30;		// MMDのフレームレート
	// VMDファイルからVMDデータを抽出
    ifstream ifs(filename, ios::binary);
	if (ifs.fail()) throw TEXT("ファイルがありません");
	MmdStruct::VmdHeader vmdHeader;
	ifs.read((char*)&vmdHeader, sizeof(MmdStruct::VmdHeader));
	unsigned long numVmdMotion;
	ifs.read((char*)&numVmdMotion, sizeof(numVmdMotion));
	vector<MmdStruct::VmdMotion> vmdMotions(numVmdMotion);
	ifs.read((char*)&vmdMotions[0], sizeof(MmdStruct::VmdMotion)*numVmdMotion);
	// KeyFramesに格納
	keyFrames.resize(bones->size());
	for (unsigned int i = 0; i < vmdMotions.size(); ++i) {
		KeyFrame keyFrame;
		keyFrame.boneName = vmdMotions[i].boneName;
		keyFrame.frameNo = vmdMotions[i].frameNo;
		keyFrame.frameNo *= frame_rate/mmd_frame_rate;
		keyFrame.position = D3DXVECTOR3(vmdMotions[i].location[0], vmdMotions[i].location[1], vmdMotions[i].location[2]);
		keyFrame.position *= MmdStruct::scale;
		keyFrame.rotation = D3DXQUATERNION(vmdMotions[i].rotation[0], vmdMotions[i].rotation[1], vmdMotions[i].rotation[2], vmdMotions[i].rotation[3]);
		keyFrame.interpolation_x[0] = D3DXVECTOR2(vmdMotions[i].interpolation[0],	vmdMotions[i].interpolation[4]);
		keyFrame.interpolation_x[1] = D3DXVECTOR2(vmdMotions[i].interpolation[8],	vmdMotions[i].interpolation[12]);
		keyFrame.interpolation_y[0] = D3DXVECTOR2(vmdMotions[i].interpolation[1],	vmdMotions[i].interpolation[5]);
		keyFrame.interpolation_y[1] = D3DXVECTOR2(vmdMotions[i].interpolation[9],	vmdMotions[i].interpolation[13]);
		keyFrame.interpolation_z[0] = D3DXVECTOR2(vmdMotions[i].interpolation[2],	vmdMotions[i].interpolation[6]);
		keyFrame.interpolation_z[1] = D3DXVECTOR2(vmdMotions[i].interpolation[10],	vmdMotions[i].interpolation[14]);
		keyFrame.interpolation_r[0] = D3DXVECTOR2(vmdMotions[i].interpolation[3],	vmdMotions[i].interpolation[7]);
		keyFrame.interpolation_r[1] = D3DXVECTOR2(vmdMotions[i].interpolation[11],	vmdMotions[i].interpolation[15]);
		for (unsigned int j = 0; j < bones->size(); ++j) {
			if (keyFrame.boneName == (*bones)[j].name) {	// ボーン名からボーン番号を探す
				keyFrames[j].push_back(keyFrame);
				break;
			}
		}
	}
	for (unsigned int i = 0; i < bones->size(); ++i) {
		keyFrames[i].sort();
		ite_keyFrames.push_back(keyFrames[i].begin());
		boneRot.push_back(D3DXQUATERNION(0, 0, 0, 0));
		bonePos.push_back(D3DXVECTOR3(0, 0, 0));
	}
	UpdateBoneMatrix();
}

void VmdMotionController::UpdateBoneMatrix() {
	for (unsigned int i = 0; i < bones->size(); i++) {
		// キーフレーム補完
		unsigned long t0, t1;
		D3DXQUATERNION q0, q1;
		D3DXVECTOR3 p0, p1;
		if (ite_keyFrames[i] != keyFrames[i].end()) {
			t0 = (*ite_keyFrames[i]).frameNo;
			boneRot[i] = q0 = (*ite_keyFrames[i]).rotation;
			bonePos[i] = p0 = (*ite_keyFrames[i]).position;
			if (++ite_keyFrames[i] != keyFrames[i].end()) {
				t1 = (*ite_keyFrames[i]).frameNo;
				q1 = (*ite_keyFrames[i]).rotation;
				p1 = (*ite_keyFrames[i]).position;
				float s = (float)(time - t0)/(float)(t1 - t0);	// 線形補完
				D3DXQuaternionSlerp(&boneRot[i], &q0, &q1, s);
				bonePos[i] = p0 + (p1 - p0)*s;
				if (time != t1) --ite_keyFrames[i];
			}
		}
		// 親ボーン座標系のボーン行列を求める
		D3DXMATRIX rot, trans;
		D3DXMatrixRotationQuaternion(&rot, &boneRot[i]);
		D3DXMatrixTranslation(&trans, bonePos[i].x, bonePos[i].y, bonePos[i].z);
		(*bones)[i].boneMat = rot*trans*(*bones)[i].initMat;		// IK影響下のボーンも含めてリセットする → 動きが不連続的になりやすい
		//if ((*bones)[i].type != 4) (*bones)[i].boneMat = rot*trans*(*bones)[i].initMat;	// IK影響下のボーンをリセットしない → 動きが連続的になるが時間と共にねじれていく
	}
	for (unsigned int i = 0; i < pmdIkData->size(); ++i) UpdateIK((*pmdIkData)[i]);
}

void VmdMotionController::UpdateIK(const MmdStruct::PmdIkData& ikData) {
	D3DXVECTOR3 localEffectorPos, localTargetPos;
	for (unsigned int j = 0; j < ikData.iterations; ++j) {
		for (unsigned int i = 0; i < ikData.ik_child_bone_index.size(); ++i) {
			unsigned short attentionIdx = ikData.ik_child_bone_index[i];
			D3DXVECTOR3 effectorPos = (*bones)[ikData.ik_target_bone_index].GetModelLocalPosition();		// エフェクタの位置
			D3DXVECTOR3 targetPos = (*bones)[ikData.ik_bone_index].GetModelLocalPosition();				// ターゲットの位置
			D3DXMATRIX invCoord;
			D3DXMatrixInverse(&invCoord, 0, &(*bones)[attentionIdx].GetModelLocalBoneMat());
			D3DXVec3TransformCoord(&localEffectorPos, &effectorPos, &invCoord);
			D3DXVec3TransformCoord(&localTargetPos, &targetPos, &invCoord);
			D3DXVECTOR3 localEffectorDir;															// エフェクタのローカル方向（注目ボーンの位置基準）
			D3DXVec3Normalize(&localEffectorDir, &localEffectorPos);
			D3DXVECTOR3 localTargetDir;																// ターゲットのローカル方向（注目ボーンの位置基準）
			D3DXVec3Normalize(&localTargetDir, &localTargetPos);
			if ((*bones)[attentionIdx].name.find("ひざ") != string::npos) {
				localEffectorDir = D3DXVECTOR3(0, localEffectorDir.y, localEffectorDir.z);
				D3DXVec3Normalize(&localEffectorDir, &localEffectorDir);
				localTargetDir = D3DXVECTOR3(0, localTargetDir.y, localTargetDir.z);
				D3DXVec3Normalize(&localTargetDir, &localTargetDir);
			}
			float p = D3DXVec3Dot(&localEffectorDir, &localTargetDir);
			if (p > 1 - 1.0e-5f) continue;	// 計算誤差により1を越えるとacos()が発散するので注意!
			float angle = acos(p);
			if (angle > 4*ikData.control_weight) angle = 4.0f*ikData.control_weight;
			D3DXVECTOR3 axis;
			D3DXVec3Cross(&axis, &localEffectorDir, &localTargetDir);
			D3DXMATRIX rotation;
			D3DXMatrixRotationAxis(&rotation, &axis, angle);
			if ((*bones)[attentionIdx].name.find("ひざ") != string::npos) {
				D3DXMATRIX inv;
				D3DXMatrixInverse(&inv, 0, &(*bones)[attentionIdx].initMat);
				D3DXMATRIX def = rotation*(*bones)[attentionIdx].boneMat*inv;
				D3DXVECTOR3 t(0, 0, 1);
				D3DXVec3TransformCoord(&t, &t, &def);
				if (t.y < 0) D3DXMatrixRotationAxis(&rotation, &axis, -2.0f*angle);
			}
			(*bones)[attentionIdx].boneMat = rotation*(*bones)[attentionIdx].boneMat;
		}
		const float errToleranceSq = 0.000001f;
		if (D3DXVec3LengthSq(&(localEffectorPos - localTargetPos)) < errToleranceSq) {
			return;
		}
	}
}

void VmdMotionController::AdvanceTime() { ++time; }

