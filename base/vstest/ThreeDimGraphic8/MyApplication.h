#pragma once
#include "DirectXFramework.h"
#include "CollisionDetection.h"
#include "XFileAnimationMesh.h"
#include "SkinMesh.h"
#include "Shadow.h"
#include "Coord.h"

/// �A�v���P�[�V�����̓����N���X
class MyApplication {
private:
	DirectXFramework* directXFramework;
	SkinMesh* skinMesh;
	Coord* coord;

public:
	/// �R���X�g���N�^
	MyApplication(HWND hWnd);

	/// �f�X�g���N�^
	~MyApplication();

	/// 1/60�b���ɍs�Ȃ�����
	void Run();
};

