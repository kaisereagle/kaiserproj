#pragma once
#include "DirectXFramework.h"
#include "CollisionDetection.h"
#include "XFileAnimationMesh.h"
#include "SkinMesh.h"
#include "Shadow.h"
#include "Coord.h"

/// アプリケーションの統括クラス
class MyApplication {
private:
	DirectXFramework* directXFramework;
	SkinMesh* skinMesh;
	Coord* coord;

public:
	/// コンストラクタ
	MyApplication(HWND hWnd);

	/// デストラクタ
	~MyApplication();

	/// 1/60秒毎に行なう処理
	void Run();
};

