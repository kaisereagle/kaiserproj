#pragma once

/// 衝突判定
class CollisionDetection {
private:
	/// ポリゴン番号から頂点を取得
	static void GetVertex(LPD3DXMESH pMesh, unsigned long index, D3DXVECTOR3* vertex);

	/// ローカル座標でレイとメッシュの交点を取得
	/// 引数はLayToMesh()参照
	static bool GetIntersectionPoint(D3DXVECTOR3 rayStart, D3DXVECTOR3 rayDirection, LPD3DXMESH pMesh, D3DXVECTOR3* intersectionPoint, D3DXVECTOR3* normalVector);

public:
	/// レイとメッシュの交差
	/// @return					交差したか否か
	/// @param rayStart			レイの開始地点
	/// @param rayDirection		レイの方向
	/// @param pMesh			メッシュ
	/// @param meshPosition		メッシュの位置
	/// @param meshRotation		メッシュの回転
	/// @param intesectionPoint	交差点を入れるアドレス (0のとき無効)
	/// @param normalVector		交差点の法線ベクトルを入れるアドレス (0のとき無効)
	static bool LayToMesh(D3DXVECTOR3 rayStart, D3DXVECTOR3 rayDirection, LPD3DXMESH pMesh, D3DXVECTOR3 meshPosition, D3DXMATRIX meshRotation, D3DXVECTOR3* intersectionPoint = 0, D3DXVECTOR3* normalVector = 0);
};