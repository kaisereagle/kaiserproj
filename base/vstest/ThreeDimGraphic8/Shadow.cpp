#include "StdAfx.h"
#include "Shadow.h"

Shadow::Shadow(LPDIRECT3DDEVICE9 pDevice, LPD3DXMESH pMesh, int NumMaterial) : pDevice(pDevice), pMesh(pMesh), NumMaterial(NumMaterial) {
	ZeroMemory(&shadow_material, sizeof(D3DMATERIAL9));
	shadow_material.Diffuse.a = 0.5f;
}

void Shadow::StencileEnable() {
	pDevice->SetRenderState(D3DRS_STENCILENABLE, TRUE);
	pDevice->SetRenderState(D3DRS_STENCILFUNC, D3DCMP_NOTEQUAL);
	pDevice->SetRenderState(D3DRS_STENCILFAIL, D3DSTENCILOP_KEEP);
	pDevice->SetRenderState(D3DRS_STENCILZFAIL, D3DSTENCILOP_KEEP);
	pDevice->SetRenderState(D3DRS_STENCILPASS, D3DSTENCILOP_REPLACE);
	pDevice->SetRenderState(D3DRS_STENCILREF, 1);
}

void Shadow::StencileDisable() { pDevice->SetRenderState(D3DRS_STENCILENABLE, FALSE); }

void Shadow::SetPlane(D3DXPLANE* p) { plane = *p; }

void Shadow::Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera) {
	D3DXMATRIX shadow, trans, view, projection;
	D3DXMatrixShadow(&shadow, &D3DXVECTOR4(light->Direction, 0), &plane);			// w = 0 ==> directional light, w = 1 ==> point light
	D3DXMatrixTranslation(&trans, position->x, position->y, position->z);
	shadow = (*rotation)*trans*shadow;
	camera->GetMatrix(&view, &projection);
    pDevice->SetTransform(D3DTS_WORLD, &shadow);
	pDevice->SetTransform(D3DTS_VIEW, &view);
	pDevice->SetTransform(D3DTS_PROJECTION, &projection);
	pDevice->SetMaterial(&shadow_material);
	StencileEnable();
	for (int i = 0; i < NumMaterial; ++i) pMesh->DrawSubset(i);
	StencileDisable();
}

///// 影用フレーム描画用関数群 /////
void FrameRenderingForShadow::RenderMeshContainer(LPDIRECT3DDEVICE9 pDevice, MYMESHCONTAINER* pMeshContainer, MYFRAME* pFrame) {
	if (pMeshContainer->pSkinInfo) {		// スキンメッシュの場合
		LPD3DXBONECOMBINATION pBoneCombination = (LPD3DXBONECOMBINATION)pMeshContainer->pBoneBuffer->GetBufferPointer();
		for (unsigned long i = 0; i < pMeshContainer->numBoneCombination; ++i) {
			unsigned long numBlendMatrix = 0;
			for (unsigned long k = 0; k < pMeshContainer->maxFaceInfl; ++k) if (pBoneCombination[i].BoneId[k] != UINT_MAX) numBlendMatrix = k + 1;
			pDevice->SetRenderState(D3DRS_VERTEXBLEND, numBlendMatrix - 1);
			for (unsigned long k = 0; k < pMeshContainer->maxFaceInfl; ++k) {
				unsigned int boneId = pBoneCombination[i].BoneId[k];
				if (boneId != UINT_MAX) pDevice->SetTransform(D3DTS_WORLDMATRIX(k), &(pMeshContainer->pBoneOffsetMatrices[boneId]*(*pMeshContainer->ppBoneMatrix[boneId])));
			}
			pMeshContainer->MeshData.pMesh->DrawSubset(i);
			pDevice->SetRenderState(D3DRS_VERTEXBLEND, D3DVBF_DISABLE);
		}
	}
	else {									// 通常メッシュの場合
		pDevice->SetTransform(D3DTS_WORLD, &pFrame->CombinedTransformationMatrix);
		for (unsigned long i = 0; i < pMeshContainer->NumMaterials; ++i) pMeshContainer->MeshData.pMesh->DrawSubset(i);
	}
}


///// XFileアニメーションシャドウ /////
XFileAnimationShadow::XFileAnimationShadow(LPDIRECT3DDEVICE9 pDevice, LPD3DXFRAME pFrame) : Shadow(pDevice, 0, 0), pFrame(pFrame) {
}

void XFileAnimationShadow::Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera) {
	D3DXMATRIX shadow, trans, view, proj;
	D3DXMatrixShadow(&shadow, &D3DXVECTOR4(light->Direction, 0), &plane);			// w = 0 ==> directional light, w = 1 ==> point light
	D3DXMatrixTranslation(&trans, position->x, position->y, position->z);
	shadow = (*rotation)*trans*shadow;
	pDevice->SetMaterial(&shadow_material);
	camera->GetMatrix(&view, &proj);
	pDevice->SetTransform(D3DTS_VIEW, &view);
	pDevice->SetTransform(D3DTS_PROJECTION, &proj);
	frameRenderingForShadow.UpdateMatrices(pFrame, &shadow);
	StencileEnable();
	frameRenderingForShadow.Draw(pDevice, pFrame);
	StencileDisable();
}


///// シャドウボリューム /////
ShadowVolume::ShadowVolume(LPDIRECT3DDEVICE9 pDevice, LPD3DXMESH pMesh) : pDevice(pDevice), pMesh(pMesh) {
}

void ShadowVolume::CreateVolume(D3DXVECTOR3 direction) {
    byte* vertexBuffer;
    pMesh->LockVertexBuffer(0, (void**)&vertexBuffer);		// 頂点バッファ
	unsigned short *indexBuffer;
    pMesh->LockIndexBuffer(0, (void**)&indexBuffer);		// インデックスバッファ
	unsigned long stride = pMesh->GetNumBytesPerVertex();	// ストライド
	unsigned long numPoly = pMesh->GetNumFaces();			// 全ポリゴン数
	unsigned long numEdge = 0;								// 抽出されたポリゴンのエッジ数
	unsigned short *edge = new unsigned short[numPoly*6];	// 抽出されたポリゴン
	D3DXVECTOR3 v[5];										// 頂点データ仮置き場
  	// 全ポリゴンについてライトベクトルに対して裏となるポリゴンを抽出
    for (unsigned long i = 0; i < numPoly; ++i) {
		for (int j = 0; j < 3; ++j) v[j] = *((D3DXVECTOR3*)(vertexBuffer + stride*indexBuffer[3*i + j]));
		D3DXVECTOR3 vNormal;
        D3DXVec3Cross(&vNormal, &D3DXVECTOR3(v[2] - v[1]), &D3DXVECTOR3(v[1] - v[0]));
		const int idx[6] = {0, 1, 1, 2, 2, 0};
        if (D3DXVec3Dot(&vNormal, &direction) >= 0 ) {
			for (int j = 0; j < 6; ++j) edge[2*numEdge + j] = indexBuffer[3*i + idx[j]];
			numEdge += 3; 
        }
    }
	// 抽出されたポリゴンを引き伸ばす
	if (6*numEdge > sizeof(vertex)/sizeof(D3DXVECTOR3)) throw TEXT("Can't create shadow volume");
	numVertex = 0;
	const float length = 20.0f;	// シャドウボリュームの長さ
    for (unsigned long i = 0; i < numEdge; ++i) {
        v[1] = *((D3DXVECTOR3*)(vertexBuffer + stride*edge[2*i]));
        v[2] = *((D3DXVECTOR3*)(vertexBuffer + stride*edge[2*i + 1]));
        v[3] = direction*length - v[1];
        v[4] = direction*length - v[2];
		int idx[6] = {1, 2, 3, 2, 4, 3};
		for (int j = 0; j < 6; ++j) vertex[numVertex + j] = v[idx[j]];
		numVertex += 6;
    }
    pMesh->UnlockVertexBuffer();
    pMesh->UnlockIndexBuffer();
	delete edge;
}

void ShadowVolume::RenderVolume(D3DXMATRIX world) {
	struct COLOR_POINT {
		D3DXVECTOR3 vecCoord;
		DWORD dwColor;
	};
	COLOR_POINT* color_point = new COLOR_POINT[numVertex];
	for (unsigned int i = 0; i < numVertex; ++i) {
		color_point[i].vecCoord = vertex[i];
		color_point[i].dwColor = D3DCOLOR_ARGB(0x3F, 0x3F, 0xFF, 0x3F);
	}
	pDevice->SetRenderState(D3DRS_LIGHTING, FALSE);
	pDevice->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
	pDevice->SetTransform(D3DTS_WORLD, &world);
	pDevice->SetFVF(D3DFVF_XYZ| D3DFVF_DIFFUSE);
    pDevice->DrawPrimitiveUP(D3DPT_TRIANGLELIST, numVertex/3, color_point, sizeof(COLOR_POINT));
	pDevice->SetRenderState(D3DRS_CULLMODE, D3DCULL_CCW);
	pDevice->SetRenderState(D3DRS_LIGHTING, TRUE);
	delete[] color_point;
}

void ShadowVolume::RenderVolumeToStencil(D3DXMATRIX world) {
	// ステンシルバッファ書き込みモードにセット
	pDevice->Clear(0, NULL, D3DCLEAR_STENCIL, 0, 0, 0);	
    pDevice->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
    pDevice->SetRenderState(D3DRS_STENCILENABLE,TRUE);
    pDevice->SetRenderState(D3DRS_SHADEMODE,D3DSHADE_FLAT);
    pDevice->SetRenderState(D3DRS_STENCILFUNC, D3DCMP_ALWAYS);
    pDevice->SetRenderState(D3DRS_STENCILZFAIL,D3DSTENCILOP_KEEP);
    pDevice->SetRenderState(D3DRS_STENCILFAIL, D3DSTENCILOP_KEEP);
    pDevice->SetRenderState(D3DRS_STENCILREF,  0x1);
    pDevice->SetRenderState(D3DRS_STENCILMASK,  0xffffffff);
    pDevice->SetRenderState(D3DRS_STENCILWRITEMASK, 0xffffffff);
    pDevice->SetRenderState(D3DRS_STENCILPASS, D3DSTENCILOP_INCR);
    pDevice->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    pDevice->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_ZERO);
    pDevice->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_ONE);
	pDevice->SetRenderState(D3DRS_TWOSIDEDSTENCILMODE, TRUE);
	pDevice->SetRenderState(D3DRS_CULLMODE,  D3DCULL_NONE);
	pDevice->SetRenderState(D3DRS_CCW_STENCILFUNC,  D3DCMP_ALWAYS);
	pDevice->SetRenderState(D3DRS_CCW_STENCILZFAIL, D3DSTENCILOP_KEEP);
	pDevice->SetRenderState(D3DRS_CCW_STENCILFAIL,  D3DSTENCILOP_KEEP);
	pDevice->SetRenderState(D3DRS_CCW_STENCILPASS, D3DSTENCILOP_DECR);
	// ステンシルバッファ書き込み
	pDevice->SetTransform(D3DTS_WORLD, &world);
	pDevice->SetFVF(D3DFVF_XYZ);
    pDevice->DrawPrimitiveUP(D3DPT_TRIANGLELIST, numVertex/3, vertex, sizeof(D3DXVECTOR3));
}

void ShadowVolume::RenderShadowBoard() {
	// 板を生成
	struct COLOR_POINT {
		D3DXVECTOR4 vecCoord;
		DWORD dwColor;
	} board[4];
	float w = (float)WINDOW_WIDTH, h = (float)WINDOW_HEIGHT;
	board[0].vecCoord = D3DXVECTOR4(0, 0, 0, 1);
	board[1].vecCoord = D3DXVECTOR4(w, 0, 0, 1);
	board[2].vecCoord = D3DXVECTOR4(0, h, 0, 1);
	board[3].vecCoord = D3DXVECTOR4(w, h, 0, 1);
	for (int i = 0; i < 4; ++i) board[i].dwColor = D3DCOLOR_ARGB(100, 10, 10, 10);
	// ステンシルバッファ使用モードにセット
	pDevice->SetRenderState(D3DRS_STENCILREF, 0x1);
    pDevice->SetRenderState(D3DRS_STENCILFUNC, D3DCMP_LESSEQUAL);
    pDevice->SetRenderState(D3DRS_STENCILPASS, D3DSTENCILOP_KEEP);
    pDevice->SetRenderState(D3DRS_ZENABLE,FALSE);
    pDevice->SetRenderState(D3DRS_STENCILENABLE,TRUE);
    pDevice->SetRenderState(D3DRS_ALPHABLENDENABLE,TRUE);
    pDevice->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
    pDevice->SetRenderState(D3DRS_DESTBLEND,D3DBLEND_INVSRCALPHA);
	// ステンシルバッファで くりぬいて描画
	pDevice->SetFVF(D3DFVF_XYZRHW | D3DFVF_DIFFUSE);
	pDevice->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, board, sizeof(COLOR_POINT));
	// ステンシルバッファ使用モードを解除
	pDevice->SetRenderState(D3DRS_TWOSIDEDSTENCILMODE, FALSE);
    pDevice->SetRenderState(D3DRS_SHADEMODE, D3DSHADE_GOURAUD);
    pDevice->SetRenderState(D3DRS_CULLMODE,  D3DCULL_CCW);
    pDevice->SetRenderState(D3DRS_ZWRITEENABLE,  TRUE);
    pDevice->SetRenderState(D3DRS_STENCILENABLE, FALSE);
	pDevice->SetRenderState(D3DRS_ZENABLE,TRUE);
}

D3DXMATRIX ShadowVolume::GetWorldMatrix(D3DXVECTOR3* position, D3DXMATRIX* rotation) {
	D3DXMATRIX trans;
	D3DXMatrixTranslation(&trans, position->x, position->y, position->z);
	return (*rotation)*trans;
}

D3DXVECTOR3 ShadowVolume::GetLocalDirection(D3DXMATRIX* rotation, D3DXVECTOR3* direction) {
	D3DXMATRIX invRotation;
	D3DXVECTOR3 local_direction;
	D3DXMatrixInverse(&invRotation, 0, rotation);
	D3DXVec3TransformNormal(&local_direction, direction, &invRotation);
	return local_direction;
}

void ShadowVolume::Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera) {
	CreateVolume(GetLocalDirection(rotation, (D3DXVECTOR3*)&light->Direction));							
	RenderVolumeToStencil(GetWorldMatrix(position, rotation));
	RenderShadowBoard();
}

void ShadowVolume::DrawVolume(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light) {
	CreateVolume(GetLocalDirection(rotation, (D3DXVECTOR3*)&light->Direction));							
	RenderVolume(GetWorldMatrix(position, rotation));
}