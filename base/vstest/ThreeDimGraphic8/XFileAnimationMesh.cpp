#include "StdAfx.h"
#include "XFileAnimationMesh.h"

///// フレーム描画用関数群 /////
void FrameRendering::RenderMeshContainer(LPDIRECT3DDEVICE9 pDevice, MYMESHCONTAINER* pMeshContainer, MYFRAME* pFrame) {
	if (pMeshContainer->pSkinInfo) {		// スキンメッシュの場合
		LPD3DXBONECOMBINATION pBoneCombination = (LPD3DXBONECOMBINATION)pMeshContainer->pBoneBuffer->GetBufferPointer();
		for (unsigned long i = 0; i < pMeshContainer->numBoneCombination; ++i) {
			unsigned long numBlendMatrix = 0;
			for (unsigned long k = 0; k < pMeshContainer->maxFaceInfl; ++k) if (pBoneCombination[i].BoneId[k] != UINT_MAX) numBlendMatrix = k + 1;
			numBlendMatrix = min(numBlendMatrix, 4);	// うちのグラボの最大ブレンド行列数は4なのでそれ以上はあきらめる
			pDevice->SetRenderState(D3DRS_VERTEXBLEND, numBlendMatrix - 1);
			for (unsigned long k = 0; k < numBlendMatrix; ++k) {
				unsigned int boneId = pBoneCombination[i].BoneId[k];
				if (boneId != UINT_MAX) pDevice->SetTransform(D3DTS_WORLDMATRIX(k), &(pMeshContainer->pBoneOffsetMatrices[boneId]*(*pMeshContainer->ppBoneMatrix[boneId])));
			}
			pDevice->SetMaterial(&pMeshContainer->pMaterials[pBoneCombination[i].AttribId].MatD3D);
			pDevice->SetTexture(0, pMeshContainer->ppTextures[pBoneCombination[i].AttribId]);
			pMeshContainer->MeshData.pMesh->DrawSubset(i);
			pDevice->SetRenderState(D3DRS_VERTEXBLEND, D3DVBF_DISABLE);
		}
	}
	else {									// 通常メッシュの場合
		pDevice->SetTransform(D3DTS_WORLD, &pFrame->CombinedTransformationMatrix);
		for (unsigned long i = 0; i < pMeshContainer->NumMaterials; ++i) {
			pDevice->SetMaterial(&pMeshContainer->pMaterials[i].MatD3D);
			pDevice->SetTexture(0, pMeshContainer->ppTextures[i]);
			pMeshContainer->MeshData.pMesh->DrawSubset(i);
		}
	}
}

void FrameRendering::UpdateMatrices(LPD3DXFRAME pFrameBase, LPD3DXMATRIX pParentMatrix) {
    MYFRAME *pFrame = (MYFRAME*)pFrameBase;	
    if (pParentMatrix) D3DXMatrixMultiply(&pFrame->CombinedTransformationMatrix, &pFrame->TransformationMatrix, pParentMatrix);
    else pFrame->CombinedTransformationMatrix = pFrame->TransformationMatrix;
    if (pFrame->pFrameSibling) UpdateMatrices(pFrame->pFrameSibling, pParentMatrix);
    if (pFrame->pFrameFirstChild) UpdateMatrices(pFrame->pFrameFirstChild, &pFrame->CombinedTransformationMatrix);
}

void FrameRendering::Draw(LPDIRECT3DDEVICE9 pDevice,LPD3DXFRAME pFrameBase) {
	MYFRAME* pFrame = (MYFRAME*)pFrameBase;
    MYMESHCONTAINER* pMeshContainer = (MYMESHCONTAINER*)pFrame->pMeshContainer;	
    while (pMeshContainer) {
        RenderMeshContainer(pDevice, pMeshContainer, pFrame);
        pMeshContainer = (MYMESHCONTAINER*)pMeshContainer->pNextMeshContainer;
    }
    if (pFrame->pFrameSibling) Draw(pDevice,pFrame->pFrameSibling);
    if (pFrame->pFrameFirstChild) Draw(pDevice,pFrame->pFrameFirstChild);
}


///// 階層構造生成消去関数群 /////
HRESULT MyHierarchy::CreateFrame(LPCSTR Name, LPD3DXFRAME *ppNewFrame) {
    MYFRAME* pFrame = new MYFRAME;
	pFrame->Name = (LPSTR)(new TCHAR[strlen(Name) + 1]);	
	strcpy_s(pFrame->Name, strlen(Name) + 1, Name);
    D3DXMatrixIdentity(&pFrame->TransformationMatrix);
    D3DXMatrixIdentity(&pFrame->CombinedTransformationMatrix);
    pFrame->pMeshContainer = 0;
    pFrame->pFrameSibling = 0;
    pFrame->pFrameFirstChild = 0;
    *ppNewFrame = pFrame;
    return S_OK;
}

void MyHierarchy::CreateTexture(MYMESHCONTAINER* pMeshContainer, LPDIRECT3DDEVICE9 pDevice, DWORD NumMaterials) {
	for (DWORD iMaterial = 0; iMaterial < NumMaterials; ++iMaterial) {
		if (pMeshContainer->pMaterials[iMaterial].pTextureFilename) {
			// テクスチャファイル名のUnicode対応
			TCHAR strTexturePath[MAX_PATH] = {0};
#ifdef UNICODE
			MultiByteToWideChar(CP_OEMCP, MB_PRECOMPOSED, pMeshContainer->pMaterials[iMaterial].pTextureFilename, strlen(pMeshContainer->pMaterials[iMaterial].pTextureFilename), strTexturePath, (sizeof strTexturePath)/2);
#else
			strcpy_s(strTexturePath, pMeshContainer->pMaterials[iMaterial].pTextureFilename);
#endif
			if(FAILED( D3DXCreateTextureFromFile(pDevice, strTexturePath, &pMeshContainer->ppTextures[iMaterial]))) pMeshContainer->ppTextures[iMaterial] = NULL;
			pMeshContainer->pMaterials[iMaterial].pTextureFilename = NULL;
		}
	}
}

void MyHierarchy::SetDefaultMaterial(MYMESHCONTAINER* pMeshContainer) {
	pMeshContainer->pMaterials[0].pTextureFilename = NULL;
	memset(&pMeshContainer->pMaterials[0].MatD3D, 0, sizeof(D3DMATERIAL9));
	pMeshContainer->pMaterials[0].MatD3D.Diffuse.r = 0.5f;
	pMeshContainer->pMaterials[0].MatD3D.Diffuse.g = 0.5f;
	pMeshContainer->pMaterials[0].MatD3D.Diffuse.b = 0.5f;
	pMeshContainer->pMaterials[0].MatD3D.Specular = pMeshContainer->pMaterials[0].MatD3D.Diffuse;
}

HRESULT MyHierarchy::CreateMeshContainer(LPCSTR Name, CONST D3DXMESHDATA* pMeshData, CONST D3DXMATERIAL* pMaterials, CONST D3DXEFFECTINSTANCE* pEffectInstances,
										 DWORD NumMaterials, CONST DWORD *pAdjacency, LPD3DXSKININFO pSkinInfo, LPD3DXMESHCONTAINER *ppMeshContainer) {
    LPD3DXMESH pMesh = pMeshData->pMesh;
    INT iFacesAmount = pMesh->GetNumFaces();  
    MYMESHCONTAINER* pMeshContainer = new MYMESHCONTAINER;
    ZeroMemory(pMeshContainer, sizeof(MYMESHCONTAINER));
	pMeshContainer->Name = (LPSTR)(new TCHAR[strlen(Name) + 1]);	
	strcpy_s((char*)pMeshContainer->Name, strlen(Name) + 1, Name);
	pMeshContainer->MeshData.pMesh = pMesh;
	pMeshContainer->MeshData.Type = D3DXMESHTYPE_MESH;
    pMeshContainer->NumMaterials = max(1, NumMaterials);
    pMeshContainer->pMaterials = new D3DXMATERIAL[pMeshContainer->NumMaterials];
    pMeshContainer->ppTextures = new LPDIRECT3DTEXTURE9[pMeshContainer->NumMaterials];
    pMeshContainer->pAdjacency = new DWORD[iFacesAmount*3];
    memcpy(pMeshContainer->pAdjacency, pAdjacency, sizeof(DWORD)*iFacesAmount*3);
    memset(pMeshContainer->ppTextures, 0, sizeof(LPDIRECT3DTEXTURE9)*pMeshContainer->NumMaterials);
    if (NumMaterials > 0) {
	    LPDIRECT3DDEVICE9 pDevice = 0;
		pMesh->GetDevice(&pDevice);
        memcpy(pMeshContainer->pMaterials, pMaterials, sizeof(D3DXMATERIAL)*NumMaterials);
		CreateTexture(pMeshContainer, pDevice, NumMaterials);
		SAFE_RELEASE(pDevice);
    }
    else SetDefaultMaterial(pMeshContainer);
	if (pSkinInfo) {			//当該メッシュがスキン情報を持っている場合（スキンメッシュ固有の処理）
		pMeshContainer->pSkinInfo = pSkinInfo;
        pSkinInfo->AddRef();
		DWORD numBone = pSkinInfo->GetNumBones();
		pMeshContainer->pBoneOffsetMatrices = new D3DXMATRIX[numBone];
		for (DWORD i = 0; i < numBone; ++i) memcpy(&pMeshContainer->pBoneOffsetMatrices[i], pMeshContainer->pSkinInfo->GetBoneOffsetMatrix(i), sizeof(D3DMATRIX));
		if(FAILED(pMeshContainer->pSkinInfo->ConvertToBlendedMesh(pMesh, NULL, pMeshContainer->pAdjacency, NULL, NULL, NULL, 
			&pMeshContainer->maxFaceInfl, &pMeshContainer->numBoneCombination, &pMeshContainer->pBoneBuffer, &pMeshContainer->MeshData.pMesh))) return E_FAIL;
	}
	*ppMeshContainer = pMeshContainer;
    return S_OK;
}

HRESULT MyHierarchy::DestroyFrame(LPD3DXFRAME pFrameToFree) {
    SAFE_DELETE_ARRAY(pFrameToFree->Name);
	SAFE_DELETE(pFrameToFree);
    return S_OK; 
}

HRESULT MyHierarchy::DestroyMeshContainer(LPD3DXMESHCONTAINER pMeshContainerBase) {
    DWORD iMaterial;
    MYMESHCONTAINER *pMeshContainer = (MYMESHCONTAINER*)pMeshContainerBase;
    SAFE_DELETE_ARRAY(pMeshContainer->Name);
	SAFE_RELEASE(pMeshContainer->pSkinInfo);
    SAFE_DELETE_ARRAY(pMeshContainer->pAdjacency);
    SAFE_DELETE_ARRAY(pMeshContainer->pMaterials);
	SAFE_DELETE_ARRAY(pMeshContainer->ppBoneMatrix);
    if (pMeshContainer->ppTextures) for (iMaterial = 0; iMaterial < pMeshContainer->NumMaterials; ++iMaterial) SAFE_RELEASE(pMeshContainer->ppTextures[iMaterial]);
    SAFE_RELEASE(pMeshContainer->MeshData.pMesh);
	SAFE_RELEASE(pMeshContainer->pBoneBuffer);
	SAFE_DELETE_ARRAY(pMeshContainer->pBoneOffsetMatrices);
	SAFE_DELETE_ARRAY(pMeshContainer->ppBoneMatrix);
    SAFE_DELETE(pMeshContainer);
    return S_OK;
}

///// アニメーションメッシュ /////
AnimationMesh::AnimationMesh(LPDIRECT3DDEVICE9 pDev) : pDevice(pDev), pFrameRoot(0), pAnimController(0) {
}

AnimationMesh::~AnimationMesh() {
	FreeAnim(pFrameRoot);
	myHierarchy.DestroyFrame(pFrameRoot);
	SAFE_RELEASE(pAnimController);
}

void AnimationMesh::FreeAnim(LPD3DXFRAME pF) {
	if (pF->pMeshContainer) myHierarchy.DestroyMeshContainer(pF->pMeshContainer);
    if (pF->pFrameSibling) FreeAnim(pF->pFrameSibling);
    if (pF->pFrameFirstChild) FreeAnim(pF->pFrameFirstChild);
}

HRESULT AnimationMesh::AllocateBoneMatrix(LPD3DXMESHCONTAINER pMeshContainerBase) {
    MYFRAME *pFrame = NULL;
	DWORD numBone = 0;
    MYMESHCONTAINER *pMeshContainer = (MYMESHCONTAINER*)pMeshContainerBase;
    if (!pMeshContainer->pSkinInfo) return S_OK;
	numBone = pMeshContainer->pSkinInfo->GetNumBones();
    pMeshContainer->ppBoneMatrix = new D3DXMATRIX*[numBone];
    for (DWORD i = 0; i < numBone; ++i) {
		pFrame = (MYFRAME*)D3DXFrameFind(pFrameRoot, pMeshContainer->pSkinInfo->GetBoneName(i));
        if (!pFrame) return E_FAIL;
		pMeshContainer->ppBoneMatrix[i] = &pFrame->CombinedTransformationMatrix;
	}    
    return S_OK;
}

HRESULT AnimationMesh::AllocateAllBoneMatrices(LPD3DXFRAME pFrame) {
    if (pFrame->pMeshContainer) if (FAILED(AllocateBoneMatrix(pFrame->pMeshContainer))) return E_FAIL;
    if (pFrame->pFrameSibling) if (FAILED(AllocateAllBoneMatrices(pFrame->pFrameSibling))) return E_FAIL;
    if (pFrame->pFrameFirstChild) if (FAILED(AllocateAllBoneMatrices(pFrame->pFrameFirstChild))) return E_FAIL;
    return S_OK;
}

void AnimationMesh::Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera) {
	D3DXMATRIX trans, world, view, proj;
	D3DXMatrixTranslation(&trans, position->x, position->y, position->z);
	world = (*rotation)*trans;
	camera->GetMatrix(&view, &proj);
	pDevice->SetLight(0, light);
	pDevice->LightEnable(0, TRUE );
    pDevice->SetTransform(D3DTS_WORLD, &world);
	pDevice->SetTransform(D3DTS_VIEW, &view);
	pDevice->SetTransform(D3DTS_PROJECTION, &proj);
	frameOperator.UpdateMatrices(pFrameRoot, &world);
	frameOperator.Draw(pDevice, pFrameRoot);
}

void AnimationMesh::AdvanceTime(double time) {
	pAnimController->AdvanceTime(time, 0);
}

void AnimationMesh::ChangeMotion(int motion_number) {
	int N = pAnimController->GetNumAnimationSets();
	for (int i = 0; i < N; ++i) pAnimController->SetTrackEnable(i, false);
	pAnimController->SetTrackEnable(motion_number, true);
}

LPD3DXFRAME AnimationMesh::GetFrame() { return pFrameRoot; }



/// Xファイルから読込んだアニメーションメッシュ
XFileAnimationMesh::XFileAnimationMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDev) : AnimationMesh(pDev) {
	if (FAILED(D3DXLoadMeshHierarchyFromX(filename, D3DXMESH_MANAGED, pDevice, &myHierarchy, NULL, &pFrameRoot, &pAnimController))) throw TEXT("Xファイルの読み込みに失敗しました");
	for(DWORD i = 0; i < pAnimController->GetNumAnimationSets(); ++i) {	//アニメーショントラックを得る
		D3DXTRACK_DESC TrackDesc;
		ZeroMemory(&TrackDesc,sizeof(TrackDesc));
		TrackDesc.Weight = 1;
		TrackDesc.Speed = 1;
		TrackDesc.Enable = 1;
		pAnimController->SetTrackDesc(i, &TrackDesc);
		pAnimController->GetAnimationSet(i, &pAnimSet[i]);
		pAnimController->SetTrackAnimationSet(i, pAnimSet[i]);
	}
	AllocateAllBoneMatrices(pFrameRoot);
	ChangeMotion(0);
}
