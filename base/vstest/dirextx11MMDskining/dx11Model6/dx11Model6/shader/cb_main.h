//C++ÇÃÉwÉbÉ_Ç∆ìØÇ∂ç\ë¢ëÃ
struct SCBScene
{
    matrix mtxProj;
    matrix mtxView;	
};

struct SCBLight
{
    float4 vDirLight[4];
    float4 colDirLight[4];
	float4 vHemiDir;
	float4 colHemiSky;
	float4 colHemiGrd;
	float4 posPLight[4];
	float4 colPLight[4];
	float4 prmPLight[4];
};

struct SBCModel
{
	float4 colModel;
};

struct SBCObject
{
    matrix mtxWorld;
};

struct SBCBoneMatrix
{
	matrix mtxBone[4];
};

cbuffer CB1 : register( b1 )
{
	SCBScene Scene;
};


cbuffer CB2 : register( b2 )
{
	SCBLight Light;
};

cbuffer CB3 : register( b3 )
{
	SBCModel Model;
};

cbuffer CB4 : register( b4 )
{
    SBCObject Object;
}

cbuffer CB5 : register( b5 )
{
	float4 vDiffuse;
}
cbuffer CB7 : register( b7 )
{
	float4x3 BoneMatrix[2] :  packoffset(c0);
	float4x3 BoneMatrixDmmy[253] :  packoffset(c6);
	float4x3 BoneMatrixEnd :  packoffset(c765);
}
float4x3 getBoneMatrix(int idx)
{
	return BoneMatrix[idx];
}
