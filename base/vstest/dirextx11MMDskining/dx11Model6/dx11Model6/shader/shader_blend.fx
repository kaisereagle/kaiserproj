Texture2D txDiffuse : register( t0 );
SamplerState samLinear : register( s0 );

#include "cb_main.h"


//ボーンインデックスとウエイト有
struct VS_INPUT_BONE
{
	float4 Pos : POSITION;
	float3 Nor : NORMAL;
	float2 Tex : TEXCOORD0;
	uint4 Bidx : BONEINDEX;
	uint4 Bwgt : BONEWEIGHT;
};

struct PS_INPUT
{
	float4 Pos : SV_POSITION;
	float3 Nor : NORMAL;
	float2 Tex : TEXCOORD0;
};


//-------------------------
//ボーン変形＆頂点ブレンド
PS_INPUT vsBasicBlend( VS_INPUT_BONE input)
{
	PS_INPUT output = (PS_INPUT)0;
	float4 pos = input.Pos;
	float3 bpos0 = input.Bwgt[0]*mul(pos, getBoneMatrix(input.Bidx.x));
	float3 bpos1 = input.Bwgt[1]*mul(pos, getBoneMatrix(input.Bidx.y));
	float3 bpos2 = input.Bwgt[2]*mul(pos, getBoneMatrix(input.Bidx.z));
	float3 bpos3 = input.Bwgt[3]*mul(pos, getBoneMatrix(input.Bidx.w));
	pos.xyz = (bpos0.xyz + bpos1.xyz + bpos2.xyz + bpos3.xyz)/100;
	
	pos = mul( pos, Object.mtxWorld );
	pos = mul( pos, Scene.mtxView );
	output.Pos = mul( pos, Scene.mtxProj );
	output.Tex = input.Tex;	
	output.Nor = input.Nor;
	return output;
}

//-------------------------
//輪郭線用ボーン変形＆頂点ブレンド
PS_INPUT vsOutlineBlend( VS_INPUT_BONE input)
{
	PS_INPUT output = (PS_INPUT)0;
	float4 pos = input.Pos;
	pos.xyz += input.Nor*0.05;
	float3 bpos0 = input.Bwgt[0]*mul(pos, getBoneMatrix(input.Bidx.x));
	float3 bpos1 = input.Bwgt[1]*mul(pos, getBoneMatrix(input.Bidx.y));
	float3 bpos2 = input.Bwgt[2]*mul(pos, getBoneMatrix(input.Bidx.z));
	float3 bpos3 = input.Bwgt[3]*mul(pos, getBoneMatrix(input.Bidx.w));
	pos.xyz = (bpos0.xyz + bpos1.xyz + bpos2.xyz + bpos3.xyz)/100;
	pos = mul( pos, Object.mtxWorld );
	pos = mul( pos, Scene.mtxView );
	output.Pos = mul( pos, Scene.mtxProj );
	output.Tex = input.Tex;
	return output;
}

//-------------------------
//ピクセルシェーダ
float4 psBasic( PS_INPUT input ) : SV_Target
{
	float3 nor = normalize(input.Nor);

	float l = saturate( dot(nor, Light.vDirLight[0].xyz) );
	if(l > 0.25){
		l=1;
	}else{
		l=0.9;
	}
	return float4(l,l,l,1)*txDiffuse.Sample( samLinear, input.Tex)*vDiffuse;
}

//-------------------------
//輪郭線用ピクセルシェーダ
float4 psBlack( PS_INPUT input ) : SV_Target
{
	return float4(0.5,0.5,0.5,1)*txDiffuse.Sample( samLinear, input.Tex)*vDiffuse;
}

//-------------------------
//シェーダエフェクト定義
#ifdef ZGFX

	technique BasicBlend
	{
		pass P0
		{
			CullMode = FRONT;
			VertexShader = compile vs_4_0 vsOutlineBlend();
			PixelShader = compile ps_4_0 psBlack();
		}	
		pass P1
		{
			CullMode = NONE;
			VertexShader = compile vs_4_0 vsBasicBlend();
			PixelShader = compile ps_4_0 psBasic();
		}
	}

#endif//ZGFX

