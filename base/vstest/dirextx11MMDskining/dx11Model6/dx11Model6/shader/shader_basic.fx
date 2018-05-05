Texture2D txDiffuse : register( t0 );
SamplerState samLinear : register( s0 );

#include "cb_main.h"

//--------------------------------------------------------------------------------------
struct VS_INPUT
{
	float4 Pos : POSITION;
	float3 Nor : NORMAL;
	float2 Tex : TEXCOORD0;
};

struct PS_INPUT
{
	float4 Pos : SV_POSITION;
	float3 Nor : NORMAL;
	float2 Tex : TEXCOORD0;
};

//--------------------------
// �e�N�X�`���̂�
PS_INPUT vsBasic( VS_INPUT input)
{
	PS_INPUT output = (PS_INPUT)0;
	float4 pos = input.Pos;
	pos = mul( pos, Object.mtxWorld );
	pos = mul( pos, Scene.mtxView );
	output.Pos = mul( pos, Scene.mtxProj );
	output.Tex = input.Tex;	
	output.Nor = input.Nor;
	return output;
}

//-------------------------
//�s�N�Z���V�F�[�_
float4 psBasic( PS_INPUT input ) : SV_Target
{
	return txDiffuse.Sample( samLinear, input.Tex)*vDiffuse;
}

//-------------------------
//�V�F�[�_�G�t�F�N�g��`
#ifdef ZGFX

	//�P���`��
	technique BasicShader
	{
		pass P0
		{
			CullMode = NONE;
			VertexShader = compile vs_4_0 vsBasic();
			PixelShader = compile ps_4_0 psBasic();
		}	
	}

#endif//ZGFX

