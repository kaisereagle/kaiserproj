// -------------------------------------------------------------------------------
// 
// 		�c�w���C�u����		Android�pOgg�֌W�w�b�_�t�@�C��
// 
// 				Ver 3.19 
// 
// -------------------------------------------------------------------------------

#ifndef __DXUSECLIBOGGANDROID_H__
#define __DXUSECLIBOGGANDROID_H__

// �C���N���[�h ------------------------------------------------------------------
#include "../DxCompileConfig.h"
#include "../DxLib.h"
#include "DxUseCLibOggAndroid.h"
//#include <alloca.h>

#if !defined( DX_NON_OGGVORBIS ) && !defined( DX_NON_OGGTHEORA )

//namespace DxLib
//{

// �}�N����` --------------------------------------------------------------------

// �\���̒�` --------------------------------------------------------------------

// Ogg Theora �f�R�[�h�����p���ˑ��f�[�^�\����
struct DECODE_THEORA_PF
{
	int									Dummy ;
} ;

// �������ϐ��錾 --------------------------------------------------------------

// �֐��v���g�^�C�v�錾-----------------------------------------------------------

//}

#endif // #if !defined( DX_NON_OGGVORBIS ) || !defined( DX_NON_OGGTHEORA )

#endif // __DXUSECLIBOGGANDROID_H__