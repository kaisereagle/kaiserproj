// -------------------------------------------------------------------------------
// 
// 		�c�w���C�u����		Android�p���O�v���O�����w�b�_�t�@�C��
// 
// 				Ver 3.18e
// 
// -------------------------------------------------------------------------------

#ifndef __DXLOGANDROID_H__
#define __DXLOGANDROID_H__

// �C���N���[�h ------------------------------------------------------------------
#include "../DxCompileConfig.h"

#ifndef DX_NON_LOG

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// �}�N����` --------------------------------------------------------------------

// �\���̒�` --------------------------------------------------------------------

// Android�p���O�����f�[�^�\����
struct LOGDATA_ANDR
{
	int		InitializeFlag ;
	char	ExternalDataPath[ 2048 ] ;
	FILE	*fp ;
} ;

// �e�[�u��-----------------------------------------------------------------------

// �������ϐ��錾 --------------------------------------------------------------

extern LOGDATA_ANDR LogData_Android ;

// �֐��v���g�^�C�v�錾-----------------------------------------------------------

#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // DX_NON_LOG

#endif // __DXLOGANDROID_H__
