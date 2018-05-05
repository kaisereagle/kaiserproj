// -------------------------------------------------------------------------------
// 
// 		�c�w���C�u����		Android�p�������֌W�v���O�����w�b�_�t�@�C��
// 
// 				Ver 3.18f
// 
// -------------------------------------------------------------------------------

#ifndef __DXMEMORYANDROID_H__
#define __DXMEMORYANDROID_H__

// �C���N���[�h ------------------------------------------------------------------
#include "../DxCompileConfig.h"
#include "../DxHeap.h"

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// �}�N����` --------------------------------------------------------------------

#define DXMEMORY_SMALLHEAP_UNITSIZE		( 4 * 1024 * 1024)			// �������������m�ۗp�̃q�[�v�̒P�ʊm�ۃT�C�Y
#define DXMEMORY_BIGHEAP_UNITSIZE		(32 * 1024 * 1024)			// �傫���������m�ۗp�̃q�[�v�̒P�ʊm�ۃT�C�Y
#define DXMEMORY_SMALL_MAX_SIZE			(64 * 1024)					// �������������m�ۗp�̃q�[�v���g�p����ő�T�C�Y

// �\���̒�` --------------------------------------------------------------------

#ifndef DX_NON_NAMESPACE
 
}

#endif // DX_NON_NAMESPACE

// Android�p�q�[�v�̏��
struct PLATFORMHEAP_ANDR
{
	void *					AllocAddress ;						// �m�ۂ����������̈�̃A�h���X
	size_t					AllocSize ;							// �m�ۂ����������̈�̃T�C�Y
} ;

// �e�[�u��-----------------------------------------------------------------------

// �������ϐ��錾 --------------------------------------------------------------

// �֐��v���g�^�C�v�錾-----------------------------------------------------------

#ifndef DX_NON_NAMESPACE
 
namespace DxLib
{

#endif // DX_NON_NAMESPACE


#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // __DXMEMORYANDROID_H__