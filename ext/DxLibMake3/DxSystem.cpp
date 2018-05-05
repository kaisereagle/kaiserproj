// -------------------------------------------------------------------------------
// 
// 		�c�w���C�u����		�V�X�e���v���O����
// 
// 				Ver 3.18e
// 
// -------------------------------------------------------------------------------

// �c�w���C�u�����쐬���p��`
#define __DX_MAKE

// �C���N���[�h ------------------------------------------------------------------
#include "DxSystem.h"
#include "DxBaseFunc.h"
#include "DxUseCLib.h"

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// �}�N����` --------------------------------------------------------------------

// �\���̒�` --------------------------------------------------------------------

// �e�[�u��-----------------------------------------------------------------------

// �������ϐ��錾 --------------------------------------------------------------

DXSYSTEMDATA DxSysData ;

// �֐��v���g�^�C�v�錾-----------------------------------------------------------

// �v���O���� --------------------------------------------------------------------

// �������E�I���֌W

// DxSysData �֌W�̏��������s��
extern int DxLib_SysInit( void )
{
	// �I�����N�G�X�g�̃t���O��|��
	DxSysData.EndRequestFlag = FALSE ;

	// �I��
	return 0 ;
}

// �I�����N�G�X�g���s��
extern int DxLib_EndRequest( void )
{
	DxSysData.EndRequestFlag = TRUE ;

	// �I��
	return 0 ;
}

// �I�����N�G�X�g�̏�Ԃ��擾����
extern int DxLib_GetEndRequest( void )
{
	return DxSysData.EndRequestFlag ;
}







// ���C�u����������������Ă��邩�ǂ������擾����( �߂�l: TRUE=����������Ă���  FALSE=����Ă��Ȃ� )
extern int NS_DxLib_IsInit( void )
{
	return DxSysData.DxLib_InitializeFlag ;
}






// �G���[�����֐�

// �Ō�ɔ��������G���[�̃G���[�R�[�h���擾����( �߂�l�@0:�G���[���������Ă��Ȃ��A���̓G���[�R�[�h�o�͂ɑΉ������G���[���������Ă��Ȃ��@�@0�ȊO�F�G���[�R�[�h�ADX_ERRORCODE_WIN_DESKTOP_24BIT_COLOR �Ȃ� )
extern int NS_GetLastErrorCode( void )
{
	return DxSysData.LastErrorCode ;
}

// �Ō�ɔ��������G���[�̃G���[���b�Z�[�W���w��̕�����o�b�t�@�Ɏ擾����
extern int NS_GetLastErrorMessage( TCHAR *StringBuffer, int StringBufferBytes )
{
	ConvString( ( char * )DxSysData.LastErrorMessage, WCHAR_T_CHARCODEFORMAT, ( char * )StringBuffer, StringBufferBytes, _TCHARCODEFORMAT ) ;

	return 0 ;
}

// �G���[�R�[�h�E���b�Z�[�W��ݒ肷��
extern int DxLib_SetLastError( int ErrorCode/* DX_ERRORCODE_WIN_24BIT_COLOR �Ȃ� */, const wchar_t *ErrorMessage )
{
	DxSysData.LastErrorCode = ErrorCode ;

	if( ErrorMessage == NULL )
	{
		DxSysData.LastErrorMessage[ 0 ] = L'\0' ;
	}
	else
	{
		CL_strcpy_s( WCHAR_T_CHARCODEFORMAT, ( char * )DxSysData.LastErrorMessage, sizeof( DxSysData.LastErrorMessage ), ( char * )ErrorMessage ) ;
	}

	return 0 ;
}

// �����t�����C�u�����̃G���[�������s��
extern int DxLib_FmtError( const wchar_t *FormatString , ... )
{
	va_list VaList ;
	wchar_t String[ 1024 ];

	// ���O�o�͗p�̃��X�g���Z�b�g����
	va_start( VaList , FormatString ) ;

	// �ҏW��̕�������擾����
	_VSWNPRINTF( String , sizeof( String ) / 2, FormatString , VaList ) ;

	// �ϒ����X�g�̃|�C���^�����Z�b�g����
	va_end( VaList ) ;

	// �G���[�����ɂ܂킷
	return DxLib_Error( String ) ;
}

// �����t�����C�u�����̃G���[�������s��
extern int DxLib_FmtErrorUTF16LE( const char *FormatString , ... )
{
	va_list VaList ;
	char String[ 2048 ];

	// ���O�o�͗p�̃��X�g���Z�b�g����
	va_start( VaList , FormatString ) ;

	// �ҏW��̕�������擾����
	CL_vsnprintf( DX_CHARCODEFORMAT_UTF16LE, TRUE, CHAR_CHARCODEFORMAT, WCHAR_T_CHARCODEFORMAT, String, sizeof( String ) / 2, FormatString, VaList ) ;

	// �ϒ����X�g�̃|�C���^�����Z�b�g����
	va_end( VaList ) ;

	// �G���[�����ɂ܂킷
	return DxLib_ErrorUTF16LE( String ) ;
}






// �ǂݍ��ݏ����n�̊֐��Ŕ񓯊��ǂݍ��݂��s�����ǂ�����ݒ肷��( �񓯊��ǂݍ��݂ɑΉ����Ă���֐��̂ݗL�� )( TRUE:�񓯊��ǂݍ��݂��s��  FALSE:�񓯊��ǂݍ��݂��s��Ȃ�( �f�t�H���g ) )
extern int NS_SetUseASyncLoadFlag( int Flag )
{
	DxSysData.ASyncLoadFlag = Flag ;

	// �I��
	return 0 ;
}

// �񓯊��ǂݍ��݂��s�����ǂ������擾����( TRUE:�񓯊��ǂݍ��݂��s��   FALSE:�񓯊��ǂݍ��݂��s��Ȃ� )
extern int GetASyncLoadFlag( void )
{
	return DxSysData.ASyncLoadFlag ? TRUE : FALSE ;
}

// �c�w���C�u�����̃E�C���h�E�֘A�̋@�\���g�p���Ȃ��t���O
extern int NS_SetNotWinFlag( int Flag )
{
	if( Flag == TRUE ) DxSysData.NotDrawFlag = TRUE;
	DxSysData.NotWinFlag = Flag ;

	return 0 ;
}

// �`��@�\���g�����ǂ����̃t���O���Z�b�g����
extern int NS_SetNotDrawFlag( int Flag )
{
	DxSysData.NotDrawFlag = Flag ;

	return 0 ;
}

// �`��@�\���g�����ǂ����̃t���O���擾����
extern int NS_GetNotDrawFlag( void )
{
	return DxSysData.NotDrawFlag ;
}

// �T�E���h�@�\���g�����ǂ����̃t���O���Z�b�g����
extern int NS_SetNotSoundFlag( int Flag )
{
	DxSysData.NotSoundFlag = Flag ;

	return 0;
}

// ���͏�Ԏ擾�@�\���g�����ǂ����̃t���O���Z�b�g����
extern int NS_SetNotInputFlag( int Flag )
{
	DxSysData.NotInputFlag = Flag ;

	return 0;
}














// �E�G�C�g�n�֐�

// �w��̎��Ԃ����������Ƃ߂�
extern int NS_WaitTimer( int WaitTime )
{
	LONGLONG StartTime, EndTime ;

	StartTime = NS_GetNowHiPerformanceCount( FALSE ) ;

	// 4msec�O�܂ŐQ��
	WaitTime *= 1000 ;
	if( WaitTime > 4000 )
	{
		// �w�莞�Ԃ̊ԃ��b�Z�[�W���[�v
		EndTime = StartTime + WaitTime - 4000 ;
		while( ProcessMessage() == 0 && EndTime > NS_GetNowHiPerformanceCount( FALSE ) )
		{
			Thread_Sleep( 1 ) ;
		}
	}

	// 4msec�ȉ��̕��͐��m�ɑ҂�
	EndTime = StartTime + WaitTime ;
	while( EndTime > NS_GetNowHiPerformanceCount( FALSE ) ){}

	// �I��
	return 0 ;
}

#ifndef DX_NON_INPUT

// �L�[�̓��͑҂�
extern int NS_WaitKey( void )
{
	int BackCode = 0 ;

	while( ProcessMessage() == 0 && CheckHitKeyAll() != 0 )
	{
		Thread_Sleep( 1 ) ;
	}

	while( ProcessMessage() == 0 && ( BackCode = CheckHitKeyAll() ) == 0 )
	{
		Thread_Sleep( 1 ) ;
	}

//	while( ProcessMessage() == 0 && CheckHitKeyAll() != 0 )
//	{
//		Thread_Sleep( 1 ) ;
//	}

	return BackCode ;
}

#endif // DX_NON_INPUT















#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE




