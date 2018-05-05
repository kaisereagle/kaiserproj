//-----------------------------------------------------------------------------
// 
// 		�c�w���C�u����		Android�p���O�v���O����
// 
//  	Ver 3.18f
// 
//-----------------------------------------------------------------------------

// �c�w���C�u�����쐬���p��`
#define __DX_MAKE

#include "../DxCompileConfig.h"

// �C���N���[�h ---------------------------------------------------------------
#ifndef DX_NON_LOG

#include "DxLogAndroid.h"
#include "DxFileAndroid.h"
#include "DxSystemAndroid.h"
#include "../DxStatic.h"
#include "../DxLib.h"
#include "../DxLog.h"
#include "../DxBaseFunc.h"
#include "../DxMemory.h"
#include "../DxGraphics.h"
#include "DxGraphicsAndroid.h"
#include <wchar.h>
#include <android/log.h>

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// �}�N����` -----------------------------------------------------------------

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "DxLibLog", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "DxLibLog", __VA_ARGS__))

// �\���̐錾 -----------------------------------------------------------------

// �f�[�^��` -----------------------------------------------------------------

LOGDATA_ANDR LogData_Android ;

// �֐��錾 -------------------------------------------------------------------

// �v���O���� -----------------------------------------------------------------

// ���O�t�@�C�������������鏈���̊��ˑ�����
extern int LogFileInitialize_PF( const wchar_t *LogFilePath )
{
	// ���ɏ������ς݂̏ꍇ�͉������Ȃ�
	if( LogData_Android.InitializeFlag == TRUE )
	{
		return 0 ;
	}

	// �������t���O�𗧂Ă�
	LogData_Android.InitializeFlag = TRUE ;

	// �G���[���O�t�@�C���̃p�X���R�s�[
	if( g_AndroidSys.NativeActivity == NULL ||
		g_AndroidSys.NativeActivity->externalDataPath == NULL )
	{
		LogData_Android.ExternalDataPath[ 0 ] = 0 ;
	}
	else
	{
		CL_strcpy_s( DX_CHARCODEFORMAT_UTF8, LogData_Android.ExternalDataPath, sizeof( LogData_Android.ExternalDataPath ), g_AndroidSys.NativeActivity->externalDataPath ) ;
	}

	// �G���[���O�t�@�C�����č쐬����
	if( LogData_Android.ExternalDataPath[ 0 ] != '\0' )
	{
		char TempLogFilePath[ 1024 ] ;
		int Length ;

		Length = CL_strlen( DX_CHARCODEFORMAT_UTF8, LogData_Android.ExternalDataPath ) ;
		if( Length + 1 < sizeof( TempLogFilePath ) - 16 )
		{
			FILE *fp ;

			CL_strcpy( DX_CHARCODEFORMAT_UTF8, TempLogFilePath, LogData_Android.ExternalDataPath ) ;
			if( TempLogFilePath[ Length - 1 ] != '/' )
			{
				TempLogFilePath[ Length ] = '/' ;
				TempLogFilePath[ Length + 1 ] = '\0' ;
			}
			CL_strcat( DX_CHARCODEFORMAT_UTF8, TempLogFilePath, "Log.txt" ) ;
			remove( TempLogFilePath ) ;
			fp = fopen( TempLogFilePath, "wb" ) ;
			if( fp != NULL )
			{
				BYTE HEAD[ 3 ] = { 0xef, 0xbb, 0xbf } ;
				fwrite( &HEAD, 3, 1, fp ) ;

				fclose( fp ) ;
			}
		}
	}

	// �I��
	return 0 ;
}

// ���O�t�@�C���֕�����������o�������̊��ˑ�����
extern int LogFileAdd_WCHAR_T_PF( const wchar_t *LogFilePath, const wchar_t *ErrorStr )
{
	char ErrorStrDefaultBuffer[ 1024 ] ;
	char *ErrorStrTempBuffer = NULL ;
	char *ErrorStrUseBuffer = NULL ;
	int ErrorStrUseBufferBytes ;
	int Bytes ;
	int StringBytes ;

	// UTF8 �̕�����ɕϊ�����
	StringBytes = ( ( int )_WCSLEN( ErrorStr ) + 1 ) * sizeof( wchar_t ) ;
	if( StringBytes > sizeof( ErrorStrDefaultBuffer ) )
	{
		ErrorStrTempBuffer = ( char * )DXALLOC( StringBytes ) ;
		if( ErrorStrTempBuffer == NULL )
		{
			return -1 ;
		}
		ErrorStrUseBuffer = ErrorStrTempBuffer ;
		ErrorStrUseBufferBytes = StringBytes ;
	}
	else
	{
		ErrorStrUseBuffer = ErrorStrDefaultBuffer ;
		ErrorStrUseBufferBytes = sizeof( ErrorStrDefaultBuffer ) ;
	}

	Bytes = ConvString( ( char * )ErrorStr, WCHAR_T_CHARCODEFORMAT, ErrorStrUseBuffer, ErrorStrUseBufferBytes, DX_CHARCODEFORMAT_UTF8 ) ;

	// �G���[���O�t�@�C�����J��
	if( Bytes > 1 &&
		LogData_Android.ExternalDataPath[ 0 ] != '\0' )
	{
		char TempLogFilePath[ 1024 ] ;
		int Length ;

		Length = CL_strlen( DX_CHARCODEFORMAT_UTF8, LogData_Android.ExternalDataPath ) ;
		if( Length + 1 < sizeof( TempLogFilePath ) - 16 )
		{
			FILE *fp ;

			CL_strcpy( DX_CHARCODEFORMAT_UTF8, TempLogFilePath, LogData_Android.ExternalDataPath ) ;
			if( TempLogFilePath[ Length - 1 ] != '/' )
			{
				TempLogFilePath[ Length ] = '/' ;
				TempLogFilePath[ Length + 1 ] = '\0' ;
			}
			CL_strcat( DX_CHARCODEFORMAT_UTF8, TempLogFilePath, "Log.txt" ) ;
			fp = fopen( TempLogFilePath, "ab" ) ;
			if( fp != NULL )
			{
				// �G���[���O�t�@�C���ɏ����o��
				fwrite( ErrorStrUseBuffer, ( DWORD )( Bytes - 1 ), 1, fp ) ;

				// �t�@�C�������
				fclose( fp ) ;
			}
		}
	}

	// �R���\�[���ɂ��o��
	LOGI( "%s", ErrorStrUseBuffer ) ;

	if( ErrorStrTempBuffer != NULL )
	{
		DXFREE( ErrorStrTempBuffer ) ;
		ErrorStrTempBuffer = NULL ;
	}

	// �I��
	return 0 ;
}


// ���O�@�\�̏��������s�����ǂ������擾����
extern int IsInitializeLog( void )
{
	return TRUE ;
}


#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // DX_NON_LOG

