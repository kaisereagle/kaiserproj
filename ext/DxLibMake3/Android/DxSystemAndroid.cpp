// -------------------------------------------------------------------------------
// 
// 		�c�w���C�u����		Android�p�V�X�e���v���O����
// 
// 				Ver 3.18e
// 
// -------------------------------------------------------------------------------

// �c�w���C�u�����쐬���p��`
#define __DX_MAKE

#include "DxSystemAndroid.h"

// �C���N���[�h ------------------------------------------------------------------
#include "DxGraphicsAndroid.h"
#include "DxGraphicsFilterAndroid.h"
#include "DxLogAndroid.h"
#include "DxMaskAndroid.h"
#include "DxModelAndroid.h"
#include "DxJavaAndroid.h"
#include "../DxLib.h"
#include "../DxArchive_.h"
#include "../DxSystem.h"
#include "../DxGraphics.h"
#include "../DxModel.h"
#include "../DxMask.h"
#include "../DxFile.h"
#include "../DxNetwork.h"
#include "../DxInputString.h"
#include "../DxInput.h"
#include "../DxSound.h"
#include "../DxSoundConvert.h"
#include "../DxBaseImage.h"
#include "../DxSoftImage.h"
#include "../DxMovie.h"
#include "../DxFont.h"
#include "../DxLog.h"
#include "../DxASyncLoad.h"
#include "../DxUseCLib.h"
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <android/window.h>
#include <android/log.h>
#include <GLES2/gl2.h>

#include "../DxBaseFunc.h"
#include "../DxMemory.h"



#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// �}�N����` --------------------------------------------------------------------

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "AndroidProject1.NativeActivity", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "AndroidProject1.NativeActivity", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "threaded_app", __VA_ARGS__))

#ifndef NDEBUG
#  define LOGV(...)  ((void)__android_log_print(ANDROID_LOG_VERBOSE, "threaded_app", __VA_ARGS__))
#else
#  define LOGV(...)  ((void)0)
#endif

#define TIME_DISTANCE( now, time )			( (now) < (time) ? 0x7fffffff - (time) + (now) : (now) - (time) )

// �\���̒�` --------------------------------------------------------------------

// �e�Z���T�[�̌Œ���
struct ANDROID_SENSOR_BASE_INFO
{
	int TypeID ;				// �^�C�vID ( ASENSOR_TYPE_ACCELEROMETER �Ȃ� )
	int LooperEventID ;			// Looper �̃C�x���gID( DX_LOOPER_ID_SENSOR_ACCELEROMETER �Ȃ� )
} ;

// �e�[�u��-----------------------------------------------------------------------

static ANDROID_SENSOR_BASE_INFO g_AndroidSensorBaseInfos[ DX_ANDROID_SENSOR_NUM ] =
{
	{ ASENSOR_TYPE_ACCELEROMETER,				DX_LOOPER_ID_SENSOR_ACCELEROMETER			},				// �����x�Z���T�[
	{ ASENSOR_TYPE_MAGNETIC_FIELD,				DX_LOOPER_ID_SENSOR_MAGNETIC_FIELD			},				// ���E�Z���T�[
	{ ASENSOR_TYPE_GYROSCOPE,					DX_LOOPER_ID_SENSOR_GYROSCOPE				},				// �W���C���X�R�[�v�Z���T�[
	{ ASENSOR_TYPE_LIGHT,						DX_LOOPER_ID_SENSOR_LIGHT					},				// �Ɠx�Z���T�[
	{ ASENSOR_TYPE_PROXIMITY,					DX_LOOPER_ID_SENSOR_PROXIMITY				},				// �ߐڃZ���T�[
	{ 6/*ASENSOR_TYPE_PRESSURE*/,				DX_LOOPER_ID_SENSOR_PRESSURE				},				// �����Z���T�[
	{ 13/*ASENSOR_TYPE_AMBIENT_TEMPERATURE*/,	DX_LOOPER_ID_SENSOR_AMBIENT_TEMPERATURE		},				// ���x�Z���T�[
} ;

// �������ϐ��錾 --------------------------------------------------------------

DXLIB_ANDROID_SYSTEMINFO g_AndroidSys ;
int g_AndroidRunFlag ;

// �֐��v���g�^�C�v�錾-----------------------------------------------------------

// ��������̓_�C�A���O��\������
static int StartInputStringDialogStatic( JNIEnv *env, const TCHAR *Title ) ;

// �v���O���� --------------------------------------------------------------------

// UTF16LE �̏���������� UTF8 �̃p�����[�^����������O�o�͂���
static void OutputAndroidOSInfo_LogAddUTF8( const char *UTF16LEFormatStr, const char *UTF8Str )
{
	char TempStr[ 1024 ] ;

	ConvString( UTF8Str, DX_CHARCODEFORMAT_UTF8, TempStr, sizeof( TempStr ), DX_CHARCODEFORMAT_UTF16LE ) ;
	DXST_LOGFILEFMT_ADDUTF16LE(( UTF16LEFormatStr, TempStr )) ;
}

// �n�r�����o�͂���
extern int OutputAndroidOSInfo( JNIEnv *env )
{
	// �߂�l�̏����l�� -1
	int res = -1 ;

	if( env == NULL )
	{
		return res ;
	}

	jclass class_Build        = env->FindClass( "android/os/Build" ) ;
	jclass class_BuildVERSION = env->FindClass( "android/os/Build$VERSION" ) ;
	if( class_Build        != NULL &&
		class_BuildVERSION != NULL )
	{
		jfieldID field_BOARD		= env->GetStaticFieldID( class_Build,		 "BOARD"		, "Ljava/lang/String;"	) ;
		jfieldID field_BOOTLOADER	= env->GetStaticFieldID( class_Build,		 "BOOTLOADER"	, "Ljava/lang/String;"	) ;
		jfieldID field_BRAND		= env->GetStaticFieldID( class_Build,		 "BRAND"		, "Ljava/lang/String;"	) ;
		jfieldID field_CPU_ABI		= env->GetStaticFieldID( class_Build,		 "CPU_ABI"		, "Ljava/lang/String;"	) ;
		jfieldID field_CPU_ABI2		= env->GetStaticFieldID( class_Build,		 "CPU_ABI2"		, "Ljava/lang/String;"	) ;
		jfieldID field_DEVICE		= env->GetStaticFieldID( class_Build,		 "DEVICE"		, "Ljava/lang/String;"	) ;
		jfieldID field_DISPLAY		= env->GetStaticFieldID( class_Build,		 "DISPLAY"		, "Ljava/lang/String;"	) ;
		jfieldID field_FINGERPRINT	= env->GetStaticFieldID( class_Build,		 "FINGERPRINT"	, "Ljava/lang/String;"	) ;
		jfieldID field_HARDWARE		= env->GetStaticFieldID( class_Build,		 "HARDWARE"		, "Ljava/lang/String;"	) ;
		jfieldID field_HOST			= env->GetStaticFieldID( class_Build,		 "HOST"			, "Ljava/lang/String;"	) ;
		jfieldID field_ID			= env->GetStaticFieldID( class_Build,		 "ID"			, "Ljava/lang/String;"	) ;
		jfieldID field_MANUFACTURER	= env->GetStaticFieldID( class_Build,		 "MANUFACTURER"	, "Ljava/lang/String;"	) ;
		jfieldID field_MODEL		= env->GetStaticFieldID( class_Build,		 "MODEL"		, "Ljava/lang/String;"	) ;
		jfieldID field_PRODUCT		= env->GetStaticFieldID( class_Build,		 "PRODUCT"		, "Ljava/lang/String;"	) ;
		jfieldID field_RADIO		= env->GetStaticFieldID( class_Build,		 "RADIO"		, "Ljava/lang/String;"	) ;
		jfieldID field_TAGS			= env->GetStaticFieldID( class_Build,		 "TAGS"			, "Ljava/lang/String;"	) ;
		jfieldID field_TIME			= env->GetStaticFieldID( class_Build,		 "TIME"			, "J"					) ;
		jfieldID field_TYPE			= env->GetStaticFieldID( class_Build,		 "TYPE"			, "Ljava/lang/String;"	) ;
		jfieldID field_UNKNOWN		= env->GetStaticFieldID( class_Build,		 "UNKNOWN"		, "Ljava/lang/String;"	) ;
		jfieldID field_USER			= env->GetStaticFieldID( class_Build,		 "USER"			, "Ljava/lang/String;"	) ;
		jfieldID field_SDK_INT		= env->GetStaticFieldID( class_BuildVERSION, "SDK_INT"		, "I"					) ;
		if( field_BOARD				!= NULL &&
			field_BOOTLOADER		!= NULL &&
			field_BRAND				!= NULL &&
			field_CPU_ABI			!= NULL &&
			field_CPU_ABI2			!= NULL &&
			field_DEVICE			!= NULL &&
			field_DISPLAY			!= NULL &&
			field_FINGERPRINT		!= NULL &&
			field_HARDWARE			!= NULL &&
			field_HOST				!= NULL &&
			field_ID				!= NULL &&
			field_MANUFACTURER		!= NULL &&
			field_MODEL				!= NULL &&
			field_PRODUCT			!= NULL &&
			field_RADIO				!= NULL &&
			field_TAGS				!= NULL &&
			field_TIME				!= NULL &&
			field_TYPE				!= NULL &&
			field_UNKNOWN			!= NULL &&
			field_USER				!= NULL &&
			field_SDK_INT			!= NULL )
		{
			jstring stringfield_BOARD			= ( jstring )env->GetStaticObjectField( class_Build,		field_BOARD			) ;
			jstring stringfield_BOOTLOADER		= ( jstring )env->GetStaticObjectField( class_Build,		field_BOOTLOADER	) ;
			jstring stringfield_BRAND			= ( jstring )env->GetStaticObjectField( class_Build,		field_BRAND			) ;
			jstring stringfield_CPU_ABI			= ( jstring )env->GetStaticObjectField( class_Build,		field_CPU_ABI		) ;
			jstring stringfield_CPU_ABI2		= ( jstring )env->GetStaticObjectField( class_Build,		field_CPU_ABI2		) ;
			jstring stringfield_DEVICE			= ( jstring )env->GetStaticObjectField( class_Build,		field_DEVICE		) ;
			jstring stringfield_DISPLAY			= ( jstring )env->GetStaticObjectField( class_Build,		field_DISPLAY		) ;
			jstring stringfield_FINGERPRINT		= ( jstring )env->GetStaticObjectField( class_Build,		field_FINGERPRINT	) ;
			jstring stringfield_HARDWARE		= ( jstring )env->GetStaticObjectField( class_Build,		field_HARDWARE		) ;
			jstring stringfield_HOST			= ( jstring )env->GetStaticObjectField( class_Build,		field_HOST			) ;
			jstring stringfield_ID				= ( jstring )env->GetStaticObjectField( class_Build,		field_ID			) ;
			jstring stringfield_MANUFACTURER	= ( jstring )env->GetStaticObjectField( class_Build,		field_MANUFACTURER	) ;
			jstring stringfield_MODEL			= ( jstring )env->GetStaticObjectField( class_Build,		field_MODEL			) ;
			jstring stringfield_PRODUCT			= ( jstring )env->GetStaticObjectField( class_Build,		field_PRODUCT		) ;
			jstring stringfield_RADIO			= ( jstring )env->GetStaticObjectField( class_Build,		field_RADIO			) ;
			jstring stringfield_TAGS			= ( jstring )env->GetStaticObjectField( class_Build,		field_TAGS			) ;
//			jlong   longfield_TIME				=            env->GetStaticLongField(   class_Build,		field_TIME			) ;
			jstring stringfield_TYPE			= ( jstring )env->GetStaticObjectField( class_Build,		field_TYPE			) ;
			jstring stringfield_UNKNOWN			= ( jstring )env->GetStaticObjectField( class_Build,		field_UNKNOWN		) ;
			jstring stringfield_USER			= ( jstring )env->GetStaticObjectField( class_Build,		field_USER			) ;
			jint    intfield_SDK_INT			=            env->GetStaticIntField(    class_BuildVERSION,	field_SDK_INT		) ;

			if( stringfield_BOARD			!= NULL &&
				stringfield_BOOTLOADER		!= NULL &&
				stringfield_BRAND			!= NULL &&
				stringfield_CPU_ABI			!= NULL &&
				stringfield_CPU_ABI2		!= NULL &&
				stringfield_DEVICE			!= NULL &&
				stringfield_DISPLAY			!= NULL &&
				stringfield_FINGERPRINT		!= NULL &&
				stringfield_HARDWARE		!= NULL &&
				stringfield_HOST			!= NULL &&
				stringfield_ID				!= NULL &&
				stringfield_MANUFACTURER	!= NULL &&
				stringfield_MODEL			!= NULL &&
				stringfield_PRODUCT			!= NULL &&
				stringfield_RADIO			!= NULL &&
				stringfield_TAGS			!= NULL &&
				stringfield_TYPE			!= NULL &&
				stringfield_UNKNOWN			!= NULL &&
				stringfield_USER			!= NULL )
			{
				const char *charp_BOARD				= env->GetStringUTFChars( stringfield_BOARD			, NULL ) ;
				const char *charp_BOOTLOADER		= env->GetStringUTFChars( stringfield_BOOTLOADER	, NULL ) ;		
				const char *charp_BRAND				= env->GetStringUTFChars( stringfield_BRAND			, NULL ) ;
				const char *charp_CPU_ABI			= env->GetStringUTFChars( stringfield_CPU_ABI		, NULL ) ;		
				const char *charp_CPU_ABI2			= env->GetStringUTFChars( stringfield_CPU_ABI2		, NULL ) ;
				const char *charp_DEVICE			= env->GetStringUTFChars( stringfield_DEVICE		, NULL ) ;		
				const char *charp_DISPLAY			= env->GetStringUTFChars( stringfield_DISPLAY		, NULL ) ;		
				const char *charp_FINGERPRINT		= env->GetStringUTFChars( stringfield_FINGERPRINT	, NULL ) ;		
				const char *charp_HARDWARE			= env->GetStringUTFChars( stringfield_HARDWARE		, NULL ) ;
				const char *charp_HOST				= env->GetStringUTFChars( stringfield_HOST			, NULL ) ;
				const char *charp_ID				= env->GetStringUTFChars( stringfield_ID			, NULL ) ;		
				const char *charp_MANUFACTURER		= env->GetStringUTFChars( stringfield_MANUFACTURER	, NULL ) ;
				const char *charp_MODEL				= env->GetStringUTFChars( stringfield_MODEL			, NULL ) ;
				const char *charp_PRODUCT			= env->GetStringUTFChars( stringfield_PRODUCT		, NULL ) ;		
				const char *charp_RADIO				= env->GetStringUTFChars( stringfield_RADIO			, NULL ) ;
				const char *charp_TAGS				= env->GetStringUTFChars( stringfield_TAGS			, NULL ) ;
				const char *charp_TYPE				= env->GetStringUTFChars( stringfield_TYPE			, NULL ) ;	
				const char *charp_UNKNOWN			= env->GetStringUTFChars( stringfield_UNKNOWN		, NULL ) ;	
				const char *charp_USER				= env->GetStringUTFChars( stringfield_USER			, NULL ) ;
	
				if( charp_BOARD			!= NULL &&
					charp_BOOTLOADER	!= NULL &&
					charp_BRAND			!= NULL &&
					charp_CPU_ABI		!= NULL &&
					charp_CPU_ABI2		!= NULL &&
					charp_DEVICE		!= NULL &&
					charp_DISPLAY		!= NULL &&
					charp_FINGERPRINT	!= NULL &&
					charp_HARDWARE		!= NULL &&
					charp_HOST			!= NULL &&
					charp_ID			!= NULL &&
					charp_MANUFACTURER	!= NULL &&
					charp_MODEL			!= NULL &&
					charp_PRODUCT		!= NULL &&
					charp_RADIO			!= NULL &&
					charp_TAGS			!= NULL &&
					charp_TYPE			!= NULL &&
					charp_UNKNOWN		!= NULL &&
					charp_USER			!= NULL )
				{
					DXST_LOGFILE_ADDUTF16LE(( "\x4f\x00\x53\x00\xc5\x60\x31\x58\xfa\x51\x9b\x52\x0a\x00\x00"/*@ L"OS���o��\n" @*/ )) ;
					NS_LogFileTabAdd() ;

					DXST_LOGFILEFMT_ADDUTF16LE((   "\x41\x00\x50\x00\x49\x00\x20\x00\x4c\x00\x65\x00\x76\x00\x65\x00\x6c\x00\x1a\xff\x25\x00\x64\x00\x00"/*@ L"API Level�F%d" @*/, intfield_SDK_INT )) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xdc\x30\xfc\x30\xc9\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�{�[�h�F%s" @*/,				charp_BOARD			) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xd6\x30\xfc\x30\xc8\x30\xed\x30\xfc\x30\xc0\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�u�[�g���[�_�F%s" @*/,		charp_BOOTLOADER	) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xd6\x30\xe9\x30\xf3\x30\xc9\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�u�����h�F%s" @*/,			charp_BRAND			) ;
					OutputAndroidOSInfo_LogAddUTF8( "\x7d\x54\xe4\x4e\xbb\x30\xc3\x30\xc8\x30\x11\xff\x1a\xff\x25\x00\x73\x00\x00"/*@ L"���߃Z�b�g�P�F%s" @*/,		charp_CPU_ABI		) ;
					OutputAndroidOSInfo_LogAddUTF8( "\x7d\x54\xe4\x4e\xbb\x30\xc3\x30\xc8\x30\x12\xff\x1a\xff\x25\x00\x73\x00\x00"/*@ L"���߃Z�b�g�Q�F%s" @*/,		charp_CPU_ABI2		) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xc7\x30\xd0\x30\xa4\x30\xb9\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�f�o�C�X�F%s" @*/,			charp_DEVICE		) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xc7\x30\xa3\x30\xb9\x30\xd7\x30\xec\x30\xa4\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�f�B�X�v���C�F%s" @*/,		charp_DISPLAY		) ;
	//				OutputAndroidOSInfo_LogAddUTF8( "\x58\x8b\x25\x52\x50\x5b\x1a\xff\x25\x00\x73\x00\x00"/*@ L"���ʎq�F%s" @*/,				charp_FINGERPRINT	) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xcf\x30\xfc\x30\xc9\x30\xa6\x30\xa7\x30\xa2\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�n�[�h�E�F�A�F%s" @*/,		charp_HARDWARE		) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xdb\x30\xb9\x30\xc8\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�z�X�g�F%s" @*/,				charp_HOST			) ;
					OutputAndroidOSInfo_LogAddUTF8( "\x29\xff\x24\xff\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�h�c�F%s" @*/,				charp_ID			) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xfd\x88\x20\x90\x05\x80\x0d\x54\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�����Җ��F%s" @*/,			charp_MANUFACTURER	) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xe2\x30\xc7\x30\xeb\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"���f���F%s" @*/,				charp_MODEL			) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xfd\x88\xc1\x54\x0d\x54\x1a\xff\x25\x00\x73\x00\x00"/*@ L"���i���F%s" @*/,				charp_PRODUCT		) ;
					OutputAndroidOSInfo_LogAddUTF8( "\x21\x71\xda\x7d\xd5\x30\xa1\x30\xfc\x30\xe0\x30\xa6\x30\xa7\x30\xa2\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�����t�@�[���E�F�A�F%s" @*/,	charp_RADIO			) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xd3\x30\xeb\x30\xc9\x30\xbf\x30\xb0\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�r���h�^�O�F%s" @*/,			charp_TAGS			) ;
	//				DXST_LOGFILEFMT_ADDUTF16LE((   "\xbf\x30\xa4\x30\xe0\x30\x1a\xff\x25\x00\x64\x00\x00"/*@ L"�^�C���F%d" @*/,              ( int )longfield_TIME )) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xd3\x30\xeb\x30\xc9\x30\xbf\x30\xa4\x30\xd7\x30\x1a\xff\x25\x00\x73\x00\x00"/*@ L"�r���h�^�C�v�F%s" @*/,		charp_TYPE			) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xc5\x60\x31\x58\x0d\x4e\x0e\x66\x42\x66\x6e\x30\x58\x8b\x25\x52\x50\x5b\x1a\xff\x25\x00\x73\x00\x00"/*@ L"���s�����̎��ʎq�F%s" @*/,	charp_UNKNOWN		) ;
					OutputAndroidOSInfo_LogAddUTF8( "\xe6\x30\xfc\x30\xb6\x30\xc5\x60\x31\x58\x1a\xff\x25\x00\x73\x00\x00"/*@ L"���[�U���F%s" @*/,			charp_USER			) ;

					NS_LogFileTabSub() ;

					res = 0 ;
				}

				if( charp_BOARD			!= NULL ){ env->ReleaseStringUTFChars(	stringfield_BOARD			, charp_BOARD			) ; }
				if( charp_BOOTLOADER	!= NULL ){ env->ReleaseStringUTFChars(	stringfield_BOOTLOADER		, charp_BOOTLOADER		) ;	}
				if( charp_BRAND			!= NULL ){ env->ReleaseStringUTFChars(	stringfield_BRAND			, charp_BRAND			) ;	}
				if( charp_CPU_ABI		!= NULL ){ env->ReleaseStringUTFChars(	stringfield_CPU_ABI			, charp_CPU_ABI			) ;	}
				if( charp_CPU_ABI2		!= NULL ){ env->ReleaseStringUTFChars(	stringfield_CPU_ABI2		, charp_CPU_ABI2		) ;	}
				if( charp_DEVICE		!= NULL ){ env->ReleaseStringUTFChars(	stringfield_DEVICE			, charp_DEVICE			) ;	}
				if( charp_DISPLAY		!= NULL ){ env->ReleaseStringUTFChars(	stringfield_DISPLAY			, charp_DISPLAY			) ;	}
				if( charp_FINGERPRINT	!= NULL ){ env->ReleaseStringUTFChars(	stringfield_FINGERPRINT		, charp_FINGERPRINT		) ;	}
				if( charp_HARDWARE		!= NULL ){ env->ReleaseStringUTFChars(	stringfield_HARDWARE		, charp_HARDWARE		) ;	}
				if( charp_HOST			!= NULL ){ env->ReleaseStringUTFChars(	stringfield_HOST			, charp_HOST			) ;	}
				if( charp_ID			!= NULL ){ env->ReleaseStringUTFChars(	stringfield_ID				, charp_ID				) ;	}
				if( charp_MANUFACTURER	!= NULL ){ env->ReleaseStringUTFChars(	stringfield_MANUFACTURER	, charp_MANUFACTURER	) ;	}
				if( charp_MODEL			!= NULL ){ env->ReleaseStringUTFChars(	stringfield_MODEL			, charp_MODEL			) ;	}
				if( charp_PRODUCT		!= NULL ){ env->ReleaseStringUTFChars(	stringfield_PRODUCT			, charp_PRODUCT			) ;	}
				if( charp_RADIO			!= NULL ){ env->ReleaseStringUTFChars(	stringfield_RADIO			, charp_RADIO			) ;	}
				if( charp_TAGS			!= NULL ){ env->ReleaseStringUTFChars(	stringfield_TAGS			, charp_TAGS			) ;	}
				if( charp_TYPE			!= NULL ){ env->ReleaseStringUTFChars(	stringfield_TYPE			, charp_TYPE			) ;	}
				if( charp_UNKNOWN		!= NULL ){ env->ReleaseStringUTFChars(	stringfield_UNKNOWN			, charp_UNKNOWN			) ;	}
				if( charp_USER			!= NULL ){ env->ReleaseStringUTFChars(	stringfield_USER			, charp_USER			) ;	}
			}

			if( stringfield_BOARD			!= NULL ){ env->DeleteLocalRef( stringfield_BOARD			) ;	}
			if( stringfield_BOOTLOADER		!= NULL ){ env->DeleteLocalRef( stringfield_BOOTLOADER		) ;	}
			if( stringfield_BRAND			!= NULL ){ env->DeleteLocalRef( stringfield_BRAND			) ;	}
			if( stringfield_CPU_ABI			!= NULL ){ env->DeleteLocalRef( stringfield_CPU_ABI			) ;	}
			if( stringfield_CPU_ABI2		!= NULL ){ env->DeleteLocalRef( stringfield_CPU_ABI2		) ;	}
			if( stringfield_DEVICE			!= NULL ){ env->DeleteLocalRef( stringfield_DEVICE			) ;	}
			if( stringfield_DISPLAY			!= NULL ){ env->DeleteLocalRef( stringfield_DISPLAY			) ;	}
			if( stringfield_FINGERPRINT		!= NULL ){ env->DeleteLocalRef( stringfield_FINGERPRINT		) ;	}
			if( stringfield_HARDWARE		!= NULL ){ env->DeleteLocalRef( stringfield_HARDWARE		) ;	}
			if( stringfield_HOST			!= NULL ){ env->DeleteLocalRef( stringfield_HOST			) ;	}
			if( stringfield_ID				!= NULL ){ env->DeleteLocalRef( stringfield_ID				) ;	}
			if( stringfield_MANUFACTURER	!= NULL ){ env->DeleteLocalRef( stringfield_MANUFACTURER	) ;	}
			if( stringfield_MODEL			!= NULL ){ env->DeleteLocalRef( stringfield_MODEL			) ;	}
			if( stringfield_PRODUCT			!= NULL ){ env->DeleteLocalRef( stringfield_PRODUCT			) ;	}
			if( stringfield_RADIO			!= NULL ){ env->DeleteLocalRef( stringfield_RADIO			) ;	}
			if( stringfield_TAGS			!= NULL ){ env->DeleteLocalRef( stringfield_TAGS			) ;	}
			if( stringfield_TYPE			!= NULL ){ env->DeleteLocalRef( stringfield_TYPE			) ;	}
			if( stringfield_UNKNOWN			!= NULL ){ env->DeleteLocalRef( stringfield_UNKNOWN			) ;	}
			if( stringfield_USER			!= NULL ){ env->DeleteLocalRef( stringfield_USER			) ; }
		}
	}

	if( class_Build        != NULL ){ env->DeleteLocalRef( class_Build        ) ; }
	if( class_BuildVERSION != NULL ){ env->DeleteLocalRef( class_BuildVERSION ) ; }

	// �߂�l��Ԃ�
	return res ;
}

// ���C�u�����������֐�
extern int NS_DxLib_Init( void )
{
	// ���ɏ������ς݂̏ꍇ�͉��������I��
	if( DxSysData.DxLib_InitializeFlag == TRUE )
	{
		return 0 ;
	}

	DXST_LOGFILEFMT_ADDUTF16LE(( "\x24\xff\x38\xff\xe9\x30\xa4\x30\xd6\x30\xe9\x30\xea\x30\x6e\x30\x1d\x52\x1f\x67\x16\x53\xe6\x51\x06\x74\x8b\x95\xcb\x59\x00"/*@ L"�c�w���C�u�����̏����������J�n" @*/ )) ;
	DXST_LOGFILE_TABADD ;

	// ���������t���O�𗧂Ă�
	DxSysData.DxLib_RunInitializeFlag = TRUE ;

#ifndef DX_NON_LITERAL_STRING
	// �c�w���C�u�����̃o�[�W�������o�͂���
	{
		char UTF16LE_Buffer[ 128 ] ;
		char DestBuffer[ 128 ] ;
		ConvString( ( const char * )DXLIB_VERSION_STR_W, WCHAR_T_CHARCODEFORMAT, UTF16LE_Buffer, sizeof( UTF16LE_Buffer ), DX_CHARCODEFORMAT_UTF16LE ) ;
		CL_snprintf( DX_CHARCODEFORMAT_UTF16LE, TRUE, DX_CHARCODEFORMAT_SHIFTJIS, DX_CHARCODEFORMAT_UTF16LE, DestBuffer, sizeof( DestBuffer ) / 2, "\x24\xff\x38\xff\xe9\x30\xa4\x30\xd6\x30\xe9\x30\xea\x30\x20\x00\x56\x00\x65\x00\x72\x00\x25\x00\x73\x00\x0a\x00\x00"/*@ L"�c�w���C�u���� Ver%s\n" @*/, UTF16LE_Buffer ) ;

		DXST_LOGFILE_ADDUTF16LE( DestBuffer ) ;
	}
#endif

	// OS���o��
	{
		pthread_mutex_lock( &g_AndroidSys.NativeActivityMutex ) ;

		if( g_AndroidSys.NativeActivity == NULL )
		{
			pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
			return -1 ;
		}

		// JavaVM �ƃ\�t�g���s�p�X���b�h���֘A�t��
		JNIEnv *env ;
		if( g_AndroidSys.NativeActivity->vm->AttachCurrentThreadAsDaemon( &env, NULL ) != JNI_OK )
		{
			pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
			return -1 ;
		}

		OutputAndroidOSInfo( env ) ;

		// JavaVM �Ƃ��̃X���b�h�̊֘A�t���I��
		g_AndroidSys.NativeActivity->vm->DetachCurrentThread() ;

		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
	}

	// DxSysData �̋��ʏ���������
	DxLib_SysInit() ;

	// DxBaseFunc �̏�����
	_INIT_BASEFUNC() ;

	// �L�����N�^�[�R�[�h�֌W�̏��������s��
	InitCharCode() ;

	// �g�p���镶���Z�b�g���Z�b�g
	_SET_DEFAULT_CHARCODEFORMAT() ;

#ifndef DX_NON_ASYNCLOAD
	// �񓯊��ǂݍ��ݏ����̏�����
	InitializeASyncLoad( Thread_GetCurrentId() ) ;
#endif // DX_NON_ASYNCLOAD

	// �t�@�C���A�N�Z�X�����̏�����
	InitializeFile() ;

#ifndef DX_NON_OGGTHEORA
	// Theora �p�̏�����
	TheoraDecode_GrobalInitialize() ;
#endif

	// �A�[�J�C�u�t�@�C���A�N�Z�X�p�̃f�[�^��������
#ifndef DX_NON_DXA
	DXA_DIR_Initialize() ;
#endif

	// �X�g���[���f�[�^�ǂݍ��ݐ���p�|�C���^�\���̂̃f�t�H���g�l���Z�b�g
	NS_ChangeStreamFunction( NULL ) ;

#ifndef DX_NON_LOG
	// ���O�t�@�C���̏�����
	LogFileInitialize() ;
#endif

	// �V�X�e�����O���o��
//	OutSystemInfo() ;

#ifndef DX_NON_GRAPHICS
	// �f�t�H���g�̃O���t�B�b�N�����֐���o�^
	NS_SetRestoreGraphCallback( NULL ) ;
#endif // DX_NON_GRAPHICS

	// �e�����n�̏�����
	if( DxSysData.NotInputFlag == FALSE )
	{
#ifndef DX_NON_INPUT
		if( InitializeInputSystem() == -1 ) goto ERROR_DX ;			// ���̓V�X�e���̏�����
#endif // DX_NON_INPUT
	}

	if( DxSysData.NotSoundFlag == FALSE )
	{
#ifndef DX_NON_SOUND
		InitializeSoundConvert() ;									// �T�E���h�ϊ������̏�����
		InitializeSoundSystem() ;									// �T�E���h�V�X�e���̂̏�����
#endif // DX_NON_SOUND
	}
	if( DxSysData.NotDrawFlag == FALSE )
	{
		InitializeBaseImageManage() ;
#ifndef DX_NON_SOFTIMAGE
		InitializeSoftImageManage() ;
#endif // DX_NON_SOFTIMAGE
#ifndef DX_NON_MOVIE
		InitializeMovieManage() ;
#endif

#ifndef DX_NON_GRAPHICS
		if( Graphics_Initialize() < 0 ) goto ERROR_DX ;
#endif // DX_NON_GRAPHICS
	}
#ifndef DX_NON_INPUTSTRING
	InitializeInputCharBuf() ;									// �����R�[�h�o�b�t�@�̏�����
#endif // DX_NON_INPUTSTRING

	// �c�w���C�u���������������t���O�����Ă�
	DxSysData.DxLib_InitializeFlag = TRUE ;

	// �u�r�x�m�b�҂�������
//	NS_SetWaitVSyncFlag( TRUE ) ;

#if !defined( DX_NON_LOG ) && !defined( DX_NON_PRINTF_DX )
	// ���O�o�͏����̏��������s��
	InitializeLog() ;
#endif

#ifndef DX_NON_GRAPHICS
	// �`���̕ύX
	NS_SetDrawScreen( DX_SCREEN_BACK ) ;
	NS_SetDrawScreen( DX_SCREEN_FRONT ) ;
#endif // DX_NON_GRAPHICS

	if( DxSysData.NotDrawFlag == FALSE )
	{
#ifndef DX_NON_MODEL
		// ���f���o�[�W�����P�̏�����
		if( MV1Initialize() < 0 )
		{
			goto ERROR_DX ;
		}
#endif
	}

#ifndef DX_NON_ASYNCLOAD
	// �񓯊��ǂݍ��ݏ������s���X���b�h�𗧂Ă�
	if( SetupASyncLoadThread( 3 ) < 0 )
	{
		DXST_LOGFILE_ADDUTF16LE( "\x5e\x97\x0c\x54\x1f\x67\xad\x8a\x7f\x30\xbc\x8f\x7f\x30\xe6\x51\x06\x74\x92\x30\x4c\x88\x46\x30\xb9\x30\xec\x30\xc3\x30\xc9\x30\x6e\x30\xcb\x7a\x61\x30\x0a\x4e\x52\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"�񓯊��ǂݍ��ݏ������s���X���b�h�̗����グ�Ɏ��s���܂���\n" @*/ ) ;
		goto ERROR_DX ;
	}
#endif // DX_NON_ASYNCLOAD

	// ���������t���O��|��
	DxSysData.DxLib_RunInitializeFlag = FALSE ;

	DXST_LOGFILE_TABSUB ;
	DXST_LOGFILEFMT_ADDUTF16LE(( "\x24\xff\x38\xff\xe9\x30\xa4\x30\xd6\x30\xe9\x30\xea\x30\x6e\x30\x1d\x52\x1f\x67\x16\x53\xe6\x51\x06\x74\x42\x7d\x86\x4e\x00"/*@ L"�c�w���C�u�����̏����������I��" @*/ ) ) ;

	// �I��
	return 0 ;

ERROR_DX:
	NS_DxLib_End() ;

	DXST_LOGFILE_TABSUB ;
	DXST_LOGFILEFMT_ADDUTF16LE(( "\x24\xff\x38\xff\xe9\x30\xa4\x30\xd6\x30\xe9\x30\xea\x30\x6e\x30\x1d\x52\x1f\x67\x16\x53\xe6\x51\x06\x74\x31\x59\x57\x65\x00"/*@ L"�c�w���C�u�����̏������������s" @*/ ) ) ;

	// ���������t���O��|��
	DxSysData.DxLib_RunInitializeFlag = FALSE ;

	return -1 ;
} 

// ���C�u�����g�p�̏I���֐�
extern int NS_DxLib_End( void )
{
	// ���ɏI���������s���Ă��邩�A������������������Ă��Ȃ��ꍇ�͉������Ȃ�
	if( DxSysData.DxLib_InitializeFlag == FALSE )
	{
		return 0 ;
	}

#ifndef DX_NON_SOFTIMAGE
	// �o�^�����S�Ẵ\�t�g�C���[�W���폜
	InitSoftImage() ;
#endif // DX_NON_SOFTIMAGE

	// �e�����n�̏I��
#if !defined( DX_NON_LOG ) && !defined( DX_NON_PRINTF_DX )
	TerminateLog() ;			// ���O�����̌�n��
#endif

#ifndef DX_NON_NETWORK
	TerminateNetWork() ;		// �v�����r�������������֌W�̏I��
#endif

#ifndef DX_NON_SOUND
	NS_StopMusic() ;			// �l�h�c�h�����t����Ă����Ԃ̏ꍇ������~�߂�
#endif // DX_NON_SOUND

#ifndef DX_NON_MODEL
	MV1Terminate() ;			// ���f���o�[�W�����P�̌�n��
#endif

#ifndef DX_NON_GRAPHICS
	Graphics_Terminate() ;
#endif // DX_NON_GRAPHICS

	TerminateBaseImageManage() ;

#ifndef DX_NON_SOFTIMAGE
	TerminateSoftImageManage() ;
#endif // DX_NON_SOFTIMAGE

#ifndef DX_NON_MOVIE
	TerminateMovieManage() ;
#endif

#ifndef DX_NON_INPUT
	TerminateInputSystem() ;	// ���̓V�X�e���̏I��
#endif // DX_NON_INPUT

#ifndef DX_NON_SOUND
	TerminateSoundSystem() ;	// �T�E���h�V�X�e���̌�n��
	TerminateSoundConvert() ;	// �T�E���h�ϊ������̏I��
#endif // DX_NON_SOUND

#ifndef DX_NON_LOG
	// ���O�t�@�C���̌�n��
	LogFileTerminate() ;
#endif

	// �c�w���C�u���������������t���O��|��
	DxSysData.DxLib_InitializeFlag = FALSE ;

	// �A�[�J�C�u�t�@�C���A�N�Z�X�p�̃f�[�^�̌�n��
#ifndef DX_NON_DXA
	DXA_DIR_Terminate() ;
#endif

#ifndef DX_NON_ASYNCLOAD
	// �񓯊��ǂݍ��ݏ����p�̃X���b�h�����
	CloseASyncLoadThread() ;
#endif // DX_NON_ASYNCLOAD

	// �t�@�C���A�N�Z�X�����̌�n��
	TerminateFile() ;

#ifndef DX_NON_ASYNCLOAD
	// �񓯊��ǂݍ��ݏ����̌�n��
	TerminateASyncLoad() ;
#endif // DX_NON_ASYNCLOAD

#ifdef DX_USE_DXLIB_MEM_DUMP
	// �������_���v���s��
	NS_DxDumpAlloc() ;
#endif

	// �������̌�n�����s��
	MemoryTerminate() ;

	// �I��
	return 0 ;
}

// ���C�u�����̓����Ŏg�p���Ă���\���̂��[�����������āADxLib_Init �̑O�ɍs�����ݒ�𖳌�������( DxLib_Init �̑O�ł̂ݗL�� )
extern int NS_DxLib_GlobalStructInitialize( void )
{
//	_MEMSET( &GRA2, 0, sizeof( GRA2 ) ) ;

#ifndef DX_NON_GRAPHICS
	_MEMSET( &GraphicsSysData, 0, sizeof( GraphicsSysData ) );
#endif // DX_NON_GRAPHICS

	_MEMSET( &BaseImageManage, 0, sizeof( BaseImageManage ) ) ;
#ifndef DX_NON_SOUND
	_MEMSET( &SoundSysData, 0, sizeof( SoundSysData ) );
#endif // DX_NON_SOUND

	return 0;
}











// �G���[�����֐�

// �G���[����
extern int DxLib_Error( const wchar_t *ErrorStr )
{
	// �G���[���O�̔r�o
	DXST_LOGFILE_ADDW( ErrorStr ) ;
	DXST_LOGFILE_ADDW( L"\n" ) ;

	// �e�����n�̏I��
	NS_DxLib_End() ;

	exit( -1 ) ;

	return -1 ;
}

// ���C�u�����̃G���[�������s��( UTF16LE�� )
extern int DxLib_ErrorUTF16LE( const char *ErrorStr )
{
	int Result ;

	CHAR_TO_WCHAR_T_STRING_BEGIN( ErrorStr )
	CHAR_TO_WCHAR_T_STRING_SETUP( ErrorStr, return -1, DX_CHARCODEFORMAT_UTF16LE )

	Result = DxLib_Error( UseErrorStrBuffer ) ;

	CHAR_TO_WCHAR_T_STRING_END( ErrorStr )

	return Result ;
}



























// �J�E���^�y�ю����擾�n�֐�

// �~���b�P�ʂ̐��x�����J�E���^�̌��ݒl�𓾂�
extern int NS_GetNowCount( int /*UseRDTSCFlag*/ )
{
	LONGLONG ResultLL ;
	int Result ;

	ResultLL  = NS_GetNowHiPerformanceCount() / 1000 ;
	ResultLL &= 0x7fffffff ;
	Result    = ( int )ResultLL ;

	return Result ;
}

// GetNowTime�̍����x�o�[�W����
extern LONGLONG NS_GetNowHiPerformanceCount( int /*UseRDTSCFlag*/ )
{
	LONGLONG NowTime ;
	timeval ltimeval ;

	gettimeofday( &ltimeval, NULL ) ;

	NowTime = ( LONGLONG )ltimeval.tv_sec * 1000000 + ltimeval.tv_usec ;

	return NowTime ;
}

// ���ݎ������擾����
extern int NS_GetDateTime( DATEDATA *DateBuf )
{
	time_t nowtime ;
	tm *datetime ;

	time( &nowtime ) ;

	datetime = localtime( &nowtime ) ;

	// ���[�J�������f�[�^�����ɐ�p�̃f�[�^�^�f�[�^�Ɏ�����ɉh������
	DateBuf->Year	= datetime->tm_year + 1900 ;
	DateBuf->Mon	= datetime->tm_mon + 1 ;
	DateBuf->Day	= datetime->tm_mday ;
	DateBuf->Hour	= datetime->tm_hour ;
	DateBuf->Min	= datetime->tm_min ;
	DateBuf->Sec	= datetime->tm_sec ;

	// �I��
	return 0 ;
}



// �����擾

#ifndef DX_NON_MERSENNE_TWISTER

// �����̏����l��ݒ肷��
extern int NS_SRand( int Seed )
{
	// �����l�Z�b�g
	srandMT( ( unsigned int )Seed ) ;

	// �I��
	return 0 ;
}

// �������擾����( RandMax : �Ԃ��ė���l�̍ő�l )
extern int NS_GetRand( int RandMax )
{
	int Result ;
	LONGLONG RandMaxLL ;

	RandMaxLL = RandMax ;
	RandMaxLL ++ ;
	Result = ( int )( ( ( LONGLONG )randMT() * RandMaxLL ) >> 32 ) ;

	return Result ;
}

#else // DX_NON_MERSENNE_TWISTER

// �����̏����l��ݒ肷��
extern int NS_SRand( int Seed )
{
	// �����l�Z�b�g
	srand( Seed ) ;

	// �I��
	return 0 ;
}

// �������擾����( RandMax : �Ԃ��ė���l�̍ő�l )
extern int NS_GetRand( int RandMax )
{
	int Result ;
	LONGLONG RandMaxLL ;

	RandMaxLL = RandMax ;
	RandMaxLL ++ ;
	Result = ( int )( ( ( LONGLONG )rand() * RandMaxLL ) / ( ( LONGLONG )RAND_MAX + 1 ) ) ;

	return Result ;
}

#endif // DX_NON_MERSENNE_TWISTER

#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE































// �X�e�[�^�X�o�[�ƃi�r�Q�[�V�����o�[���\���ɂ���
extern int SetAndroidWindowStyle( JNIEnv *env )
{
	// �߂�l�̏����l�� -1
	int res = -1 ;

	if( env == NULL )
	{
		return res ;
	}

	// API Level �̒萔�����N���X�̎擾
	jclass class_BuildVERSION = env->FindClass( "android/os/Build$VERSION" ) ;
	if( class_BuildVERSION == NULL )
	{
		return res ;
	}

	// API Level �̒萔��ID���擾
	jfieldID field_SDK_INT = env->GetStaticFieldID( class_BuildVERSION, "SDK_INT", "I" ) ;
	if( field_SDK_INT == NULL )
	{
		return res ;
	}

	// API Level �̒l���擾
	jint intfield_SDK_INT = env->GetStaticIntField( class_BuildVERSION, field_SDK_INT ) ;

	// setSystemUiVisibility �Ăяo���ɕK�v�ȃN���X���擾
	jclass class_NativeActivity = env->GetObjectClass( g_AndroidSys.NativeActivity->clazz ) ;
	jclass class_Window         = env->FindClass( "android/view/Window" ) ;
	jclass class_View           = env->FindClass( "android/view/View"   ) ;

	// �N���X�̎擾���ł����ꍇ�̂� if ���̒��ɓ���
	if( class_NativeActivity != NULL &&
		class_Window         != NULL &&
		class_View           != NULL )
	{
		// setSystemUiVisibility �Ăяo���ɕK�v�Ȋ֐����擾����
		jmethodID methodID_getWindow             = env->GetMethodID( class_NativeActivity, "getWindow",             "()Landroid/view/Window;" ) ;
		jmethodID methodID_getDecorView          = env->GetMethodID( class_Window,         "getDecorView",          "()Landroid/view/View;"   ) ;
		jmethodID methodID_setSystemUiVisibility = env->GetMethodID( class_View,           "setSystemUiVisibility", "(I)V"                    ) ;

		// �֐��̎擾���ł����ꍇ�̂� if ���̒��ɓ���
		if( methodID_getWindow             != NULL &&
			methodID_getDecorView          != NULL && 
			methodID_setSystemUiVisibility != NULL )
		{
			// �\�t�g�� window �̎擾
			jobject object_Window = env->CallObjectMethod( g_AndroidSys.NativeActivity->clazz, methodID_getWindow ) ;
			if( object_Window != NULL )
			{
				// �\�t�g�� DecorView ���擾
				jobject object_DecorView = env->CallObjectMethod( object_Window, methodID_getDecorView ) ;
				if( object_DecorView != NULL )
				{
					// API Level �ɉ����� setSystemUiVisibility �ɓn���t���O��ύX����
					jint flags = 0 ;

					// API Level 14
					if( intfield_SDK_INT >= 14 )
					{
						jfieldID field_SYSTEM_UI_FLAG_HIDE_NAVIGATION = env->GetStaticFieldID( class_View, "SYSTEM_UI_FLAG_HIDE_NAVIGATION", "I" ) ;
						jfieldID field_SYSTEM_UI_FLAG_LOW_PROFILE     = env->GetStaticFieldID( class_View, "SYSTEM_UI_FLAG_LOW_PROFILE",     "I" ) ;
						if( field_SYSTEM_UI_FLAG_HIDE_NAVIGATION != NULL &&
							field_SYSTEM_UI_FLAG_LOW_PROFILE     != NULL )
						{
							jint intfield_SYSTEM_UI_FLAG_HIDE_NAVIGATION = env->GetStaticIntField( class_View, field_SYSTEM_UI_FLAG_HIDE_NAVIGATION ) ;
							jint intfield_SYSTEM_UI_FLAG_LOW_PROFILE     = env->GetStaticIntField( class_View, field_SYSTEM_UI_FLAG_LOW_PROFILE ) ;
							flags |= intfield_SYSTEM_UI_FLAG_HIDE_NAVIGATION |
									 intfield_SYSTEM_UI_FLAG_LOW_PROFILE ;
						}
					}

					// API Level 16
					if( intfield_SDK_INT >= 16 )
					{
						jfieldID field_SYSTEM_UI_FLAG_FULLSCREEN = env->GetStaticFieldID( class_View, "SYSTEM_UI_FLAG_FULLSCREEN", "I" ) ;
						if( field_SYSTEM_UI_FLAG_FULLSCREEN != NULL )
						{
							jint intfield_SYSTEM_UI_FLAG_FULLSCREEN = env->GetStaticIntField( class_View, field_SYSTEM_UI_FLAG_FULLSCREEN ) ;
							flags |= intfield_SYSTEM_UI_FLAG_FULLSCREEN ;
						}
					}

					// API Level 19
					if( intfield_SDK_INT >= 19 )
					{
						jfieldID field_SYSTEM_UI_FLAG_IMMERSIVE_STICKY = env->GetStaticFieldID( class_View, "SYSTEM_UI_FLAG_IMMERSIVE_STICKY", "I" ) ;
						jfieldID field_SYSTEM_UI_FLAG_IMMERSIVE        = env->GetStaticFieldID( class_View, "SYSTEM_UI_FLAG_IMMERSIVE",        "I" ) ;
						if( field_SYSTEM_UI_FLAG_IMMERSIVE_STICKY != NULL &&
							field_SYSTEM_UI_FLAG_IMMERSIVE        != NULL )
						{
							jint intfield_SYSTEM_UI_FLAG_IMMERSIVE_STICKY = env->GetStaticIntField( class_View, field_SYSTEM_UI_FLAG_IMMERSIVE_STICKY ) ;
							jint intfield_SYSTEM_UI_FLAG_IMMERSIVE        = env->GetStaticIntField( class_View, field_SYSTEM_UI_FLAG_IMMERSIVE        ) ;
							flags |= intfield_SYSTEM_UI_FLAG_IMMERSIVE_STICKY |
									 intfield_SYSTEM_UI_FLAG_IMMERSIVE ;
						}
					}

					// setSystemUiVisibility �̌Ăяo��
					env->CallVoidMethod( object_DecorView, methodID_setSystemUiVisibility, flags ) ;

					// �����܂ł��ꂽ�ꍇ�̂ݖ߂�l�� 0 �ɂ���
					res = 0 ;

					// �擾�����Q�Ƃ̌�n��
					env->DeleteLocalRef( object_DecorView ) ;
					object_DecorView = NULL ;
				}

				// �擾�����Q�Ƃ̌�n��
				env->DeleteLocalRef( object_Window ) ;
				object_Window = NULL ;
			}
		}
	}

	// �擾�����Q�Ƃ̌�n��
	if( class_BuildVERSION != NULL )
	{
		env->DeleteLocalRef( class_BuildVERSION ) ;
		class_BuildVERSION = NULL ;
	}

	if( class_NativeActivity != NULL )
	{
		env->DeleteLocalRef( class_NativeActivity ) ;
		class_NativeActivity = NULL ;
	}

	if( class_Window != NULL )
	{
		env->DeleteLocalRef( class_Window ) ;
		class_Window = NULL ;
	}

	if( class_View != NULL )
	{
		env->DeleteLocalRef( class_View ) ;
		class_View = NULL ;
	}

	// �߂�l��Ԃ�
	return res ;
}

// android_main �֐�
extern int android_main( void ) ;

// �\�t�g�p�X���b�h�̃G���g���[�|�C���g
static void* android_app_entry( void * )
{
	// �c�w���C�u�����̃t�@�C���V�X�e���ɃA�Z�b�g�}�l�[�W���[��o�^
	SetAssetManager( g_AndroidSys.NativeActivity->assetManager ) ;

	// �c�w���C�u�����̃t�@�C���V�X�e���� InternalDataPath �� ExternalDataPath ��o�^
	SetInternalAndExternalDataPath(
		g_AndroidSys.NativeActivity->internalDataPath,
		g_AndroidSys.NativeActivity->externalDataPath
	) ;

#ifndef DX_NON_FONT
	// �t�H���g�^�C�v�̓A���`�G�C���A�X�^�C�v�ɌŒ�
	SetAntialiasingFontOnlyFlag( TRUE ) ;
#endif // DX_NON_FONT


#ifndef DX_NON_DXA
	// �c�w�A�[�J�C�u�̃p�X��啶���ɂ��Ȃ��悤�ɂ���
	DXA_DIR_SetNotArchivePathCharUp( TRUE ) ;
#endif // DX_NON_DXA

	// �R�[���o�b�N���� Looper �̎擾
    g_AndroidSys.Looper = ALooper_prepare( ALOOPER_PREPARE_ALLOW_NON_CALLBACKS ) ;

	// ���C���X���b�h����̃��b�Z�[�W�󂯎��o�^
    ALooper_addFd(
		g_AndroidSys.Looper,
		g_AndroidSys.MessageRead,
		DX_LOOPER_ID_MAIN,
		ALOOPER_EVENT_INPUT,
		NULL,
		NULL
	) ;

	// �Z���T�[�̏�����
	{
		int i ;

		g_AndroidSys.SensorManager					= ASensorManager_getInstance() ;
		for( i = 0 ; i < DX_ANDROID_SENSOR_NUM ; i ++ )
		{
			g_AndroidSys.SensorInfos[ i ].Sensor			= ASensorManager_getDefaultSensor( g_AndroidSys.SensorManager, g_AndroidSensorBaseInfos[ i ].TypeID ) ;
			g_AndroidSys.SensorInfos[ i ].SensorEventQueue	= ASensorManager_createEventQueue( g_AndroidSys.SensorManager, g_AndroidSys.Looper, g_AndroidSensorBaseInfos[ i ].LooperEventID, NULL, NULL ) ;
		}
	}

	// �\�t�g�p�X���b�h�J�n�������t���O�𗧂Ă�
    pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
	g_AndroidSys.MutexLockIndex = 1 << 7 ;
    g_AndroidSys.SoftThreadRunning = 1 ;
    pthread_cond_broadcast( &g_AndroidSys.Cond ) ;
	g_AndroidSys.MutexLockIndex &= ~( 1 << 7 ) ;
    pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;

	// �\�t�g�p�X���b�h���҂���Ԃ��ǂ����̃t���O��|��
	g_AndroidSys.SoftThreadWait = 0 ;

	// �X�e�[�^�X�o�[�ƃi�r�Q�[�V�����o�[���\���ɂ���
	{
		JNIEnv *env ;
		int res = -1 ;

		// JavaVM �ƃ\�t�g���s�p�X���b�h���֘A�t��
		if( g_AndroidSys.NativeActivity->vm->AttachCurrentThreadAsDaemon( &env, NULL ) == JNI_OK )
		{
			// �X�e�[�^�X�o�[�ƃi�r�Q�[�V�����o�[���\���ɂ���֐��Ăяo��
			res = SetAndroidWindowStyle( env ) ;

			// JavaVM �Ƃ��̃X���b�h�̊֘A�t���I��
			g_AndroidSys.NativeActivity->vm->DetachCurrentThread() ;
		}

		// �X�e�[�^�X�o�[�ƃi�r�Q�[�V�����o�[�̔�\�������Ɏ��s������A���o�[�W�����p�̃t���X�N���[�������������s����
		if( res < 0 )
		{
			ANativeActivity_setWindowFlags( g_AndroidSys.NativeActivity, AWINDOW_FLAG_FULLSCREEN, AWINDOW_FLAG_FULLSCREEN ) ;
		}
	}

	// android_main �̌Ăяo��
	android_main() ;

	// onDestroy ���������Ă��Ȃ��ꍇ�� finish ���Ăяo���� onDestroy ����������܂ő҂�
	if( g_AndroidSys.DestroyRequested == 0 )
	{
		ANativeActivity_finish( g_AndroidSys.NativeActivity ) ;
		while( NS_ProcessMessage() == 0 )
		{
			Thread_Sleep( 10 ) ;
		}
	}

	// ���̓C�x���g�L���[�̌�n��
    pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
	g_AndroidSys.MutexLockIndex = 1 << 8 ;
    if( g_AndroidSys.InputQueue != NULL )
	{
		AInputQueue_detachLooper( ( AInputQueue * )g_AndroidSys.InputQueue ) ;
		g_AndroidSys.InputQueue = NULL ;
    }
	
	// �Z���T�[�C�x���g�L���[�̌�n��
	{
		int i ;

		for( i = 0 ; i < DX_ANDROID_SENSOR_NUM ; i ++ )
		{
			if( g_AndroidSys.SensorInfos[ i ].SensorEventQueue != NULL )
			{
				ASensorManager_destroyEventQueue( g_AndroidSys.SensorManager, g_AndroidSys.SensorInfos[ i ].SensorEventQueue ) ;
				g_AndroidSys.SensorInfos[ i ].SensorEventQueue = NULL ;
			}
		}
	}

    pthread_cond_broadcast( &g_AndroidSys.Cond ) ;
	g_AndroidSys.MutexLockIndex &= ~( 1 << 8 ) ;
    pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;

	// onDestroy ���I������̂�҂�
	while( g_AndroidSys.onDestroyEnd == 0 )
	{
		Thread_Sleep( 10 ) ;
	}

	// �\�t�g���s�p�X���b�h�Ƃ̒ʐM�p�̃p�C�v�����
	if( g_AndroidSys.MessageRead != 0 )
	{
		close( g_AndroidSys.MessageRead ) ;
		g_AndroidSys.MessageRead = 0 ;
	}
	if( g_AndroidSys.MessageWrite != 0 )
	{
		close( g_AndroidSys.MessageWrite ) ;
		g_AndroidSys.MessageWrite = 0 ;
	}

//	// Java �̎Q�ƂȂǂ����
//	TerminateJavaAndroidInfo( g_AndroidSys.NativeActivity->env ) ;

	// �~���[�e�b�N�X�Ə����ϐ��̎g�p���I��
	pthread_cond_destroy( &g_AndroidSys.Cond ) ;
	g_AndroidSys.MutexLockIndex &= ~( 1 << 20 ) ;
	pthread_mutex_destroy( &g_AndroidSys.Mutex ) ;
	pthread_mutex_destroy( &g_AndroidSys.NativeActivityMutex ) ;

	// �\�t�g���s�p�X���b�h���I���������ǂ����̃t���O�𗧂Ă�
    g_AndroidSys.SoftThreadDestroyed = 1 ;

    return NULL;
}

// �\�t�g���s�p�X���b�h�ɃR�}���h�𑗐M����
static void AndroidWriteCommand( int8_t cmd )
{
	write( g_AndroidSys.MessageWrite, &cmd, sizeof( cmd ) ) ;
}

// �\�t�g���s�p�X���b�h�� ActiveState ��ύX����R�}���h�𑗐M����
static void AndroidSetActivityState( int8_t cmd )
{
	pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
	g_AndroidSys.MutexLockIndex = 1 << 1 ;

	// �\�t�g���s�p�X���b�h�ɐV���� ActivityState �Ƃ��ăR�}���h�����̂܂ܑ��M
    AndroidWriteCommand( cmd ) ;

	// ActivityState ���X�V�����܂ő҂�
    while( g_AndroidSys.ActivityState != cmd )
	{
        pthread_cond_wait( &g_AndroidSys.Cond, &g_AndroidSys.Mutex ) ;
    }
	g_AndroidSys.MutexLockIndex &= ~( 1 << 1 ) ;
    pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
}

// onDestroy �̃R�[���o�b�N�֐�
static void onDestroy( ANativeActivity* NativeActivity )
{
	int i ;
	
	pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
	g_AndroidSys.MutexLockIndex = 1 << 9 ;

	// ���X�g����O��
	for( i = 0; i < g_AndroidSys.NativeActivityNum; i++ )
	{
		if( g_AndroidSys.NativeActivityBuffer[ i ] == NativeActivity )
		{
			g_AndroidSys.NativeActivityNum-- ;
			if( g_AndroidSys.NativeActivityNum > i )
			{
				_MEMMOVE( &g_AndroidSys.NativeActivityBuffer[ i ], &g_AndroidSys.NativeActivityBuffer[ i + 1 ], sizeof( ANativeActivity * ) * ( g_AndroidSys.NativeActivityNum - i ) ) ;
			}
			break ;
		}
	}

	if( g_AndroidSys.NativeActivity == NativeActivity )
	{
		// onDestroy ���Ă΂ꂽ�R�}���h���\�t�g���s�p�X���b�h�ɑ��M
		AndroidWriteCommand( DX_ANDR_CMD_DESTROY ) ;

		// ���b�Z�[�W���͂��܂ő҂�
		while( g_AndroidSys.DestroyRequested == 0 )
		{
			pthread_cond_wait( &g_AndroidSys.Cond, &g_AndroidSys.Mutex ) ;
		}

		// �\�t�g���s�p�X���b�h���I������܂ő҂�
	//	while( g_AndroidSys.SoftThreadDestroyed == 0 )
	//	{
	//		pthread_cond_wait( &g_AndroidSys.Cond, &g_AndroidSys.Mutex ) ;
	//	}
	//
	//	g_AndroidSys.SoftThreadDestroyed = 0 ;

		g_AndroidSys.MutexLockIndex &= ~( 1 << 9 ) ;
		pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;

	//	// �\�t�g���s�p�X���b�h�Ƃ̒ʐM�p�̃p�C�v�����
	//	if( g_AndroidSys.MessageRead != 0 )
	//	{
	//		close( g_AndroidSys.MessageRead ) ;
	//		g_AndroidSys.MessageRead = 0 ;
	//	}
	//	if( g_AndroidSys.MessageWrite != 0 )
	//	{
	//		close( g_AndroidSys.MessageWrite ) ;
	//		g_AndroidSys.MessageWrite = 0 ;
	//	}
	//
	//	// �~���[�e�b�N�X�Ə����ϐ��̎g�p���I��
	//	pthread_cond_destroy( &g_AndroidSys.Cond ) ;
	//	g_AndroidSys.MutexLockIndex &= ~( 1 << 20 ) ;
	//	pthread_mutex_destroy( &g_AndroidSys.Mutex ) ;

		pthread_mutex_lock( &g_AndroidSys.NativeActivityMutex ) ;

		// Java �̎Q�ƂȂǂ����
		TerminateJavaAndroidInfo( NativeActivity->env ) ;

		g_AndroidSys.NativeActivity = NULL ;

		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;

		g_AndroidSys.onDestroyEnd = 1 ;
	}
	else
	{
		g_AndroidSys.MutexLockIndex &= ~( 1 << 9 ) ;
		pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
	}
}

// onStart �̃R�[���o�b�N�֐�
static void onStart( ANativeActivity *NativeActivity )
{
	// �\�t�g���s�p�X���b�h�� ActivityState �� DX_ANDR_CMD_START �ɂ���
	AndroidSetActivityState( DX_ANDR_CMD_START ) ;
}

// onResume �̃R�[���o�b�N�֐�
static void onResume( ANativeActivity *NativeActivity )
{
	// �\�t�g���s�p�X���b�h�� ActivityState �� DX_ANDR_CMD_RESUME �ɂ���
    AndroidSetActivityState( DX_ANDR_CMD_RESUME ) ;

	// �X�e�[�^�X�o�[�ƃi�r�Q�[�V�����o�[���\���ɂ���
	if( SetAndroidWindowStyle( NativeActivity->env ) < 0 )
	{
		// ���s�����狌�o�[�W�����p�̃t���X�N���[�������������s����
		ANativeActivity_setWindowFlags( NativeActivity, AWINDOW_FLAG_FULLSCREEN, AWINDOW_FLAG_FULLSCREEN ) ;
	}

	ANativeActivity_showSoftInput( NativeActivity, ANATIVEACTIVITY_SHOW_SOFT_INPUT_IMPLICIT ) ;

//	StartInputStringDialogStatic( NativeActivity->env, "test" ) ;
}

// onPause �̃R�[���o�b�N�֐�
static void onPause( ANativeActivity *NativeActivity )
{
	// �\�t�g���s�p�X���b�h�� ActivityState �� DX_ANDR_CMD_PAUSE �ɂ���
	if( g_AndroidSys.NativeActivity == NativeActivity )
	{
		AndroidSetActivityState( DX_ANDR_CMD_PAUSE ) ;
	}
}

// onStop �̃R�[���o�b�N�֐�
static void onStop( ANativeActivity *NativeActivity )
{
	// �\�t�g���s�p�X���b�h�� ActivityState �� DX_ANDR_CMD_STOP �ɂ���
	if( g_AndroidSys.NativeActivity == NativeActivity )
	{
		AndroidSetActivityState( DX_ANDR_CMD_STOP ) ;
	}
}

// onWindowFocusChanged �̃R�[���o�b�N�֐�
static void onWindowFocusChanged( ANativeActivity *NativeActivity, int focused )
{
	// focused �ɂ���ă\�t�g���s�p�X���b�h�ɑ��M����R�}���h�𕪂���
    AndroidWriteCommand( focused ? DX_ANDR_CMD_GAINED_FOCUS : DX_ANDR_CMD_LOST_FOCUS ) ;
}

// onNativeWindowCreated �� onNativeWindowDestroyed �̏������s���֐�
static void onNativeWindowBase( ANativeWindow *NativeWindow )
{
    pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
	g_AndroidSys.MutexLockIndex = 1 << 10 ;

	// �V�����E�C���h�E�̃A�h���X���Z�b�g
    g_AndroidSys.NewNativeWindow = NativeWindow ;

	// �\�t�g���s�p�X���b�h�ɃE�C���h�E�̕ύX�R�}���h�𑗐M
	AndroidWriteCommand( DX_ANDR_CMD_WINDOW_CHANGED ) ;

	// �E�C���h�E�̕ύX�����f�����܂ő҂�
    while( g_AndroidSys.NativeWindow != g_AndroidSys.NewNativeWindow )
	{
        pthread_cond_wait( &g_AndroidSys.Cond, &g_AndroidSys.Mutex ) ;
    }
	g_AndroidSys.MutexLockIndex &= ~( 1 << 10 ) ;
    pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
}

// onNativeWindowCreated �̃R�[���o�b�N�֐�
static void onNativeWindowCreated( ANativeActivity *NativeActivity, ANativeWindow* NativeWindow )
{
	// onNativeWindowBase �֐��ŏ�������
	onNativeWindowBase( NativeWindow ) ;
}

// onNativeWindowDestroyed �̃R�[���o�b�N�֐�
static void onNativeWindowDestroyed( ANativeActivity *NativeActivity, ANativeWindow* NativeWindow )
{
	// onNativeWindowBase �֐��ŏ�������
	if( g_AndroidSys.NativeActivity == NativeActivity )
	{
		onNativeWindowBase( NULL ) ;
	}
}

// onInputQueueCreated �� onInputQueueDestroyed �̏������s���֐�
static void onInputQueueBase( AInputQueue *InputQueue )
{
    pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
	g_AndroidSys.MutexLockIndex = 1 << 2 ;

	// �V�������̓C�x���g���󂯎�邽�߂̃L���[�̃A�h���X���Z�b�g
    g_AndroidSys.NewInputQueue = InputQueue ;

	// �\�t�g���s�p�X���b�h�ɓ��̓C�x���g���󂯎�邽�߂̃L���[�̕ύX�R�}���h�𑗐M
    AndroidWriteCommand( DX_ANDR_CMD_INPUT_CHANGED ) ;

	// ���̓C�x���g���󂯎�邽�߂̃L���[�̕ύX�����f�����܂ő҂�
    while( g_AndroidSys.InputQueue != g_AndroidSys.NewInputQueue )
	{
        pthread_cond_wait( &g_AndroidSys.Cond, &g_AndroidSys.Mutex ) ;
    }
	g_AndroidSys.MutexLockIndex &= ~( 1 << 2 ) ;
    pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
}

// onInputQueueCreated �̃R�[���o�b�N�֐�
static void onInputQueueCreated( ANativeActivity *NativeActivity, AInputQueue *queue )
{
	// onInputQueueBase �ŏ�������
	onInputQueueBase( queue ) ;
}

// onInputQueueDestroyed �̃R�[���o�b�N�֐�
static void onInputQueueDestroyed( ANativeActivity *NativeActivity, AInputQueue *queue )
{
	// onInputQueueBase �ŏ�������
	if( g_AndroidSys.NativeActivity == NativeActivity )
	{
		onInputQueueBase( NULL ) ;
	}
}

// �O���[�o���ϐ����[������������
static void InitializeGlobalData( void )
{
	static int Flag = 0 ;

	// ����͎��s���Ȃ�
	if( Flag == 0 )
	{
		Flag = 1 ;
		return ;
	}

	memset( &g_AndroidSys, 0, sizeof( g_AndroidSys ) ) ;
	memset( &g_JavaAndroidInfo, 0, sizeof( g_JavaAndroidInfo ) ) ;
	memset( &DxSysData, 0, sizeof( DxSysData ) ) ;

#ifndef DX_NON_DXA
    memset( &DX_ArchiveDirData, 0, sizeof( DX_ArchiveDirData ) ) ;
#endif // DX_NON_DXA

#ifndef DX_NON_ASYNCLOAD
    memset( &GASyncLoadData, 0, sizeof( GASyncLoadData ) ) ;
#endif // DX_NON_ASYNCLOAD

#ifndef DX_NON_FONT
	memset( &FontSystem, 0, sizeof( FontSystem ) ) ;
#endif // DX_NON_FONT

#ifndef DX_NON_GRAPHICS
    memset( &GraphicsHardDataAndroid, 0, sizeof( GraphicsHardDataAndroid ) ) ;
	memset( &GraphicsSysData, 0, sizeof( GraphicsSysData ) ) ;

	#ifndef DX_NON_FILTER
		memset( &GraphFilterSystemInfoAndroid, 0, sizeof( GraphFilterSystemInfoAndroid ) ) ;
		memset( &GraphFilterShaderHandle, 0, sizeof( GraphFilterShaderHandle ) ) ;
	#endif // DX_NON_FILTER

	#ifndef DX_NON_MODEL
		memset( &MV1Man_Android, 0, sizeof( MV1Man_Android ) ) ;
		memset( &MV1Man, 0, sizeof( MV1Man ) ) ;
	#endif // DX_NON_MODEL

#endif // DX_NON_GRAPHICS

#ifndef DX_NON_LOG
    memset( &LogData_Android, 0, sizeof( LogData_Android ) ) ;
	memset( &LogData, 0, sizeof( LogData ) ) ;
#endif // DX_NON_LOG

#ifndef DX_NON_MASK
    memset( &MaskManageData_Android, 0, sizeof( MaskManageData_Android ) ) ;
	memset( &MaskManageData, 0, sizeof( MaskManageData ) ) ;
#endif // DX_NON_MASK

#ifndef DX_NON_NETWORK
	memset( &SockData, 0, sizeof( SockData ) ) ;
	memset( &HttpData, 0, sizeof( HttpData ) ) ;
#endif // DX_NON_NETWORK

#ifndef DX_NON_SOFTIMAGE
	memset( &SoftImageManage, 0, sizeof( SoftImageManage ) ) ;
#endif // DX_NON_SOFTIMAGE

#ifndef DX_NON_MOVIE
	memset( &MovieGraphManageData, 0, sizeof( MovieGraphManageData ) ) ;
#endif // DX_NON_MOVIE

#ifndef DX_NON_SOUND
	memset( &SoundSysData, 0, sizeof( SoundSysData ) ) ;
	memset( &MidiSystemData, 0, sizeof( MidiSystemData ) ) ;	
	memset( &GSoundConvertData, 0, sizeof( GSoundConvertData ) ) ;
#endif // DX_NON_SOUND

	memset( &MemImgManage, 0, sizeof( MemImgManage ) ) ;
	memset( &MemData, 0, sizeof( MemData ) ) ;

    memset( &g_BaseFuncSystem, 0, sizeof( g_BaseFuncSystem ) ) ;
    memset( &BaseImageManage, 0, sizeof( BaseImageManage ) ) ;
	memset( &g_CharCodeSystem, 0, sizeof( g_CharCodeSystem ) ) ;

	memset( &HandleManageArray, 0, sizeof( HandleManageArray ) ) ;

#ifndef DX_NON_INPUT
	memset( &InputSysData, 0, sizeof( InputSysData ) ) ;
#endif // DX_NON_INPUT	

#ifndef DX_NON_INPUTSTRING
	memset( &CharBuf, 0, sizeof( CharBuf ) ) ;
#endif // DX_NON_INPUTSTRING

//	size_t g_AndroidSysSize = sizeof( g_AndroidSys )  ;
//	size_t DxSysDataSize = sizeof( DxSysData )  ;
//	size_t DX_ArchiveDirDataSize = sizeof( DX_ArchiveDirData )  ;
//	size_t FontSystemSize = sizeof( FontSystem )  ;
//	size_t GraphicsHardDataAndroidSize = sizeof( GraphicsHardDataAndroid )  ;
//	size_t GraphicsSysDataSize = sizeof( GraphicsSysData )  ;
//	size_t GraphFilterSystemInfoAndroidSize = sizeof( GraphFilterSystemInfoAndroid )  ;
//	size_t GraphFilterShaderHandleSize = sizeof( GraphFilterShaderHandle )  ;
//	size_t MV1Man_AndroidSize = sizeof( MV1Man_Android )  ;
//	size_t MV1ManSize = sizeof( MV1Man )  ;
//	size_t LogData_AndroidSize = sizeof( LogData_Android )  ;
//	size_t LogDataSize = sizeof( LogData )  ;
//	size_t MaskManageData_AndroidSize = sizeof( MaskManageData_Android )  ;
//	size_t MaskManageDataSize = sizeof( MaskManageData )  ;
//	size_t SoftImageManageSize = sizeof( SoftImageManage )  ;
//	size_t MovieGraphManageDataSize = sizeof( MovieGraphManageData )  ;
//	size_t SoundSysDataSize = sizeof( SoundSysData )  ;
//	size_t MidiSystemDataSize = sizeof( MidiSystemData )  ;	
//	size_t GSoundConvertDataSize = sizeof( GSoundConvertData )  ;
//	size_t MemImgManageSize = sizeof( MemImgManage )  ;
//	size_t MemDataSize = sizeof( MemData )  ;
//	size_t g_BaseFuncSystemSize = sizeof( g_BaseFuncSystem )  ;
//	size_t BaseImageManageSize = sizeof( BaseImageManage )  ;
//	size_t g_CharCodeSystemSize = sizeof( g_CharCodeSystem )  ;
//	size_t HandleManageArraySize = sizeof( HandleManageArray )  ;
//	size_t InputSysDataSize = sizeof( InputSysData )  ;
//	size_t GRAPHICS_HARDWARE_ANDROID_SHADER_MODELSize = sizeof( GRAPHICS_HARDWARE_ANDROID_SHADER_MODEL ) ;
//	size_t GRAPHICS_HARDWARE_ANDROID_SHADER_BASE3DSize = sizeof( GRAPHICS_HARDWARE_ANDROID_SHADER_BASE3D ) ;
//	size_t GRAPHICS_HARDWARE_ANDROID_SHADER_BASESize = sizeof( GRAPHICS_HARDWARE_ANDROID_SHADER_BASE ) ;
}

// �\�t�g�̐V���� Activity ���쐬���ꂽ�Ƃ��ɌĂ΂��֐��A�����̃G���g���[�|�C���g
void ANativeActivity_onCreate( ANativeActivity *NativeActivity, void *savedState, size_t savedStateSize )
{
	int lDoubleStartFlag = g_AndroidRunFlag ;

	g_AndroidRunFlag = TRUE ;

	if( lDoubleStartFlag )
	{
		// ���C���X���b�h�̏I���������J�n����Ă�����I������̂�҂�
		if( g_AndroidSys.SoftThreadDestroyedStart != 0 ||
			g_AndroidSys.SoftThreadDestroyed != 0 )
		{
			while( g_AndroidSys.SoftThreadDestroyed == 0 )
			{
				Thread_Sleep( 10 ) ;
			}

			lDoubleStartFlag = FALSE ;
		}
	}

	// �O���[�o���ϐ��̏�����
	if( lDoubleStartFlag == FALSE )
	{
		InitializeGlobalData() ;
	}

	// �R�[���o�b�N�̓o�^
    NativeActivity->callbacks->onStart					= onStart ;
    NativeActivity->callbacks->onResume					= onResume ;
    NativeActivity->callbacks->onPause					= onPause ;
    NativeActivity->callbacks->onStop					= onStop ;
    NativeActivity->callbacks->onDestroy				= onDestroy ;
    NativeActivity->callbacks->onWindowFocusChanged		= onWindowFocusChanged ;
    NativeActivity->callbacks->onNativeWindowCreated	= onNativeWindowCreated ;
    NativeActivity->callbacks->onNativeWindowDestroyed	= onNativeWindowDestroyed ;
    NativeActivity->callbacks->onInputQueueCreated		= onInputQueueCreated ;
    NativeActivity->callbacks->onInputQueueDestroyed	= onInputQueueDestroyed ;

	// �V���O���C���X�^���X�̂ݑΉ��Ȃ̂ŁAinstance �ɂ� NULL ����
	NativeActivity->instance = NULL ;

	if( lDoubleStartFlag )
	{
		pthread_mutex_lock( &g_AndroidSys.Mutex ) ;

		// NativeActivity �̃A�h���X��ۑ�
		pthread_mutex_lock( &g_AndroidSys.NativeActivityMutex ) ;

		g_AndroidSys.NativeActivity = NativeActivity ;

		// Java����������
		SetupJavaAndroidInfo( NativeActivity->env ) ;

		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;

		g_AndroidSys.DestroyRequested = 0 ;

		// NativeActivity �A�h���X��ǉ�
		if( g_AndroidSys.NativeActivityNum >= g_AndroidSys.NativeActivityBufferLength )
		{
			ANativeActivity **NewNativeActivityBuffer ;
			int NewNativeActivityBufferLength = g_AndroidSys.NativeActivityBufferLength * 2 ;

			NewNativeActivityBuffer = ( ANativeActivity ** )malloc( sizeof( ANativeActivity * ) * NewNativeActivityBufferLength ) ;
			_MEMCPY( NewNativeActivityBuffer, g_AndroidSys.NativeActivityBuffer, sizeof( ANativeActivity * ) * g_AndroidSys.NativeActivityBufferLength ) ;
			free( g_AndroidSys.NativeActivityBuffer ) ;

			g_AndroidSys.NativeActivityBuffer = NewNativeActivityBuffer ;
			g_AndroidSys.NativeActivityBufferLength = NewNativeActivityBufferLength ;
		}
		g_AndroidSys.NativeActivityBuffer[ g_AndroidSys.NativeActivityNum ] = NativeActivity ;
		g_AndroidSys.NativeActivityNum ++ ;

		// �X�e�[�^�X�o�[�ƃi�r�Q�[�V�����o�[���\���ɂ���
		if( SetAndroidWindowStyle( NativeActivity->env ) < 0 )
		{
			// ���s�����狌�o�[�W�����p�̃t���X�N���[�������������s����
			ANativeActivity_setWindowFlags( NativeActivity, AWINDOW_FLAG_FULLSCREEN, AWINDOW_FLAG_FULLSCREEN ) ;
		}

		pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
	}
	else
	{
		// NativeActivity �̃A�h���X��ۑ�
		g_AndroidSys.NativeActivity = NativeActivity ;

		// NativeActivity �A�h���X�i�[�p�o�b�t�@�̊m��
		g_AndroidSys.NativeActivityBufferLength = 16 ;
		g_AndroidSys.NativeActivityBuffer = ( ANativeActivity ** )malloc( sizeof( ANativeActivity * ) * g_AndroidSys.NativeActivityBufferLength ) ;
		g_AndroidSys.NativeActivityBuffer[ 0 ] = NativeActivity ;
		g_AndroidSys.NativeActivityNum = 1 ;

		// Java����������
		SetupJavaAndroidInfo( NativeActivity->env ) ;

		// �����_���W����������
#ifndef DX_NON_MERSENNE_TWISTER
		srandMT( ( unsigned int )NS_GetNowCount() ) ;
#else
		srand( NS_GetNowCount() ) ;
#endif

		// �~���[�e�b�N�X�g�����ϐ���������
	    pthread_mutex_init( &g_AndroidSys.Mutex, NULL ) ;
		pthread_cond_init(  &g_AndroidSys.Cond,  NULL ) ;
		pthread_mutex_init( &g_AndroidSys.NativeActivityMutex, NULL ) ;

		// �\�t�g���s�p�X���b�h�Ƃ̒ʐM�p�̃p�C�v����
		int msgpipe[ 2 ] ;
		if( pipe( msgpipe ) )
		{
			// �p�C�v�������s
			LOGE( "could not create pipe: %s", strerror( errno ) ) ;
			return ;
		}
		g_AndroidSys.MessageRead  = msgpipe[ 0 ] ;
		g_AndroidSys.MessageWrite = msgpipe[ 1 ] ;

		// �\�t�g���s�p�X���b�h����
		pthread_attr_t attr ;
		pthread_attr_init( &attr ) ;
		pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_DETACHED ) ;
		pthread_attr_setstacksize( &attr, 4 * 1024 * 1024 ) ;	// �X�^�b�N�T�C�Y�� 4MB
		pthread_create( &g_AndroidSys.SoftThread, &attr, android_app_entry, NULL ) ;

		// �X���b�h�̊J�n��ҋ@
		pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
		g_AndroidSys.MutexLockIndex = 1 << 3 ;
		while( g_AndroidSys.SoftThreadRunning == 0 )
		{
			pthread_cond_wait( &g_AndroidSys.Cond, &g_AndroidSys.Mutex ) ;
		}
		g_AndroidSys.MutexLockIndex &= ~( 1 << 3 ) ;
		pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
	}
}












#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// �E�C���h�E�Y�̃��b�Z�[�W���[�v�ɑ��鏈�����s��
extern int NS_ProcessMessage( void )
{
	static int EndFlag = FALSE ;

	// �����t���O�������Ă�����Ȃɂ������I��
	if( EndFlag )
	{
		return 0 ;
	}

	// �t�@�C�������̎����I�������s��
//	ReadOnlyFileAccessProcessAll() ;

#ifndef DX_NON_SOUND
	{
		// �T�E���h�̎����I�������s��
//		NS_ProcessStreamSoundMemAll() ;
//		ST_SoftSoundPlayerProcessAll() ;
		ProcessPlayFinishDeleteSoundMemAll() ;
//		SoundBuffer_Apply_StopSoundBufferList() ;
//		ProcessPlay3DSoundMemAll() ;
	}
#endif // DX_NON_SOUND

#ifndef DX_NON_ASYNCLOAD
	// ���C���X���b�h����������񓯊��ǂݍ��݂̏������s��
	ProcessASyncLoadRequestMainThread() ;
#endif // DX_NON_ASYNCLOAD

	// ���t�̎����I�������s��
#ifndef DX_NON_SOUND
	NS_ProcessMusicMem() ;
#endif // DX_NON_SOUND

#ifndef DX_NON_INPUT
	// �L�[�{�[�h���͂̍X�V�������s��
	UpdateKeyboardInputState( FALSE ) ;

	// �p�b�h�̎����I�������s��
	JoypadEffectProcess() ;
#endif // DX_NON_INPUT

#ifndef DX_NON_NETWORK
	// �ʐM�֌W�̃��b�Z�[�W�������s��
	NS_ProcessNetMessage( TRUE ) ;
#endif

	// �������֌W�̎����I�������s��
	MemoryProcess() ;

#ifndef DX_NON_GRAPHICS
	// ��ʊ֌W�̎����������s��
	Graphics_Android_FrontScreenProcess() ;
#endif // DX_NON_GRAPHICS

#ifndef DX_NON_KEYEX
	// �L�[���͏������s��
	{
		// �t���O�����Ă�
		EndFlag = TRUE ;

		NS_ProcessActKeyInput() ;

		// �t���O��|��
		EndFlag = FALSE ;
	}
#endif

	// �C�x���g�������[�v
	for(;;)
	{
		int   events ;
		void *source ;

		// �C�x���g�̎擾�A�\�t�g���s�p�X���b�h���҂���Ԃ̏ꍇ�̓C�x���g������܂ŉ��X�Ƒ҂�
		int ident = ALooper_pollAll( ( g_AndroidSys.SoftThreadWait && g_AndroidSys.NonActiveRunFlag == FALSE ) ? -1 : 0, NULL, &events, &source ) ;
		if( ident < 0 ) 
		{
			break ;
		}

		// �C�x���gID ���ɏ����𕪊�
		switch( ident )
		{
		// ���C���X���b�h���瑗���Ă����R�}���h�̏ꍇ
		case DX_LOOPER_ID_MAIN :
			{
				int8_t cmd ;

				// ���C���X���b�h���瑗���Ă����R�}���h���擾
				if( read( g_AndroidSys.MessageRead, &cmd, sizeof( cmd ) ) == sizeof( cmd ) )
				{
					// �R�}���h�̎�ނɂ���ď����𕪊�
					switch( cmd ) 
					{
					// ���̓C�x���g���󂯎��L���[�̕ύX
					case DX_ANDR_CMD_INPUT_CHANGED :
						pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
						g_AndroidSys.MutexLockIndex = 1 << 4 ;

						// ���܂ł̓��̓C�x���g�󂯎��L���[�� Looper ����f�^�b�`
						if( g_AndroidSys.InputQueue != NULL )
						{
							AInputQueue_detachLooper( ( AInputQueue * )g_AndroidSys.InputQueue ) ;
						}

						// �V�������̓C�x���g�󂯎��L���[�̃A�h���X��ۑ�
						g_AndroidSys.InputQueue = g_AndroidSys.NewInputQueue ;

						// �V�������̓C�x���g�󂯎��L���[�� Looper �ɓo�^
						if( g_AndroidSys.InputQueue != NULL )
						{
							AInputQueue_attachLooper(
								( AInputQueue * )g_AndroidSys.InputQueue,
								g_AndroidSys.Looper,
								DX_LOOPER_ID_INPUT,
								NULL,
								NULL
							) ;
						}

						pthread_cond_broadcast( &g_AndroidSys.Cond ) ;
						g_AndroidSys.MutexLockIndex &= ~( 1 << 4 ) ;
						pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
						break;

					// ActivityState ��ύX���邾���̃R�}���h
					case DX_ANDR_CMD_RESUME :
					case DX_ANDR_CMD_START :
					case DX_ANDR_CMD_PAUSE :
					case DX_ANDR_CMD_STOP :
						pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
						g_AndroidSys.MutexLockIndex = 1 << 5 ;

			#ifndef DX_NON_INPUT
						// �����^�b�`���Ă��Ȃ����Ƃɂ���
						{
							TOUCHINPUTDATA TouchInputData ;

							TouchInputData.PointNum = 0 ;
							TouchInputData.Time = ( LONGLONG )NS_GetNowCount() ;

							AddTouchInputData( &TouchInputData ) ;
						}
			#endif // DX_NON_INPUT

						// �V���� ActivityState ��ۑ�
						g_AndroidSys.ActivityState = cmd ;

						pthread_cond_broadcast( &g_AndroidSys.Cond ) ;
						g_AndroidSys.MutexLockIndex &= ~( 1 << 5 ) ;
						pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
						break;

					// onDestroy ���Ă΂ꂽ
					case DX_ANDR_CMD_DESTROY :
						// onDestroy ���Ă΂ꂽ�t���O�𗧂Ă�
						pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
						g_AndroidSys.DestroyRequested = 1 ;
						pthread_cond_broadcast( &g_AndroidSys.Cond ) ;
						pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;

						// onDestroy ���Ă΂ꂽ�������L�^
						g_AndroidSys.DestroyRequestedTime = NS_GetNowCount() ;
						break ;

					// �E�C���h�E�̕ύX
					case DX_ANDR_CMD_WINDOW_CHANGED :
						pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
						g_AndroidSys.MutexLockIndex = 1 << 6 ;

						// �����̃E�C���h�E������ꍇ�̓E�C���h�E�̌�n�����s��
						if( g_AndroidSys.NativeWindow != NULL )
						{
							pthread_cond_broadcast( &g_AndroidSys.Cond ) ;

					#ifndef DX_NON_GRAPHICS
							if( DxSysData.NotDrawFlag == FALSE )
							{
					#ifndef DX_NON_MASK
								// �}�X�N�T�[�t�F�X���ꎞ�폜
								Mask_ReleaseSurface() ;
					#endif // DX_NON_MASK
								// �O���t�B�b�N�n���h�������� OpenGL ES �I�u�W�F�N�g�̉��
								Graphics_Android_ReleaseObjectAll() ;

								// �V�X�e�������� OpenGL ES �I�u�W�F�N�g�̉��
								Graphics_Android_Terminate() ;
							}
					#endif // DX_NON_GRAPHICS

							g_AndroidSys.NativeWindow = NULL ;
						}

						// �V�����E�C���h�E������ꍇ�̓E�C���h�E�̏��������s��
						if( g_AndroidSys.NewNativeWindow != NULL )
						{
							g_AndroidSys.NativeWindow = g_AndroidSys.NewNativeWindow ;

				#ifndef DX_NON_GRAPHICS
							if( DxSysData.DxLib_InitializeFlag )
							{
								// �O���t�B�b�N�V�X�e���̕��A����
								NS_RestoreGraphSystem() ;

								if( g_AndroidSys.SoundAndMoviePause == TRUE )
								{
									g_AndroidSys.SoundAndMoviePause = FALSE ;

					#ifndef DX_NON_MOVIE
									// ���[�r�[�O���t�B�b�N�̍Đ���Ԃ�߂�
									PlayMovieAll() ;
					#endif

					#ifndef DX_NON_SOUND
									// �T�E���h�̍Đ����ĊJ����
									PauseSoundMemAll( FALSE ) ;
									PauseSoftSoundAll( FALSE ) ;
					#endif // DX_NON_SOUND

									// �R�[���o�b�N�֐����o�^����Ă���ꍇ�͌Ă�
									if( g_AndroidSys.GainedFocusCallbackFunction != NULL )
									{
										g_AndroidSys.GainedFocusCallbackFunction( ( void * )g_AndroidSys.GainedFocusCallbackFunctionData ) ;
									}
								}

								// �\�t�g���s�p�X���b�h��҂���Ԃ������
								g_AndroidSys.SoftThreadWait = 0 ;
							}
				#endif // DX_NON_GRAPHICS
						}

						pthread_cond_broadcast( &g_AndroidSys.Cond ) ;
						g_AndroidSys.MutexLockIndex &= ~( 1 << 6 ) ;
						pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
						break ;

					// �E�C���h�E���A�N�e�B�u�ɂȂ���
					case DX_ANDR_CMD_GAINED_FOCUS :
						// �Z���T�[��L���ɂ���
						for( int i = 0 ; i < DX_ANDROID_SENSOR_NUM ; i ++ )
						{
							if( g_AndroidSys.SensorInfos[ i ].Sensor           != NULL &&
								g_AndroidSys.SensorInfos[ i ].SensorEventQueue != NULL )
							{
								ASensorEventQueue_enableSensor(
									g_AndroidSys.SensorInfos[ i ].SensorEventQueue,
									g_AndroidSys.SensorInfos[ i ].Sensor
								) ;

								ASensorEventQueue_setEventRate(
									g_AndroidSys.SensorInfos[ i ].SensorEventQueue,
									g_AndroidSys.SensorInfos[ i ].Sensor, 
									( 1000L / 60 ) * 1000
								) ;
							}
						}

						if( g_AndroidSys.SoundAndMoviePause == TRUE )
						{
							g_AndroidSys.SoundAndMoviePause = FALSE ;

			#ifndef DX_NON_MOVIE
							// ���[�r�[�O���t�B�b�N�̍Đ���Ԃ�߂�
							PlayMovieAll() ;
			#endif

			#ifndef DX_NON_SOUND
							// �T�E���h�̍Đ����ĊJ����
							PauseSoundMemAll( FALSE ) ;
							PauseSoftSoundAll( FALSE ) ;
			#endif // DX_NON_SOUND

							// �R�[���o�b�N�֐����o�^����Ă���ꍇ�͌Ă�
							if( g_AndroidSys.GainedFocusCallbackFunction != NULL )
							{
								g_AndroidSys.GainedFocusCallbackFunction( ( void * )g_AndroidSys.GainedFocusCallbackFunctionData ) ;
							}
						}

						// �\�t�g���s�p�X���b�h��҂���Ԃ������
						g_AndroidSys.SoftThreadWait = 0 ;
						break ;

					// �E�C���h�E����A�N�e�B�u�ɂȂ���
					case DX_ANDR_CMD_LOST_FOCUS :
						// �Z���T�[�𖳌��ɂ���
						for( int i = 0 ; i < DX_ANDROID_SENSOR_NUM ; i ++ )
						{
							if( g_AndroidSys.SensorInfos[ i ].Sensor           != NULL &&
								g_AndroidSys.SensorInfos[ i ].SensorEventQueue != NULL )
							{
								ASensorEventQueue_disableSensor(
									g_AndroidSys.SensorInfos[ i ].SensorEventQueue,
									g_AndroidSys.SensorInfos[ i ].Sensor
								) ;
							}
						}

						if( g_AndroidSys.SoundAndMoviePause == FALSE )
						{
							g_AndroidSys.SoundAndMoviePause = TRUE ;

				#ifndef DX_NON_MOVIE
							// ���[�r�[�O���t�B�b�N�̍Đ���Ԃ��Ƃ߂�
							PauseMovieAll() ;
				#endif

				#ifndef DX_NON_SOUND
							// �T�E���h�̍Đ����~�߂�
							PauseSoundMemAll( TRUE ) ;
							PauseSoftSoundAll( TRUE ) ;
				#endif // DX_NON_SOUND

							// �R�[���o�b�N�֐����o�^����Ă���ꍇ�͌Ă�
							if( g_AndroidSys.LostFocusCallbackFunction != NULL )
							{
								g_AndroidSys.LostFocusCallbackFunction( ( void * )g_AndroidSys.LostFocusCallbackFunctionData ) ;
							}
						}

						// �\�t�g���s�p�X���b�h��҂���Ԃɂ���
						g_AndroidSys.SoftThreadWait = 1 ;
						break ;
					}
				}
			}
			break ;

		// ���̓C�x���g�̏ꍇ
		case DX_LOOPER_ID_INPUT :
			{
				AInputEvent *event = NULL ;

				// ���̓C�x���g������ꍇ�̓��[�v
				while( AInputQueue_getEvent( ( AInputQueue * )g_AndroidSys.InputQueue, &event ) >= 0 )
				{
					// ���̓C�x���g���f�B�X�p�b�`
					if( AInputQueue_preDispatchEvent( ( AInputQueue * )g_AndroidSys.InputQueue, event ) )
					{
						continue ;
					}
					int32_t handled = 0 ;

			#ifndef DX_NON_INPUT
					// ���̓C�x���g������
					handled = ProcessInputEvent( event ) ;
			#endif // DX_NON_INPUT

					// ���̓C�x���g��������Ԃɂ���
					AInputQueue_finishEvent( ( AInputQueue * )g_AndroidSys.InputQueue, event, handled ) ;
				}
			}
			break ;

		// �Z���T�[�C�x���g�̏ꍇ
		case DX_LOOPER_ID_SENSOR_ACCELEROMETER :
		case DX_LOOPER_ID_SENSOR_MAGNETIC_FIELD :
		case DX_LOOPER_ID_SENSOR_GYROSCOPE :
		case DX_LOOPER_ID_SENSOR_LIGHT :
		case DX_LOOPER_ID_SENSOR_PROXIMITY :
		case DX_LOOPER_ID_SENSOR_PRESSURE :
		case DX_LOOPER_ID_SENSOR_AMBIENT_TEMPERATURE :
			{
				int SensorType = ident - DX_LOOPER_ID_SENSOR_ACCELEROMETER ;

				if( g_AndroidSys.SensorInfos[ SensorType ].Sensor != NULL &&
					g_AndroidSys.SensorInfos[ SensorType ].SensorEventQueue != NULL )
				{
					// �C�x���g�����擾
					while( ASensorEventQueue_getEvents( g_AndroidSys.SensorInfos[ SensorType ].SensorEventQueue, &g_AndroidSys.SensorInfos[ SensorType ].SensorEvent, 1 ) > 0 ){}
				}
			}
			break ;
		}

		// onDestroy ���Ă΂�Ă��邩�Ag_AndroidSys.SoftThreadDestroyedStart �� 1 �̏ꍇ�̓��[�v�𔲂���
		if( g_AndroidSys.DestroyRequested != 0 ||
			g_AndroidSys.SoftThreadDestroyedStart != 0 )
		{
			break ;
		}
	}

	// g_AndroidSys.SoftThreadDestroyedStart �� 1 �������� -1 ��Ԃ�
	if( g_AndroidSys.SoftThreadDestroyedStart != 0 )
	{
		return -1 ;
	}
	else
	// onDestroy ���Ă΂�Ď��� NativeActivity �����s���ꂸ�A���� 0.5�b�ȏ�o�߂����ꍇ�� -1 ��Ԃ�
	if( g_AndroidSys.DestroyRequested != 0 )
	{
		int ActivityNum = 0 ;

		pthread_mutex_lock( &g_AndroidSys.Mutex ) ;
		ActivityNum = g_AndroidSys.NativeActivityNum ;

		if( ActivityNum == 0 )
		{
			int NowTime = NS_GetNowCount() ;
			if( TIME_DISTANCE( NowTime, g_AndroidSys.DestroyRequestedTime ) > 500 )
			{
				// �I���J�n�t���O�𗧂Ă�
				g_AndroidSys.SoftThreadDestroyedStart = 1 ;

				pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
				return -1 ;
			}
		}
		else
		{
			pthread_cond_broadcast( &g_AndroidSys.Cond ) ;
		}

		pthread_mutex_unlock( &g_AndroidSys.Mutex ) ;
	}

	// �ʏ�I��
	return 0 ;
}

// �A�v�����A�N�e�B�u�ł͂Ȃ���Ԃł������𑱍s���邩�A�t���O���Z�b�g����
extern int NS_SetAlwaysRunFlag( int Flag )
{
	// �t���O���Z�b�g
	g_AndroidSys.NonActiveRunFlag = Flag ;
	
	// �I��
	return 0 ;
}

// �\�t�g�̃f�[�^�ۑ��p�̃f�B���N�g���p�X���擾����
extern int GetInternalDataPath( TCHAR *PathBuffer, int PathBufferBytes )
{
	TCHAR *TempBuffer ;
	int Result ;

	if( g_AndroidSys.NativeActivity == NULL ||
		g_AndroidSys.NativeActivity->internalDataPath == NULL )
	{
		return -1 ;
	}

	if( PathBuffer == NULL )
	{
		size_t Length ;
		size_t BufferSize ;

		Length = CL_strlen( DX_CHARCODEFORMAT_UTF8, g_AndroidSys.NativeActivity->internalDataPath ) ;
		BufferSize = ( Length + 1 ) * 16 ;
		TempBuffer = ( TCHAR * )DXALLOC( BufferSize ) ;
		if( TempBuffer == NULL )
		{
			return -1 ;
		}
		Result = ConvString( g_AndroidSys.NativeActivity->internalDataPath, DX_CHARCODEFORMAT_UTF8, TempBuffer, BufferSize, _TCHARCODEFORMAT ) ;
		DXFREE( TempBuffer ) ;
		TempBuffer = NULL ;
	}
	else
	{
		Result = ConvString( g_AndroidSys.NativeActivity->internalDataPath, DX_CHARCODEFORMAT_UTF8, PathBuffer, PathBufferBytes, _TCHARCODEFORMAT ) ;
	}

	// �I��
	return Result ;
}

// �\�t�g�̊O���f�[�^�ۑ��p�̃f�B���N�g���p�X���擾����
extern int GetExternalDataPath( TCHAR *PathBuffer, int PathBufferBytes )
{
	TCHAR *TempBuffer ;
	int Result ;

	if( g_AndroidSys.NativeActivity == NULL ||
		g_AndroidSys.NativeActivity->externalDataPath == NULL )
	{
		return -1 ;
	}

	if( PathBuffer == NULL )
	{
		size_t Length ;
		size_t BufferSize ;

		Length = CL_strlen( DX_CHARCODEFORMAT_UTF8, g_AndroidSys.NativeActivity->externalDataPath ) ;
		BufferSize = ( Length + 1 ) * 16 ;
		TempBuffer = ( TCHAR * )DXALLOC( BufferSize ) ;
		if( TempBuffer == NULL )
		{
			return -1 ;
		}
		Result = ConvString( g_AndroidSys.NativeActivity->externalDataPath, DX_CHARCODEFORMAT_UTF8, TempBuffer, BufferSize, _TCHARCODEFORMAT ) ;
		DXFREE( TempBuffer ) ;
		TempBuffer = NULL ;
	}
	else
	{
		Result = ConvString( g_AndroidSys.NativeActivity->externalDataPath, DX_CHARCODEFORMAT_UTF8, PathBuffer, PathBufferBytes, _TCHARCODEFORMAT ) ;
	}

	// �I��
	return Result ;
}

// �[���ɐݒ肳��Ă��錾����擾����
extern int GetLanguage( TCHAR *StringBuffer, int StringBufferSize )
{
	JNIEnv *env ;
	jobject object_Locale = NULL ;
	jstring jstring_Language = NULL ;
	int res = -1 ;
	TCHAR *LanguageName = NULL ;

	pthread_mutex_lock( &g_AndroidSys.NativeActivityMutex ) ;
	
	if( g_AndroidSys.NativeActivity == NULL )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// JavaVM �ƃ\�t�g���s�p�X���b�h���֘A�t��
	if( g_AndroidSys.NativeActivity->vm->AttachCurrentThreadAsDaemon( &env, NULL ) != JNI_OK )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// ���P�[���̎擾
	object_Locale = env->CallStaticObjectMethod( JAVAANDR.class_Locale, JAVAANDR.methodID_Locale_getDefault ) ;
	if( object_Locale == NULL )
	{
		goto END ;
	}

	// ����̎擾
	jstring_Language = ( jstring )env->CallObjectMethod( object_Locale, JAVAANDR.methodID_Locale_getLanguage );
	if( jstring_Language == NULL )
	{
		goto END ;
	}

	// TCHAR ������ɕϊ�
	if( Java_Create_TCHAR_string_From_jstring( env, jstring_Language, &LanguageName ) < 0 )
	{
		goto END ;
	}

	// �o�b�t�@�ɃR�s�[����
	if( StringBuffer != NULL )
	{
		CL_strcpy_s( _TCHARCODEFORMAT, StringBuffer, StringBufferSize, LanguageName ) ;  
	}

	res = ( CL_strlen( _TCHARCODEFORMAT, StringBuffer ) + 1 ) * GetCharCodeFormatUnitSize( _TCHARCODEFORMAT ) ;

END :

	if( LanguageName != NULL )
	{
		DXFREE( LanguageName ) ;
		LanguageName = NULL ;
	}

	if( object_Locale != NULL )
	{
		env->DeleteLocalRef( object_Locale ) ;
		object_Locale = NULL ;
	}

	if( jstring_Language != NULL )
	{
		env->DeleteLocalRef( jstring_Language ) ;
		jstring_Language = NULL ;
	}

	// JavaVM �Ƃ��̃X���b�h�̊֘A�t���I��
	g_AndroidSys.NativeActivity->vm->DetachCurrentThread() ;

	pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;

	return res ;
}

// �[���ɐݒ肳��Ă��鍑���擾����( �߂�l�@-1�F�G���[�@0�ȏ�F����������̊i�[�ɕK�v�ȃo�C�g�� )
extern int GetCountry( TCHAR *StringBuffer, int StringBufferSize )
{
	JNIEnv *env ;
	jobject object_Locale = NULL ;
	jstring jstring_Country = NULL ;
	int res = -1 ;
	TCHAR *CountryName = NULL ;

	pthread_mutex_lock( &g_AndroidSys.NativeActivityMutex ) ;
	
	if( g_AndroidSys.NativeActivity == NULL )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// JavaVM �ƃ\�t�g���s�p�X���b�h���֘A�t��
	if( g_AndroidSys.NativeActivity->vm->AttachCurrentThreadAsDaemon( &env, NULL ) != JNI_OK )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// ���P�[���̎擾
	object_Locale = env->CallStaticObjectMethod( JAVAANDR.class_Locale, JAVAANDR.methodID_Locale_getDefault ) ;
	if( object_Locale == NULL )
	{
		goto END ;
	}

	// ����̎擾
	jstring_Country = ( jstring )env->CallObjectMethod( object_Locale, JAVAANDR.methodID_Locale_getCountry );
	if( jstring_Country == NULL )
	{
		goto END ;
	}

	// TCHAR ������ɕϊ�
	if( Java_Create_TCHAR_string_From_jstring( env, jstring_Country, &CountryName ) < 0 )
	{
		goto END ;
	}

	// �o�b�t�@�ɃR�s�[����
	if( StringBuffer != NULL )
	{
		CL_strcpy_s( _TCHARCODEFORMAT, StringBuffer, StringBufferSize, CountryName ) ;  
	}

	res = ( CL_strlen( _TCHARCODEFORMAT, StringBuffer ) + 1 ) * GetCharCodeFormatUnitSize( _TCHARCODEFORMAT ) ;

END :

	if( CountryName != NULL )
	{
		DXFREE( CountryName ) ;
		CountryName = NULL ;
	}

	if( object_Locale != NULL )
	{
		env->DeleteLocalRef( object_Locale ) ;
		object_Locale = NULL ;
	}

	if( jstring_Country != NULL )
	{
		env->DeleteLocalRef( jstring_Country ) ;
		jstring_Country = NULL ;
	}

	// JavaVM �Ƃ��̃X���b�h�̊֘A�t���I��
	g_AndroidSys.NativeActivity->vm->DetachCurrentThread() ;

	pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;

	return res ;
}

// ��������̓_�C�A���O��\������
static int StartInputStringDialogStatic( JNIEnv *env, const TCHAR *Title )
{
	int res = -1 ;

	// EditText �̍쐬
	{
		jobject object_EditText = env->NewObject(
			g_JavaAndroidInfo.class_EditText,
			g_JavaAndroidInfo.methodID_EditText_newEditText,
			g_AndroidSys.NativeActivity->clazz
		) ;
		if( object_EditText != NULL )
		{
			g_AndroidSys.object_EditText = env->NewGlobalRef( object_EditText ) ;
			env->DeleteLocalRef( object_EditText ) ;
			object_EditText = NULL ;
		}
		if( g_AndroidSys.object_EditText == NULL )
		{
			DXST_LOGFILEFMT_ADDUTF16LE(( "\x45\x00\x64\x00\x69\x00\x74\x00\x54\x00\x65\x00\x78\x00\x74\x00\x20\x00\x6e\x30\x20\x00\x6e\x00\x65\x00\x77\x00\x20\x00\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x00"/*@ L"EditText �� new �Ɏ��s���܂���" @*/ )) ;
			goto END ;
		}
	}

	// AlertDialog.Builder �̍쐬
	{
		jobject object_AlertDialog_Builder = env->NewObject(
			g_JavaAndroidInfo.class_AlertDialog_Builder,
			g_JavaAndroidInfo.methodID_AlertDialog_Builder_newAlertDialog_Builder,
			g_AndroidSys.NativeActivity->clazz
		) ;
		if( object_AlertDialog_Builder != NULL )
		{
			g_AndroidSys.object_AlertDialog_Builder = env->NewGlobalRef( object_AlertDialog_Builder ) ;
			env->DeleteLocalRef( object_AlertDialog_Builder ) ;
			object_AlertDialog_Builder = NULL ;
		}
		if( g_AndroidSys.object_AlertDialog_Builder == NULL )
		{
			DXST_LOGFILEFMT_ADDUTF16LE(( "\x41\x00\x6c\x00\x65\x00\x72\x00\x74\x00\x44\x00\x69\x00\x61\x00\x6c\x00\x6f\x00\x67\x00\x2e\x00\x42\x00\x75\x00\x69\x00\x6c\x00\x64\x00\x65\x00\x72\x00\x20\x00\x6e\x30\x20\x00\x6e\x00\x65\x00\x77\x00\x20\x00\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x00"/*@ L"AlertDialog.Builder �� new �Ɏ��s���܂���" @*/ )) ;
			goto END ;
		}
	}

	// AlertDialog.Builder �̐ݒ�
	{
		jobject object_AlertDialog_Builder ;

		// �^�C�g���̐ݒ�
		{
			char UTF16LE_Title[ 256 * 4 ] ;
			int UTF16LE_TitleLength ;
		
			UTF16LE_TitleLength = ConvString( ( char * )Title, _TCHARCODEFORMAT, UTF16LE_Title, sizeof( UTF16LE_Title ), DX_CHARCODEFORMAT_UTF16LE ) ;
			UTF16LE_TitleLength = UTF16LE_TitleLength / 2 - 1 ;
			if( UTF16LE_TitleLength > 0 )
			{
				jstring jstring_Title = env->NewString( ( jchar * )UTF16LE_Title, ( jsize )UTF16LE_TitleLength ) ;
				if( jstring_Title != NULL )
				{
					object_AlertDialog_Builder = env->CallObjectMethod(
						g_AndroidSys.object_AlertDialog_Builder,
						g_JavaAndroidInfo.methodID_AlertDialog_Builder_setTitle,
						jstring_Title
					) ;
					if( object_AlertDialog_Builder != NULL )
					{
						env->DeleteLocalRef( object_AlertDialog_Builder ) ;
						object_AlertDialog_Builder = NULL ;
					}

					env->DeleteLocalRef( jstring_Title ) ;
					jstring_Title = NULL ;
				}
			}
		}

		// �r���[�̐ݒ�
		object_AlertDialog_Builder = env->CallObjectMethod(
			g_AndroidSys.object_AlertDialog_Builder,
			g_JavaAndroidInfo.methodID_AlertDialog_Builder_setView,
			g_AndroidSys.object_EditText
		) ;
		if( object_AlertDialog_Builder != NULL )
		{
			env->DeleteLocalRef( object_AlertDialog_Builder ) ;
			object_AlertDialog_Builder = NULL ;
		}

		// OK�{�^���̐ݒ�
		{
			jstring jstring_OK = env->NewStringUTF( "OK" ) ;
			if( jstring_OK != NULL )
			{
				object_AlertDialog_Builder = env->CallObjectMethod(
					g_AndroidSys.object_AlertDialog_Builder,
					g_JavaAndroidInfo.methodID_AlertDialog_Builder_setPositiveButton,
					jstring_OK,
					NULL
				) ;
				if( object_AlertDialog_Builder != NULL )
				{
					env->DeleteLocalRef( object_AlertDialog_Builder ) ;
					object_AlertDialog_Builder = NULL ;
				}

				env->DeleteLocalRef( jstring_OK ) ;
				jstring_OK = NULL ;
			}
		}
	}

	// �_�C�A���O��\��
	{
		jobject object_Dialog = env->CallObjectMethod(
			g_AndroidSys.object_AlertDialog_Builder,
			g_JavaAndroidInfo.methodID_AlertDialog_Builder_show
		) ;
		if( object_Dialog != NULL )
		{
			g_AndroidSys.object_Dialog = env->NewGlobalRef( object_Dialog ) ;
			env->DeleteLocalRef( object_Dialog ) ;
			object_Dialog = NULL ;
		}
		if( g_AndroidSys.object_Dialog == NULL )
		{
			DXST_LOGFILEFMT_ADDUTF16LE(( "\x41\x00\x6c\x00\x65\x00\x72\x00\x74\x00\x44\x00\x69\x00\x61\x00\x6c\x00\x6f\x00\x67\x00\x2e\x00\x42\x00\x75\x00\x69\x00\x6c\x00\x64\x00\x65\x00\x72\x00\x20\x00\x6e\x30\x20\x00\x73\x00\x68\x00\x6f\x00\x77\x00\x20\x00\x4c\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x00"/*@ L"AlertDialog.Builder �� show �����s���܂���" @*/ )) ;
			goto END ;
		}
	}

	res = 0 ;

END :

	if( res < 0 )
	{
		if( g_AndroidSys.object_EditText != NULL )
		{
			env->DeleteGlobalRef( g_AndroidSys.object_EditText ) ;
			g_AndroidSys.object_EditText = NULL ;
		}

		if( g_AndroidSys.object_Dialog != NULL )
		{
			env->DeleteGlobalRef( g_AndroidSys.object_Dialog ) ;
			g_AndroidSys.object_Dialog = NULL ;
		}

		if( g_AndroidSys.object_AlertDialog_Builder != NULL )
		{
			env->DeleteGlobalRef( g_AndroidSys.object_AlertDialog_Builder ) ;
			g_AndroidSys.object_AlertDialog_Builder = NULL ;
		}
	}

	return res ;
}

// ��������̓_�C�A���O��\������
extern int StartInputStringDialog( const TCHAR *Title )
{
	JNIEnv *env ;
	int res = -1 ;

	// JavaVM �ƃ\�t�g���s�p�X���b�h���֘A�t��
	if( g_AndroidSys.NativeActivity->vm->AttachCurrentThreadAsDaemon( &env, NULL ) != JNI_OK )
	{
		return res ;
	}

	res = StartInputStringDialogStatic( env, Title ) ;

//	jclass class_AppNativeActivityClass = env->GetObjectClass( g_AndroidSys.NativeActivity->clazz ) ;
//
//	jmethodID methodID_StartInputDialog = env->GetMethodID( class_AppNativeActivityClass, "StartInputDialog", "()V" ) ;
//	env->CallVoidMethod( g_AndroidSys.NativeActivity->clazz, methodID_StartInputDialog ) ;
//
//	env->DeleteLocalRef( class_AppNativeActivityClass ) ;

	// JavaVM �Ƃ��̃X���b�h�̊֘A�t���I��
	g_AndroidSys.NativeActivity->vm->DetachCurrentThread() ;

	return res ;
}

// �A�v���Ŏg�p���Ă��� NativeActivity ���擾����
extern const ANativeActivity *GetNativeActivity( void )
{
	return g_AndroidSys.NativeActivity ;
}

// �f�B�X�v���C�ɐݒ肳��Ă���𑜓x���擾����
extern int GetAndroidDisplayResolution( int *SizeX, int *SizeY )
{
#ifdef DX_NON_GRAPHICS
	return -1 ;
#else // DX_NON_GRAPHICS
	// �T�C�Y�擾�O�̏ꍇ�̓G���[
	if( GANDR.Device.Screen.Width  == 0 ||
		GANDR.Device.Screen.Height == 0 )
	{
		return -1 ;
	}

	if( SizeX != NULL ) *SizeX = ( int )GANDR.Device.Screen.Width ;
	if( SizeY != NULL ) *SizeY = ( int )GANDR.Device.Screen.Height ;

	return 0 ;
#endif // DX_NON_GRAPHICS
}

// �����x�Z���T�[�̃x�N�g���l���擾����
extern VECTOR GetAccelerometerVector( void )
{
	VECTOR Result ;

	if( g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_ACCELEROMETER ].Sensor != NULL &&
		g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_ACCELEROMETER ].SensorEventQueue != NULL )
	{
		Result.x = g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_ACCELEROMETER ].SensorEvent.acceleration.x ;
		Result.y = g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_ACCELEROMETER ].SensorEvent.acceleration.y ;
		Result.z = g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_ACCELEROMETER ].SensorEvent.acceleration.z ;
	}
	else
	{
		Result.x = 0.0f ;
		Result.y = 0.0f ;
		Result.z = 0.0f ;
	}

	return Result ;
}

// �Z���T�[�̃x�N�g���l���擾����
extern VECTOR GetAndroidSensorVector( int SensorType /* DX_ANDROID_SENSOR_ACCELEROMETER �Ȃ� */ )
{
	VECTOR Result = { -1.0f, -1.0f, -1.0f } ;

	if( SensorType < 0 || SensorType >= DX_ANDROID_SENSOR_NUM )
	{
		return Result ;
	}

	if( g_AndroidSys.SensorInfos[ SensorType ].Sensor != NULL &&
		g_AndroidSys.SensorInfos[ SensorType ].SensorEventQueue != NULL )
	{
		Result.x = g_AndroidSys.SensorInfos[ SensorType ].SensorEvent.acceleration.x ;
		Result.y = g_AndroidSys.SensorInfos[ SensorType ].SensorEvent.acceleration.y ;
		Result.z = g_AndroidSys.SensorInfos[ SensorType ].SensorEvent.acceleration.z ;
	}
	else
	{
		Result.x = 0.0f ;
		Result.y = 0.0f ;
		Result.z = 0.0f ;
	}

	return Result ;
}

// �Z���T�[���瓾������p���擾����
extern VECTOR GetOrientationVector( void )
{
	VECTOR Result = { -1.0f, -1.0f, -1.0f } ;
	jfloatArray floatArray_AccelerometerValues = NULL ;
	jfloatArray floatArray_MagneticFieldValues = NULL ;
	jfloatArray floatArray_rotationMatrix = NULL ;
	jfloatArray floatArray_inclinationMatrix = NULL ;
	jfloatArray floatArray_remapedMatrix = NULL ;
	jfloatArray floatArray_orientationValues = NULL ;
	jobject object_Temp = NULL ;

	if( g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_ACCELEROMETER ].Sensor            == NULL ||
		g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_ACCELEROMETER ].SensorEventQueue  == NULL ||
		g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_MAGNETIC_FIELD ].Sensor           == NULL ||
		g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_MAGNETIC_FIELD ].SensorEventQueue == NULL )
	{
		return Result ;
	}

	pthread_mutex_lock( &g_AndroidSys.NativeActivityMutex ) ;

	if( g_AndroidSys.NativeActivity == NULL )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return Result ;
	}

	// JavaVM �ƃ\�t�g���s�p�X���b�h���֘A�t��
	JNIEnv *env ;
	if( g_AndroidSys.NativeActivity->vm->AttachCurrentThreadAsDaemon( &env, NULL ) != JNI_OK )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return Result ;
	}

	// �����x�Z���T�[�̒l������
	{
		VECTOR TempVector ;

		TempVector.x = g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_ACCELEROMETER ].SensorEvent.acceleration.x ;
		TempVector.y = g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_ACCELEROMETER ].SensorEvent.acceleration.y ;
		TempVector.z = g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_ACCELEROMETER ].SensorEvent.acceleration.z ;

		floatArray_AccelerometerValues = Java_Create_floatArray_From_VECTOR( env, &TempVector ) ;
		if( floatArray_AccelerometerValues == NULL )
		{
			goto END ;
		}
	}

	// ���E�Z���T�[�̒l������
	{
		VECTOR TempVector ;

		TempVector.x = g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_MAGNETIC_FIELD ].SensorEvent.magnetic.x ;
		TempVector.y = g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_MAGNETIC_FIELD ].SensorEvent.magnetic.y ;
		TempVector.z = g_AndroidSys.SensorInfos[ DX_ANDROID_SENSOR_MAGNETIC_FIELD ].SensorEvent.magnetic.z ;

		floatArray_MagneticFieldValues = Java_Create_floatArray_From_VECTOR( env, &TempVector ) ;
		if( floatArray_MagneticFieldValues == NULL )
		{
			goto END ;
		}
	}

	// ���Z�p�̔z�������
	{
		floatArray_rotationMatrix    = env->NewFloatArray( 16 ) ;
		floatArray_inclinationMatrix = env->NewFloatArray( 16 ) ;
		floatArray_remapedMatrix     = env->NewFloatArray( 16 ) ;
		floatArray_orientationValues = env->NewFloatArray( 3 ) ;
		if( floatArray_rotationMatrix    == NULL ||
			floatArray_inclinationMatrix == NULL ||
			floatArray_remapedMatrix     == NULL ||
			floatArray_orientationValues == NULL )
		{
			goto END ;
		}
	}

	// �����x�Z���T�[�̒l�Ǝ��C�Z���T�[�̒l�����]�s����擾
	env->CallStaticBooleanMethod(
		JAVAANDR.class_SensorManager,
		JAVAANDR.methodID_SensorManager_getRotationMatrix,
		floatArray_rotationMatrix,
		floatArray_inclinationMatrix,
		floatArray_AccelerometerValues,
		floatArray_MagneticFieldValues
	) ;

	// ��]�s�񂩂���p�����擾����
	env->CallStaticBooleanMethod(
		JAVAANDR.class_SensorManager,
		JAVAANDR.methodID_SensorManager_remapCoordinateSystem,
		floatArray_rotationMatrix,
		JAVAANDR.fieldint_SensorManager_AXIS_X,
		JAVAANDR.fieldint_SensorManager_AXIS_Z,
		floatArray_remapedMatrix
	) ;
	object_Temp = env->CallStaticObjectMethod(
		JAVAANDR.class_SensorManager,
		JAVAANDR.methodID_SensorManager_getOrientation,
		floatArray_remapedMatrix,
		floatArray_orientationValues
	) ;

	// float Array ����l���擾
	if( Java_Get_VECTOR_From_floatArray( env, floatArray_orientationValues, &Result ) < 0 )
	{
		goto END ;
	}

END :

	// �擾�����Q�Ƃ̌�n��
	if( object_Temp != NULL )
	{
		env->DeleteLocalRef( object_Temp ) ;
		object_Temp = NULL ;
	}

	if( floatArray_AccelerometerValues != NULL )
	{
		env->DeleteLocalRef( floatArray_AccelerometerValues ) ;
		floatArray_AccelerometerValues = NULL ;
	}

	if( floatArray_MagneticFieldValues != NULL )
	{
		env->DeleteLocalRef( floatArray_MagneticFieldValues ) ;
		floatArray_MagneticFieldValues = NULL ;
	}

	if( floatArray_rotationMatrix != NULL )
	{
		env->DeleteLocalRef( floatArray_rotationMatrix ) ;
		floatArray_rotationMatrix = NULL ;
	}

	if( floatArray_inclinationMatrix != NULL )
	{
		env->DeleteLocalRef( floatArray_inclinationMatrix ) ;
		floatArray_inclinationMatrix = NULL ;
	}

	if( floatArray_remapedMatrix != NULL )
	{
		env->DeleteLocalRef( floatArray_remapedMatrix ) ;
		floatArray_remapedMatrix = NULL ;
	}

	if( floatArray_orientationValues != NULL )
	{
		env->DeleteLocalRef( floatArray_orientationValues ) ;
		floatArray_orientationValues = NULL ;
	}


	// JavaVM �Ƃ��̃X���b�h�̊֘A�t���I��
	g_AndroidSys.NativeActivity->vm->DetachCurrentThread() ;

	pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;

	// �߂�l��Ԃ�
	return Result ;
}

// �ʒm�𔭍s����
static int AndroidNotification_WCHAR_T(
	const wchar_t *Title, const wchar_t *SubTitle, int Icon, int ShowWhen,
	int AutoCancel, int NotifyId, int *Vibrate, int VibrateLength,
	unsigned int LightColor, int LightOnTime, int LightOffTime )
{
	jobject object_ApplicationContext = NULL ;
//	jobjectArray objectArray_Intent = NULL ;
	jobject object_Intent = NULL ;
//	jobject object_Intent2 = NULL ;
	jobject object_PendingIntent = NULL ;
	jobject object_Notification = NULL ;
	jobject object_NotificationManager = NULL ;
	jobject object_Notification_Builder = NULL ;
	jobject object_Temp = NULL ;
//	jstring string_Title = NULL ;
//	jstring string_SubTitle = NULL ;
//	jstring string_TickerText = NULL ;
	jlongArray longArray_Vibrate = NULL ;
	jobject object_TitleCharSeq = NULL ;
	jobject object_SubTitleCharSeq = NULL ;
	jobject object_TickerTextCharSeq = NULL ;
	jclass class_NativeActivity = NULL ;
	jint int_Icon = 0 ;
	DWORD LightColorARGB = 0 ;
	int LightColorR, LightColorG, LightColorB ;
	int Result = -1 ;
	int StrLength ;

	pthread_mutex_lock( &g_AndroidSys.NativeActivityMutex ) ;

	if( g_AndroidSys.NativeActivity == NULL )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// JavaVM �ƃ\�t�g���s�p�X���b�h���֘A�t��
	JNIEnv *env ;
	if( g_AndroidSys.NativeActivity->vm->AttachCurrentThreadAsDaemon( &env, NULL ) != JNI_OK )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// �A�C�R���̒l������
	if( Icon >= 0 )
	{
		int_Icon = Icon ;
	}
	else
	{
		int_Icon = JAVAANDR.fieldint_R_mipmap_sym_def_app_icon ;
	}

	// Title �� SubTitle �� CharSequence �ɕϊ�
	object_TitleCharSeq      = Java_Create_CharSequence_From_wchar_t( env, Title ) ;
	object_SubTitleCharSeq   = Java_Create_CharSequence_From_wchar_t( env, SubTitle ) ;
	object_TickerTextCharSeq = Java_Create_CharSequence_From_wchar_t( env, L"" ) ;

	// �A�v���P�[�V�����R���e�L�X�g�̎擾
	object_ApplicationContext = env->CallObjectMethod( g_AndroidSys.NativeActivity->clazz, JAVAANDR.methodID_Context_getApplicationContext ) ;
	if( object_ApplicationContext == NULL )
	{
		goto END ;
	}

	// Native Activity �̃N���X�I�u�W�F�N�g���擾
	class_NativeActivity = env->GetObjectClass( g_AndroidSys.NativeActivity->clazz ) ;
	if( class_NativeActivity == NULL )
	{
		goto END ;
	}

	// �U���p�� long�z��̏���
	if( Vibrate != NULL && VibrateLength > 0 )
	{
		jlong *jlong_Element = NULL ;

		longArray_Vibrate = env->NewLongArray( VibrateLength ) ;
		if( longArray_Vibrate == NULL )
		{
			goto END ;
		}

		jlong_Element = env->GetLongArrayElements( longArray_Vibrate, NULL ) ;
		if( jlong_Element != NULL )
		{
			int i ;
			for( i = 0 ; i < VibrateLength ; i ++ )
			{
				jlong_Element[ i ] = ( jlong )Vibrate[ i ] ;
			}

			env->ReleaseLongArrayElements( longArray_Vibrate, jlong_Element, 0 ) ;
		}
	}

	// ���C�g�p�����[�^�̏���
	{
		NS_GetColor2( LightColor, &LightColorR, &LightColorG, &LightColorB ) ;
		LightColorARGB = 0xff000000 | ( LightColorR << 16 ) | ( LightColorG << 8 ) | LightColorB ;

		if( LightOnTime <= 0 )
		{
			LightOnTime = 0 ;
		}
		if( LightOffTime <= 0 )
		{
			LightOffTime = 0 ;
		}
	}

//	objectArray_Intent = env->NewObjectArray( 2, JAVAANDR.class_Intent, NULL ) ;
//	if( objectArray_Intent == NULL )
//	{
//		goto END ;
//	}

	// Intent �̍쐬
	object_Intent = env->NewObject( JAVAANDR.class_Intent, JAVAANDR.methodID_Intent_newIntent, /*g_AndroidSys.NativeActivity->clazz*/object_ApplicationContext, class_NativeActivity ) ;
	if( object_Intent == NULL )
	{
		goto END ;
	}

//	object_Intent2 = env->NewObject( JAVAANDR.class_Intent, JAVAANDR.methodID_Intent_newIntent, object_ApplicationContext, class_NativeActivity ) ;
//	if( object_Intent2 == NULL )
//	{
//		goto END ;
//	}

//	env->SetObjectArrayElement( objectArray_Intent, 0, object_Intent ) ;
//	env->SetObjectArrayElement( objectArray_Intent, 1, object_Intent2 ) ;

//	object_Intent = env->GetObjectArrayElement( objectArray_Intent, 0 );
//	object_Intent2 = env->GetObjectArrayElement( objectArray_Intent, 1 );

	// Intent �̃t���O���Z�b�g
	object_Temp = env->CallObjectMethod( object_Intent, JAVAANDR.methodID_Intent_setFlags, JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_NEW_TASK | JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_CLEAR_TASK | JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_NO_ANIMATION ) ;
//	object_Temp = env->CallObjectMethod( object_Intent, JAVAANDR.methodID_Intent_setFlags, JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_NEW_TASK /* | JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_CLEAR_TOP */ ) ;
//	object_Temp = env->CallObjectMethod( object_Intent, JAVAANDR.methodID_Intent_setFlags, JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_NO_HISTORY ) ;
	if( object_Temp != NULL )
	{
		env->DeleteLocalRef( object_Temp ) ;
		object_Temp = NULL ;
	}

	// PendingIntent �̍쐬
	object_PendingIntent = env->CallStaticObjectMethod( JAVAANDR.class_PendingIntent, JAVAANDR.methodID_PendingIntent_getActivity, g_AndroidSys.NativeActivity->clazz, 0, object_Intent, JAVAANDR.fieldint_PendingIntent_FLAG_UPDATE_CURRENT ) ;
//	object_PendingIntent = env->CallStaticObjectMethod( JAVAANDR.class_PendingIntent, JAVAANDR.methodID_PendingIntent_getActivities, g_AndroidSys.NativeActivity->clazz, 0, objectArray_Intent, JAVAANDR.fieldint_PendingIntent_FLAG_UPDATE_CURRENT ) ;
	if( object_PendingIntent == NULL )
	{
		goto END ;
	}

	// API ���x���ɂ���ĕ���
	if( JAVAANDR.fieldint_Build_VERSION_SDK_INT < 11 )
	{
		// Notification �̍쐬
		object_Notification = env->NewObject( JAVAANDR.class_Notification, JAVAANDR.methodID_Notification_newNotification ) ;
		if( object_Notification == NULL )
		{
			goto END ;
		}

		// �A�C�R���̐ݒ�
		env->SetIntField( object_Notification, JAVAANDR.fieldID_Notification_icon, int_Icon ) ;

		// tickerText�̐ݒ�
		env->SetObjectField( object_Notification, JAVAANDR.fieldID_Notification_tickerText, object_TickerTextCharSeq ) ;

		// �U���̐ݒ�
		if( Vibrate != NULL && VibrateLength > 0 )
		{
			env->SetObjectField( object_Notification, JAVAANDR.fieldID_Notification_vibrate, longArray_Vibrate ) ;
		}

		// ���C�g�̐ݒ�
		if( LightOnTime > 0 && LightOffTime > 0 )
		{
			env->SetIntField( object_Notification, JAVAANDR.fieldID_Notification_ledARGB, LightColorARGB ) ;
			env->SetIntField( object_Notification, JAVAANDR.fieldID_Notification_ledOnMS, LightOnTime ) ;
			env->SetIntField( object_Notification, JAVAANDR.fieldID_Notification_ledOffMS, LightOffTime ) ;
		}

		// �^�C�g���e�L�X�g�̐ݒ�
		env->CallVoidMethod( object_Notification, JAVAANDR.methodID_Notification_setLatestEventInfo, object_ApplicationContext, object_TitleCharSeq, object_SubTitleCharSeq, object_PendingIntent ) ;
	}
	else
	{
		// Notification.Builder ���쐬
		object_Notification_Builder = env->NewObject( JAVAANDR.class_Notification_Builder, JAVAANDR.methodID_Notification_Builder_newNotification_Builder, g_AndroidSys.NativeActivity->clazz ) ;
		if( object_Notification_Builder == NULL )
		{
			goto END ;
		}

		// AutoCancel ���Z�b�g
		if( AutoCancel )
		{
			object_Temp = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_setAutoCancel, JNI_TRUE ) ;
			if( object_Temp != NULL )
			{
				env->DeleteLocalRef( object_Temp ) ;
				object_Temp = NULL ;
			}
		}

		// �^�C�g�����Z�b�g
		object_Temp = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_setContentTitle, object_TitleCharSeq ) ;
		if( object_Temp != NULL )
		{
			env->DeleteLocalRef( object_Temp ) ;
			object_Temp = NULL ;
		}

		// �T�u�^�C�g�����Z�b�g
		object_Temp = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_setContentText, object_SubTitleCharSeq ) ;
		if( object_Temp != NULL )
		{
			env->DeleteLocalRef( object_Temp ) ;
			object_Temp = NULL ;
		}

		// tickerText���Z�b�g
		object_Temp = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_setTicker, object_TickerTextCharSeq ) ;
		if( object_Temp != NULL )
		{
			env->DeleteLocalRef( object_Temp ) ;
			object_Temp = NULL ;
		}

		// �A�C�R�����Z�b�g
		object_Temp = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_setSmallIcon, int_Icon ) ;
		if( object_Temp != NULL )
		{
			env->DeleteLocalRef( object_Temp ) ;
			object_Temp = NULL ;
		}

		// �����\���̎w��
		if( ShowWhen == FALSE && JAVAANDR.fieldint_Build_VERSION_SDK_INT >= 19 )
		{
			object_Temp = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_setShowWhen, JNI_FALSE ) ;
			if( object_Temp != NULL )
			{
				env->DeleteLocalRef( object_Temp ) ;
				object_Temp = NULL ;
			}
		}

		// �U���̐ݒ�
		if( Vibrate != NULL && VibrateLength > 0 )
		{
			object_Temp = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_setVibrate, longArray_Vibrate ) ;
			if( object_Temp != NULL )
			{
				env->DeleteLocalRef( object_Temp ) ;
				object_Temp = NULL ;
			}
		}

		// ���C�g�̐ݒ�
		if( LightColor != NULL )
		{
			object_Temp = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_setLights, LightColorARGB, LightOnTime, LightOffTime ) ;
			if( object_Temp != NULL )
			{
				env->DeleteLocalRef( object_Temp ) ;
				object_Temp = NULL ;
			}
		}

		// PendingIntent �Z�b�g
		object_Temp = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_setContentIntent, object_PendingIntent ) ;
		if( object_Temp != NULL )
		{
			env->DeleteLocalRef( object_Temp ) ;
			object_Temp = NULL ;
		}

		// SDK�̃o�[�W�����ɂ���ď����𕪊�
		if( JAVAANDR.fieldint_Build_VERSION_SDK_INT >= 16 )
		{
			object_Notification = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_build ) ;
		}
		else
		{
			object_Notification = env->CallObjectMethod( object_Notification_Builder, JAVAANDR.methodID_Notification_Builder_getNotification ) ;
		}
	}

	// NotificationManager �̎擾
	object_NotificationManager = env->CallObjectMethod( g_AndroidSys.NativeActivity->clazz, JAVAANDR.methodID_Context_getSystemService, JAVAANDR.fieldobject_Context_NOTIFICATION_SERVICE ) ;
	if( object_NotificationManager == NULL )
	{
		goto END ;
	}

	// Notification ��ʒm
	env->CallVoidMethod( object_NotificationManager, JAVAANDR.methodID_NotificationManager_notify, NotifyId, object_Notification ) ;

	Result = 0 ;

END :

	// �擾�����Q�Ƃ̌�n��
	if( longArray_Vibrate != NULL )
	{
		env->DeleteLocalRef( longArray_Vibrate ) ;
		longArray_Vibrate = NULL ;
	}

	if( object_Notification_Builder != NULL )
	{
		env->DeleteLocalRef( object_Notification_Builder ) ;
		object_Notification_Builder = NULL ;
	}

	if( object_Temp != NULL )
	{
		env->DeleteLocalRef( object_Temp ) ;
		object_Temp = NULL ;
	}

	if( object_NotificationManager != NULL )
	{
		env->DeleteLocalRef( object_NotificationManager ) ;
		object_NotificationManager = NULL ;
	}

	if( object_TitleCharSeq != NULL )
	{
		env->DeleteLocalRef( object_TitleCharSeq ) ;
		object_TitleCharSeq = NULL ;
	}

	if( object_SubTitleCharSeq != NULL )
	{
		env->DeleteLocalRef( object_SubTitleCharSeq ) ;
		object_SubTitleCharSeq = NULL ;
	}

	if( object_TickerTextCharSeq != NULL )
	{
		env->DeleteLocalRef( object_TickerTextCharSeq ) ;
		object_TickerTextCharSeq = NULL ;
	}

//	if( string_Title != NULL )
//	{
//		env->DeleteLocalRef( string_Title ) ;
//		string_Title = NULL ;
//	}
//
//	if( string_SubTitle != NULL )
//	{
//		env->DeleteLocalRef( string_SubTitle ) ;
//		string_SubTitle = NULL ;
//	}
//
//	if( string_TickerText != NULL )
//	{
//		env->DeleteLocalRef( string_TickerText ) ;
//		string_TickerText = NULL ;
//	}

	if( object_Notification != NULL )
	{
		env->DeleteLocalRef( object_Notification ) ;
		object_Notification = NULL ;
	}

	if( object_PendingIntent != NULL )
	{
		env->DeleteLocalRef( object_PendingIntent ) ;
		object_PendingIntent = NULL ;
	}

	if( object_Intent != NULL )
	{
		env->DeleteLocalRef( object_Intent ) ;
		object_Intent = NULL ;
	}

//	if( object_Intent2 != NULL )
//	{
//		env->DeleteLocalRef( object_Intent2 ) ;
//		object_Intent2 = NULL ;
//	}

//	if( objectArray_Intent != NULL )
//	{
//		env->DeleteLocalRef( objectArray_Intent ) ;
//		objectArray_Intent = NULL ;
//	}

	if( class_NativeActivity != NULL )
	{
		env->DeleteLocalRef( class_NativeActivity ) ;
		class_NativeActivity = NULL ;
	}

	if( object_ApplicationContext != NULL )
	{
		env->DeleteLocalRef( object_ApplicationContext ) ;
		object_ApplicationContext = NULL ;
	}


	// JavaVM �Ƃ��̃X���b�h�̊֘A�t���I��
	g_AndroidSys.NativeActivity->vm->DetachCurrentThread() ;

	pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;

	// �߂�l��Ԃ�
	return Result ;
}
extern int AndroidNotification(
	const TCHAR *Title, const TCHAR *SubTitle, int Icon, int ShowWhen,
	int AutoCancel, int NotifyId, int *Vibrate, int VibrateLength,
	unsigned int LightColor, int LightOnTime, int LightOffTime )
{
#ifdef UNICODE
	return AndroidNotification_WCHAR_T( Title, SubTitle, Icon, ShowWhen, AutoCancel, NotifyId, Vibrate, VibrateLength, LightColor, LightOnTime, LightOffTime ) ;
#else
	int Result = -1 ;
	CHAR_TO_WCHAR_T_STRING_BEGIN( Title )
	CHAR_TO_WCHAR_T_STRING_BEGIN( SubTitle )
	CHAR_TO_WCHAR_T_STRING_SETUP( Title,    goto END, _TCHARCODEFORMAT )
	CHAR_TO_WCHAR_T_STRING_SETUP( SubTitle, goto END, _TCHARCODEFORMAT )

	Result = AndroidNotification_WCHAR_T( UseTitleBuffer, UseSubTitleBuffer, Icon, ShowWhen, AutoCancel, NotifyId, Vibrate, VibrateLength, LightColor, LightOnTime, LightOffTime ) ;

END :

	TCHAR_TO_WCHAR_T_STRING_END( SubTitle )
	TCHAR_TO_WCHAR_T_STRING_END( Title )

	return Result ;
#endif
}

// �ʒm���L�����Z������
// NotifyID : �ʒmID
extern int AndroidNotificationCancel( int NotifyId )
{
	jobject object_NotificationManager = NULL ;

	pthread_mutex_lock( &g_AndroidSys.NativeActivityMutex ) ;

	if( g_AndroidSys.NativeActivity == NULL )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// JavaVM �ƃ\�t�g���s�p�X���b�h���֘A�t��
	JNIEnv *env ;
	if( g_AndroidSys.NativeActivity->vm->AttachCurrentThreadAsDaemon( &env, NULL ) != JNI_OK )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// NotificationManager �̎擾
	object_NotificationManager = env->CallObjectMethod( g_AndroidSys.NativeActivity->clazz, JAVAANDR.methodID_Context_getSystemService, JAVAANDR.fieldobject_Context_NOTIFICATION_SERVICE ) ;
	if( object_NotificationManager == NULL )
	{
		goto END ;
	}

	// �ʒm���L�����Z��
	env->CallVoidMethod( object_NotificationManager, JAVAANDR.methodID_NotificationManager_cancel, NotifyId ) ;

END :

	if( object_NotificationManager != NULL )
	{
		env->DeleteLocalRef( object_NotificationManager ) ;
		object_NotificationManager = NULL ;
	}

	// JavaVM �Ƃ��̃X���b�h�̊֘A�t���I��
	g_AndroidSys.NativeActivity->vm->DetachCurrentThread() ;

	pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;

	// �I��
	return 0 ;
}

// �S�Ă̒ʒm���L�����Z������
extern int AndroidNotificationCancelAll( void )
{
	jobject object_NotificationManager = NULL ;

	pthread_mutex_lock( &g_AndroidSys.NativeActivityMutex ) ;

	if( g_AndroidSys.NativeActivity == NULL )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// JavaVM �ƃ\�t�g���s�p�X���b�h���֘A�t��
	JNIEnv *env ;
	if( g_AndroidSys.NativeActivity->vm->AttachCurrentThreadAsDaemon( &env, NULL ) != JNI_OK )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// NotificationManager �̎擾
	object_NotificationManager = env->CallObjectMethod( g_AndroidSys.NativeActivity->clazz, JAVAANDR.methodID_Context_getSystemService, JAVAANDR.fieldobject_Context_NOTIFICATION_SERVICE ) ;
	if( object_NotificationManager == NULL )
	{
		goto END ;
	}

	// �ʒm��S�ăL�����Z��
	env->CallVoidMethod( object_NotificationManager, JAVAANDR.methodID_NotificationManager_cancelAll ) ;

END :

	if( object_NotificationManager != NULL )
	{
		env->DeleteLocalRef( object_NotificationManager ) ;
		object_NotificationManager = NULL ;
	}

	// JavaVM �Ƃ��̃X���b�h�̊֘A�t���I��
	g_AndroidSys.NativeActivity->vm->DetachCurrentThread() ;

	pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;

	// �I��
	return 0 ;
}


// �w��� URL ���u���E�U�ŊJ��( BrowserAppPackageName �� BrowserAppClassName �� NULL �̏ꍇ�͕W���u���E�U�ŊJ�� )
// URL                   : �J��URL
// BrowserAppPackageName : �u���E�U�̃p�b�P�[�W��( NULL �ŕW���u���E�U )
// BrowserAppClassName   : �u���E�U�̃N���X��( NULL �ŕW���u���E�U )
static int AndroidJumpURL_WCHAR_T( const wchar_t *URL, const wchar_t *BrowserAppPackageName, const wchar_t *BrowserAppClassName )
{
	jstring string_URL = NULL ;
	jstring string_BrowserAppPackageName = NULL ;
	jstring string_BrowserAppClassName = NULL ;
	jobject object_Intent = NULL ;
	jobject object_Uri = NULL ;
	jobject object_Temp = NULL ;
	int Result = -1 ;

	pthread_mutex_lock( &g_AndroidSys.NativeActivityMutex ) ;

	if( g_AndroidSys.NativeActivity == NULL )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// JavaVM �ƃ\�t�g���s�p�X���b�h���֘A�t��
	JNIEnv *env ;
	if( g_AndroidSys.NativeActivity->vm->AttachCurrentThreadAsDaemon( &env, NULL ) != JNI_OK )
	{
		pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;
		return -1 ;
	}

	// jstring �̏���
	string_URL                   = Java_Create_jstring_From_wchar_t( env, URL ) ;
	string_BrowserAppPackageName = Java_Create_jstring_From_wchar_t( env, BrowserAppPackageName ) ;
	string_BrowserAppClassName   = Java_Create_jstring_From_wchar_t( env, BrowserAppClassName ) ;

	// Uri �̍쐬
	object_Uri = env->CallStaticObjectMethod( JAVAANDR.class_Uri, JAVAANDR.methodID_Uri_parse, string_URL ) ;
	if( object_Uri == NULL )
	{
		goto END ;
	}

	// Intent �̍쐬
	object_Intent = env->NewObject( JAVAANDR.class_Intent, JAVAANDR.methodID_Intent_newIntent_Uri, JAVAANDR.fieldstring_Intent_ACTION_VIEW, object_Uri ) ;
	if( object_Intent == NULL )
	{
		goto END ;
	}

	// �N���X���̐ݒ�
	if( BrowserAppPackageName != NULL && BrowserAppClassName != NULL )
	{
		object_Temp = env->CallObjectMethod( object_Intent, JAVAANDR.methodID_Intent_setClassName, string_BrowserAppPackageName, string_BrowserAppClassName ) ;
		if( object_Temp != NULL )
		{
			env->DeleteLocalRef( object_Temp ) ;
			object_Temp = NULL ;
		}
	}

	// Intent �̊J�n
	env->CallVoidMethod( g_AndroidSys.NativeActivity->clazz, JAVAANDR.methodID_Activity_startActivity, object_Intent ) ;

	Result = 0 ;

END :

	// �擾�����Q�Ƃ̌�n��
	if( object_Intent != NULL )
	{
		env->DeleteLocalRef( object_Intent ) ;
		object_Intent = NULL ;
	}

	if( object_Uri != NULL )
	{
		env->DeleteLocalRef( object_Uri ) ;
		object_Uri = NULL ;
	}

	if( string_URL != NULL )
	{
		env->DeleteLocalRef( string_URL ) ;
		string_URL = NULL ;
	}

	if( string_BrowserAppPackageName != NULL )
	{
		env->DeleteLocalRef( string_BrowserAppPackageName ) ;
		string_BrowserAppPackageName = NULL ;
	}

	if( string_BrowserAppClassName != NULL )
	{
		env->DeleteLocalRef( string_BrowserAppClassName ) ;
		string_BrowserAppClassName = NULL ;
	}

	// JavaVM �Ƃ��̃X���b�h�̊֘A�t���I��
	g_AndroidSys.NativeActivity->vm->DetachCurrentThread() ;

	pthread_mutex_unlock( &g_AndroidSys.NativeActivityMutex ) ;

	// �߂�l��Ԃ�
	return Result ;
}
extern int AndroidJumpURL( const TCHAR *URL, const TCHAR *BrowserAppPackageName, const TCHAR *BrowserAppClassName )
{
#ifdef UNICODE
	return AndroidJumpURL_WCHAR_T( URL, BrowserAppPackageName, BrowserAppClassName ) ;
#else
	int Result = -1 ;
	CHAR_TO_WCHAR_T_STRING_BEGIN( URL )
	CHAR_TO_WCHAR_T_STRING_BEGIN( BrowserAppPackageName )
	CHAR_TO_WCHAR_T_STRING_BEGIN( BrowserAppClassName )
	CHAR_TO_WCHAR_T_STRING_SETUP( URL,                   goto END, _TCHARCODEFORMAT )
	CHAR_TO_WCHAR_T_STRING_SETUP( BrowserAppPackageName, goto END, _TCHARCODEFORMAT )
	CHAR_TO_WCHAR_T_STRING_SETUP( BrowserAppClassName,   goto END, _TCHARCODEFORMAT )

	Result = AndroidJumpURL_WCHAR_T( UseURLBuffer, UseBrowserAppPackageNameBuffer, UseBrowserAppClassNameBuffer ) ;

END :

	TCHAR_TO_WCHAR_T_STRING_END( URL )
	TCHAR_TO_WCHAR_T_STRING_END( BrowserAppPackageName )
	TCHAR_TO_WCHAR_T_STRING_END( BrowserAppClassName )

	return Result ;
#endif
}

// �\�t�g����A�N�e�B�u�ɂȂ����ۂɌĂ΂��R�[���o�b�N�֐���o�^����
extern int SetAndroidLostFocusCallbackFunction( void (* Callback )( void *Data ), void *CallbackData )
{
	g_AndroidSys.LostFocusCallbackFunction     = ( volatile void ( * )( void * ) )Callback ;
	g_AndroidSys.LostFocusCallbackFunctionData = ( volatile void * )CallbackData ;

	return 0 ;
}

// �\�t�g���A�N�e�B�u�ɂȂ����ۂɌĂ΂��R�[���o�b�N�֐���o�^����
extern int SetAndroidGainedFocusCallbackFunction( void (* Callback )( void *Data ), void *CallbackData )
{
	g_AndroidSys.GainedFocusCallbackFunction     = ( volatile void ( * )( void * ) )Callback ;
	g_AndroidSys.GainedFocusCallbackFunctionData = ( volatile void * )CallbackData ;

	return 0 ;
}

// �X�N���[�����W���c�w���C�u������ʍ��W�ɕϊ�����
extern int ConvScreenPositionToDxScreenPosition( int ScreenX, int ScreenY, int *DxScreenX, int *DxScreenY )
{
#ifdef DX_NON_GRAPHICS
	*DxScreenX = ScreenX ;
	*DxScreenY = ScreenY ;
#else // DX_NON_GRAPHICS

	RECT DestRect ;
	DWORD DestW ;
	DWORD DestH ;

	if( GSYS.Screen.FullScreenFitScalingFlag )
	{
		if( DxScreenX != NULL )
		{
			*DxScreenX = ScreenX * GANDR.Device.Screen.SubBackBufferTextureSizeX / GANDR.Device.Screen.Width ;
		}

		if( DxScreenY != NULL )
		{
			*DxScreenY = ScreenY * GANDR.Device.Screen.SubBackBufferTextureSizeY / GANDR.Device.Screen.Height ;
		}
	}
	else
	{
		DestW = GANDR.Device.Screen.Width ;
		DestH = GANDR.Device.Screen.Width * GANDR.Device.Screen.SubBackBufferTextureSizeY / GANDR.Device.Screen.SubBackBufferTextureSizeX ;
		if( DestH > GANDR.Device.Screen.Height )
		{
			DestW = GANDR.Device.Screen.Height * GANDR.Device.Screen.SubBackBufferTextureSizeX / GANDR.Device.Screen.SubBackBufferTextureSizeY ;
			DestH = GANDR.Device.Screen.Height ;
		}

		DestRect.left   = ( GANDR.Device.Screen.Width  - DestW ) / 2 ;
		DestRect.top    = ( GANDR.Device.Screen.Height - DestH ) / 2 ;
		DestRect.right  = DestRect.left + DestW ;
		DestRect.bottom = DestRect.top  + DestH ;

		if( DxScreenX != NULL )
		{
			*DxScreenX = ( ScreenX - DestRect.left ) * GANDR.Device.Screen.SubBackBufferTextureSizeX / DestW ;
		}

		if( DxScreenY != NULL )
		{
			*DxScreenY = ( ScreenY - DestRect.top  ) * GANDR.Device.Screen.SubBackBufferTextureSizeY / DestH ;
		}
	}

#endif // DX_NON_GRAPHICS

	// �I��
	return 0 ;
}

// �A�N�e�B�u�ɂȂ�܂ŉ������Ȃ�
extern void DxActiveWait_Android( void )
{
	while(
		g_AndroidSys.NonActiveRunFlag == FALSE &&
		(
			( g_AndroidSys.ActivityState != DX_ANDR_CMD_START &&
			  g_AndroidSys.ActivityState != DX_ANDR_CMD_RESUME ) ||
			g_AndroidSys.NativeWindow == NULL
	#ifndef DX_NON_GRAPHICS
			|| GANDR.Device.Screen.Context == EGL_NO_CONTEXT
	#endif // DX_NON_GRAPHICS
		)
	)
	{
		if( NS_ProcessMessage() < 0 )
		{
			break ;
		}
		Thread_Sleep( 32 ) ;
	}
}

#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE
