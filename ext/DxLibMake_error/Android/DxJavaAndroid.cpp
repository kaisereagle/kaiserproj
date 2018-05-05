// -------------------------------------------------------------------------------
// 
// 		ＤＸライブラリ		Android用Java関係プログラム
// 
// 				Ver 3.18f
// 
// -------------------------------------------------------------------------------

// ＤＸライブラリ作成時用定義
#define __DX_MAKE

#include "../DxCompileConfig.h"

// インクルード ------------------------------------------------------------------
#include "DxJavaAndroid.h"
#include "DxBaseFuncAndroid.h"
#include "DxSystemAndroid.h"
#include "../DxFont.h"
#include "../DxMemory.h"
#include "../DxBaseFunc.h"
#include "../DxSystem.h"
#include "../DxArchive_.h"
#include "../DxLog.h"



#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// マクロ定義 --------------------------------------------------------------------

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "AndroidProject1.NativeActivity", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "AndroidProject1.NativeActivity", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "threaded_app", __VA_ARGS__))

#ifndef NDEBUG
#  define LOGV(...)  ((void)__android_log_print(ANDROID_LOG_VERBOSE, "threaded_app", __VA_ARGS__))
#else
#  define LOGV(...)  ((void)0)
#endif

// 構造体定義 --------------------------------------------------------------------

DXLIB_JAVA_ANDROID_INFO g_JavaAndroidInfo ;

// テーブル-----------------------------------------------------------------------

// 内部大域変数宣言 --------------------------------------------------------------

// 関数プロトタイプ宣言-----------------------------------------------------------

// プログラム --------------------------------------------------------------------

// UTF16LE の書式文字列と UTF8 のパラメータ文字列をログ出力する
static void JavaAndroid_LogAddUTF8( const char *UTF16LEFormatStr, const char *UTF8Str )
{
	char TempStr[ 1024 ] ;

	ConvString( UTF8Str, DX_CHARCODEFORMAT_UTF8, TempStr, sizeof( TempStr ), DX_CHARCODEFORMAT_UTF16LE ) ;
	DXST_LOGFILEFMT_ADDUTF16LE(( UTF16LEFormatStr, TempStr )) ;
}

// Java のクラスのグローバル参照を取得する
extern jclass Java_FindClass_Global( JNIEnv *env, const char *name )
{
	jclass class_local = env->FindClass( name ) ;
	if( class_local == NULL )
	{
		return NULL ;
	}

	jclass class_global = ( jclass )env->NewGlobalRef( class_local ) ;
	env->DeleteLocalRef( class_local ) ;
	class_local = NULL ;

	return class_global ;
}

// Java のスタティックオブジェクトフィールドのグローバル参照を取得する
extern jobject Java_GetStaticObjectField_Global( JNIEnv *env, jclass clazz, jfieldID fieldID )
{
	jobject fieldobject_local = env->GetStaticObjectField( clazz, fieldID ) ;
	if( fieldobject_local == NULL )
	{
		return NULL ;
	}

	jobject fieldobject_global = env->NewGlobalRef( fieldobject_local ) ;
	env->DeleteLocalRef( fieldobject_local ) ;
	fieldobject_local = NULL ;

	return fieldobject_global ;
}

// Java のスタティック文字列フィールドのグローバル参照を取得する
extern jstring Java_GetStaticStringField_Global( JNIEnv *env, jclass clazz, jfieldID fieldID )
{
	jobject fieldobject_local = env->GetStaticObjectField( clazz, fieldID ) ;
	if( fieldobject_local == NULL )
	{
		return NULL ;
	}

	jobject fieldobject_global = env->NewGlobalRef( fieldobject_local ) ;
	env->DeleteLocalRef( fieldobject_local ) ;
	fieldobject_local = NULL ;

	return ( jstring )fieldobject_global ;
}

// wchar_t の文字列から jstring を作成する( Local Ref )
extern jstring Java_Create_jstring_From_wchar_t( JNIEnv *env, const wchar_t *wchar_t_string )
{
	char UTF16LE_String[ 1024 * 2 ] ;
	int UTF16LE_StringLength ;

	if( wchar_t_string == NULL )
	{
		return NULL ;
	}

	UTF16LE_StringLength = ConvString( ( char * )wchar_t_string, WCHAR_T_CHARCODEFORMAT, UTF16LE_String, sizeof( UTF16LE_String ), DX_CHARCODEFORMAT_UTF16LE ) ;
	UTF16LE_StringLength = UTF16LE_StringLength / 2 - 1 ;
	if( UTF16LE_StringLength < 0 )
	{
		return NULL ;
	}

	return env->NewString( ( jchar * )UTF16LE_String, ( jsize )UTF16LE_StringLength ) ;
}

// wchar_t の文字列から CharSequence を作成する( Local Ref )
extern jobject Java_Create_CharSequence_From_wchar_t( JNIEnv *env, const wchar_t *wchar_t_string )
{
	char UTF16LE_String[ 1024 * 2 ] ;
	int UTF16LE_StringLength ;
	jstring jstring_Temp = NULL ;
	jobject jobject_Result = NULL ;

	if( wchar_t_string == NULL )
	{
		return NULL ;
	}

	UTF16LE_StringLength = ConvString( ( char * )wchar_t_string, WCHAR_T_CHARCODEFORMAT, UTF16LE_String, sizeof( UTF16LE_String ), DX_CHARCODEFORMAT_UTF16LE ) ;
	UTF16LE_StringLength = UTF16LE_StringLength / 2 - 1 ;
	if( UTF16LE_StringLength > 0 )
	{
		jsize StrLength = 0 ;

		jstring_Temp = env->NewString( ( jchar * )UTF16LE_String, ( jsize )UTF16LE_StringLength ) ;
		if( jstring_Temp == NULL )
		{
			goto END ;
		}

		StrLength = env->GetStringLength( jstring_Temp ) ;
		jobject_Result = env->CallObjectMethod( jstring_Temp, JAVAANDR.methodID_String_subSequence, 0, StrLength ) ;
	}

END :
	if( jstring_Temp != NULL )
	{
		env->DeleteLocalRef( jstring_Temp ) ;
		jstring_Temp = NULL ;
	}

	return jobject_Result ;
}

// jstring から wchar_t の文字列を作成する( DXFREE で解放する )
extern int Java_Create_wchar_t_string_From_jstring( JNIEnv *env, jstring _jstring, wchar_t **wchar_t_stringP )
{
	const char *utf8_charp = NULL ;
	int CharNum = 0 ;
	size_t BufferBytes = 0 ;

	// ポインタを初期化
	*wchar_t_stringP = NULL ;

	// UTF-8 の文字列を取得
	utf8_charp = env->GetStringUTFChars( _jstring, NULL ) ;
	if( utf8_charp == NULL )
	{
		return -1 ;
	}

	// 文字数を取得
	CharNum = CL_strlen( DX_CHARCODEFORMAT_UTF8, utf8_charp ) ;

	// メモリの確保
	BufferBytes = ( CharNum + 1 ) * sizeof( wchar_t ) * 2 ;
	*wchar_t_stringP = ( wchar_t * )DXALLOC( BufferBytes ) ;

	// UTF-8 を wchar_t に変換
	ConvString( utf8_charp, DX_CHARCODEFORMAT_UTF8, ( char * )*wchar_t_stringP, BufferBytes, WCHAR_T_CHARCODEFORMAT ) ;

	// UTF-8 の文字列を解放
	env->ReleaseStringUTFChars( _jstring, utf8_charp ) ;

	// 正常終了
	return 0 ;
}

// jstring から TCHAR の文字列を作成する( DXFREE で解放する )
extern int Java_Create_TCHAR_string_From_jstring( JNIEnv *env, jstring _jstring, TCHAR **tchar_stringP )
{
	const char *utf8_charp = NULL ;
	int CharNum = 0 ;
	size_t BufferBytes = 0 ;

	// ポインタを初期化
	*tchar_stringP = NULL ;

	// UTF-8 の文字列を取得
	utf8_charp = env->GetStringUTFChars( _jstring, NULL ) ;
	if( utf8_charp == NULL )
	{
		return -1 ;
	}

	// 文字数を取得
	CharNum = CL_strlen( DX_CHARCODEFORMAT_UTF8, utf8_charp ) ;

	// メモリの確保
	BufferBytes = ( CharNum + 1 ) * sizeof( TCHAR ) * 6 ;
	*tchar_stringP = ( TCHAR * )DXALLOC( BufferBytes ) ;

	// UTF-8 を TCHAR に変換
	ConvString( utf8_charp, DX_CHARCODEFORMAT_UTF8, ( char * )*tchar_stringP, BufferBytes, _TCHARCODEFORMAT ) ;

	// UTF-8 の文字列を解放
	env->ReleaseStringUTFChars( _jstring, utf8_charp ) ;

	// 正常終了
	return 0 ;
}

// VECTOR から長さ 3 の float Array を作成する( Local Ref )
extern jfloatArray Java_Create_floatArray_From_VECTOR( JNIEnv *env, const VECTOR *Vector )
{
	jfloat *jfloat_Element = NULL ;
	jfloatArray jfloatArray_Result = NULL ;

	jfloatArray_Result = env->NewFloatArray( 3 ) ;
	if( jfloatArray_Result == NULL )
	{
		return NULL ;
	}

	jfloat_Element = env->GetFloatArrayElements( jfloatArray_Result, NULL ) ;
	if( jfloat_Element == NULL )
	{
		env->DeleteLocalRef( jfloatArray_Result ) ;
		return NULL ;
	}

	jfloat_Element[ 0 ] = Vector->x ;
	jfloat_Element[ 1 ] = Vector->y ;
	jfloat_Element[ 2 ] = Vector->z ;

	env->ReleaseFloatArrayElements( jfloatArray_Result, jfloat_Element, 0 ) ;

	return jfloatArray_Result ;
}

// 長さ 3 以上の float Array から VECTOR の値を取得する( 0:正常終了  -1:エラー )
extern int Java_Get_VECTOR_From_floatArray( JNIEnv *env, jfloatArray floatArray, VECTOR *Buffer )
{
	jfloat *jfloat_Element = NULL ;

	if( env->GetArrayLength( floatArray ) < 3 )
	{
		return -1 ;
	}

	jfloat_Element = env->GetFloatArrayElements( floatArray, NULL ) ;
	if( jfloat_Element == NULL )
	{
		return -1 ;
	}

	Buffer->x = jfloat_Element[ 0 ] ;
	Buffer->y = jfloat_Element[ 1 ] ;
	Buffer->z = jfloat_Element[ 2 ] ;

	env->ReleaseFloatArrayElements( floatArray, jfloat_Element, 0 ) ;

	return 0 ;
}

// Java のクラスやメソッドの参照を取得する
#define FINDCLASS( obj, name )			\
	{\
		obj = Java_FindClass_Global( env, name ) ;\
		if( obj == NULL )\
		{\
			JavaAndroid_LogAddUTF8( "\x4a\x00\x61\x00\x76\x00\x61\x00\x20\x00\x43\x00\x6c\x00\x61\x00\x73\x00\x73\x00\x20\x00\x25\x00\x73\x00\x20\x00\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x00"/*@ L"Java Class %s の取得に失敗しました" @*/, name ) ;\
			TerminateJavaAndroidInfo( env ) ;\
			return -1 ;\
		}\
	}

#define GETMETHOD( obj, class_, name, sig )		\
	{\
		obj = env->GetMethodID( class_, name, sig ) ;\
		if( obj == NULL )\
		{\
			JavaAndroid_LogAddUTF8( "\x4a\x00\x61\x00\x76\x00\x61\x00\x20\x00\x4d\x00\x65\x00\x74\x00\x68\x00\x6f\x00\x64\x00\x20\x00\x25\x00\x73\x00\x20\x00\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x00"/*@ L"Java Method %s の取得に失敗しました" @*/, name ) ;\
			TerminateJavaAndroidInfo( env ) ;\
			return -1 ;\
		}\
	}

#define GETSTATICMETHOD( obj, class_, name, sig )		\
	{\
		obj = env->GetStaticMethodID( class_, name, sig ) ;\
		if( obj == NULL )\
		{\
			JavaAndroid_LogAddUTF8( "\x4a\x00\x61\x00\x76\x00\x61\x00\x20\x00\x53\x00\x74\x00\x61\x00\x74\x00\x69\x00\x63\x00\x20\x00\x4d\x00\x65\x00\x74\x00\x68\x00\x6f\x00\x64\x00\x20\x00\x25\x00\x73\x00\x20\x00\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x00"/*@ L"Java Static Method %s の取得に失敗しました" @*/, name ) ;\
			TerminateJavaAndroidInfo( env ) ;\
			return -1 ;\
		}\
	}

#define GETFIELD( obj, class_, name, sig )		\
	{\
		obj = env->GetFieldID( class_, name, sig ) ;\
		if( obj == NULL )\
		{\
			JavaAndroid_LogAddUTF8( "\x4a\x00\x61\x00\x76\x00\x61\x00\x20\x00\x46\x00\x69\x00\x65\x00\x6c\x00\x64\x00\x20\x00\x25\x00\x73\x00\x20\x00\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x00"/*@ L"Java Field %s の取得に失敗しました" @*/, name ) ;\
			TerminateJavaAndroidInfo( env ) ;\
			return -1 ;\
		}\
	}

#define GETSTATICFIELD( obj, class_, name, sig )		\
	{\
		obj = env->GetStaticFieldID( class_, name, sig ) ;\
		if( obj == NULL )\
		{\
			JavaAndroid_LogAddUTF8( "\x4a\x00\x61\x00\x76\x00\x61\x00\x20\x00\x53\x00\x74\x00\x61\x00\x74\x00\x69\x00\x63\x00\x20\x00\x46\x00\x69\x00\x65\x00\x6c\x00\x64\x00\x20\x00\x25\x00\x73\x00\x20\x00\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x00"/*@ L"Java Static Field %s の取得に失敗しました" @*/, name ) ;\
			TerminateJavaAndroidInfo( env ) ;\
			return -1 ;\
		}\
	}

#define GETSTATICOBJECTFIELD( obj, class_, field_, name )		\
	{\
		obj = Java_GetStaticObjectField_Global( env, class_, field_ ) ;\
		if( obj == NULL )\
		{\
			JavaAndroid_LogAddUTF8( "\x4a\x00\x61\x00\x76\x00\x61\x00\x20\x00\x53\x00\x74\x00\x61\x00\x74\x00\x69\x00\x63\x00\x20\x00\x4f\x00\x62\x00\x6a\x00\x65\x00\x63\x00\x74\x00\x20\x00\x46\x00\x69\x00\x65\x00\x6c\x00\x64\x00\x20\x00\x25\x00\x73\x00\x20\x00\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x00"/*@ L"Java Static Object Field %s の取得に失敗しました" @*/, name ) ;\
			TerminateJavaAndroidInfo( env ) ;\
			return -1 ;\
		}\
	}

#define GETSTATICSTRINGFIELD( obj, class_, field_, name )		\
	{\
		obj = Java_GetStaticStringField_Global( env, class_, field_ ) ;\
		if( obj == NULL )\
		{\
			JavaAndroid_LogAddUTF8( "\x4a\x00\x61\x00\x76\x00\x61\x00\x20\x00\x53\x00\x74\x00\x61\x00\x74\x00\x69\x00\x63\x00\x20\x00\x53\x00\x74\x00\x72\x00\x69\x00\x6e\x00\x67\x00\x20\x00\x46\x00\x69\x00\x65\x00\x6c\x00\x64\x00\x20\x00\x25\x00\x73\x00\x20\x00\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x00"/*@ L"Java Static String Field %s の取得に失敗しました" @*/, name ) ;\
			TerminateJavaAndroidInfo( env ) ;\
			return -1 ;\
		}\
	}
	
extern int SetupJavaAndroidInfo( JNIEnv *env )
{
	jint API_LV ;

	if( JAVAANDR.InitializeFlag == TRUE )
	{
		return -1 ;
	}

	if( env == NULL )
	{
		return -1 ;
	}

	// APIレベルだけ先に取得する
	FINDCLASS( JAVAANDR.class_Build_VERSION, "android/os/Build$VERSION" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Build_VERSION_SDK_INT, JAVAANDR.class_Build_VERSION, "SDK_INT", "I" ) ;
	JAVAANDR.fieldint_Build_VERSION_SDK_INT = env->GetStaticIntField( JAVAANDR.class_Build_VERSION, JAVAANDR.fieldID_Build_VERSION_SDK_INT ) ;
	API_LV = JAVAANDR.fieldint_Build_VERSION_SDK_INT ;

	if( API_LV >= 11 )
	{
		FINDCLASS( JAVAANDR.class_Notification_Builder, "android/app/Notification$Builder" ) ;

		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_newNotification_Builder, JAVAANDR.class_Notification_Builder, "<init>",              "(Landroid/content/Context;)V" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setWhen,                 JAVAANDR.class_Notification_Builder, "setWhen",             "(J)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 19 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setShowWhen,             JAVAANDR.class_Notification_Builder, "setShowWhen",         "(Z)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 16 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setUsesChronometer,      JAVAANDR.class_Notification_Builder, "setUsesChronometer",  "(Z)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setSmallIcon,            JAVAANDR.class_Notification_Builder, "setSmallIcon",        "(II)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setContentTitle,         JAVAANDR.class_Notification_Builder, "setContentTitle",     "(Ljava/lang/CharSequence;)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setContentText,          JAVAANDR.class_Notification_Builder, "setContentText",      "(Ljava/lang/CharSequence;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 16 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setSubText,              JAVAANDR.class_Notification_Builder, "setSubText",          "(Ljava/lang/CharSequence;)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setNumber,               JAVAANDR.class_Notification_Builder, "setNumber",           "(I)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setContentInfo,          JAVAANDR.class_Notification_Builder, "setContentInfo",      "(Ljava/lang/CharSequence;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 14 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setProgress,             JAVAANDR.class_Notification_Builder, "setProgress",         "(IIZ)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setContent,              JAVAANDR.class_Notification_Builder, "setContent",          "(Landroid/widget/RemoteViews;)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setContentIntent,        JAVAANDR.class_Notification_Builder, "setContentIntent",    "(Landroid/app/PendingIntent;)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setDeleteIntent,         JAVAANDR.class_Notification_Builder, "setDeleteIntent",     "(Landroid/app/PendingIntent;)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setFullScreenIntent,     JAVAANDR.class_Notification_Builder, "setFullScreenIntent", "(Landroid/app/PendingIntent;Z)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setTicker,               JAVAANDR.class_Notification_Builder, "setTicker",           "(Ljava/lang/CharSequence;Landroid/widget/RemoteViews;)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setLargeIcon,            JAVAANDR.class_Notification_Builder, "setLargeIcon",        "(Landroid/graphics/Bitmap;)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setSound,                JAVAANDR.class_Notification_Builder, "setSound",            "(Landroid/net/Uri;I)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setVibrate,              JAVAANDR.class_Notification_Builder, "setVibrate",          "([J)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setLights,               JAVAANDR.class_Notification_Builder, "setLights",           "(III)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setOngoing,              JAVAANDR.class_Notification_Builder, "setOngoing",          "(Z)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setOnlyAlertOnce,        JAVAANDR.class_Notification_Builder, "setOnlyAlertOnce",    "(Z)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setAutoCancel,           JAVAANDR.class_Notification_Builder, "setAutoCancel",       "(Z)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 20 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setLocalOnly,            JAVAANDR.class_Notification_Builder, "setLocalOnly",        "(Z)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_setDefaults,             JAVAANDR.class_Notification_Builder, "setDefaults",         "(I)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 16 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setPriority,             JAVAANDR.class_Notification_Builder, "setPriority",         "(I)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 21 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setCategory,             JAVAANDR.class_Notification_Builder, "setCategory",         "(Ljava/lang/String;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 21 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_addPerson,               JAVAANDR.class_Notification_Builder, "addPerson",           "(Ljava/lang/String;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 20 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setGroup,                JAVAANDR.class_Notification_Builder, "setGroup",            "(Ljava/lang/String;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 20 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setGroupSummary,         JAVAANDR.class_Notification_Builder, "setGroupSummary",     "(Z)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 20 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setSortKey,              JAVAANDR.class_Notification_Builder, "setSortKey",          "(Ljava/lang/String;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 20 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_addExtras,               JAVAANDR.class_Notification_Builder, "addExtras",           "(Landroid/os/Bundle;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 19 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setExtras,               JAVAANDR.class_Notification_Builder, "setExtras",           "(Landroid/os/Bundle;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 21 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_getExtras,               JAVAANDR.class_Notification_Builder, "getExtras",           "()Landroid/os/Bundle;" ) ;
		if( API_LV >= 16 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_addAction,               JAVAANDR.class_Notification_Builder, "addAction",           "(ILjava/lang/CharSequence;Landroid/app/PendingIntent;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 22 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setStyle,                JAVAANDR.class_Notification_Builder, "setStyle",            "(Landroid/app/Notification$Style;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 21 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setVisibility,           JAVAANDR.class_Notification_Builder, "setVisibility",       "(I)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 21 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setPublicVersion,        JAVAANDR.class_Notification_Builder, "setPublicVersion",    "(Landroid/app/Notification;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 22 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_extend,                  JAVAANDR.class_Notification_Builder, "extend",              "(Landroid/app/Notification$Extender;)Landroid/app/Notification$Builder;" ) ;
		if( API_LV >= 21 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_setColor,                JAVAANDR.class_Notification_Builder, "setColor",            "(I)Landroid/app/Notification$Builder;" ) ;
		                   GETMETHOD( JAVAANDR.methodID_Notification_Builder_getNotification,         JAVAANDR.class_Notification_Builder, "getNotification",     "()Landroid/app/Notification;" ) ;
		if( API_LV >= 16 ) GETMETHOD( JAVAANDR.methodID_Notification_Builder_build,                   JAVAANDR.class_Notification_Builder, "build",               "()Landroid/app/Notification;" ) ;
	}

	FINDCLASS( JAVAANDR.class_Bitmap,               "android/graphics/Bitmap"            ) ;
	FINDCLASS( JAVAANDR.class_BitmapConfig,         "android/graphics/Bitmap$Config"     ) ;
	FINDCLASS( JAVAANDR.class_Canvas,               "android/graphics/Canvas"            ) ;
	FINDCLASS( JAVAANDR.class_Typeface,             "android/graphics/Typeface"          ) ;
	FINDCLASS( JAVAANDR.class_Paint,                "android/graphics/Paint"             ) ;
	FINDCLASS( JAVAANDR.class_PaintFontMetrics,     "android/graphics/Paint$FontMetrics" ) ;
	FINDCLASS( JAVAANDR.class_Color,                "android/graphics/Color"             ) ;

	FINDCLASS( JAVAANDR.class_TextView,             "android/widget/TextView"            ) ;
	FINDCLASS( JAVAANDR.class_TextView_BufferType,  "android/widget/TextView$BufferType" ) ;
	FINDCLASS( JAVAANDR.class_EditText,             "android/widget/EditText"            ) ;
	FINDCLASS( JAVAANDR.class_CharSequence,         "java/lang/CharSequence"             ) ;
	FINDCLASS( JAVAANDR.class_String,               "java/lang/String"                   ) ;
	FINDCLASS( JAVAANDR.class_Locale,               "java/util/Locale"                   ) ;
	FINDCLASS( JAVAANDR.class_Dialog,               "android/app/Dialog"                 ) ;
	FINDCLASS( JAVAANDR.class_AlertDialog,          "android/app/AlertDialog"            ) ;
	FINDCLASS( JAVAANDR.class_AlertDialog_Builder,  "android/app/AlertDialog$Builder"    ) ;
	FINDCLASS( JAVAANDR.class_Notification,         "android/app/Notification"           ) ;
	FINDCLASS( JAVAANDR.class_NotificationManager,  "android/app/NotificationManager"    ) ;
	FINDCLASS( JAVAANDR.class_PendingIntent,        "android/app/PendingIntent"          ) ;
	FINDCLASS( JAVAANDR.class_Intent,               "android/content/Intent"             ) ;
	FINDCLASS( JAVAANDR.class_Context,              "android/content/Context"            ) ;
	FINDCLASS( JAVAANDR.class_SensorManager,        "android/hardware/SensorManager"     ) ;
	FINDCLASS( JAVAANDR.class_R_mipmap,             "android/R$mipmap"                   ) ;
	FINDCLASS( JAVAANDR.class_Uri,                  "android/net/Uri"                    ) ;
	FINDCLASS( JAVAANDR.class_Activity,             "android/app/Activity"               ) ;


	GETSTATICMETHOD( JAVAANDR.methodID_Bitmap_createBitmap                        , JAVAANDR.class_Bitmap,               "createBitmap",          "(IILandroid/graphics/Bitmap$Config;)Landroid/graphics/Bitmap;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Bitmap_setPixels                           , JAVAANDR.class_Bitmap,               "setPixels",             "([IIIIIII)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_Bitmap_getPixels                           , JAVAANDR.class_Bitmap,               "getPixels",             "([IIIIIII)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_Canvas_newCanvas                           , JAVAANDR.class_Canvas,               "<init>",                "(Landroid/graphics/Bitmap;)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_Canvas_drawText                            , JAVAANDR.class_Canvas,               "drawText",              "([CIIFFLandroid/graphics/Paint;)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_Canvas_drawARGB                            , JAVAANDR.class_Canvas,               "drawARGB",              "(IIII)V" ) ;
	GETSTATICMETHOD( JAVAANDR.methodID_Typeface_create                            , JAVAANDR.class_Typeface,             "create",                "(Ljava/lang/String;I)Landroid/graphics/Typeface;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Paint_newPaint                             , JAVAANDR.class_Paint,                "<init>",                "()V" ) ;
	GETMETHOD(       JAVAANDR.methodID_Paint_getFontMetrics                       , JAVAANDR.class_Paint,                "getFontMetrics",        "()Landroid/graphics/Paint$FontMetrics;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Paint_setTextSize                          , JAVAANDR.class_Paint,                "setTextSize",           "(F)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_Paint_setTypeface                          , JAVAANDR.class_Paint,                "setTypeface",           "(Landroid/graphics/Typeface;)Landroid/graphics/Typeface;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Paint_setAntiAlias                         , JAVAANDR.class_Paint,                "setAntiAlias",          "(Z)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_Paint_setARGB                              , JAVAANDR.class_Paint,                "setARGB",               "(IIII)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_Paint_measureText                          , JAVAANDR.class_Paint,                "measureText",           "([CII)F" ) ;

	GETMETHOD(       JAVAANDR.methodID_EditText_newEditText                       , JAVAANDR.class_EditText,             "<init>",                "(Landroid/content/Context;)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_EditText_setText                           , JAVAANDR.class_EditText,             "setText",               "(Ljava/lang/CharSequence;Landroid/widget/TextView$BufferType;)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_TextView_getText                           , JAVAANDR.class_TextView,             "getText",               "()Ljava/lang/CharSequence;" ) ;
	GETMETHOD(       JAVAANDR.methodID_CharSequence_toString                      , JAVAANDR.class_CharSequence,         "toString",              "()Ljava/lang/String;" ) ;
	GETMETHOD(       JAVAANDR.methodID_String_subSequence                         , JAVAANDR.class_String,               "subSequence",           "(II)Ljava/lang/CharSequence;" ) ;
	GETSTATICMETHOD( JAVAANDR.methodID_Locale_getDefault                          , JAVAANDR.class_Locale,               "getDefault",            "()Ljava/util/Locale;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Locale_getLanguage                         , JAVAANDR.class_Locale,               "getLanguage",           "()Ljava/lang/String;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Locale_getCountry                          , JAVAANDR.class_Locale,               "getCountry",            "()Ljava/lang/String;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Dialog_isShowing                           , JAVAANDR.class_Dialog,               "isShowing",             "()Z" ) ;
	GETMETHOD(       JAVAANDR.methodID_AlertDialog_Builder_newAlertDialog_Builder , JAVAANDR.class_AlertDialog_Builder,  "<init>",                "(Landroid/content/Context;)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_AlertDialog_Builder_setIcon                , JAVAANDR.class_AlertDialog_Builder,  "setIcon",               "(Landroid/graphics/drawable/Drawable;)Landroid/app/AlertDialog$Builder;" ) ;
	GETMETHOD(       JAVAANDR.methodID_AlertDialog_Builder_setTitle               , JAVAANDR.class_AlertDialog_Builder,  "setTitle",              "(Ljava/lang/CharSequence;)Landroid/app/AlertDialog$Builder;" ) ;
	GETMETHOD(       JAVAANDR.methodID_AlertDialog_Builder_setView                , JAVAANDR.class_AlertDialog_Builder,  "setView",               "(Landroid/view/View;)Landroid/app/AlertDialog$Builder;" ) ;
	GETMETHOD(       JAVAANDR.methodID_AlertDialog_Builder_setPositiveButton      , JAVAANDR.class_AlertDialog_Builder,  "setPositiveButton",     "(Ljava/lang/CharSequence;Landroid/content/DialogInterface$OnClickListener;)Landroid/app/AlertDialog$Builder;" ) ;
	GETMETHOD(       JAVAANDR.methodID_AlertDialog_Builder_setNegativeButton      , JAVAANDR.class_AlertDialog_Builder,  "setNegativeButton",     "(Ljava/lang/CharSequence;Landroid/content/DialogInterface$OnClickListener;)Landroid/app/AlertDialog$Builder;" ) ;
	GETMETHOD(       JAVAANDR.methodID_AlertDialog_Builder_show                   , JAVAANDR.class_AlertDialog_Builder,  "show",                  "()Landroid/app/AlertDialog;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Notification_newNotification               , JAVAANDR.class_Notification,         "<init>",                "()V" ) ;
	if( API_LV < 11 )
	{
		GETMETHOD(   JAVAANDR.methodID_Notification_setLatestEventInfo            , JAVAANDR.class_Notification,         "setLatestEventInfo",    "(Landroid/content/Context;Ljava/lang/CharSequence;Ljava/lang/CharSequence;Landroid/app/PendingIntent;)V" ) ;
	}
	GETMETHOD(       JAVAANDR.methodID_NotificationManager_notify                 , JAVAANDR.class_NotificationManager,  "notify",                "(ILandroid/app/Notification;)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_NotificationManager_cancel                 , JAVAANDR.class_NotificationManager,  "cancel",                "(I)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_NotificationManager_cancelAll              , JAVAANDR.class_NotificationManager,  "cancelAll",             "()V" ) ;
	GETSTATICMETHOD( JAVAANDR.methodID_PendingIntent_getActivity                  , JAVAANDR.class_PendingIntent,        "getActivity",           "(Landroid/content/Context;ILandroid/content/Intent;I)Landroid/app/PendingIntent;" ) ;
	GETSTATICMETHOD( JAVAANDR.methodID_PendingIntent_getActivities                , JAVAANDR.class_PendingIntent,        "getActivities",         "(Landroid/content/Context;I[Landroid/content/Intent;I)Landroid/app/PendingIntent;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Intent_newIntent                           , JAVAANDR.class_Intent,               "<init>",                "(Landroid/content/Context;Ljava/lang/Class;)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_Intent_newIntent_Uri                       , JAVAANDR.class_Intent,               "<init>",                "(Ljava/lang/String;Landroid/net/Uri;)V" ) ;
	GETMETHOD(       JAVAANDR.methodID_Intent_setFlags                            , JAVAANDR.class_Intent,               "setFlags",              "(I)Landroid/content/Intent;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Intent_setClassName                        , JAVAANDR.class_Intent,               "setClassName",          "(Ljava/lang/String;Ljava/lang/String;)Landroid/content/Intent;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Context_getApplicationContext              , JAVAANDR.class_Context,              "getApplicationContext", "()Landroid/content/Context;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Context_getSystemService                   , JAVAANDR.class_Context,              "getSystemService",      "(Ljava/lang/String;)Ljava/lang/Object;" ) ;
	GETSTATICMETHOD( JAVAANDR.methodID_SensorManager_getRotationMatrix            , JAVAANDR.class_SensorManager,        "getRotationMatrix",     "([F[F[F[F)Z" ) ;
	GETSTATICMETHOD( JAVAANDR.methodID_SensorManager_remapCoordinateSystem        , JAVAANDR.class_SensorManager,        "remapCoordinateSystem", "([FII[F)Z" ) ;
	GETSTATICMETHOD( JAVAANDR.methodID_SensorManager_getOrientation               , JAVAANDR.class_SensorManager,        "getOrientation",        "([F[F)[F" ) ;
	GETSTATICMETHOD( JAVAANDR.methodID_Uri_parse                                  , JAVAANDR.class_Uri,                  "parse",                 "(Ljava/lang/String;)Landroid/net/Uri;" ) ;
	GETMETHOD(       JAVAANDR.methodID_Activity_startActivity                     , JAVAANDR.class_Activity,             "startActivity",         "(Landroid/content/Intent;)V" ) ;


	GETSTATICFIELD( JAVAANDR.fieldID_BitmapConfig_ARGB_8888                     , JAVAANDR.class_BitmapConfig,            "ARGB_8888",                           "Landroid/graphics/Bitmap$Config;" ) ;
	GETFIELD(       JAVAANDR.fieldID_PaintFontMetrics_ascent                    , JAVAANDR.class_PaintFontMetrics,        "ascent",                              "F" ) ;
	GETFIELD(       JAVAANDR.fieldID_PaintFontMetrics_bottom                    , JAVAANDR.class_PaintFontMetrics,        "bottom",                              "F" ) ;
	GETFIELD(       JAVAANDR.fieldID_PaintFontMetrics_descent                   , JAVAANDR.class_PaintFontMetrics,        "descent",                             "F" ) ;
	GETFIELD(       JAVAANDR.fieldID_PaintFontMetrics_leading                   , JAVAANDR.class_PaintFontMetrics,        "leading",                             "F" ) ;
	GETFIELD(       JAVAANDR.fieldID_PaintFontMetrics_top                       , JAVAANDR.class_PaintFontMetrics,        "top",                                 "F" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Typeface_NORMAL                            , JAVAANDR.class_Typeface,                "NORMAL",                              "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Typeface_BOLD                              , JAVAANDR.class_Typeface,                "BOLD",                                "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Typeface_ITALIC                            , JAVAANDR.class_Typeface,                "ITALIC",                              "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Typeface_BOLD_ITALIC                       , JAVAANDR.class_Typeface,                "BOLD_ITALIC",                         "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_BLACK                                , JAVAANDR.class_Color,                   "BLACK",                               "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_BLUE                                 , JAVAANDR.class_Color,                   "BLUE",                                "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_CYAN                                 , JAVAANDR.class_Color,                   "CYAN",                                "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_DKGRAY                               , JAVAANDR.class_Color,                   "DKGRAY",                              "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_GRAY                                 , JAVAANDR.class_Color,                   "GRAY",                                "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_GREEN                                , JAVAANDR.class_Color,                   "GREEN",                               "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_LTGRAY                               , JAVAANDR.class_Color,                   "LTGRAY",                              "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_MAGENTA                              , JAVAANDR.class_Color,                   "MAGENTA",                             "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_RED                                  , JAVAANDR.class_Color,                   "RED",                                 "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_TRANSPARENT                          , JAVAANDR.class_Color,                   "TRANSPARENT",                         "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_WHITE                                , JAVAANDR.class_Color,                   "WHITE",                               "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Color_YELLOW                               , JAVAANDR.class_Color,                   "YELLOW",                              "I" ) ;

	GETSTATICFIELD( JAVAANDR.fieldID_TextView_BufferType_EDITABLE               , JAVAANDR.class_TextView_BufferType,     "EDITABLE",                            "Landroid/widget/TextView$BufferType;" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_TextView_BufferType_NORMAL                 , JAVAANDR.class_TextView_BufferType,     "NORMAL",                              "Landroid/widget/TextView$BufferType;" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_TextView_BufferType_SPANNABLE              , JAVAANDR.class_TextView_BufferType,     "SPANNABLE",                           "Landroid/widget/TextView$BufferType;" ) ;

	GETSTATICFIELD( JAVAANDR.fieldID_Notification_DEFAULT_LIGHTS                , JAVAANDR.class_Notification,            "DEFAULT_LIGHTS",                      "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Notification_DEFAULT_VIBRATE               , JAVAANDR.class_Notification,            "DEFAULT_VIBRATE",                     "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Notification_FLAG_SHOW_LIGHTS              , JAVAANDR.class_Notification,            "FLAG_SHOW_LIGHTS",                    "I" ) ;
	GETFIELD(       JAVAANDR.fieldID_Notification_icon                          , JAVAANDR.class_Notification,            "icon",                                "I" ) ;
	GETFIELD(       JAVAANDR.fieldID_Notification_tickerText                    , JAVAANDR.class_Notification,            "tickerText",                          "Ljava/lang/CharSequence;" ) ;
	GETFIELD(       JAVAANDR.fieldID_Notification_defaults                      , JAVAANDR.class_Notification,            "defaults",                            "I" ) ;
	GETFIELD(       JAVAANDR.fieldID_Notification_vibrate                       , JAVAANDR.class_Notification,            "vibrate",                             "[J" ) ;
	GETFIELD(       JAVAANDR.fieldID_Notification_ledARGB                       , JAVAANDR.class_Notification,            "ledARGB",                             "I" ) ;
	GETFIELD(       JAVAANDR.fieldID_Notification_ledOffMS                      , JAVAANDR.class_Notification,            "ledOffMS",                            "I" ) ;
	GETFIELD(       JAVAANDR.fieldID_Notification_ledOnMS                       , JAVAANDR.class_Notification,            "ledOnMS",                             "I" ) ;
	GETFIELD(       JAVAANDR.fieldID_Notification_flags                         , JAVAANDR.class_Notification,            "flags",                               "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_PendingIntent_FLAG_CANCEL_CURRENT	        , JAVAANDR.class_PendingIntent,           "FLAG_CANCEL_CURRENT",                 "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_PendingIntent_FLAG_NO_CREATE               , JAVAANDR.class_PendingIntent,           "FLAG_NO_CREATE",                      "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_PendingIntent_FLAG_ONE_SHOT                , JAVAANDR.class_PendingIntent,           "FLAG_ONE_SHOT",                       "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_PendingIntent_FLAG_UPDATE_CURRENT          , JAVAANDR.class_PendingIntent,           "FLAG_UPDATE_CURRENT",                 "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Intent_ACTION_VIEW                         , JAVAANDR.class_Intent,                  "ACTION_VIEW",                         "Ljava/lang/String;" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_NEW_TASK              , JAVAANDR.class_Intent,                  "FLAG_ACTIVITY_NEW_TASK",              "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_SINGLE_TOP            , JAVAANDR.class_Intent,                  "FLAG_ACTIVITY_SINGLE_TOP",            "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_CLEAR_TOP             , JAVAANDR.class_Intent,                  "FLAG_ACTIVITY_CLEAR_TOP",             "I" ) ;
	if( API_LV >= 11 )
	{
		GETSTATICFIELD( JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_CLEAR_TASK            , JAVAANDR.class_Intent,                  "FLAG_ACTIVITY_CLEAR_TASK",            "I" ) ;
	}
	GETSTATICFIELD( JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_NO_ANIMATION          , JAVAANDR.class_Intent,                  "FLAG_ACTIVITY_NO_ANIMATION",          "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_NO_HISTORY            , JAVAANDR.class_Intent,                  "FLAG_ACTIVITY_NO_HISTORY",            "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_LAUNCHED_FROM_HISTORY , JAVAANDR.class_Intent,                  "FLAG_ACTIVITY_LAUNCHED_FROM_HISTORY", "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_EXCLUDE_FROM_RECENTS  , JAVAANDR.class_Intent,                  "FLAG_ACTIVITY_EXCLUDE_FROM_RECENTS",  "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_PREVIOUS_IS_TOP       , JAVAANDR.class_Intent,                  "FLAG_ACTIVITY_PREVIOUS_IS_TOP",       "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_REORDER_TO_FRONT      , JAVAANDR.class_Intent,                  "FLAG_ACTIVITY_REORDER_TO_FRONT",      "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Context_NOTIFICATION_SERVICE               , JAVAANDR.class_Context,                 "NOTIFICATION_SERVICE",                "Ljava/lang/String;" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_Context_SENSOR_SERVICE                     , JAVAANDR.class_Context,                 "SENSOR_SERVICE",                      "Ljava/lang/String;" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_SensorManager_AXIS_MINUS_X                 , JAVAANDR.class_SensorManager,           "AXIS_MINUS_X",                        "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_SensorManager_AXIS_MINUS_Y                 , JAVAANDR.class_SensorManager,           "AXIS_MINUS_Y",                        "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_SensorManager_AXIS_MINUS_Z                 , JAVAANDR.class_SensorManager,           "AXIS_MINUS_Z",                        "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_SensorManager_AXIS_X                       , JAVAANDR.class_SensorManager,           "AXIS_X",                              "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_SensorManager_AXIS_Y                       , JAVAANDR.class_SensorManager,           "AXIS_Y",                              "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_SensorManager_AXIS_Z                       , JAVAANDR.class_SensorManager,           "AXIS_Z",                              "I" ) ;
	GETSTATICFIELD( JAVAANDR.fieldID_R_mipmap_sym_def_app_icon                  , JAVAANDR.class_R_mipmap,                "sym_def_app_icon",                    "I" ) ;
	


	GETSTATICOBJECTFIELD( JAVAANDR.fieldobject_BitmapConfig_ARGB_8888,        JAVAANDR.class_BitmapConfig,        JAVAANDR.fieldID_BitmapConfig_ARGB_8888, "BitmapConfig_ARGB_8888" ) ;
	JAVAANDR.fieldint_Typeface_NORMAL           = env->GetStaticIntField(     JAVAANDR.class_Typeface,            JAVAANDR.fieldID_Typeface_NORMAL      ) ;
	JAVAANDR.fieldint_Typeface_BOLD             = env->GetStaticIntField(     JAVAANDR.class_Typeface,            JAVAANDR.fieldID_Typeface_BOLD        ) ;
	JAVAANDR.fieldint_Typeface_ITALIC           = env->GetStaticIntField(     JAVAANDR.class_Typeface,            JAVAANDR.fieldID_Typeface_ITALIC      ) ;
	JAVAANDR.fieldint_Typeface_BOLD_ITALIC      = env->GetStaticIntField(     JAVAANDR.class_Typeface,            JAVAANDR.fieldID_Typeface_BOLD_ITALIC ) ;
	JAVAANDR.fieldint_Color_BLACK               = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_BLACK          ) ;
	JAVAANDR.fieldint_Color_BLUE                = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_BLUE           ) ;
	JAVAANDR.fieldint_Color_CYAN                = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_CYAN           ) ;
	JAVAANDR.fieldint_Color_DKGRAY              = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_DKGRAY         ) ;
	JAVAANDR.fieldint_Color_GRAY                = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_GRAY           ) ;
	JAVAANDR.fieldint_Color_GREEN               = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_GREEN          ) ;
	JAVAANDR.fieldint_Color_LTGRAY              = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_LTGRAY         ) ;
	JAVAANDR.fieldint_Color_MAGENTA             = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_MAGENTA        ) ;
	JAVAANDR.fieldint_Color_RED                 = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_RED            ) ;
	JAVAANDR.fieldint_Color_TRANSPARENT         = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_TRANSPARENT    ) ;
	JAVAANDR.fieldint_Color_WHITE               = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_WHITE          ) ;
	JAVAANDR.fieldint_Color_YELLOW              = env->GetStaticIntField(     JAVAANDR.class_Color,               JAVAANDR.fieldID_Color_YELLOW         ) ;

	GETSTATICOBJECTFIELD( JAVAANDR.fieldobject_TextView_BufferType_EDITABLE,  JAVAANDR.class_TextView_BufferType, JAVAANDR.fieldID_TextView_BufferType_EDITABLE,  "TextView_BufferType_EDITABLE"  ) ;
	GETSTATICOBJECTFIELD( JAVAANDR.fieldobject_TextView_BufferType_NORMAL,    JAVAANDR.class_TextView_BufferType, JAVAANDR.fieldID_TextView_BufferType_NORMAL,    "TextView_BufferType_NORMAL"    ) ;
	GETSTATICOBJECTFIELD( JAVAANDR.fieldobject_TextView_BufferType_SPANNABLE, JAVAANDR.class_TextView_BufferType, JAVAANDR.fieldID_TextView_BufferType_SPANNABLE, "TextView_BufferType_SPANNABLE" ) ;

	JAVAANDR.fieldint_Notification_DEFAULT_LIGHTS                = env->GetStaticIntField( JAVAANDR.class_Notification,        JAVAANDR.fieldID_Notification_DEFAULT_LIGHTS   ) ;
	JAVAANDR.fieldint_Notification_DEFAULT_VIBRATE               = env->GetStaticIntField( JAVAANDR.class_Notification,        JAVAANDR.fieldID_Notification_DEFAULT_VIBRATE  ) ;
	JAVAANDR.fieldint_Notification_FLAG_SHOW_LIGHTS              = env->GetStaticIntField( JAVAANDR.class_Notification,        JAVAANDR.fieldID_Notification_FLAG_SHOW_LIGHTS ) ;
	JAVAANDR.fieldint_PendingIntent_FLAG_CANCEL_CURRENT          = env->GetStaticIntField( JAVAANDR.class_PendingIntent,       JAVAANDR.fieldID_PendingIntent_FLAG_CANCEL_CURRENT ) ;
	JAVAANDR.fieldint_PendingIntent_FLAG_NO_CREATE               = env->GetStaticIntField( JAVAANDR.class_PendingIntent,       JAVAANDR.fieldID_PendingIntent_FLAG_NO_CREATE      ) ;
	JAVAANDR.fieldint_PendingIntent_FLAG_ONE_SHOT                = env->GetStaticIntField( JAVAANDR.class_PendingIntent,       JAVAANDR.fieldID_PendingIntent_FLAG_ONE_SHOT       ) ;
	JAVAANDR.fieldint_PendingIntent_FLAG_UPDATE_CURRENT          = env->GetStaticIntField( JAVAANDR.class_PendingIntent,       JAVAANDR.fieldID_PendingIntent_FLAG_UPDATE_CURRENT ) ;
	GETSTATICSTRINGFIELD( JAVAANDR.fieldstring_Intent_ACTION_VIEW,              JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_ACTION_VIEW,            "Ljava/lang/String;" ) ;
	JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_NEW_TASK              = env->GetStaticIntField( JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_NEW_TASK     ) ;
	JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_SINGLE_TOP            = env->GetStaticIntField( JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_SINGLE_TOP   ) ;
	JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_CLEAR_TOP             = env->GetStaticIntField( JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_CLEAR_TOP    ) ;
	if( API_LV >= 11 )
	{
		JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_CLEAR_TASK            = env->GetStaticIntField( JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_CLEAR_TASK            ) ;
	}
	JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_NO_ANIMATION          = env->GetStaticIntField( JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_NO_ANIMATION          ) ;
	JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_NO_HISTORY            = env->GetStaticIntField( JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_NO_HISTORY            ) ;
	JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_LAUNCHED_FROM_HISTORY = env->GetStaticIntField( JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_LAUNCHED_FROM_HISTORY ) ;
	JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_EXCLUDE_FROM_RECENTS  = env->GetStaticIntField( JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_EXCLUDE_FROM_RECENTS  ) ;
	JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_PREVIOUS_IS_TOP       = env->GetStaticIntField( JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_PREVIOUS_IS_TOP       ) ;
	JAVAANDR.fieldint_Intent_FLAG_ACTIVITY_REORDER_TO_FRONT      = env->GetStaticIntField( JAVAANDR.class_Intent,              JAVAANDR.fieldID_Intent_FLAG_ACTIVITY_REORDER_TO_FRONT      ) ;
	GETSTATICOBJECTFIELD( JAVAANDR.fieldobject_Context_NOTIFICATION_SERVICE,    JAVAANDR.class_Context,             JAVAANDR.fieldID_Context_NOTIFICATION_SERVICE,  "Context_NOTIFICATION_SERVICE" ) ;
	GETSTATICOBJECTFIELD( JAVAANDR.fieldobject_Context_SENSOR_SERVICE,          JAVAANDR.class_Context,             JAVAANDR.fieldID_Context_SENSOR_SERVICE,        "Context_SENSOR_SERVICE"       ) ;
	JAVAANDR.fieldint_SensorManager_AXIS_MINUS_X                 = env->GetStaticIntField( JAVAANDR.class_SensorManager,       JAVAANDR.fieldID_SensorManager_AXIS_MINUS_X ) ;
	JAVAANDR.fieldint_SensorManager_AXIS_MINUS_Y                 = env->GetStaticIntField( JAVAANDR.class_SensorManager,       JAVAANDR.fieldID_SensorManager_AXIS_MINUS_Y ) ;
	JAVAANDR.fieldint_SensorManager_AXIS_MINUS_Z                 = env->GetStaticIntField( JAVAANDR.class_SensorManager,       JAVAANDR.fieldID_SensorManager_AXIS_MINUS_Z ) ;
	JAVAANDR.fieldint_SensorManager_AXIS_X                       = env->GetStaticIntField( JAVAANDR.class_SensorManager,       JAVAANDR.fieldID_SensorManager_AXIS_X       ) ;
	JAVAANDR.fieldint_SensorManager_AXIS_Y                       = env->GetStaticIntField( JAVAANDR.class_SensorManager,       JAVAANDR.fieldID_SensorManager_AXIS_Y       ) ;
	JAVAANDR.fieldint_SensorManager_AXIS_Z                       = env->GetStaticIntField( JAVAANDR.class_SensorManager,       JAVAANDR.fieldID_SensorManager_AXIS_Z       ) ;
	JAVAANDR.fieldint_R_mipmap_sym_def_app_icon                  = env->GetStaticIntField( JAVAANDR.class_R_mipmap,            JAVAANDR.fieldID_R_mipmap_sym_def_app_icon  ) ;


	JAVAANDR.InitializeFlag = TRUE ;

	// 正常終了
	return 0 ;
}
#undef FINDCLASS
#undef GETMETHOD
#undef GETSTATICMETHOD
#undef GETFIELD
#undef GETSTATICFIELD

// Java のクラスやメソッドの参照の後始末をする
#define JAVA_DELETE_GLOBAL_REF( object )\
	if( object != NULL )\
	{\
		env->DeleteGlobalRef( object ) ;\
		object = NULL ;\
	}
extern int TerminateJavaAndroidInfo( JNIEnv *env )
{
	if( JAVAANDR.InitializeFlag == FALSE )
	{
		return -1 ;
	}

	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Build_VERSION ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Bitmap ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_BitmapConfig ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Canvas ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Typeface ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Paint ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_PaintFontMetrics ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Color ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_TextView ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_TextView_BufferType ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_EditText ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_String ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Locale ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_CharSequence ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Dialog ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_AlertDialog ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_AlertDialog_Builder ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Notification ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_NotificationManager ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Notification_Builder ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_PendingIntent ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Intent ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Context ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_SensorManager ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_R_mipmap ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Uri ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.class_Activity ) ;

	JAVA_DELETE_GLOBAL_REF( JAVAANDR.fieldobject_BitmapConfig_ARGB_8888 ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.fieldobject_TextView_BufferType_EDITABLE ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.fieldobject_TextView_BufferType_NORMAL ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.fieldobject_TextView_BufferType_SPANNABLE ) ;
	JAVA_DELETE_GLOBAL_REF( JAVAANDR.fieldobject_Context_NOTIFICATION_SERVICE ) ;

	JAVAANDR.InitializeFlag = FALSE ;

	return 0 ;
}
#undef JAVA_DELETE_GLOBAL_REF

#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE
