// -------------------------------------------------------------------------------
// 
// 		ＤＸライブラリ		Android用Java関係ヘッダファイル
// 
// 				Ver 3.19 
// 
// -------------------------------------------------------------------------------

#ifndef __DXJAVAANDROID_H__
#define __DXJAVAANDROID_H__

// インクルード ------------------------------------------------------------------
#include "../DxCompileConfig.h"
#include <android/configuration.h>
#include <android/native_activity.h>

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// マクロ定義 --------------------------------------------------------------------

#define JAVAANDR		g_JavaAndroidInfo	

// 構造体定義 --------------------------------------------------------------------

struct DXLIB_JAVA_ANDROID_INFO
{
	int						InitializeFlag ;

	jclass					class_Build_VERSION ;
	jclass					class_Bitmap ;
	jclass					class_BitmapConfig ;
	jclass					class_Canvas ;
	jclass					class_Typeface ;
	jclass					class_Paint ;
	jclass					class_PaintFontMetrics ;
	jclass					class_Color ;
	jclass					class_TextView ;
	jclass					class_TextView_BufferType ;
	jclass					class_EditText ;
	jclass					class_String ;
	jclass					class_Locale ;
	jclass					class_Calendar ;
	jclass					class_CharSequence ;
	jclass					class_Dialog ;
	jclass					class_AlertDialog ;
	jclass					class_AlertDialog_Builder ;
	jclass					class_Notification ;
	jclass					class_NotificationManager ;
	jclass					class_Notification_Builder ;
	jclass					class_PendingIntent ;
	jclass					class_Intent ;
	jclass					class_Context ;
	jclass					class_SensorManager ;
	jclass					class_R_mipmap ;
	jclass					class_Uri ;
	jclass					class_Activity ;


	jmethodID				methodID_Bitmap_createBitmap ;
	jmethodID				methodID_Bitmap_setPixels ;
	jmethodID				methodID_Bitmap_getPixels ;
	jmethodID				methodID_Canvas_newCanvas ;
	jmethodID				methodID_Canvas_drawText ;
	jmethodID				methodID_Canvas_drawARGB ;
	jmethodID				methodID_Typeface_create ;
	jmethodID				methodID_Paint_newPaint ;
	jmethodID				methodID_Paint_getFontMetrics ;
	jmethodID				methodID_Paint_setTextSize ;
	jmethodID				methodID_Paint_setTypeface ;
	jmethodID				methodID_Paint_setAntiAlias ;
	jmethodID				methodID_Paint_setARGB ;
	jmethodID				methodID_Paint_measureText ;

	jmethodID				methodID_EditText_newEditText ;
	jmethodID				methodID_EditText_setText ;
	jmethodID				methodID_TextView_getText ;
	jmethodID				methodID_CharSequence_toString ;
	jmethodID				methodID_String_subSequence ;
	jmethodID				methodID_Locale_getDefault ;
	jmethodID				methodID_Locale_getLanguage ;
	jmethodID				methodID_Locale_getCountry ;
	jmethodID				methodID_Calendar_getInstance ;
	jmethodID				methodID_Calendar_get ;
	jmethodID				methodID_Dialog_isShowing ;
	jmethodID				methodID_AlertDialog_Builder_newAlertDialog_Builder ;
	jmethodID				methodID_AlertDialog_Builder_setIcon ;
	jmethodID				methodID_AlertDialog_Builder_setTitle ;
	jmethodID				methodID_AlertDialog_Builder_setView ;
	jmethodID				methodID_AlertDialog_Builder_setPositiveButton ;
	jmethodID				methodID_AlertDialog_Builder_setNegativeButton ;
	jmethodID				methodID_AlertDialog_Builder_show ;
	jmethodID				methodID_Notification_newNotification ;
	jmethodID				methodID_Notification_setLatestEventInfo ;
	jmethodID				methodID_NotificationManager_notify ;
	jmethodID				methodID_NotificationManager_cancel ;
	jmethodID				methodID_NotificationManager_cancelAll ;
	jmethodID				methodID_Notification_Builder_newNotification_Builder ;
	jmethodID				methodID_Notification_Builder_setWhen ;
	jmethodID				methodID_Notification_Builder_setShowWhen ;
	jmethodID				methodID_Notification_Builder_setUsesChronometer ;
	jmethodID				methodID_Notification_Builder_setSmallIcon ;
	jmethodID				methodID_Notification_Builder_setContentTitle ;
	jmethodID				methodID_Notification_Builder_setContentText ;
	jmethodID				methodID_Notification_Builder_setSubText ;
	jmethodID				methodID_Notification_Builder_setNumber ;
	jmethodID				methodID_Notification_Builder_setContentInfo ;
	jmethodID				methodID_Notification_Builder_setProgress ;
	jmethodID				methodID_Notification_Builder_setContent ;
	jmethodID				methodID_Notification_Builder_setContentIntent ;
	jmethodID				methodID_Notification_Builder_setDeleteIntent ;
	jmethodID				methodID_Notification_Builder_setFullScreenIntent ;
	jmethodID				methodID_Notification_Builder_setTicker ;
	jmethodID				methodID_Notification_Builder_setLargeIcon ;
	jmethodID				methodID_Notification_Builder_setSound ;
	jmethodID				methodID_Notification_Builder_setVibrate ;
	jmethodID				methodID_Notification_Builder_setLights ;
	jmethodID				methodID_Notification_Builder_setOngoing ;
	jmethodID				methodID_Notification_Builder_setOnlyAlertOnce ;
	jmethodID				methodID_Notification_Builder_setAutoCancel ;
	jmethodID				methodID_Notification_Builder_setLocalOnly ;
	jmethodID				methodID_Notification_Builder_setDefaults ;
	jmethodID				methodID_Notification_Builder_setPriority ;
	jmethodID				methodID_Notification_Builder_setCategory ;
	jmethodID				methodID_Notification_Builder_addPerson ;
	jmethodID				methodID_Notification_Builder_setGroup ;
	jmethodID				methodID_Notification_Builder_setGroupSummary ;
	jmethodID				methodID_Notification_Builder_setSortKey ;
	jmethodID				methodID_Notification_Builder_addExtras ;
	jmethodID				methodID_Notification_Builder_setExtras ;
	jmethodID				methodID_Notification_Builder_getExtras ;
	jmethodID				methodID_Notification_Builder_addAction ;
	jmethodID				methodID_Notification_Builder_setStyle ;
	jmethodID				methodID_Notification_Builder_setVisibility ;
	jmethodID				methodID_Notification_Builder_setPublicVersion ;
	jmethodID				methodID_Notification_Builder_extend ;
	jmethodID				methodID_Notification_Builder_setColor ;
	jmethodID				methodID_Notification_Builder_getNotification ;
	jmethodID				methodID_Notification_Builder_build ;
	jmethodID				methodID_PendingIntent_getActivity ;
	jmethodID				methodID_PendingIntent_getActivities ;
	jmethodID				methodID_Intent_newIntent ;
	jmethodID				methodID_Intent_newIntent_Uri ;
	jmethodID				methodID_Intent_setFlags ;
	jmethodID				methodID_Intent_setClassName ;
	jmethodID				methodID_Context_getApplicationContext ;
	jmethodID				methodID_Context_getSystemService ;
	jmethodID				methodID_SensorManager_getRotationMatrix ;
	jmethodID				methodID_SensorManager_remapCoordinateSystem ;
	jmethodID				methodID_SensorManager_getOrientation ;
	jmethodID				methodID_Uri_parse ;
	jmethodID				methodID_Activity_startActivity ;


	jfieldID				fieldID_Build_VERSION_SDK_INT ;
	jfieldID				fieldID_BitmapConfig_ARGB_8888 ;
	jfieldID				fieldID_PaintFontMetrics_ascent ;
	jfieldID				fieldID_PaintFontMetrics_bottom ;
	jfieldID				fieldID_PaintFontMetrics_descent ;
	jfieldID				fieldID_PaintFontMetrics_leading ;
	jfieldID				fieldID_PaintFontMetrics_top ;
	jfieldID				fieldID_Typeface_NORMAL ;
	jfieldID				fieldID_Typeface_BOLD ;
	jfieldID				fieldID_Typeface_ITALIC ;
	jfieldID				fieldID_Typeface_BOLD_ITALIC ;
	jfieldID				fieldID_Color_BLACK ;
	jfieldID				fieldID_Color_BLUE ;
	jfieldID				fieldID_Color_CYAN ;
	jfieldID				fieldID_Color_DKGRAY ;
	jfieldID				fieldID_Color_GRAY ;
	jfieldID				fieldID_Color_GREEN ;
	jfieldID				fieldID_Color_LTGRAY ;
	jfieldID				fieldID_Color_MAGENTA ;
	jfieldID				fieldID_Color_RED ;
	jfieldID				fieldID_Color_TRANSPARENT ;
	jfieldID				fieldID_Color_WHITE ;
	jfieldID				fieldID_Color_YELLOW ;        

	jfieldID				fieldID_TextView_BufferType_EDITABLE ;
	jfieldID				fieldID_TextView_BufferType_NORMAL ;
	jfieldID				fieldID_TextView_BufferType_SPANNABLE ;

	jfieldID				fieldID_Notification_DEFAULT_LIGHTS ;
	jfieldID				fieldID_Notification_DEFAULT_VIBRATE ;
	jfieldID				fieldID_Notification_FLAG_SHOW_LIGHTS ;
	jfieldID				fieldID_Notification_icon ;
	jfieldID				fieldID_Notification_tickerText ;
	jfieldID				fieldID_Notification_defaults ;
	jfieldID				fieldID_Notification_vibrate ;
	jfieldID				fieldID_Notification_ledARGB ;
	jfieldID				fieldID_Notification_ledOffMS ;
	jfieldID				fieldID_Notification_ledOnMS ;
	jfieldID				fieldID_Notification_flags ;
	jfieldID				fieldID_PendingIntent_FLAG_CANCEL_CURRENT ;
	jfieldID				fieldID_PendingIntent_FLAG_NO_CREATE ;
	jfieldID				fieldID_PendingIntent_FLAG_ONE_SHOT ;
	jfieldID				fieldID_PendingIntent_FLAG_UPDATE_CURRENT ;
	jfieldID				fieldID_Intent_ACTION_VIEW ;
	jfieldID				fieldID_Intent_FLAG_ACTIVITY_NEW_TASK ;
	jfieldID				fieldID_Intent_FLAG_ACTIVITY_SINGLE_TOP ;
	jfieldID				fieldID_Intent_FLAG_ACTIVITY_CLEAR_TOP ;
	jfieldID				fieldID_Intent_FLAG_ACTIVITY_CLEAR_TASK ;
	jfieldID				fieldID_Intent_FLAG_ACTIVITY_NO_ANIMATION ;
	jfieldID				fieldID_Intent_FLAG_ACTIVITY_NO_HISTORY ;
	jfieldID				fieldID_Intent_FLAG_ACTIVITY_LAUNCHED_FROM_HISTORY ;
	jfieldID				fieldID_Intent_FLAG_ACTIVITY_EXCLUDE_FROM_RECENTS ;
	jfieldID				fieldID_Intent_FLAG_ACTIVITY_PREVIOUS_IS_TOP ;
	jfieldID				fieldID_Intent_FLAG_ACTIVITY_REORDER_TO_FRONT ;      
	jfieldID				fieldID_Context_NOTIFICATION_SERVICE ;
	jfieldID				fieldID_Context_SENSOR_SERVICE ;
	jfieldID				fieldID_SensorManager_AXIS_MINUS_X ;
	jfieldID				fieldID_SensorManager_AXIS_MINUS_Y ;
	jfieldID				fieldID_SensorManager_AXIS_MINUS_Z ;
	jfieldID				fieldID_SensorManager_AXIS_X ;
	jfieldID				fieldID_SensorManager_AXIS_Y ;
	jfieldID				fieldID_SensorManager_AXIS_Z ;
	jfieldID				fieldID_R_mipmap_sym_def_app_icon ;
	jfieldID				fieldID_Calendar_DAY_OF_WEEK ;
	jfieldID				fieldID_Calendar_WEEK_OF_MONTH ;
	jfieldID				fieldID_Calendar_DAY_OF_WEEK_IN_MONTH ;


	jint					fieldint_Build_VERSION_SDK_INT ;
	jobject					fieldobject_BitmapConfig_ARGB_8888 ;
	jint					fieldint_Typeface_NORMAL ;
	jint					fieldint_Typeface_BOLD ;
	jint					fieldint_Typeface_ITALIC ;
	jint					fieldint_Typeface_BOLD_ITALIC ;
	jint					fieldint_Color_BLACK ;
	jint					fieldint_Color_BLUE ;
	jint					fieldint_Color_CYAN ;
	jint					fieldint_Color_DKGRAY ;
	jint					fieldint_Color_GRAY ;
	jint					fieldint_Color_GREEN ;
	jint					fieldint_Color_LTGRAY ;
	jint					fieldint_Color_MAGENTA ;
	jint					fieldint_Color_RED ;
	jint					fieldint_Color_TRANSPARENT ;
	jint					fieldint_Color_WHITE ;
	jint					fieldint_Color_YELLOW ;

	jobject					fieldobject_TextView_BufferType_EDITABLE ;
	jobject					fieldobject_TextView_BufferType_NORMAL ;
	jobject					fieldobject_TextView_BufferType_SPANNABLE ;

	jint					fieldint_Notification_DEFAULT_LIGHTS ;
	jint					fieldint_Notification_DEFAULT_VIBRATE ;
	jint					fieldint_Notification_FLAG_SHOW_LIGHTS ;
	jint					fieldint_PendingIntent_FLAG_CANCEL_CURRENT ;
	jint					fieldint_PendingIntent_FLAG_NO_CREATE ;
	jint					fieldint_PendingIntent_FLAG_ONE_SHOT ;
	jint					fieldint_PendingIntent_FLAG_UPDATE_CURRENT ;
	jstring					fieldstring_Intent_ACTION_VIEW ;
	jint					fieldint_Intent_FLAG_ACTIVITY_NEW_TASK ;
	jint					fieldint_Intent_FLAG_ACTIVITY_SINGLE_TOP ;
	jint					fieldint_Intent_FLAG_ACTIVITY_CLEAR_TOP ;
	jint					fieldint_Intent_FLAG_ACTIVITY_CLEAR_TASK ;
	jint					fieldint_Intent_FLAG_ACTIVITY_NO_ANIMATION ;
	jint					fieldint_Intent_FLAG_ACTIVITY_NO_HISTORY ;
	jint					fieldint_Intent_FLAG_ACTIVITY_LAUNCHED_FROM_HISTORY ;
	jint					fieldint_Intent_FLAG_ACTIVITY_EXCLUDE_FROM_RECENTS ;
	jint					fieldint_Intent_FLAG_ACTIVITY_PREVIOUS_IS_TOP ;
	jint					fieldint_Intent_FLAG_ACTIVITY_REORDER_TO_FRONT ;
	jobject					fieldobject_Context_NOTIFICATION_SERVICE ;
	jobject					fieldobject_Context_SENSOR_SERVICE ;
	jint					fieldint_SensorManager_AXIS_MINUS_X ;
	jint					fieldint_SensorManager_AXIS_MINUS_Y ;
	jint					fieldint_SensorManager_AXIS_MINUS_Z ;
	jint					fieldint_SensorManager_AXIS_X ;
	jint					fieldint_SensorManager_AXIS_Y ;
	jint					fieldint_SensorManager_AXIS_Z ;
	jint					fieldint_R_mipmap_sym_def_app_icon ;
	jint					fieldint_Calendar_DAY_OF_WEEK ;
	jint					fieldint_Calendar_WEEK_OF_MONTH ;
	jint					fieldint_Calendar_DAY_OF_WEEK_IN_MONTH ;
} ;

// テーブル-----------------------------------------------------------------------

// 内部大域変数宣言 --------------------------------------------------------------

extern DXLIB_JAVA_ANDROID_INFO g_JavaAndroidInfo ;

// 関数プロトタイプ宣言-----------------------------------------------------------

// Java のクラスやメソッドの参照を取得する
extern int SetupJavaAndroidInfo( JNIEnv *env ) ;

// Java のクラスやメソッドの参照の後始末をする
extern int TerminateJavaAndroidInfo( JNIEnv *env ) ;

// Java のクラスのグローバル参照を取得する
extern jclass Java_FindClass_Global( JNIEnv *env, const char *name ) ;

// Java のスタティックオブジェクトフィールドのグローバル参照を取得する
extern jobject Java_GetStaticObjectField_Global( JNIEnv *env, jclass clazz, jfieldID fieldID ) ;

// Java のスタティック文字列フィールドのグローバル参照を取得する
extern jstring Java_GetStaticStringField_Global( JNIEnv *env, jclass clazz, jfieldID fieldID ) ;

// wchar_t の文字列から jstring を作成する( Local Ref )
extern jstring Java_Create_jstring_From_wchar_t( JNIEnv *env, const wchar_t *wchar_t_string ) ;

// wchar_t の文字列から CharSequence を作成する( Local Ref )
extern jobject Java_Create_CharSequence_From_wchar_t( JNIEnv *env, const wchar_t *wchar_t_string ) ;

// jstring から wchar_t の文字列を作成する( DXFREE で解放する )
extern int Java_Create_wchar_t_string_From_jstring( JNIEnv *env, jstring _jstring, wchar_t **wchar_t_stringP ) ;

// jstring から TCHAR の文字列を作成する( DXFREE で解放する )
extern int Java_Create_TCHAR_string_From_jstring( JNIEnv *env, jstring _jstring, TCHAR **tchar_stringP ) ;

// VECTOR から長さ 3 の float Array を作成する( Local Ref )
extern jfloatArray Java_Create_floatArray_From_VECTOR( JNIEnv *env, const struct tagVECTOR *Vector ) ;

// 長さ 3 以上の float Array から VECTOR の値を取得する( 0:正常終了  -1:エラー )
extern int Java_Get_VECTOR_From_floatArray( JNIEnv *env, jfloatArray floatArray, struct tagVECTOR *Buffer ) ;

#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // __DXJAVAANDROID_H__
