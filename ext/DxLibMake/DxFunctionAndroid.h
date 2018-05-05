// -------------------------------------------------------------------------------
// 
// 		ＤＸライブラリ		Android専用関数プロトタイプ宣言用ヘッダファイル
// 
// 				Ver 3.19 
// 
// -------------------------------------------------------------------------------

#ifndef __DXFUNCTIONANDROID
#define __DXFUNCTIONANDROID

// インクルード ------------------------------------------------------------------
#include <android/native_activity.h>
#include <android/window.h>

// 定義---------------------------------------------------------------------------

// 関数プロトタイプ宣言-----------------------------------------------------------

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// ソフトのデータ保存用のディレクトリパスを取得する
extern int GetInternalDataPath( TCHAR *PathBuffer, int PathBufferBytes ) ;

// ソフトの外部データ保存用のディレクトリパスを取得する
extern int GetExternalDataPath( TCHAR *PathBuffer, int PathBufferBytes ) ;

// 端末に設定されている言語を取得する( 戻り値　-1：エラー　0以上：言語名文字列の格納に必要なバイト数 )
extern int GetLanguage( TCHAR *StringBuffer, int StringBufferSize ) ;

// 端末に設定されている国を取得する( 戻り値　-1：エラー　0以上：国名文字列の格納に必要なバイト数 )
extern int GetCountry( TCHAR *StringBuffer, int StringBufferSize ) ;

// アプリで使用している NativeActivity を取得する
extern const ANativeActivity *GetNativeActivity( void ) ;

// ディスプレイの解像度を取得する
extern int GetAndroidDisplayResolution( int *SizeX, int *SizeY ) ;

// 加速度センサーのベクトル値を取得する
extern VECTOR GetAccelerometerVector( void ) ;

// センサーから得られる方角を取得する
// 縦持ち( 地面に対して端末を垂直に持った状態 )の場合に正しく値が取得できます
// 戻り値の x:方角( 単位：ラジアン  0.0f:北  π/2.0f:東  -π/2.0f:西  -π or π:南 )
// 戻り値の y:前後の傾斜( 単位：ラジアン  0.0f:端末が縦持ちで垂直の状態  π/2.0f:前方方向に90度倒した状態( 画面が空を向いている状態 )  -π/2.0f:後方方向に90度倒した状態( 画面が地面を向いている状態 )  -π or π:端末が上下さかさまで垂直の状態 )
// 戻り値の z:左右の傾斜( 単位：ラジアン  0.0f:端末が縦持ちで垂直の状態  π/2.0f:右方向に90度倒した状態  -π/2.0f:左方向に90度倒した状態  -π or π:端末が上下さかさまで垂直の状態 )
extern VECTOR GetOrientationVector( void ) ;

// センサーのベクトル値を取得する
extern VECTOR GetAndroidSensorVector( int SensorType /* DX_ANDROID_SENSOR_ACCELEROMETER など */ ) ;

// 曜日や週に関する情報を取得する
// DayOfWeek : 曜日( 1:日曜日 2:月曜日 3:火曜日 4:水曜日 5:木曜日 6:金曜日 7:土曜日 )
// WeekOfMonth : 今日が今月の何週目なのかの値( 1:１週目  2:２週目  3:３週目  4:４週目  5:５週目 )
// DayOfWeekInMonth : 今日の曜日が今月何回目に当たるか、の値( 1:１回目  2:２回目  3:３回目  4:４回目 ... )
extern int GetAndroidWeekInfo( int *DayOfWeek, int *WeekOfMonth, int *DayOfWeekInMonth ) ;

// 通知を発行する
// Title         : タイトル
// SubTitle      : サブタイトル
// Icon          : アイコン ( -1 でデフォルトのアイコン )
// ShowWhen      : 通知時刻を表示するか ( TRUE : 表示する  FALSE : 表示しない )
// AutoCancel    : 通知がタップされたら通知を削除するか ( TRUE : タップされたら通知を削除する   FALSE : Cancel されるまで通知を削除しない )
// NotifyID      : 通知ID
// Vibrate       : 振動パターン用の int型配列( 時間の単位はミリ秒 )、NULL の場合は振動なし( Vibrate[0]:振動停止時間  Vibrate[1]:振動時間  Vibrate[2]:振動停止時間  Vibrate[3]:振動時間  ... 以下繰り返し )
// VibrateLength : Vibrate 配列の要素数
// LightColor    : 通知時のライトの色( GetColor で取得 )、LightOnTime か LightOffTime が 0 の場合はライトの点灯無し
// LightOnTime   : ライトの点灯時間( 単位：ミリ秒 )
// LightOffTime  : ライトの消灯時間( 単位：ミリ秒 )
extern int AndroidNotification(
	const TCHAR *Title, const TCHAR *SubTitle, int Icon = -1, int ShowWhen = TRUE,
	int AutoCancel = TRUE, int NotifyId = 0, int *Vibrate = NULL, int VibrateLength = 0,
	unsigned int LightColor = 0, int LightOnTime = 0, int LightOffTime = 0 ) ;

// 通知をキャンセルする
// NotifyID : 通知ID
extern int AndroidNotificationCancel( int NotifyId ) ;

// 全ての通知をキャンセルする
extern int AndroidNotificationCancelAll( void ) ;

// 指定の URL をブラウザで開く( BrowserAppPackageName か BrowserAppClassName が NULL の場合は標準ブラウザで開く )
// URL                   : 開くURL
// BrowserAppPackageName : ブラウザのパッケージ名( NULL で標準ブラウザ )
// BrowserAppClassName   : ブラウザのクラス名( NULL で標準ブラウザ )
extern int AndroidJumpURL( const TCHAR *URL, const TCHAR *BrowserAppPackageName = NULL, const TCHAR *BrowserAppClassName = NULL ) ;

// アプリが非アクティブになった際に呼ばれるコールバック関数を登録する
extern int SetAndroidLostFocusCallbackFunction( void (* Callback )( void *Data ), void *CallbackData ) ;

// アプリがアクティブになった際に呼ばれるコールバック関数を登録する
extern int SetAndroidGainedFocusCallbackFunction( void (* Callback )( void *Data ), void *CallbackData ) ;

// OpenGL 関係の情報を取得する
extern int GetOpenGLInfo(
	TCHAR **Version = NULL,
	TCHAR **Renderer = NULL,
	TCHAR **Vendor = NULL,
	TCHAR ***ExtensionNames = NULL,
	TCHAR **Shader_Language_Version = NULL,
	int *Shader_Max_Vertex_Attribs = NULL,						// GL_MAX_VERTEX_ATTRIBS
	int *Shader_Max_Vertex_Uniform_Vectors = NULL,				// GL_MAX_VERTEX_UNIFORM_VECTORS
	int *Shader_Max_Varying_Vectors = NULL,						// GL_MAX_VARYING_VECTORS
	int *Shader_Max_Combined_Texture_Image_Units = NULL,		// GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS
	int *Shader_Max_Vertex_Texture_Image_Units = NULL,			// GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS
	int *Shader_Max_Texture_Image_Units = NULL,					// GL_MAX_TEXTURE_IMAGE_UNITS
	int *Shader_Max_Fragment_Uniform_Vectors = NULL				// GL_MAX_FRAGMENT_UNIFORM_VECTORS
) ;


#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // __DXFUNCTIONANDROID

