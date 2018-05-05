// -------------------------------------------------------------------------------
// 
// 		ＤＸライブラリ		システムプログラム
// 
// 				Ver 3.19 
// 
// -------------------------------------------------------------------------------

// ＤＸライブラリ作成時用定義
#define __DX_MAKE

// インクルード ------------------------------------------------------------------
#include "DxSystem.h"
#include "DxBaseFunc.h"
#include "DxUseCLib.h"

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// マクロ定義 --------------------------------------------------------------------

// 構造体定義 --------------------------------------------------------------------

// テーブル-----------------------------------------------------------------------

// 内部大域変数宣言 --------------------------------------------------------------

DXSYSTEMDATA DxSysData ;

// 関数プロトタイプ宣言-----------------------------------------------------------

// プログラム --------------------------------------------------------------------

// 初期化・終了関係

// DxSysData 関係の初期化を行う
extern int DxLib_SysInit( void )
{
	// 終了リクエストのフラグを倒す
	DxSysData.EndRequestFlag = FALSE ;

	// 終了
	return 0 ;
}

// 終了リクエストを行う
extern int DxLib_EndRequest( void )
{
	DxSysData.EndRequestFlag = TRUE ;

	// 終了
	return 0 ;
}

// 終了リクエストの状態を取得する
extern int DxLib_GetEndRequest( void )
{
	return DxSysData.EndRequestFlag ;
}







// ライブラリが初期化されているかどうかを取得する( 戻り値: TRUE=初期化されている  FALSE=されていない )
extern int NS_DxLib_IsInit( void )
{
	return DxSysData.DxLib_InitializeFlag ;
}






// エラー処理関数

// 最後に発生したエラーのエラーコードを取得する( 戻り値　0:エラーが発生していない、又はエラーコード出力に対応したエラーが発生していない　　0以外：エラーコード、DX_ERRORCODE_WIN_DESKTOP_24BIT_COLOR など )
extern int NS_GetLastErrorCode( void )
{
	return DxSysData.LastErrorCode ;
}

// 最後に発生したエラーのエラーメッセージを指定の文字列バッファに取得する
extern int NS_GetLastErrorMessage( TCHAR *StringBuffer, int StringBufferBytes )
{
	ConvString( ( char * )DxSysData.LastErrorMessage, WCHAR_T_CHARCODEFORMAT, ( char * )StringBuffer, StringBufferBytes, _TCHARCODEFORMAT ) ;

	return 0 ;
}

// エラーコード・メッセージを設定する
extern int DxLib_SetLastError( int ErrorCode/* DX_ERRORCODE_WIN_24BIT_COLOR など */, const wchar_t *ErrorMessage )
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

// 書式付きライブラリのエラー処理を行う
extern int DxLib_FmtError( const wchar_t *FormatString , ... )
{
	va_list VaList ;
	wchar_t String[ 1024 ];

	// ログ出力用のリストをセットする
	va_start( VaList , FormatString ) ;

	// 編集後の文字列を取得する
	_VSWNPRINTF( String , sizeof( String ) / 2, FormatString , VaList ) ;

	// 可変長リストのポインタをリセットする
	va_end( VaList ) ;

	// エラー処理にまわす
	return DxLib_Error( String ) ;
}

// 書式付きライブラリのエラー処理を行う
extern int DxLib_FmtErrorUTF16LE( const char *FormatString , ... )
{
	va_list VaList ;
	char String[ 2048 ];

	// ログ出力用のリストをセットする
	va_start( VaList , FormatString ) ;

	// 編集後の文字列を取得する
	CL_vsnprintf( DX_CHARCODEFORMAT_UTF16LE, TRUE, CHAR_CHARCODEFORMAT, WCHAR_T_CHARCODEFORMAT, String, sizeof( String ) / 2, FormatString, VaList ) ;

	// 可変長リストのポインタをリセットする
	va_end( VaList ) ;

	// エラー処理にまわす
	return DxLib_ErrorUTF16LE( String ) ;
}






// 読み込み処理系の関数で非同期読み込みを行うかどうかを設定する( 非同期読み込みに対応している関数のみ有効 )( TRUE:非同期読み込みを行う  FALSE:非同期読み込みを行わない( デフォルト ) )
extern int NS_SetUseASyncLoadFlag( int Flag )
{
	DxSysData.ASyncLoadFlag = Flag ;

	// 終了
	return 0 ;
}

// 非同期読み込みを行うかどうかを取得する( TRUE:非同期読み込みを行う   FALSE:非同期読み込みを行わない )
extern int GetASyncLoadFlag( void )
{
	return DxSysData.ASyncLoadFlag ? TRUE : FALSE ;
}

// ＤＸライブラリのウインドウ関連の機能を使用しないフラグ
extern int NS_SetNotWinFlag( int Flag )
{
	if( Flag == TRUE ) DxSysData.NotDrawFlag = TRUE;
	DxSysData.NotWinFlag = Flag ;

	return 0 ;
}

// 描画機能を使うかどうかのフラグをセットする
extern int NS_SetNotDrawFlag( int Flag )
{
	DxSysData.NotDrawFlag = Flag ;

	return 0 ;
}

// 描画機能を使うかどうかのフラグを取得する
extern int NS_GetNotDrawFlag( void )
{
	return DxSysData.NotDrawFlag ;
}

// サウンド機能を使うかどうかのフラグをセットする
extern int NS_SetNotSoundFlag( int Flag )
{
	DxSysData.NotSoundFlag = Flag ;

	return 0;
}

// 入力状態取得機能を使うかどうかのフラグをセットする
extern int NS_SetNotInputFlag( int Flag )
{
	DxSysData.NotInputFlag = Flag ;

	return 0;
}














// ウエイト系関数

// 指定の時間だけ処理をとめる
extern int NS_WaitTimer( int WaitTime )
{
	LONGLONG StartTime, EndTime ;

	StartTime = NS_GetNowHiPerformanceCount( FALSE ) ;

	// 4msec前まで寝る
	WaitTime *= 1000 ;
	if( WaitTime > 4000 )
	{
		// 指定時間の間メッセージループ
		EndTime = StartTime + WaitTime - 4000 ;
		while( ProcessMessage() == 0 && EndTime > NS_GetNowHiPerformanceCount( FALSE ) )
		{
			Thread_Sleep( 1 ) ;
		}
	}

	// 4msec以下の分は正確に待つ
	EndTime = StartTime + WaitTime ;
	while( EndTime > NS_GetNowHiPerformanceCount( FALSE ) ){}

	// 終了
	return 0 ;
}

#ifndef DX_NON_INPUT

// キーの入力待ち
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





