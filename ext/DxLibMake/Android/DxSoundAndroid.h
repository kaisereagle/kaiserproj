// -------------------------------------------------------------------------------
// 
// 		ＤＸライブラリ		Android用サウンドプログラムヘッダファイル
// 
// 				Ver 3.19 
// 
// -------------------------------------------------------------------------------

#ifndef __DXSOUNDANDROID_H__
#define __DXSOUNDANDROID_H__

#include "../DxCompileConfig.h"

#ifndef DX_NON_SOUND

// インクルード ------------------------------------------------------------------
#include "../DxLib.h"
#include "../DxThread.h"

#include <SLES/OpenSLES.h>
#include <SLES/OpenSLES_Android.h>

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// マクロ定義 --------------------------------------------------------------------

// ストリーム関係の定義
#define STS_BUFSEC_FILE						(256)			// ストリーム風サウンド再生時に確保するサウンドバッファの時間( 単位は STS_DIVNUM分の1秒 )( ファイルからのストリーム再生用 )
#define STS_ADVANCECOPYSEC_FILE				(192)			// ストリーム風サウンド再生時に音声データを先行展開しておく最大時間( 単位は STS_DIVNUM分の1秒 )( ファイルからのストリーム再生用 )
#define STS_MINADVANCECOPYSEC_FILE			(96)			// ストリーム風サウンド再生時に音声データを先行展開しておく最小時間( 単位は STS_DIVNUM分の1秒 )( ファイルからのストリーム再生用 )

// 一つのバッファーキューのバッファの数
#define BUFFERQUEUE_BUFFER_NUM				(16)

// サウンドバッファ再生処理用情報の最大数
#define SOUNDBUFFERPLAYINFO_MAX_NUM			(512)

// 構造体定義 --------------------------------------------------------------------

// バッファキューに渡したデータの情報
struct BUFFERENQUEUE_INFO
{
	DWORD							CompPos ;
	DWORD							AddBytes ;
} ;

// サウンドバッファ再生処理用情報構造体
struct SOUNDBUFFERPLAYINFO
{
	int								SetupFlag ;							// 出力オブジェクトなどの作成が済んでいるかどうかのフラグ
	int								UseFlag ;							// 再生のために使用しているかどうかのフラグ

	DWORD							Channels ;							// チャンネル数
	DWORD							BitsPerSample ;						// １サンプルあたりのビット数
	DWORD							SamplesPerSec ;						// 再生周波数

	SLObjectItf						OutputMixObject ;					// 出力オブジェクト
	SLObjectItf						PlayerObject;						// プレイヤーオブジェクト
	SLPlayItf						PlayerPlayInterface ;				// プレイヤー再生インタフェース
	SLAndroidSimpleBufferQueueItf	PlayerBufferQueueInterface ;		// プレイヤーバッファキューインタフェース
	SLVolumeItf						PlayerVolumeInterface ;				// プレイヤー音量インタフェース
} ;

// サウンドバッファ環境依存情報構造体
struct SOUNDBUFFER_PF
{
	DX_CRITICAL_SECTION				CriticalSection ;					// クリティカルセクション

	volatile int					EnqueueInfoNum ;					// バッファキューに渡したデータの数
	volatile BUFFERENQUEUE_INFO		EnqueueInfo[ BUFFERQUEUE_BUFFER_NUM ] ;	// バッファキューに渡したデータの情報

	int								BufferEndPlayEnqueueNum ;			// バッファキューに再生終了後のキューを積んだ回数

	int								PlaySetupComp ;						// 再生準備ができているか( TRUE:できている  FALSE:できていない )
	int								UseSoundBufferPlayInfoIndex ;		// 使用している再生処理用情報
//	SLObjectItf						OutputMixObject ;					// 出力オブジェクト
//	SLObjectItf						PlayerObject;						// プレイヤーオブジェクト
//	SLPlayItf						PlayerPlayInterface ;				// プレイヤー再生インタフェース
//	SLAndroidSimpleBufferQueueItf	PlayerBufferQueueInterface ;		// プレイヤーバッファキューインタフェース
//	SLVolumeItf						PlayerVolumeInterface ;				// プレイヤー音量インタフェース

	int								StopSoundBufferValid ;
	struct SOUNDBUFFER *			StopSoundBufferPrev ;
	struct SOUNDBUFFER *			StopSoundBufferNext ;
} ;

// サウンドシステム用環境依存データ構造体
struct SOUNDSYSTEMDATA_PF
{
	int							EngineObjectInitialize ;
	SLObjectItf					EngineObject ;							// エンジンオブジェクト
	SLEngineItf					EngineInterface ;						// エンジンインタフェース

	pthread_t					ProcessSoundThread ;					// ProcessStreamSoundMemAll 等を呼ぶスレッドのID
	volatile int				ProcessSoundThreadEndRequest ;			// ProcessStreamSoundMemAll 等を呼ぶスレッドを終了するかどうか

	int							DXSoundProcessStart ;					// ＤＸライブラリのサウンド処理を開始したかどうか

	DX_CRITICAL_SECTION			StopSoundBufferCriticalSection ;		// クリティカルセクション
	struct SOUNDBUFFER *		StopSoundBuffer ;

	DX_CRITICAL_SECTION			SoundBufferPlayInfosCriticalSection ;					// サウンドバッファの再生処理に必要な情報用のクリティカルセクション
	int							SoundBufferPlayInfoSetupErrorNum ;						// サウンドバッファの再生処理に必要な情報のセットアップに失敗した際のセットアップが済んでいる要素数
	int							SoundBufferPlayInfoSetupNum ;							// サウンドバッファの再生処理に必要な情報のセットアップが済んでいる要素の数
	SOUNDBUFFERPLAYINFO			SoundBufferPlayInfos[ SOUNDBUFFERPLAYINFO_MAX_NUM ] ;	// サウンドバッファの再生処理に必要な情報の構造体
} ;

// ＭＩＤＩデータ環境依存情報
struct MIDIHANDLEDATA_PF
{
	int							Dummy ;
} ;

// ＭＩＤＩシステム用環境依存データ構造体
struct MIDISYSTEMDATA_PF
{
	int							Dummy ;
} ;

// 内部大域変数宣言 --------------------------------------------------------------

// 関数プロトタイプ宣言-----------------------------------------------------------

extern	int		MidiCallBackProcess( void ) ;																			// ＭＩＤＩ演奏終了時呼ばれるコールバック関数

extern	void	SoundBuffer_Apply_StopSoundBufferList( void ) ;							// 停止待ちサウンドバッファリストに登録されているサウンドバッファを停止する


#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // DX_NON_SOUND

#endif // __DXSOUNDANDROID_H__
