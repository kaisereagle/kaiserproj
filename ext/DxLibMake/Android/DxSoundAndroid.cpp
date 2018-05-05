//-----------------------------------------------------------------------------
// 
// 		ＤＸライブラリ		Android用サウンドプログラム
// 
//  	Ver 3.19 
// 
//-----------------------------------------------------------------------------

// ＤＸライブラリ作成時用定義
#define __DX_MAKE

#include "../DxCompileConfig.h"

#ifndef DX_NON_SOUND

// インクルード----------------------------------------------------------------
#include "DxSoundAndroid.h"
#include "DxFileAndroid.h"
#include "../DxSound.h"
#include "../DxSystem.h"
#include "../DxMemory.h"
#include "../DxLog.h"

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// マクロ定義------------------------------------------------------------------

// ストリーム再生用のバッファ一つ辺りのサンプル数
//#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(512)
#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(735)
//#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(1024)
//#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(2048)
//#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(4096)
//#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(8192)

// 型定義----------------------------------------------------------------------

// データ宣言------------------------------------------------------------------

// 8bit波形を16bit波形に変換するためのテーブル
static short Bit8To16Table[ 256 ] ;

// 無音データ
static BYTE g_NoneSound8bit[  STREAM_SOUND_BUFFER_UNIT_SAPMLES ] ;
static WORD g_NoneSound16bit[ STREAM_SOUND_BUFFER_UNIT_SAPMLES ] ;

// 関数プロトタイプ宣言 -------------------------------------------------------

static void RefreshSoundBufferCurrentTime( SOUNDBUFFER *Buffer, SLAndroidSimpleBufferQueueState BufferQueState, int IsCriticalsectionLock = TRUE, int EnableStopSoundBuffer = FALSE ) ;	// サウンドバッファの再生時間を更新する
static void PlayerBufferQueueCallback( SLAndroidSimpleBufferQueueItf , void *Data ) ;	// サウンドデータをサウンドバッファに追加する
static void EnqueueSoundBuffer( SOUNDBUFFER *Buffer ) ;									// サウンドデータをサウンドバッファに追加する
static void *StreamSoundThreadFunction( void *argc ) ;

static void SoundBuffer_Add_StopSoundBufferList( SOUNDBUFFER *Buffer ) ;			// 停止待ちサウンドバッファリストに指定のサウンドバッファを加える
static void SoundBuffer_Sub_StopSoundBufferList( SOUNDBUFFER *Buffer ) ;			// 停止待ちサウンドバッファリストから指定のサウンドバッファを外す

static void TerminateSoundBufferPlayInfo( int InfoIndex ) ;							// サウンドバッファ再生用情報の後始末を行う( クリティカルセクションをロックした状態で呼ぶこと )
static int NotUseSoundBufferPlayInfo( int InfoIndex ) ;								// 指定のサウンドバッファ再生用情報を使用していない状態に変更する
static int SetupSoundBufferPlayInfo( SOUNDBUFFER *Buffer ) ;						// サウンドバッファ再生用情報のセットアップと、セットアップした情報の番号を取得する


// プログラム------------------------------------------------------------------

// 停止待ちサウンドバッファリストに指定のサウンドバッファを加える
static void SoundBuffer_Add_StopSoundBufferList( SOUNDBUFFER *Buffer )
{
	// クリティカルセクションの取得
	CRITICALSECTION_LOCK( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;

	// リストに追加
	if( Buffer->PF.StopSoundBufferValid == FALSE )
	{
		Buffer->PF.StopSoundBufferValid = TRUE ;

		Buffer->PF.StopSoundBufferPrev = NULL ;
		Buffer->PF.StopSoundBufferNext = SoundSysData.PF.StopSoundBuffer ;
		if( SoundSysData.PF.StopSoundBuffer != NULL )
		{
			SoundSysData.PF.StopSoundBuffer->PF.StopSoundBufferPrev = Buffer ;
		}
		SoundSysData.PF.StopSoundBuffer = Buffer ;
	}

	// クリティカルセクションの解放
	CriticalSection_Unlock( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;
}

// 停止待ちサウンドバッファリストから指定のサウンドバッファを外す
static void SoundBuffer_Sub_StopSoundBufferList( SOUNDBUFFER *Buffer )
{
	// クリティカルセクションの取得
	CRITICALSECTION_LOCK( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;

	if( Buffer->PF.StopSoundBufferValid )
	{
		Buffer->PF.StopSoundBufferValid = FALSE ;

		if( Buffer->PF.StopSoundBufferNext != NULL )
		{
			Buffer->PF.StopSoundBufferNext->PF.StopSoundBufferPrev = Buffer->PF.StopSoundBufferPrev ;
		}

		if( Buffer->PF.StopSoundBufferPrev != NULL )
		{
			Buffer->PF.StopSoundBufferPrev->PF.StopSoundBufferNext = Buffer->PF.StopSoundBufferNext ;
		}
		else
		{
			SoundSysData.PF.StopSoundBuffer = Buffer->PF.StopSoundBufferNext ;
		}

		Buffer->PF.StopSoundBufferNext = NULL ;
		Buffer->PF.StopSoundBufferPrev = NULL ;
	}

	// クリティカルセクションの解放
	CriticalSection_Unlock( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;
}

// 停止待ちサウンドバッファリストに登録されているサウンドバッファを停止する
extern void SoundBuffer_Apply_StopSoundBufferList( void )
{
	for(;;)
	{
		SOUNDBUFFER *StopSoundBuffer = NULL ;

		// クリティカルセクションの取得
		CRITICALSECTION_LOCK( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;

		StopSoundBuffer = SoundSysData.PF.StopSoundBuffer ;

		// クリティカルセクションの解放
		CriticalSection_Unlock( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;

		// 登録されているサウンドバッファが無ければ終了
		if( StopSoundBuffer == NULL )
		{
			break ;
		}

		// 再生の停止
		SoundBuffer_Stop( StopSoundBuffer ) ;
	}

}

// サウンドバッファの再生時間を更新する
static void RefreshSoundBufferCurrentTime( SOUNDBUFFER *Buffer, SLAndroidSimpleBufferQueueState BufferQueState, int IsCriticalsectionLock, int EnableStopSoundBuffer )
{
	if( IsCriticalsectionLock )
	{
		// クリティカルセクションの取得
		CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;
	}

	// 再生位置の更新
	if( Buffer->PF.EnqueueInfoNum > 0 )
	{
		int OutputNum ;

		OutputNum = Buffer->PF.EnqueueInfoNum - BufferQueState.count ;
		if( OutputNum > 0 )
		{
			Buffer->Pos = Buffer->PF.EnqueueInfo[ OutputNum - 1 ].CompPos ;

			Buffer->PF.EnqueueInfoNum -= OutputNum ;
			if( Buffer->PF.EnqueueInfoNum > 0 )
			{
				_MEMMOVE(
					( void * )&Buffer->PF.EnqueueInfo[ 0 ],
					( void * )&Buffer->PF.EnqueueInfo[ OutputNum ],
					sizeof( BUFFERENQUEUE_INFO ) * Buffer->PF.EnqueueInfoNum
				) ;
			}

			if( Buffer->Pos == Buffer->SampleNum && Buffer->PF.BufferEndPlayEnqueueNum > 0 )
			{
				if( EnableStopSoundBuffer )
				{
					SoundBuffer_Stop( Buffer ) ;
				}
				else
				{
					// 停止待ちサウンドバッファリストに追加
					SoundBuffer_Add_StopSoundBufferList( Buffer ) ;
				}
			}
		}
	}

	if( IsCriticalsectionLock )
	{
		// クリティカルセクションの解放
		CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;
	}
}

// バッファキューインターフェースから呼ばれるコールバック関数
static void PlayerBufferQueueCallback( SLAndroidSimpleBufferQueueItf BufferQueueInterface, void *Data )
{
	EnqueueSoundBuffer( ( SOUNDBUFFER * )Data ) ;
}

// サウンドデータをサウンドバッファに追加する
static void EnqueueSoundBuffer( SOUNDBUFFER *Buffer )
{
	DWORD BytesRequired ;
	SLAndroidSimpleBufferQueueState BufferQueState ;
	int AddMaxCount ;
	int i ;

	// クリティカルセクションの取得
	CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;
	CRITICALSECTION_LOCK( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	if( Buffer->State == FALSE ||
		Buffer->PF.UseSoundBufferPlayInfoIndex < 0 ||
		Buffer->PF.UseSoundBufferPlayInfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
	{
		// クリティカルセクションの解放
		CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;
		CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;
		return ;
	}

	SOUNDBUFFERPLAYINFO *SoundBufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ Buffer->PF.UseSoundBufferPlayInfoIndex ] ;

	int CompPos ;
	int Loop ;
	void *SampleBuffer ;
	int BlockAlign ;

	if( SoundBufferPlayInfo->UseFlag == FALSE ||
		SoundBufferPlayInfo->PlayerObject == NULL ||
		SoundBufferPlayInfo->PlayerPlayInterface == NULL ||
		SoundBufferPlayInfo->PlayerBufferQueueInterface == NULL ||
		( *SoundBufferPlayInfo->PlayerBufferQueueInterface )->GetState( SoundBufferPlayInfo->PlayerBufferQueueInterface, &BufferQueState ) != SL_RESULT_SUCCESS )
	{
		// クリティカルセクションの解放
		CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;
		CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;
		return ;
	}

	// 再生位置の更新
	RefreshSoundBufferCurrentTime( Buffer, BufferQueState, FALSE ) ;

	SampleBuffer	= Buffer->Wave->DoubleSizeBuffer ? Buffer->Wave->DoubleSizeBuffer : Buffer->Wave->Buffer ;
	BlockAlign		= Buffer->Format.wBitsPerSample * Buffer->Format.nChannels / 8 ;
	BytesRequired	= STREAM_SOUND_BUFFER_UNIT_SAPMLES * BlockAlign ;

	CompPos		= Buffer->CompPos ;
	Loop		= Buffer->Loop ;
	AddMaxCount	= BUFFERQUEUE_BUFFER_NUM - BufferQueState.count ;
	for( i = 0 ; i < AddMaxCount ; i ++ )
	{
		void *AddBuffer ;
		SLuint32 AddBytes ;

		if( CompPos >= Buffer->SampleNum )
		{
			if( Loop )
			{
				CompPos = 0 ;
				goto COPYDATA ;
			}
			else
			{
//				if( i == 0 && BufferQueState.count == 0 )
//				{
//					SoundBuffer_Stop( Buffer ) ;
//				}

				// 無音バッファを積む
				switch( Buffer->Format.wBitsPerSample )
				{
				case 8 :
					AddBuffer = g_NoneSound8bit ;
					AddBytes  = sizeof( g_NoneSound8bit ) ;
					break ;

				case 16 :
					AddBuffer = g_NoneSound16bit ;
					AddBytes  = sizeof( g_NoneSound16bit ) ;
					break ;
				}

				if( Buffer->PF.PlaySetupComp )
				{
					( *SoundBufferPlayInfo->PlayerBufferQueueInterface )->Enqueue(
						SoundBufferPlayInfo->PlayerBufferQueueInterface,
						AddBuffer,
						AddBytes
					) ;
				}

				Buffer->PF.EnqueueInfo[ Buffer->PF.EnqueueInfoNum ].CompPos = Buffer->SampleNum ;
				Buffer->PF.EnqueueInfo[ Buffer->PF.EnqueueInfoNum ].AddBytes = AddBytes ;
				Buffer->PF.EnqueueInfoNum ++ ;

				Buffer->PF.BufferEndPlayEnqueueNum ++ ;
				break ;
			}
		}
		else
		{
			DWORD NowBytes ;
			DWORD AddSamples ;

COPYDATA :
			NowBytes	= ( DWORD )( CompPos * BlockAlign ) ;
			AddBuffer	= ( BYTE * )SampleBuffer + NowBytes ;
			AddSamples	= ( DWORD )( Buffer->SampleNum - CompPos ) ;
			AddBytes	= AddSamples * BlockAlign ;
			if( AddBytes > BytesRequired )
			{
				AddSamples = BytesRequired / BlockAlign ;
				AddBytes = AddSamples * BlockAlign ;
			}

			if( Buffer->PF.PlaySetupComp )
			{
				( *SoundBufferPlayInfo->PlayerBufferQueueInterface )->Enqueue(
					SoundBufferPlayInfo->PlayerBufferQueueInterface,
					AddBuffer,
					AddBytes
				) ;
			}

			Buffer->PF.EnqueueInfo[ Buffer->PF.EnqueueInfoNum ].CompPos = CompPos ;
			Buffer->PF.EnqueueInfo[ Buffer->PF.EnqueueInfoNum ].AddBytes = AddBytes ;
			Buffer->PF.EnqueueInfoNum ++ ;

			Buffer->PF.BufferEndPlayEnqueueNum = 0 ;

			CompPos += AddSamples ;
		}

		Buffer->CompPos	= CompPos ;
	}

	// クリティカルセクションの解放
	CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;
	CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;
}

// ストリームサウンド処理用スレッド
static void *StreamSoundThreadFunction( void *argc )
{
	while( SoundSysData.PF.ProcessSoundThreadEndRequest == FALSE )
	{
		// クリティカルセクションの取得
		CRITICALSECTION_LOCK( &HandleManageArray[ DX_HANDLETYPE_SOUND ].CriticalSection ) ;

		// 停止待ちサウンドバッファリストに登録されているサウンドバッファを停止する
		SoundBuffer_Apply_StopSoundBufferList() ;

		// ストリーミング処理
		NS_ProcessStreamSoundMemAll() ;

		// 再生が終了したらハンドルを削除する処理を行う
//		ProcessPlayFinishDeleteSoundMemAll() ;

		// ３Ｄサウンドを再生しているサウンドハンドルに対する処理を行う
		ProcessPlay3DSoundMemAll() ;

		// クリティカルセクションの解放
		CriticalSection_Unlock( &HandleManageArray[ DX_HANDLETYPE_SOUND ].CriticalSection ) ;


		// クリティカルセクションの取得
		CRITICALSECTION_LOCK( &HandleManageArray[ DX_HANDLETYPE_SOFTSOUND ].CriticalSection ) ;

		// ストリーミング処理
		ST_SoftSoundPlayerProcessAll() ;

		// クリティカルセクションの解放
		CriticalSection_Unlock( &HandleManageArray[ DX_HANDLETYPE_SOFTSOUND ].CriticalSection ) ;

		// 待ち
		Thread_Sleep( 10 ) ;
	}

	return NULL ;
}

// サウンドシステムを初期化する関数の環境依存処理を行う関数
extern int InitializeSoundSystem_PF_Timing0( void )
{
	int i ;

	if( SoundSysData.PF.EngineObjectInitialize )
	{
		return 0 ;
	}

	DXST_LOGFILE_ADDUTF16LE( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\x1d\x52\x1f\x67\x16\x53\x8b\x95\xcb\x59\x0a\x00\x00"/*@ L"OpenSL ES の初期化開始\n" @*/ ) ;

	DXST_LOGFILE_TABADD ;

	// ストップサウンドバッファ用のクリティカルセクションを初期化
	if( CriticalSection_Initialize( &SoundSysData.PF.StopSoundBufferCriticalSection ) < 0 )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\x5c\x50\x62\x6b\xe6\x51\x06\x74\x28\x75\x6e\x30\xaf\x30\xea\x30\xc6\x30\xa3\x30\xab\x30\xeb\x30\xbb\x30\xaf\x30\xb7\x30\xe7\x30\xf3\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES のサウンドバッファ停止処理用のクリティカルセクションの作成に失敗しました\n" @*/ )) ;
		DXST_LOGFILE_TABSUB ;
		return -1 ;
	}

	// サウンドバッファ再生処理情報用のクリティカルセクションを初期化
	if( CriticalSection_Initialize( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) < 0 )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\x8d\x51\x1f\x75\xe6\x51\x06\x74\xc5\x60\x31\x58\x28\x75\x6e\x30\xaf\x30\xea\x30\xc6\x30\xa3\x30\xab\x30\xeb\x30\xbb\x30\xaf\x30\xb7\x30\xe7\x30\xf3\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES のサウンドバッファ再生処理情報用のクリティカルセクションの作成に失敗しました\n" @*/ )) ;
		DXST_LOGFILE_TABSUB ;
		return -1 ;
	}

	// 8bit波形を16bit波形に変換するためのテーブルを初期化
	for( i = 0 ; i < 256 ; i ++ )
	{
		Bit8To16Table[ i ] = ( short )( ( ( int )i * 65535 ) / 255 - 32768 ) ;
	}

	// 無音データの初期化
	for( i = 0 ; i < STREAM_SOUND_BUFFER_UNIT_SAPMLES ; i ++ )
	{
		g_NoneSound8bit[ i ]  = 128 ;
		g_NoneSound16bit[ i ] = 0 ;
	}
 
	// エンジンオブジェクト作成
	if( slCreateEngine( &SoundSysData.PF.EngineObject, 0, NULL, 0, NULL, NULL ) != SL_RESULT_SUCCESS )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xa8\x30\xf3\x30\xb8\x30\xf3\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES のエンジンオブジェクトの作成に失敗しました\n" @*/ )) ;
		DXST_LOGFILE_TABSUB ;
		return -1 ;
	}
 
	// エンジンオブジェクトのリアライズ
    if( ( *SoundSysData.PF.EngineObject )->Realize( SoundSysData.PF.EngineObject, SL_BOOLEAN_FALSE ) != SL_RESULT_SUCCESS )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xa8\x30\xf3\x30\xb8\x30\xf3\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xea\x30\xa2\x30\xe9\x30\xa4\x30\xba\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES のエンジンオブジェクトのリアライズに失敗しました\n" @*/ )) ;
		DXST_LOGFILE_TABSUB ;
		return -1 ;
	}

	// エンジンオブジェクトのインターフェースを取得
    if( ( *SoundSysData.PF.EngineObject )->GetInterface( SoundSysData.PF.EngineObject, SL_IID_ENGINE, &SoundSysData.PF.EngineInterface ) != SL_RESULT_SUCCESS )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xa8\x30\xf3\x30\xb8\x30\xf3\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xa4\x30\xb9\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES のエンジンオブジェクトのインターフェース取得に失敗しました\n" @*/ )) ;
		DXST_LOGFILE_TABSUB ;
		return -1 ;
	}

	// ProcessStreamSoundMemAll 等を呼ぶスレッドの開始
	{
		pthread_attr_t attr ;
		int returnCode ;

		pthread_attr_init( &attr ) ;
		pthread_attr_setstacksize( &attr, 128 * 1024 ) ;

		returnCode = pthread_create(
			&SoundSysData.PF.ProcessSoundThread,
			&attr,
			StreamSoundThreadFunction,
			NULL
		) ;
		if( returnCode != 0 )
		{
			DXST_LOGFILEFMT_ADDUTF16LE(( "\x50\x00\x72\x00\x6f\x00\x63\x00\x65\x00\x73\x00\x73\x00\x53\x00\x74\x00\x72\x00\x65\x00\x61\x00\x6d\x00\x53\x00\x6f\x00\x75\x00\x6e\x00\x64\x00\x4d\x00\x65\x00\x6d\x00\x41\x00\x6c\x00\x6c\x00\x20\x00\x49\x7b\x92\x30\x7c\x54\x76\x30\xb9\x30\xec\x30\xc3\x30\xc9\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x45\x00\x72\x00\x72\x00\x6f\x00\x72\x00\x20\x00\x43\x00\x6f\x00\x64\x00\x65\x00\x20\x00\x3a\x00\x20\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x58\x00\x0a\x00\x00"/*@ L"ProcessStreamSoundMemAll 等を呼ぶスレッドの作成に失敗しました Error Code : 0x%08X\n" @*/, returnCode )) ;
			DXST_LOGFILE_TABSUB ;
			return -1 ;
		}
	}

	DXST_LOGFILE_TABSUB ;

	SoundSysData.PF.EngineObjectInitialize = TRUE ;

	DXST_LOGFILE_ADDUTF16LE( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\x1d\x52\x1f\x67\x16\x53\x8c\x5b\x86\x4e\x0a\x00\x00"/*@ L"OpenSL ES の初期化完了\n" @*/ ) ;

	// 終了
	return 0 ;
}


// サウンドシステムを初期化する関数の環境依存処理を行う関数( 実行箇所区別１ )
extern	int		InitializeSoundSystem_PF_Timing1( void )
{
#ifndef DX_NON_MULTITHREAD
	SoundSysData.PF.DXSoundProcessStart = TRUE ;
#endif // DX_NON_MULTITHREAD

	// 正常終了
	return 0 ;
}




// サウンドシステムの後始末をする関数の環境依存処理を行う関数( 実行箇所区別０ )
extern	int		TerminateSoundSystem_PF_Timing0( void )
{
#ifndef DX_NON_MULTITHREAD
	SoundSysData.PF.DXSoundProcessStart = FALSE ;
#endif // DX_NON_MULTITHREAD

	// ProcessStreamSoundMemAll 等を呼ぶスレッドを終了する
	SoundSysData.PF.ProcessSoundThreadEndRequest = TRUE ;
	pthread_join( SoundSysData.PF.ProcessSoundThread, NULL ) ;

	// 正常終了
	return 0 ;
}


// サウンドシステムの後始末をする関数の環境依存処理を行う関数( 実行箇所区別１ )
extern	int		TerminateSoundSystem_PF_Timing1( void )
{
	SoundSysData.PF.EngineObjectInitialize = FALSE ;

	// サウンドバッファ再生用情報の後始末
	{
		int i ;
		SOUNDBUFFERPLAYINFO *BufferPlayInfo ;

		// クリティカルセクションの取得
		CRITICALSECTION_LOCK( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

		BufferPlayInfo = SoundSysData.PF.SoundBufferPlayInfos ;
		for( i = 0 ; i < SOUNDBUFFERPLAYINFO_MAX_NUM ; i ++, BufferPlayInfo ++ )
		{
			if( BufferPlayInfo->SetupFlag )
			{
				TerminateSoundBufferPlayInfo( i ) ;
			}
		}

		// クリティカルセクションの解放
		CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;
	}

	// エンジンオブジェクトの後始末
    if( SoundSysData.PF.EngineObject )
	{
        ( *SoundSysData.PF.EngineObject )->Destroy( SoundSysData.PF.EngineObject ) ;
        SoundSysData.PF.EngineObject = NULL ;
    }

	// ストップサウンドバッファ用のクリティカルセクションを削除
	CriticalSection_Delete( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;

	// サウンドバッファ再生処理情報用のクリティカルセクションを削除
	CriticalSection_Delete( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	// 正常終了
	return 0 ;
}

// サウンドシステムの初期化チェックの環境依存処理を行う関数( TRUE:初期化されている  FALSE:初期化されていない )
extern	int		CheckSoundSystem_Initialize_PF( void )
{
	return SoundSysData.PF.EngineObjectInitialize ;
}

// ＭＩＤＩハンドルの後始末を行う関数の環境依存処理
extern	int		TerminateMidiHandle_PF( MIDIHANDLEDATA *MusicData )
{
	// 正常終了
	return 0 ;
}

// プリセットの３Ｄサウンド用のリバーブパラメータを取得する処理の環境依存処理を行う関数
extern	int		Get3DPresetReverbParamSoundMem_PF( SOUND3D_REVERB_PARAM *ParamBuffer, int PresetNo /* DX_REVERB_PRESET_DEFAULT 等 */ )
{
	return 0 ;
}

// ３Ｄサウンドのリスナーの位置とリスナーの前方位置とリスナーの上方向位置を設定する処理の環境依存処理を行う関数
extern int Set3DSoundListenerPosAndFrontPosAndUpVec_PF( VECTOR Position, VECTOR FrontPosition, VECTOR UpVector )
{
	return 0 ;
}

// ３Ｄサウンドのリスナーの移動速度を設定する処理の環境依存処理を行う関数
extern int Set3DSoundListenerVelocity_PF( VECTOR Velocity )
{
	return 0 ;
}

// ３Ｄサウンドのリスナーの可聴角度範囲を設定する処理の環境依存処理を行う関数
extern int Set3DSoundListenerConeAngle_PF( float InnerAngle, float OuterAngle )
{
	return 0 ;
}

// ３Ｄサウンドのリスナーの可聴角度範囲の音量倍率を設定する処理の環境依存処理を行う関数
extern int Set3DSoundListenerConeVolume_PF( float InnerAngleVolume, float OuterAngleVolume )
{
	return 0 ;
}

// LoadMusicMemByMemImage の実処理関数の環境依存処理を行う関数
extern int LoadMusicMemByMemImage_Static_PF( MIDIHANDLEDATA *MusicData, int ASyncThread )
{
	return 0 ;
}

// 読み込んだＭＩＤＩデータの演奏を開始する処理の環境依存処理を行う関数
extern int PlayMusicMem_PF( MIDIHANDLEDATA *MusicData, int PlayType )
{
	return 0 ;
}

// ＭＩＤＩデータの演奏を停止する処理の環境依存処理を行う
extern int StopMusicMem_PF( MIDIHANDLEDATA *MusicData )
{
	return 0 ;
}

// ＭＩＤＩデータが演奏中かどうかを取得する( TRUE:演奏中  FALSE:停止中 )処理の環境依存処理を行う関数
extern int CheckMusicMem_PF( MIDIHANDLEDATA *MusicData )
{
	return 0 ;
}

// ＭＩＤＩデータの周期的処理の環境依存処理を行う関数
extern int ProcessMusicMem_PF( MIDIHANDLEDATA *MusicData )
{
	return 0 ;
}

// ＭＩＤＩデータの現在の再生位置を取得する処理の環境依存処理を行う関数
extern int GetMusicMemPosition_PF( MIDIHANDLEDATA *MusicData )
{
	return 0 ;
}

// ＭＩＤＩの再生音量をセットする処理の環境依存処理を行う関数
extern int SetVolumeMusic_PF( int Volume )
{
	return 0 ;
}

// ＭＩＤＩの現在の再生位置を取得する処理の環境依存処理を行う関数
extern int GetMusicPosition_PF( void )
{
	return 0 ;
}









// サウンドバッファ再生用情報の後始末を行う( クリティカルセクションをロックした状態で呼ぶこと )
static void TerminateSoundBufferPlayInfo( int InfoIndex )
{
	SOUNDBUFFERPLAYINFO *BufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ InfoIndex ] ;

	if( InfoIndex < 0 || InfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
	{
		return ;
	}

	if( BufferPlayInfo->SetupFlag == FALSE )
	{
		return ;
	}

	BufferPlayInfo->UseFlag = FALSE ;
	BufferPlayInfo->SetupFlag = FALSE ;
	SoundSysData.PF.SoundBufferPlayInfoSetupNum -- ;

	if( BufferPlayInfo->PlayerObject )
	{
		if( BufferPlayInfo->PlayerPlayInterface != NULL )
		{
			( *BufferPlayInfo->PlayerPlayInterface )->SetPlayState( BufferPlayInfo->PlayerPlayInterface, SL_PLAYSTATE_STOPPED ) ;
		}

		( *BufferPlayInfo->PlayerObject)->Destroy( BufferPlayInfo->PlayerObject ) ;
		BufferPlayInfo->PlayerObject = NULL ;
	}

	if( BufferPlayInfo->OutputMixObject != NULL )
	{
		( *BufferPlayInfo->OutputMixObject )->Destroy( BufferPlayInfo->OutputMixObject ) ;
		BufferPlayInfo->OutputMixObject = NULL ;
	}

	BufferPlayInfo->PlayerPlayInterface = NULL ;
	BufferPlayInfo->PlayerBufferQueueInterface = NULL ;
	BufferPlayInfo->PlayerVolumeInterface = NULL ;
}

// 指定のサウンドバッファ再生用情報を使用していない状態に変更する
static int NotUseSoundBufferPlayInfo( int InfoIndex )
{
	SOUNDBUFFERPLAYINFO *BufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ InfoIndex ] ;

	if( InfoIndex < 0 || InfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
	{
		return -1 ;
	}

	// クリティカルセクションの取得
	CRITICALSECTION_LOCK( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	// 既に使用していない状態になっていたら何もせずに終了
	if( BufferPlayInfo->UseFlag == FALSE )
	{
		// クリティカルセクションの解放
		CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;
		
		return 0 ;
	}

	// 使用していない状態に変更
	BufferPlayInfo->UseFlag = FALSE ;

	// 再生を停止
	if( BufferPlayInfo->PlayerObject &&
		BufferPlayInfo->PlayerPlayInterface != NULL )
	{
		( *BufferPlayInfo->PlayerPlayInterface )->SetPlayState( BufferPlayInfo->PlayerPlayInterface, SL_PLAYSTATE_STOPPED ) ;
	}

	// バッファをクリア
	if(  BufferPlayInfo->PlayerBufferQueueInterface != NULL &&
		*BufferPlayInfo->PlayerBufferQueueInterface != NULL )
	{
		( *BufferPlayInfo->PlayerBufferQueueInterface )->Clear( BufferPlayInfo->PlayerBufferQueueInterface ) ;
	}

	// クリティカルセクションの解放
	CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	// 正常終了
	return 0 ;
}

// サウンドバッファ再生用情報のセットアップと、セットアップした情報の番号を取得する
static int SetupSoundBufferPlayInfo( SOUNDBUFFER *Buffer )
{
	SOUNDBUFFERPLAYINFO						*BufferPlayInfo ;
	SLDataLocator_AndroidSimpleBufferQueue	LocBufq ;
	SLDataFormat_PCM						FormatPcm ;
	SLDataSource							AudioSrc ;
	SLDataLocator_OutputMix					LocOutmix ;
	SLDataSink								AudioDataSink ;
	SLresult								Result ;
	const SLInterfaceID						ids[ 3 ] = { SL_IID_PLAY,     SL_IID_BUFFERQUEUE, SL_IID_VOLUME   } ;
	const SLboolean							req[ 3 ] = { SL_BOOLEAN_TRUE, SL_BOOLEAN_TRUE,    SL_BOOLEAN_TRUE } ;
	int i, j ;
	int NoUseIndex ;
	int NoSetupIndex ;
	int NewIndex ;
	int ErrorType = -1 ;

	// クリティカルセクションの取得
	CRITICALSECTION_LOCK( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	// 同じフォーマットで且つ使用されていない再生情報を探す
	BufferPlayInfo = SoundSysData.PF.SoundBufferPlayInfos ;
	NoUseIndex = -1 ;
	NoSetupIndex = -1 ;
	j = 0 ;
	for( i = 0 ; i < SOUNDBUFFERPLAYINFO_MAX_NUM && j < SoundSysData.PF.SoundBufferPlayInfoSetupNum ; i ++, BufferPlayInfo ++ )
	{
		if( BufferPlayInfo->SetupFlag == FALSE )
		{
			// セットアップされていない要素を保存
			if( NoSetupIndex == -1 )
			{
				NoSetupIndex = i ;
			}
			continue ;
		}

		j ++ ;

		if( BufferPlayInfo->UseFlag )
		{
			continue ;
		}

		// 使用されていない要素を保存
		if( NoUseIndex == -1 )
		{
			NoUseIndex = i ;
		}

		if( BufferPlayInfo->Channels      == Buffer->Format.nChannels      &&
			BufferPlayInfo->BitsPerSample == Buffer->Format.wBitsPerSample &&
			BufferPlayInfo->SamplesPerSec == Buffer->Format.nSamplesPerSec )
		{
			// 見つかったら使用状態にして返す
			BufferPlayInfo->UseFlag = TRUE ;

			// バッファキューインターフェースのコールバック関数の設定
			Result = ( *BufferPlayInfo->PlayerBufferQueueInterface )->RegisterCallback( BufferPlayInfo->PlayerBufferQueueInterface, PlayerBufferQueueCallback, Buffer ) ;
			if( Result != SL_RESULT_SUCCESS )
			{
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\xad\x30\xe5\x30\xfc\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xb3\x30\xfc\x30\xeb\x30\xd0\x30\xc3\x30\xaf\x30\xa2\x95\x70\x65\x6e\x30\x7b\x76\x32\x93\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のバッファキューインターフェースのコールバック関数の登録に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
				return -1 ;
			}

			// クリティカルセクションの解放
			CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

			// 返す
			return i ;
		}
	}

	// セットアップされていない要素があったらそこを使用する
	if( NoSetupIndex >= 0 )
	{
		NewIndex = NoSetupIndex ;
	}
	else
	// 再生情報が最大数に達している場合分岐
	if( SoundSysData.PF.SoundBufferPlayInfoSetupNum >= BUFFERQUEUE_BUFFER_NUM )
	{
		// 使用されていない要素が無かったらエラー
		if( NoUseIndex < 0 )
		{
			// クリティカルセクションの解放
			CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

			DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xbb\x30\xc3\x30\xc8\x30\xa2\x30\xc3\x30\xd7\x30\xa8\x30\xe9\x30\xfc\x30\x01\x30\x8d\x51\x1f\x75\xc5\x60\x31\x58\x70\x65\x4c\x30\x00\x67\x27\x59\x70\x65\x20\x00\x25\x00\x64\x00\x20\x00\x6b\x30\x54\x90\x57\x30\x66\x30\x44\x30\x7e\x30\x59\x30\x0a\x00\x00"/*@ L"OpenSL ES のサウンド出力セットアップエラー、再生情報数が最大数 %d に達しています\n" @*/, BUFFERQUEUE_BUFFER_NUM )) ;
			return -1 ;
		}

		// 使用されていない要素があった場合は解放処理を行う
		TerminateSoundBufferPlayInfo( NoUseIndex ) ;

		// 使用するバッファのアドレスをセット
		NewIndex = NoUseIndex ;
	}
	else
	{
		// 最大数に達していない場合は配列の使用してない要素のアドレスをセット
		NewIndex = SoundSysData.PF.SoundBufferPlayInfoSetupNum ;
	}

SETUP :

	BufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ NewIndex ] ;

	if( NewIndex < 0 || NewIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xbb\x30\xc3\x30\xc8\x30\xa2\x30\xc3\x30\xd7\x30\xa8\x30\xe9\x30\xfc\x30\x01\x30\x4e\x00\x65\x00\x77\x00\x49\x00\x6e\x00\x64\x00\x65\x00\x78\x00\x20\x00\x24\x50\x4c\x30\x0d\x4e\x63\x6b\x0a\x00\x00"/*@ L"OpenSL ES のサウンド出力セットアップエラー、NewIndex 値が不正\n" @*/ )) ;
		return -1 ;
	}

	// 最大数に達していなくても、エラーが発生した数に達していたらエラー扱い
	if( SoundSysData.PF.SoundBufferPlayInfoSetupErrorNum != 0 &&
		SoundSysData.PF.SoundBufferPlayInfoSetupErrorNum <= SoundSysData.PF.SoundBufferPlayInfoSetupNum )
	{
		goto ERR ;
	}

	// フォーマットを保存
	BufferPlayInfo->Channels      = Buffer->Format.nChannels      ;
	BufferPlayInfo->BitsPerSample = Buffer->Format.wBitsPerSample ;
	BufferPlayInfo->SamplesPerSec = Buffer->Format.nSamplesPerSec ;

	// 出力オブジェクト作成
	Result = ( *SoundSysData.PF.EngineInterface )->CreateOutputMix( SoundSysData.PF.EngineInterface, &BufferPlayInfo->OutputMixObject, 0, NULL, NULL ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 0 ;
		goto ERR ;
	}

	// 出力オブジェクトのリアライズ
	Result = ( *BufferPlayInfo->OutputMixObject )->Realize( BufferPlayInfo->OutputMixObject, SL_BOOLEAN_FALSE ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 1 ;
		goto ERR ;
	}

	FormatPcm.formatType	= SL_DATAFORMAT_PCM;
	FormatPcm.numChannels	= ( SLuint32 )Buffer->Format.nChannels ;
	FormatPcm.samplesPerSec	= ( SLuint32 )Buffer->Format.nSamplesPerSec * 1000 ;
	FormatPcm.bitsPerSample	= ( SLuint32 )Buffer->Format.wBitsPerSample ;
	FormatPcm.containerSize	= ( SLuint32 )Buffer->Format.wBitsPerSample ;
	FormatPcm.channelMask	= Buffer->Format.nChannels == 1 ? SL_SPEAKER_FRONT_CENTER : ( SL_SPEAKER_FRONT_LEFT | SL_SPEAKER_FRONT_RIGHT ) ;
	FormatPcm.endianness	= SL_BYTEORDER_LITTLEENDIAN ;

	LocBufq.locatorType		= SL_DATALOCATOR_ANDROIDSIMPLEBUFFERQUEUE ;
	LocBufq.numBuffers		= BUFFERQUEUE_BUFFER_NUM ;
	AudioSrc.pLocator		= &LocBufq ;
	AudioSrc.pFormat		= &FormatPcm ;

	LocOutmix.locatorType	= SL_DATALOCATOR_OUTPUTMIX ;
	LocOutmix.outputMix		= BufferPlayInfo->OutputMixObject ;

	AudioDataSink.pLocator	= &LocOutmix ;
	AudioDataSink.pFormat	= NULL ;

	// プレイヤーオブジェクト作成
	Result = ( *SoundSysData.PF.EngineInterface )->CreateAudioPlayer(
		SoundSysData.PF.EngineInterface,
		&BufferPlayInfo->PlayerObject,
		&AudioSrc,
		&AudioDataSink,
		3,
		ids,
		req
	) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 2 ;
		goto ERR ;
	}

	// プレイヤーオブジェクトのリアライズ
	Result = ( *BufferPlayInfo->PlayerObject )->Realize( BufferPlayInfo->PlayerObject, SL_BOOLEAN_FALSE ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 3 ;
		goto ERR ;
	}

	// 再生インターフェースの取得
	Result = ( *BufferPlayInfo->PlayerObject )->GetInterface(BufferPlayInfo->PlayerObject, SL_IID_PLAY, &BufferPlayInfo->PlayerPlayInterface ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 4 ;
		goto ERR ;
	}

	// バッファキューインターフェースの取得
	Result = ( *BufferPlayInfo->PlayerObject )->GetInterface(BufferPlayInfo->PlayerObject, SL_IID_BUFFERQUEUE, &BufferPlayInfo->PlayerBufferQueueInterface ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 5 ;
		goto ERR ;
	}

	// 音量インターフェースの取得
	Result = ( *BufferPlayInfo->PlayerObject )->GetInterface(BufferPlayInfo->PlayerObject, SL_IID_VOLUME, &BufferPlayInfo->PlayerVolumeInterface ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 6 ;
		goto ERR ;
	}

	// バッファキューインターフェースのコールバック関数の設定
	Result = ( *BufferPlayInfo->PlayerBufferQueueInterface )->RegisterCallback( BufferPlayInfo->PlayerBufferQueueInterface, PlayerBufferQueueCallback, Buffer ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 7 ;
		goto ERR ;
	}

	// セットアップ完了状態にする
	BufferPlayInfo->SetupFlag = TRUE ;
	SoundSysData.PF.SoundBufferPlayInfoSetupNum ++ ;

	// 使用状態にする
	BufferPlayInfo->UseFlag = TRUE ;

	// クリティカルセクションの解放
	CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	// 正常終了
	return NewIndex ;


	// エラー処理
ERR :

	// オブジェクトの後始末
	{
		if( BufferPlayInfo->PlayerObject )
		{
			if( BufferPlayInfo->PlayerPlayInterface != NULL )
			{
				( *BufferPlayInfo->PlayerPlayInterface )->SetPlayState( BufferPlayInfo->PlayerPlayInterface, SL_PLAYSTATE_STOPPED ) ;
			}

			( *BufferPlayInfo->PlayerObject)->Destroy( BufferPlayInfo->PlayerObject ) ;
			BufferPlayInfo->PlayerObject = NULL ;
		}

		if( BufferPlayInfo->OutputMixObject != NULL )
		{
			( *BufferPlayInfo->OutputMixObject )->Destroy( BufferPlayInfo->OutputMixObject ) ;
			BufferPlayInfo->OutputMixObject = NULL ;
		}

		BufferPlayInfo->UseFlag = FALSE ;
		BufferPlayInfo->PlayerPlayInterface = NULL ;
		BufferPlayInfo->PlayerBufferQueueInterface = NULL ;
		BufferPlayInfo->PlayerVolumeInterface = NULL ;
	}

	// セットアップされていて且つ使用されていない要素を一つ解放する
	BufferPlayInfo = SoundSysData.PF.SoundBufferPlayInfos ;
	NoUseIndex = -1 ;
	j = 0 ;
	for( i = 0 ; i < SOUNDBUFFERPLAYINFO_MAX_NUM && j < SoundSysData.PF.SoundBufferPlayInfoSetupNum ; i ++, BufferPlayInfo ++ )
	{
		if( BufferPlayInfo->SetupFlag == FALSE )
		{
			continue ;
		}

		j ++ ;

		if( BufferPlayInfo->UseFlag == FALSE )
		{
			NoUseIndex = i ;
			break ;
		}
	}

	// セットアップされていて且つ使用されていない要素が無い場合はエラー
	if( NoUseIndex == -1 )
	{
		// クリティカルセクションの解放
		CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

		// 前回エラーが発生した際とセットアップできている要素数が異なる場合のみエラーログを出力
		if( SoundSysData.PF.SoundBufferPlayInfoSetupErrorNum != 0 &&
			SoundSysData.PF.SoundBufferPlayInfoSetupErrorNum != SoundSysData.PF.SoundBufferPlayInfoSetupNum )
		{
			SoundSysData.PF.SoundBufferPlayInfoSetupErrorNum = SoundSysData.PF.SoundBufferPlayInfoSetupNum ;

			switch( ErrorType )
			{
			case 0 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のサウンド出力オブジェクトの作成に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
				break ;

			case 1 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xea\x30\xa2\x30\xe9\x30\xa4\x30\xba\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のサウンド出力オブジェクトのリアライズに失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
				break ;

			case 2 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd7\x30\xec\x30\xa4\x30\xe4\x30\xfc\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のプレイヤーオブジェクトの作成に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
				break ;

			case 3 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd7\x30\xec\x30\xa4\x30\xe4\x30\xfc\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xea\x30\xa2\x30\xe9\x30\xa4\x30\xba\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のプレイヤーオブジェクトのリアライズに失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
				break ;

			case 4 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\x8d\x51\x1f\x75\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES の再生インターフェースの取得に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
				break ;

			case 5 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\xad\x30\xe5\x30\xfc\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のバッファキューインターフェースの取得に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
				break ;

			case 6 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xf3\x97\xcf\x91\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES の音量インターフェースの取得に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
				break ;

			case 7 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\xad\x30\xe5\x30\xfc\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xb3\x30\xfc\x30\xeb\x30\xd0\x30\xc3\x30\xaf\x30\xa2\x95\x70\x65\x6e\x30\x7b\x76\x32\x93\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のバッファキューインターフェースのコールバック関数の登録に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
				break ;
			}

			DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xbb\x30\xc3\x30\xc8\x30\xa2\x30\xc3\x30\xd7\x30\xa8\x30\xe9\x30\xfc\x30\x01\x30\xbb\x30\xc3\x30\xc8\x30\xa2\x30\xc3\x30\xd7\x30\x55\x30\x8c\x30\x66\x30\x44\x30\x66\x30\x14\x4e\x64\x30\x7f\x4f\x28\x75\x55\x30\x8c\x30\x66\x30\x44\x30\x6a\x30\x44\x30\x81\x89\x20\x7d\x4c\x30\x8b\x89\x64\x30\x4b\x30\x8a\x30\x7e\x30\x5b\x30\x93\x30\x67\x30\x57\x30\x5f\x30\x28\x00\x20\x00\x0c\x54\x42\x66\x8d\x51\x1f\x75\x70\x65\x1a\xff\x25\x00\x64\x00\x20\x00\x29\x00\x0a\x00\x00"/*@ L"OpenSL ES のサウンド出力セットアップエラー、セットアップされていて且つ使用されていない要素が見つかりませんでした( 同時再生数：%d )\n" @*/, SoundSysData.PF.SoundBufferPlayInfoSetupNum )) ;
		}
		return -1 ;
	}

	// セットアップされていて且つ使用していない要素を解放して再チャレンジ
	TerminateSoundBufferPlayInfo( NoUseIndex ) ;

	// 使用するインデックスとしてセット
	NewIndex = NoUseIndex ;

	goto SETUP ;
}



// 再生の準備を行う
static int SoundBuffer_Play_Setup( SOUNDBUFFER *Buffer )
{
	// 再生準備ができている場合は何もしない
	if( Buffer->PF.PlaySetupComp == TRUE )
	{
		return 0 ;
	}

	Buffer->PF.UseSoundBufferPlayInfoIndex = SetupSoundBufferPlayInfo( Buffer ) ;
	if( Buffer->PF.UseSoundBufferPlayInfoIndex < 0 )
	{
//		DXST_LOGFILE_ADDUTF16LE( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xbb\x30\xc3\x30\xc8\x30\xa2\x30\xc3\x30\xd7\x30\x4c\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES のサウンド出力セットアップが失敗しました\n" @*/ ) ;
		return -1 ;
	}

	// 再生準備完了
	Buffer->PF.PlaySetupComp = TRUE ;

	// 音量の設定
	SoundBuffer_RefreshVolume_PF( Buffer ) ;

	// 正常終了
	return 0 ;

//	SLDataLocator_AndroidSimpleBufferQueue	LocBufq ;
//	SLDataFormat_PCM						FormatPcm ;
//	SLDataSource							AudioSrc ;
//	SLDataLocator_OutputMix					LocOutmix ;
//	SLDataSink								AudioDataSink ;
//	SLresult								Result ;
//	const SLInterfaceID						ids[ 3 ] = { SL_IID_PLAY,     SL_IID_BUFFERQUEUE, SL_IID_VOLUME   } ;
//	const SLboolean							req[ 3 ] = { SL_BOOLEAN_TRUE, SL_BOOLEAN_TRUE,    SL_BOOLEAN_TRUE } ;
//
//	// 再生準備ができている場合は何もしない
//	if( Buffer->PF.PlaySetupComp == TRUE )
//	{
//		return 0 ;
//	}
//
//	// 出力オブジェクト作成
//	Result = ( *SoundSysData.PF.EngineInterface )->CreateOutputMix( SoundSysData.PF.EngineInterface, &Buffer->PF.OutputMixObject, 0, NULL, NULL ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のサウンド出力オブジェクトの作成に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// 出力オブジェクトのリアライズ
//	Result = ( *Buffer->PF.OutputMixObject )->Realize( Buffer->PF.OutputMixObject, SL_BOOLEAN_FALSE ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xea\x30\xa2\x30\xe9\x30\xa4\x30\xba\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のサウンド出力オブジェクトのリアライズに失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	FormatPcm.formatType	= SL_DATAFORMAT_PCM;
//	FormatPcm.numChannels	= ( SLuint32 )Buffer->Format.nChannels ;
//	FormatPcm.samplesPerSec	= ( SLuint32 )Buffer->Format.nSamplesPerSec * 1000 ;
//	FormatPcm.bitsPerSample	= ( SLuint32 )Buffer->Format.wBitsPerSample ;
//	FormatPcm.containerSize	= ( SLuint32 )Buffer->Format.wBitsPerSample ;
//	FormatPcm.channelMask	= Buffer->Format.nChannels == 1 ? SL_SPEAKER_FRONT_CENTER : ( SL_SPEAKER_FRONT_LEFT | SL_SPEAKER_FRONT_RIGHT ) ;
//	FormatPcm.endianness	= SL_BYTEORDER_LITTLEENDIAN ;
//
//	LocBufq.locatorType		= SL_DATALOCATOR_ANDROIDSIMPLEBUFFERQUEUE ;
//	LocBufq.numBuffers		= BUFFERQUEUE_BUFFER_NUM ;
//	AudioSrc.pLocator		= &LocBufq ;
//	AudioSrc.pFormat		= &FormatPcm ;
//
//	LocOutmix.locatorType	= SL_DATALOCATOR_OUTPUTMIX ;
//	LocOutmix.outputMix		= Buffer->PF.OutputMixObject ;
//
//	AudioDataSink.pLocator	= &LocOutmix ;
//	AudioDataSink.pFormat	= NULL ;
//
//	// プレイヤーオブジェクト作成
//	Result = ( *SoundSysData.PF.EngineInterface )->CreateAudioPlayer(
//		SoundSysData.PF.EngineInterface,
//		&Buffer->PF.PlayerObject,
//		&AudioSrc,
//		&AudioDataSink,
//		3,
//		ids,
//		req
//	) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd7\x30\xec\x30\xa4\x30\xe4\x30\xfc\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のプレイヤーオブジェクトの作成に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// プレイヤーオブジェクトのリアライズ
//	Result = ( *Buffer->PF.PlayerObject )->Realize( Buffer->PF.PlayerObject, SL_BOOLEAN_FALSE ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd7\x30\xec\x30\xa4\x30\xe4\x30\xfc\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xea\x30\xa2\x30\xe9\x30\xa4\x30\xba\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のプレイヤーオブジェクトのリアライズに失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// 再生インターフェースの取得
//	Result = ( *Buffer->PF.PlayerObject )->GetInterface(Buffer->PF.PlayerObject, SL_IID_PLAY, &Buffer->PF.PlayerPlayInterface ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\x8d\x51\x1f\x75\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES の再生インターフェースの取得に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// バッファキューインターフェースの取得
//	Result = ( *Buffer->PF.PlayerObject )->GetInterface(Buffer->PF.PlayerObject, SL_IID_BUFFERQUEUE, &Buffer->PF.PlayerBufferQueueInterface ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\xad\x30\xe5\x30\xfc\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のバッファキューインターフェースの取得に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// 音量インターフェースの取得
//	Result = ( *Buffer->PF.PlayerObject )->GetInterface(Buffer->PF.PlayerObject, SL_IID_VOLUME, &Buffer->PF.PlayerVolumeInterface ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xf3\x97\xcf\x91\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES の音量インターフェースの取得に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// バッファキューインターフェースのコールバック関数の設定
//	Result = ( *Buffer->PF.PlayerBufferQueueInterface )->RegisterCallback( Buffer->PF.PlayerBufferQueueInterface, PlayerBufferQueueCallback, Buffer ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\xad\x30\xe5\x30\xfc\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xb3\x30\xfc\x30\xeb\x30\xd0\x30\xc3\x30\xaf\x30\xa2\x95\x70\x65\x6e\x30\x7b\x76\x32\x93\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES のバッファキューインターフェースのコールバック関数の登録に失敗しました  エラーコード:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// 再生準備完了
//	Buffer->PF.PlaySetupComp = TRUE ;
//
//	// 音量の設定
//	SoundBuffer_RefreshVolume_PF( Buffer ) ;
//
//	// 正常終了
//	return 0 ;
//
//	// エラー処理
//ERR :
//
//	if( Buffer->PF.PlayerObject )
//	{
//		if( Buffer->PF.PlayerPlayInterface != NULL )
//		{
//			( *Buffer->PF.PlayerPlayInterface )->SetPlayState( Buffer->PF.PlayerPlayInterface, SL_PLAYSTATE_STOPPED ) ;
//		}
//
//		( *Buffer->PF.PlayerObject)->Destroy( Buffer->PF.PlayerObject ) ;
//		Buffer->PF.PlayerObject = NULL ;
//	}
//
//	if( Buffer->PF.OutputMixObject != NULL )
//	{
//		( *Buffer->PF.OutputMixObject )->Destroy( Buffer->PF.OutputMixObject ) ;
//		Buffer->PF.OutputMixObject = NULL ;
//	}
//
//	Buffer->PF.PlayerPlayInterface = NULL ;
//	Buffer->PF.PlayerBufferQueueInterface = NULL ;
//	Buffer->PF.PlayerVolumeInterface = NULL ;
//
//	return -1 ;
}

// 再生の後始末を行う
static int SoundBuffer_Play_Terminate( SOUNDBUFFER *Buffer )
{
	// 再生準備がされていなければ何もせずに終了
	if( Buffer->PF.PlaySetupComp == FALSE )
	{
		return 0 ;
	}

	// サウンドバッファ再生用情報をを使用していない状態に変更
	NotUseSoundBufferPlayInfo( Buffer->PF.UseSoundBufferPlayInfoIndex ) ;
	Buffer->PF.UseSoundBufferPlayInfoIndex = -1 ;

	// セットアップされていない状態に変更
	Buffer->PF.PlaySetupComp = FALSE ;

	return 0 ;
}

extern int SoundBuffer_Initialize_Timing0_PF( SOUNDBUFFER *Buffer, DWORD Bytes, WAVEFORMATEX *Format, SOUNDBUFFER *Src, int Is3DSound )
{
	int InitializeCriticalSection = FALSE ;

	// ゼロ初期化
	_MEMSET( &Buffer->PF, 0, sizeof( Buffer->PF ) ) ;

	// クリティカルセクションを初期化
	if( CriticalSection_Initialize( &Buffer->PF.CriticalSection ) < 0 )
	{
		goto ERR ;
	}
	InitializeCriticalSection = TRUE ;

	// サウンドバッファ用のメモリを確保
	if( Src != NULL )
	{
		Buffer->SampleNum	= Src->SampleNum ;
		Buffer->Format		= Src->Format ;
		Buffer->Wave		= DuplicateWaveData( Src->Wave ) ;
	}
	else
	{
		Buffer->Format		= *Format ;
		Buffer->SampleNum	= ( int )( Bytes / Format->nBlockAlign ) ;
		Buffer->Wave		= AllocWaveData( ( int )Bytes, FALSE ) ;
	}
	if( Buffer->Wave == NULL )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\x28\x75\x6e\x30\xe1\x30\xe2\x30\xea\x30\x6e\x30\xba\x78\xdd\x4f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"サウンドバッファ用のメモリの確保に失敗しました\n" @*/ )) ;
		goto ERR ;
	}

	// 正常終了
	return 0 ;

	// エラー処理
ERR :

	if( InitializeCriticalSection )
	{
		CriticalSection_Delete( &Buffer->PF.CriticalSection ) ;
	}

	return -1 ;
}

extern int SoundBuffer_Initialize_Timing1_PF( SOUNDBUFFER *Buffer, SOUNDBUFFER *Src, int Is3DSound )
{
	return 0 ;
}

extern int SoundBuffer_Terminate_PF( SOUNDBUFFER *Buffer )
{
	// クリティカルセクションの削除
	CriticalSection_Delete( &Buffer->PF.CriticalSection ) ;

	return 0 ;
}

extern int SoundBuffer_CheckEnable_PF( SOUNDBUFFER *Buffer )
{
	return TRUE ;
}

extern int SoundBuffer_Play_PF( SOUNDBUFFER *Buffer, int Loop )
{
	// クリティカルセクションの取得
	CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;

	// 停止待ちサウンドバッファリストからサウンドバッファを外す
	SoundBuffer_Sub_StopSoundBufferList( Buffer ) ;

	// 再生準備がされていなかったら
	if( Buffer->PF.PlaySetupComp == FALSE )
	{
		// バッファキュー関係の情報の初期化
		Buffer->PF.EnqueueInfoNum = 0 ;
		Buffer->PF.BufferEndPlayEnqueueNum = 0 ;

		// 再生準備
		if( SoundBuffer_Play_Setup( Buffer ) < 0 )
		{
			Buffer->State = FALSE ;

			// クリティカルセクションの解放
			CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;
			return -1 ;
		}
	}

	Buffer->State = TRUE ;

	// 再生準備が成功した場合のみ処理を行う
	if( Buffer->PF.PlaySetupComp )
	{
		// バッファキューにサウンドデータを追加する
		EnqueueSoundBuffer( Buffer ) ;

		// 再生状態に変更
		if( Buffer->PF.PlaySetupComp )
		{
			SOUNDBUFFERPLAYINFO *SoundBufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ Buffer->PF.UseSoundBufferPlayInfoIndex ] ;

			if( Buffer->PF.UseSoundBufferPlayInfoIndex < 0 || Buffer->PF.UseSoundBufferPlayInfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
			{
				return -1 ;
			}

			( *SoundBufferPlayInfo->PlayerPlayInterface )->SetPlayState( SoundBufferPlayInfo->PlayerPlayInterface, SL_PLAYSTATE_PLAYING ) ;
		}
	}

	Buffer->StopTimeState = 1 ;
	Buffer->StopTime = 0 ;

	// クリティカルセクションの解放
	CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;

	return 0 ;
}

extern int SoundBuffer_Stop_PF(	SOUNDBUFFER *Buffer, int EffectStop )
{
	// クリティカルセクションの取得
	CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;

	// 再生準備がされている場合のみ処理
	if( Buffer->PF.PlaySetupComp )
	{
		SOUNDBUFFERPLAYINFO *SoundBufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ Buffer->PF.UseSoundBufferPlayInfoIndex ] ;

		if( Buffer->PF.UseSoundBufferPlayInfoIndex < 0 || Buffer->PF.UseSoundBufferPlayInfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
		{
			return -1 ;
		}

	//	( *Buffer->PF.PlayerPlayInterface )->SetPlayState( Buffer->PF.PlayerPlayInterface, SL_PLAYSTATE_PAUSED ) ;
		( *SoundBufferPlayInfo->PlayerPlayInterface )->SetPlayState( SoundBufferPlayInfo->PlayerPlayInterface, SL_PLAYSTATE_STOPPED ) ;

		// 再生処理の後始末
		SoundBuffer_Play_Terminate( Buffer ) ;

		// バッファキュー関係の情報の初期化
		Buffer->PF.EnqueueInfoNum = 0 ;
		Buffer->PF.BufferEndPlayEnqueueNum = 0 ;
	}

	Buffer->State = FALSE ;

	// 停止待ちサウンドバッファリストからサウンドバッファを外す
	SoundBuffer_Sub_StopSoundBufferList( Buffer ) ;

	// クリティカルセクションの解放
	CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;

	return 0 ;
}

extern int SoundBuffer_CheckPlay_PF( SOUNDBUFFER *Buffer )
{
	return Buffer->State ;
}

extern int SoundBuffer_Lock_PF( SOUNDBUFFER *Buffer, DWORD WritePos , DWORD WriteSize, void **LockPos1, DWORD *LockSize1, void **LockPos2, DWORD *LockSize2 )
{
	return 2 ;
}

extern int SoundBuffer_Unlock_PF( SOUNDBUFFER *Buffer, void *LockPos1, DWORD LockSize1, void *LockPos2, DWORD LockSize2 )
{
	DWORD i ;

	// ダブルサイズの場合はステレオデータにする
	if( Buffer->Wave->DoubleSizeBuffer != NULL )
	{
		switch( Buffer->Format.wBitsPerSample )
		{
		case 8 :
			{
				BYTE *Src8bit ;
				WORD *Dest8bit ;
				DWORD SampleNum ;

				Src8bit = ( BYTE * )LockPos1 ;
				Dest8bit = ( WORD * )Buffer->Wave->DoubleSizeBuffer + ( ( BYTE * )LockPos1 - ( BYTE * )Buffer->Wave->Buffer ) ;
				SampleNum = LockSize1 ;
				for( i = 0 ; i < SampleNum ; i ++ )
				{
					Dest8bit[ i ] = ( WORD )( Src8bit[ i ] + ( Src8bit[ i ] << 8 ) ) ;
				}

				Src8bit = ( BYTE * )LockPos2 ;
				Dest8bit = ( WORD * )Buffer->Wave->DoubleSizeBuffer + ( ( BYTE * )LockPos2 - ( BYTE * )Buffer->Wave->Buffer ) ;
				SampleNum = LockSize2 ;
				for( i = 0 ; i < SampleNum ; i ++ )
				{
					Dest8bit[ i ] = ( WORD )( Src8bit[ i ] + ( Src8bit[ i ] << 8 ) ) ;
				}
			}
			break ;

		case 16 :
			{
				WORD *Src16bit ;
				DWORD *Dest16bit ;
				DWORD SampleNum ;

				Src16bit = ( WORD * )LockPos1 ;
				Dest16bit = ( DWORD * )Buffer->Wave->DoubleSizeBuffer + ( ( WORD * )LockPos1 - ( WORD * )Buffer->Wave->Buffer ) ;
				SampleNum = LockSize1 / 2 ;
				for( i = 0 ; i < SampleNum ; i ++ )
				{
					Dest16bit[ i ] = ( DWORD )( Src16bit[ i ] + ( Src16bit[ i ] << 16 ) ) ;
				}

				Src16bit = ( WORD * )LockPos2 ;
				Dest16bit = ( DWORD * )Buffer->Wave->DoubleSizeBuffer + ( ( WORD * )LockPos2 - ( WORD * )Buffer->Wave->Buffer ) ;
				SampleNum = LockSize2 / 2 ;
				for( i = 0 ; i < SampleNum ; i ++ )
				{
					Dest16bit[ i ] = ( DWORD )( Src16bit[ i ] + ( Src16bit[ i ] << 16 ) ) ;
				}
			}
			break ;
		}
	}

	return 0 ;
}

extern int SoundBuffer_SetFrequency_PF( SOUNDBUFFER *Buffer, DWORD Frequency )
{
	return 0 ;
}

extern int SoundBuffer_GetFrequency_PF( SOUNDBUFFER *Buffer, LPDWORD Frequency )
{
	return 2 ;
}

extern int SoundBuffer_RefreshVolume_PF( SOUNDBUFFER *Buffer )
{
	LONG CalcVolume[ 2 ] ;
	LONG TempVolume[ 2 ] ;
	LONG Volume ;
	LONG Pan ;

	TempVolume[ 0 ] = Buffer->Volume[ 0 ] ;
	if( Buffer->Format.nChannels == 1 )
	{
		TempVolume[ 1 ] = Buffer->Volume[ 0 ] ;
	}
	else
	{
		TempVolume[ 1 ] = Buffer->Volume[ 1 ] ;
	}

	if( Buffer->Pan < 0 )
	{
		CalcVolume[ 0 ] = 10000 ;
		CalcVolume[ 1 ] = 10000 + Buffer->Pan ;
	}
	else
	{
		CalcVolume[ 0 ] = 10000 - Buffer->Pan ;
		CalcVolume[ 1 ] = 10000 ;
	}

	if( TempVolume[ 0 ] > 0 )
	{
		TempVolume[ 0 ] = 0 ;
	}
	else
	if( TempVolume[ 0 ] < -10000 )
	{
		TempVolume[ 0 ] = -10000 ;
	}
	if( TempVolume[ 1 ] > 0 )
	{
		TempVolume[ 1 ] = 0 ;
	}
	else
	if( TempVolume[ 1 ] < -10000 )
	{
		TempVolume[ 1 ] = -10000 ;
	}

	CalcVolume[ 0 ] = CalcVolume[ 0 ] * ( TempVolume[ 0 ] + 10000 ) / 10000 ;
	CalcVolume[ 1 ] = CalcVolume[ 1 ] * ( TempVolume[ 0 ] + 10000 ) / 10000 ;

	if( CalcVolume[ 0 ] > CalcVolume[ 1 ] )
	{
		Volume = CalcVolume[ 0 ] - 10000 ;
		Pan =    _FTOL( CalcVolume[ 1 ] * ( 10000.0f / CalcVolume[ 0 ] ) ) - 10000 ;
	}
	else
	if( CalcVolume[ 0 ] < CalcVolume[ 1 ] )
	{
		Volume = CalcVolume[ 1 ] - 10000 ;
		Pan = -( _FTOL( CalcVolume[ 0 ] * ( 10000.0f / CalcVolume[ 1 ] ) ) - 10000 ) ;
	}
	else
	{
		Volume = CalcVolume[ 0 ] - 10000 ;
		Pan = 0 ;
	}

	// 再生準備が成功した場合のみ処理を行う
	if( Buffer->PF.PlaySetupComp )
	{
		SOUNDBUFFERPLAYINFO *SoundBufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ Buffer->PF.UseSoundBufferPlayInfoIndex ] ;

		if( Buffer->PF.UseSoundBufferPlayInfoIndex < 0 || Buffer->PF.UseSoundBufferPlayInfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
		{
			return -1 ;
		}

		if( CalcVolume[ 0 ] == CalcVolume[ 1 ] )
		{
			( *SoundBufferPlayInfo->PlayerVolumeInterface )->EnableStereoPosition( SoundBufferPlayInfo->PlayerVolumeInterface, SL_BOOLEAN_FALSE ) ;
			( *SoundBufferPlayInfo->PlayerVolumeInterface )->SetStereoPosition( SoundBufferPlayInfo->PlayerVolumeInterface, 0 ) ;
		}
		else
		{
			( *SoundBufferPlayInfo->PlayerVolumeInterface )->EnableStereoPosition( SoundBufferPlayInfo->PlayerVolumeInterface, SL_BOOLEAN_TRUE ) ;
			( *SoundBufferPlayInfo->PlayerVolumeInterface )->SetStereoPosition( SoundBufferPlayInfo->PlayerVolumeInterface, ( SLmillibel )( Pan * 0x7fff / 10000 ) ) ;
		}
	
//		( *SoundBufferPlayInfo->PlayerVolumeInterface )->SetVolumeLevel( SoundBufferPlayInfo->PlayerVolumeInterface, ( SLmillibel )( Volume * 0x7fff / 10000 ) ) ;
		( *SoundBufferPlayInfo->PlayerVolumeInterface )->SetVolumeLevel( SoundBufferPlayInfo->PlayerVolumeInterface, ( SLmillibel )( Volume ) ) ;
	}

	return 0 ;
}

extern int SoundBuffer_GetCurrentPosition_PF(	SOUNDBUFFER *Buffer, LPDWORD PlayPos, LPDWORD WritePos )
{
//	SLAndroidSimpleBufferQueueState BufferQueState ;
//
//	// クリティカルセクションの取得
//	CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;
//
//	// 再生準備が成功した場合のみ処理を行う
//	if( Buffer->PF.PlaySetupComp )
//	{
//		if( ( *Buffer->PF.PlayerBufferQueueInterface )->GetState( Buffer->PF.PlayerBufferQueueInterface, &BufferQueState ) == SL_RESULT_SUCCESS )
//		{
//			// 再生位置の更新
//			RefreshSoundBufferCurrentTime( Buffer, BufferQueState, FALSE ) ;
//		}
//	}
//
//	// クリティカルセクションの解放
//	CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;

	return 2 ;
}

extern int SoundBuffer_SetCurrentPosition_PF( SOUNDBUFFER *Buffer, DWORD NewPos )
{
	int OldState = Buffer->State ;

	// クリティカルセクションの取得
	CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;

	if( OldState == TRUE )
	{
		SoundBuffer_Stop( Buffer ) ;
	}

	// 再生準備が成功した場合のみ処理を行う
	if( Buffer->PF.PlaySetupComp )
	{
		SOUNDBUFFERPLAYINFO *SoundBufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ Buffer->PF.UseSoundBufferPlayInfoIndex ] ;

		if( Buffer->PF.UseSoundBufferPlayInfoIndex < 0 || Buffer->PF.UseSoundBufferPlayInfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
		{
			return -1 ;
		}

		// バッファキューのサウンドデータをクリアする
		( *SoundBufferPlayInfo->PlayerBufferQueueInterface )->Clear( SoundBufferPlayInfo->PlayerBufferQueueInterface ) ;
	}

	Buffer->Pos     = ( int )( NewPos / Buffer->Format.nBlockAlign ) ;
	Buffer->CompPos = Buffer->Pos ;

	Buffer->PF.EnqueueInfoNum = 0 ;
	Buffer->PF.BufferEndPlayEnqueueNum = 0 ;

	if( OldState == TRUE )
	{
		SoundBuffer_Play( Buffer, Buffer->Loop ) ;
	}

	// クリティカルセクションの解放
	CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;

	return 0 ;
}

extern int SoundBuffer_CycleProcess_PF( SOUNDBUFFER *Buffer )
{
	return -1 ;
}

extern int SoundBuffer_Set3DPosition_PF( SOUNDBUFFER *Buffer, VECTOR *Position )
{
	return 0 ;
}

extern int SoundBuffer_Set3DRadius_PF( SOUNDBUFFER *Buffer, float Radius )
{
	return 0 ;
}

extern int SoundBuffer_Set3DInnerRadius_PF(	SOUNDBUFFER *Buffer, float Radius )
{
	return 0 ;
}

extern int SoundBuffer_Set3DVelocity_PF( SOUNDBUFFER *Buffer, VECTOR *Velocity )
{
	return 0 ;
}

extern int SoundBuffer_Set3DFrontPosition_PF( SOUNDBUFFER *Buffer, VECTOR *FrontPosition, VECTOR *UpVector )
{
	return 0 ;
}

extern int SoundBuffer_Set3DConeAngle_PF( SOUNDBUFFER *Buffer, float InnerAngle, float OuterAngle )
{
	return 0 ;
}

extern int SoundBuffer_Set3DConeVolume_PF( SOUNDBUFFER *Buffer, float InnerAngleVolume, float OuterAngleVolume )
{
	return 0 ;
}

extern int SoundBuffer_Refresh3DSoundParam_PF(	SOUNDBUFFER *Buffer, int AlwaysFlag )
{
	// 終了
	return 0 ;
}

extern int SoundBuffer_SetReverbParam_PF( SOUNDBUFFER *Buffer, SOUND3D_REVERB_PARAM *Param )
{
	return 0 ;
}

extern int SoundBuffer_SetPresetReverbParam_PF( SOUNDBUFFER *Buffer, int PresetNo )
{
	return 0 ;
}





















// ＭＩＤＩ演奏終了時呼ばれるコールバック関数
extern int MidiCallBackProcess( void )
{
	return 0 ;
}








#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // DX_NON_SOUND

