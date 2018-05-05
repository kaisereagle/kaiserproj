//-----------------------------------------------------------------------------
// 
// 		�c�w���C�u����		Android�p�T�E���h�v���O����
// 
//  	Ver 3.19 
// 
//-----------------------------------------------------------------------------

// �c�w���C�u�����쐬���p��`
#define __DX_MAKE

#include "../DxCompileConfig.h"

#ifndef DX_NON_SOUND

// �C���N���[�h----------------------------------------------------------------
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

// �}�N����`------------------------------------------------------------------

// �X�g���[���Đ��p�̃o�b�t�@��ӂ�̃T���v����
//#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(512)
#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(735)
//#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(1024)
//#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(2048)
//#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(4096)
//#define STREAM_SOUND_BUFFER_UNIT_SAPMLES	(8192)

// �^��`----------------------------------------------------------------------

// �f�[�^�錾------------------------------------------------------------------

// 8bit�g�`��16bit�g�`�ɕϊ����邽�߂̃e�[�u��
static short Bit8To16Table[ 256 ] ;

// �����f�[�^
static BYTE g_NoneSound8bit[  STREAM_SOUND_BUFFER_UNIT_SAPMLES ] ;
static WORD g_NoneSound16bit[ STREAM_SOUND_BUFFER_UNIT_SAPMLES ] ;

// �֐��v���g�^�C�v�錾 -------------------------------------------------------

static void RefreshSoundBufferCurrentTime( SOUNDBUFFER *Buffer, SLAndroidSimpleBufferQueueState BufferQueState, int IsCriticalsectionLock = TRUE, int EnableStopSoundBuffer = FALSE ) ;	// �T�E���h�o�b�t�@�̍Đ����Ԃ��X�V����
static void PlayerBufferQueueCallback( SLAndroidSimpleBufferQueueItf , void *Data ) ;	// �T�E���h�f�[�^���T�E���h�o�b�t�@�ɒǉ�����
static void EnqueueSoundBuffer( SOUNDBUFFER *Buffer ) ;									// �T�E���h�f�[�^���T�E���h�o�b�t�@�ɒǉ�����
static void *StreamSoundThreadFunction( void *argc ) ;

static void SoundBuffer_Add_StopSoundBufferList( SOUNDBUFFER *Buffer ) ;			// ��~�҂��T�E���h�o�b�t�@���X�g�Ɏw��̃T�E���h�o�b�t�@��������
static void SoundBuffer_Sub_StopSoundBufferList( SOUNDBUFFER *Buffer ) ;			// ��~�҂��T�E���h�o�b�t�@���X�g����w��̃T�E���h�o�b�t�@���O��

static void TerminateSoundBufferPlayInfo( int InfoIndex ) ;							// �T�E���h�o�b�t�@�Đ��p���̌�n�����s��( �N���e�B�J���Z�N�V���������b�N������ԂŌĂԂ��� )
static int NotUseSoundBufferPlayInfo( int InfoIndex ) ;								// �w��̃T�E���h�o�b�t�@�Đ��p�����g�p���Ă��Ȃ���ԂɕύX����
static int SetupSoundBufferPlayInfo( SOUNDBUFFER *Buffer ) ;						// �T�E���h�o�b�t�@�Đ��p���̃Z�b�g�A�b�v�ƁA�Z�b�g�A�b�v�������̔ԍ����擾����


// �v���O����------------------------------------------------------------------

// ��~�҂��T�E���h�o�b�t�@���X�g�Ɏw��̃T�E���h�o�b�t�@��������
static void SoundBuffer_Add_StopSoundBufferList( SOUNDBUFFER *Buffer )
{
	// �N���e�B�J���Z�N�V�����̎擾
	CRITICALSECTION_LOCK( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;

	// ���X�g�ɒǉ�
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

	// �N���e�B�J���Z�N�V�����̉��
	CriticalSection_Unlock( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;
}

// ��~�҂��T�E���h�o�b�t�@���X�g����w��̃T�E���h�o�b�t�@���O��
static void SoundBuffer_Sub_StopSoundBufferList( SOUNDBUFFER *Buffer )
{
	// �N���e�B�J���Z�N�V�����̎擾
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

	// �N���e�B�J���Z�N�V�����̉��
	CriticalSection_Unlock( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;
}

// ��~�҂��T�E���h�o�b�t�@���X�g�ɓo�^����Ă���T�E���h�o�b�t�@���~����
extern void SoundBuffer_Apply_StopSoundBufferList( void )
{
	for(;;)
	{
		SOUNDBUFFER *StopSoundBuffer = NULL ;

		// �N���e�B�J���Z�N�V�����̎擾
		CRITICALSECTION_LOCK( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;

		StopSoundBuffer = SoundSysData.PF.StopSoundBuffer ;

		// �N���e�B�J���Z�N�V�����̉��
		CriticalSection_Unlock( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;

		// �o�^����Ă���T�E���h�o�b�t�@��������ΏI��
		if( StopSoundBuffer == NULL )
		{
			break ;
		}

		// �Đ��̒�~
		SoundBuffer_Stop( StopSoundBuffer ) ;
	}

}

// �T�E���h�o�b�t�@�̍Đ����Ԃ��X�V����
static void RefreshSoundBufferCurrentTime( SOUNDBUFFER *Buffer, SLAndroidSimpleBufferQueueState BufferQueState, int IsCriticalsectionLock, int EnableStopSoundBuffer )
{
	if( IsCriticalsectionLock )
	{
		// �N���e�B�J���Z�N�V�����̎擾
		CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;
	}

	// �Đ��ʒu�̍X�V
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
					// ��~�҂��T�E���h�o�b�t�@���X�g�ɒǉ�
					SoundBuffer_Add_StopSoundBufferList( Buffer ) ;
				}
			}
		}
	}

	if( IsCriticalsectionLock )
	{
		// �N���e�B�J���Z�N�V�����̉��
		CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;
	}
}

// �o�b�t�@�L���[�C���^�[�t�F�[�X����Ă΂��R�[���o�b�N�֐�
static void PlayerBufferQueueCallback( SLAndroidSimpleBufferQueueItf BufferQueueInterface, void *Data )
{
	EnqueueSoundBuffer( ( SOUNDBUFFER * )Data ) ;
}

// �T�E���h�f�[�^���T�E���h�o�b�t�@�ɒǉ�����
static void EnqueueSoundBuffer( SOUNDBUFFER *Buffer )
{
	DWORD BytesRequired ;
	SLAndroidSimpleBufferQueueState BufferQueState ;
	int AddMaxCount ;
	int i ;

	// �N���e�B�J���Z�N�V�����̎擾
	CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;
	CRITICALSECTION_LOCK( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	if( Buffer->State == FALSE ||
		Buffer->PF.UseSoundBufferPlayInfoIndex < 0 ||
		Buffer->PF.UseSoundBufferPlayInfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
	{
		// �N���e�B�J���Z�N�V�����̉��
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
		// �N���e�B�J���Z�N�V�����̉��
		CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;
		CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;
		return ;
	}

	// �Đ��ʒu�̍X�V
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

				// �����o�b�t�@��ς�
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

	// �N���e�B�J���Z�N�V�����̉��
	CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;
	CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;
}

// �X�g���[���T�E���h�����p�X���b�h
static void *StreamSoundThreadFunction( void *argc )
{
	while( SoundSysData.PF.ProcessSoundThreadEndRequest == FALSE )
	{
		// �N���e�B�J���Z�N�V�����̎擾
		CRITICALSECTION_LOCK( &HandleManageArray[ DX_HANDLETYPE_SOUND ].CriticalSection ) ;

		// ��~�҂��T�E���h�o�b�t�@���X�g�ɓo�^����Ă���T�E���h�o�b�t�@���~����
		SoundBuffer_Apply_StopSoundBufferList() ;

		// �X�g���[�~���O����
		NS_ProcessStreamSoundMemAll() ;

		// �Đ����I��������n���h�����폜���鏈�����s��
//		ProcessPlayFinishDeleteSoundMemAll() ;

		// �R�c�T�E���h���Đ����Ă���T�E���h�n���h���ɑ΂��鏈�����s��
		ProcessPlay3DSoundMemAll() ;

		// �N���e�B�J���Z�N�V�����̉��
		CriticalSection_Unlock( &HandleManageArray[ DX_HANDLETYPE_SOUND ].CriticalSection ) ;


		// �N���e�B�J���Z�N�V�����̎擾
		CRITICALSECTION_LOCK( &HandleManageArray[ DX_HANDLETYPE_SOFTSOUND ].CriticalSection ) ;

		// �X�g���[�~���O����
		ST_SoftSoundPlayerProcessAll() ;

		// �N���e�B�J���Z�N�V�����̉��
		CriticalSection_Unlock( &HandleManageArray[ DX_HANDLETYPE_SOFTSOUND ].CriticalSection ) ;

		// �҂�
		Thread_Sleep( 10 ) ;
	}

	return NULL ;
}

// �T�E���h�V�X�e��������������֐��̊��ˑ��������s���֐�
extern int InitializeSoundSystem_PF_Timing0( void )
{
	int i ;

	if( SoundSysData.PF.EngineObjectInitialize )
	{
		return 0 ;
	}

	DXST_LOGFILE_ADDUTF16LE( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\x1d\x52\x1f\x67\x16\x53\x8b\x95\xcb\x59\x0a\x00\x00"/*@ L"OpenSL ES �̏������J�n\n" @*/ ) ;

	DXST_LOGFILE_TABADD ;

	// �X�g�b�v�T�E���h�o�b�t�@�p�̃N���e�B�J���Z�N�V������������
	if( CriticalSection_Initialize( &SoundSysData.PF.StopSoundBufferCriticalSection ) < 0 )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\x5c\x50\x62\x6b\xe6\x51\x06\x74\x28\x75\x6e\x30\xaf\x30\xea\x30\xc6\x30\xa3\x30\xab\x30\xeb\x30\xbb\x30\xaf\x30\xb7\x30\xe7\x30\xf3\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES �̃T�E���h�o�b�t�@��~�����p�̃N���e�B�J���Z�N�V�����̍쐬�Ɏ��s���܂���\n" @*/ )) ;
		DXST_LOGFILE_TABSUB ;
		return -1 ;
	}

	// �T�E���h�o�b�t�@�Đ��������p�̃N���e�B�J���Z�N�V������������
	if( CriticalSection_Initialize( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) < 0 )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\x8d\x51\x1f\x75\xe6\x51\x06\x74\xc5\x60\x31\x58\x28\x75\x6e\x30\xaf\x30\xea\x30\xc6\x30\xa3\x30\xab\x30\xeb\x30\xbb\x30\xaf\x30\xb7\x30\xe7\x30\xf3\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES �̃T�E���h�o�b�t�@�Đ��������p�̃N���e�B�J���Z�N�V�����̍쐬�Ɏ��s���܂���\n" @*/ )) ;
		DXST_LOGFILE_TABSUB ;
		return -1 ;
	}

	// 8bit�g�`��16bit�g�`�ɕϊ����邽�߂̃e�[�u����������
	for( i = 0 ; i < 256 ; i ++ )
	{
		Bit8To16Table[ i ] = ( short )( ( ( int )i * 65535 ) / 255 - 32768 ) ;
	}

	// �����f�[�^�̏�����
	for( i = 0 ; i < STREAM_SOUND_BUFFER_UNIT_SAPMLES ; i ++ )
	{
		g_NoneSound8bit[ i ]  = 128 ;
		g_NoneSound16bit[ i ] = 0 ;
	}
 
	// �G���W���I�u�W�F�N�g�쐬
	if( slCreateEngine( &SoundSysData.PF.EngineObject, 0, NULL, 0, NULL, NULL ) != SL_RESULT_SUCCESS )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xa8\x30\xf3\x30\xb8\x30\xf3\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES �̃G���W���I�u�W�F�N�g�̍쐬�Ɏ��s���܂���\n" @*/ )) ;
		DXST_LOGFILE_TABSUB ;
		return -1 ;
	}
 
	// �G���W���I�u�W�F�N�g�̃��A���C�Y
    if( ( *SoundSysData.PF.EngineObject )->Realize( SoundSysData.PF.EngineObject, SL_BOOLEAN_FALSE ) != SL_RESULT_SUCCESS )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xa8\x30\xf3\x30\xb8\x30\xf3\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xea\x30\xa2\x30\xe9\x30\xa4\x30\xba\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES �̃G���W���I�u�W�F�N�g�̃��A���C�Y�Ɏ��s���܂���\n" @*/ )) ;
		DXST_LOGFILE_TABSUB ;
		return -1 ;
	}

	// �G���W���I�u�W�F�N�g�̃C���^�[�t�F�[�X���擾
    if( ( *SoundSysData.PF.EngineObject )->GetInterface( SoundSysData.PF.EngineObject, SL_IID_ENGINE, &SoundSysData.PF.EngineInterface ) != SL_RESULT_SUCCESS )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xa8\x30\xf3\x30\xb8\x30\xf3\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xa4\x30\xb9\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES �̃G���W���I�u�W�F�N�g�̃C���^�[�t�F�[�X�擾�Ɏ��s���܂���\n" @*/ )) ;
		DXST_LOGFILE_TABSUB ;
		return -1 ;
	}

	// ProcessStreamSoundMemAll �����ĂԃX���b�h�̊J�n
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
			DXST_LOGFILEFMT_ADDUTF16LE(( "\x50\x00\x72\x00\x6f\x00\x63\x00\x65\x00\x73\x00\x73\x00\x53\x00\x74\x00\x72\x00\x65\x00\x61\x00\x6d\x00\x53\x00\x6f\x00\x75\x00\x6e\x00\x64\x00\x4d\x00\x65\x00\x6d\x00\x41\x00\x6c\x00\x6c\x00\x20\x00\x49\x7b\x92\x30\x7c\x54\x76\x30\xb9\x30\xec\x30\xc3\x30\xc9\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x45\x00\x72\x00\x72\x00\x6f\x00\x72\x00\x20\x00\x43\x00\x6f\x00\x64\x00\x65\x00\x20\x00\x3a\x00\x20\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x58\x00\x0a\x00\x00"/*@ L"ProcessStreamSoundMemAll �����ĂԃX���b�h�̍쐬�Ɏ��s���܂��� Error Code : 0x%08X\n" @*/, returnCode )) ;
			DXST_LOGFILE_TABSUB ;
			return -1 ;
		}
	}

	DXST_LOGFILE_TABSUB ;

	SoundSysData.PF.EngineObjectInitialize = TRUE ;

	DXST_LOGFILE_ADDUTF16LE( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\x1d\x52\x1f\x67\x16\x53\x8c\x5b\x86\x4e\x0a\x00\x00"/*@ L"OpenSL ES �̏���������\n" @*/ ) ;

	// �I��
	return 0 ;
}


// �T�E���h�V�X�e��������������֐��̊��ˑ��������s���֐�( ���s�ӏ���ʂP )
extern	int		InitializeSoundSystem_PF_Timing1( void )
{
#ifndef DX_NON_MULTITHREAD
	SoundSysData.PF.DXSoundProcessStart = TRUE ;
#endif // DX_NON_MULTITHREAD

	// ����I��
	return 0 ;
}




// �T�E���h�V�X�e���̌�n��������֐��̊��ˑ��������s���֐�( ���s�ӏ���ʂO )
extern	int		TerminateSoundSystem_PF_Timing0( void )
{
#ifndef DX_NON_MULTITHREAD
	SoundSysData.PF.DXSoundProcessStart = FALSE ;
#endif // DX_NON_MULTITHREAD

	// ProcessStreamSoundMemAll �����ĂԃX���b�h���I������
	SoundSysData.PF.ProcessSoundThreadEndRequest = TRUE ;
	pthread_join( SoundSysData.PF.ProcessSoundThread, NULL ) ;

	// ����I��
	return 0 ;
}


// �T�E���h�V�X�e���̌�n��������֐��̊��ˑ��������s���֐�( ���s�ӏ���ʂP )
extern	int		TerminateSoundSystem_PF_Timing1( void )
{
	SoundSysData.PF.EngineObjectInitialize = FALSE ;

	// �T�E���h�o�b�t�@�Đ��p���̌�n��
	{
		int i ;
		SOUNDBUFFERPLAYINFO *BufferPlayInfo ;

		// �N���e�B�J���Z�N�V�����̎擾
		CRITICALSECTION_LOCK( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

		BufferPlayInfo = SoundSysData.PF.SoundBufferPlayInfos ;
		for( i = 0 ; i < SOUNDBUFFERPLAYINFO_MAX_NUM ; i ++, BufferPlayInfo ++ )
		{
			if( BufferPlayInfo->SetupFlag )
			{
				TerminateSoundBufferPlayInfo( i ) ;
			}
		}

		// �N���e�B�J���Z�N�V�����̉��
		CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;
	}

	// �G���W���I�u�W�F�N�g�̌�n��
    if( SoundSysData.PF.EngineObject )
	{
        ( *SoundSysData.PF.EngineObject )->Destroy( SoundSysData.PF.EngineObject ) ;
        SoundSysData.PF.EngineObject = NULL ;
    }

	// �X�g�b�v�T�E���h�o�b�t�@�p�̃N���e�B�J���Z�N�V�������폜
	CriticalSection_Delete( &SoundSysData.PF.StopSoundBufferCriticalSection ) ;

	// �T�E���h�o�b�t�@�Đ��������p�̃N���e�B�J���Z�N�V�������폜
	CriticalSection_Delete( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	// ����I��
	return 0 ;
}

// �T�E���h�V�X�e���̏������`�F�b�N�̊��ˑ��������s���֐�( TRUE:����������Ă���  FALSE:����������Ă��Ȃ� )
extern	int		CheckSoundSystem_Initialize_PF( void )
{
	return SoundSysData.PF.EngineObjectInitialize ;
}

// �l�h�c�h�n���h���̌�n�����s���֐��̊��ˑ�����
extern	int		TerminateMidiHandle_PF( MIDIHANDLEDATA *MusicData )
{
	// ����I��
	return 0 ;
}

// �v���Z�b�g�̂R�c�T�E���h�p�̃��o�[�u�p�����[�^���擾���鏈���̊��ˑ��������s���֐�
extern	int		Get3DPresetReverbParamSoundMem_PF( SOUND3D_REVERB_PARAM *ParamBuffer, int PresetNo /* DX_REVERB_PRESET_DEFAULT �� */ )
{
	return 0 ;
}

// �R�c�T�E���h�̃��X�i�[�̈ʒu�ƃ��X�i�[�̑O���ʒu�ƃ��X�i�[�̏�����ʒu��ݒ肷�鏈���̊��ˑ��������s���֐�
extern int Set3DSoundListenerPosAndFrontPosAndUpVec_PF( VECTOR Position, VECTOR FrontPosition, VECTOR UpVector )
{
	return 0 ;
}

// �R�c�T�E���h�̃��X�i�[�̈ړ����x��ݒ肷�鏈���̊��ˑ��������s���֐�
extern int Set3DSoundListenerVelocity_PF( VECTOR Velocity )
{
	return 0 ;
}

// �R�c�T�E���h�̃��X�i�[�̉��p�x�͈͂�ݒ肷�鏈���̊��ˑ��������s���֐�
extern int Set3DSoundListenerConeAngle_PF( float InnerAngle, float OuterAngle )
{
	return 0 ;
}

// �R�c�T�E���h�̃��X�i�[�̉��p�x�͈͂̉��ʔ{����ݒ肷�鏈���̊��ˑ��������s���֐�
extern int Set3DSoundListenerConeVolume_PF( float InnerAngleVolume, float OuterAngleVolume )
{
	return 0 ;
}

// LoadMusicMemByMemImage �̎������֐��̊��ˑ��������s���֐�
extern int LoadMusicMemByMemImage_Static_PF( MIDIHANDLEDATA *MusicData, int ASyncThread )
{
	return 0 ;
}

// �ǂݍ��񂾂l�h�c�h�f�[�^�̉��t���J�n���鏈���̊��ˑ��������s���֐�
extern int PlayMusicMem_PF( MIDIHANDLEDATA *MusicData, int PlayType )
{
	return 0 ;
}

// �l�h�c�h�f�[�^�̉��t���~���鏈���̊��ˑ��������s��
extern int StopMusicMem_PF( MIDIHANDLEDATA *MusicData )
{
	return 0 ;
}

// �l�h�c�h�f�[�^�����t�����ǂ������擾����( TRUE:���t��  FALSE:��~�� )�����̊��ˑ��������s���֐�
extern int CheckMusicMem_PF( MIDIHANDLEDATA *MusicData )
{
	return 0 ;
}

// �l�h�c�h�f�[�^�̎����I�����̊��ˑ��������s���֐�
extern int ProcessMusicMem_PF( MIDIHANDLEDATA *MusicData )
{
	return 0 ;
}

// �l�h�c�h�f�[�^�̌��݂̍Đ��ʒu���擾���鏈���̊��ˑ��������s���֐�
extern int GetMusicMemPosition_PF( MIDIHANDLEDATA *MusicData )
{
	return 0 ;
}

// �l�h�c�h�̍Đ����ʂ��Z�b�g���鏈���̊��ˑ��������s���֐�
extern int SetVolumeMusic_PF( int Volume )
{
	return 0 ;
}

// �l�h�c�h�̌��݂̍Đ��ʒu���擾���鏈���̊��ˑ��������s���֐�
extern int GetMusicPosition_PF( void )
{
	return 0 ;
}









// �T�E���h�o�b�t�@�Đ��p���̌�n�����s��( �N���e�B�J���Z�N�V���������b�N������ԂŌĂԂ��� )
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

// �w��̃T�E���h�o�b�t�@�Đ��p�����g�p���Ă��Ȃ���ԂɕύX����
static int NotUseSoundBufferPlayInfo( int InfoIndex )
{
	SOUNDBUFFERPLAYINFO *BufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ InfoIndex ] ;

	if( InfoIndex < 0 || InfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
	{
		return -1 ;
	}

	// �N���e�B�J���Z�N�V�����̎擾
	CRITICALSECTION_LOCK( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	// ���Ɏg�p���Ă��Ȃ���ԂɂȂ��Ă����牽�������ɏI��
	if( BufferPlayInfo->UseFlag == FALSE )
	{
		// �N���e�B�J���Z�N�V�����̉��
		CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;
		
		return 0 ;
	}

	// �g�p���Ă��Ȃ���ԂɕύX
	BufferPlayInfo->UseFlag = FALSE ;

	// �Đ����~
	if( BufferPlayInfo->PlayerObject &&
		BufferPlayInfo->PlayerPlayInterface != NULL )
	{
		( *BufferPlayInfo->PlayerPlayInterface )->SetPlayState( BufferPlayInfo->PlayerPlayInterface, SL_PLAYSTATE_STOPPED ) ;
	}

	// �o�b�t�@���N���A
	if(  BufferPlayInfo->PlayerBufferQueueInterface != NULL &&
		*BufferPlayInfo->PlayerBufferQueueInterface != NULL )
	{
		( *BufferPlayInfo->PlayerBufferQueueInterface )->Clear( BufferPlayInfo->PlayerBufferQueueInterface ) ;
	}

	// �N���e�B�J���Z�N�V�����̉��
	CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	// ����I��
	return 0 ;
}

// �T�E���h�o�b�t�@�Đ��p���̃Z�b�g�A�b�v�ƁA�Z�b�g�A�b�v�������̔ԍ����擾����
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

	// �N���e�B�J���Z�N�V�����̎擾
	CRITICALSECTION_LOCK( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	// �����t�H�[�}�b�g�Ŋ��g�p����Ă��Ȃ��Đ�����T��
	BufferPlayInfo = SoundSysData.PF.SoundBufferPlayInfos ;
	NoUseIndex = -1 ;
	NoSetupIndex = -1 ;
	j = 0 ;
	for( i = 0 ; i < SOUNDBUFFERPLAYINFO_MAX_NUM && j < SoundSysData.PF.SoundBufferPlayInfoSetupNum ; i ++, BufferPlayInfo ++ )
	{
		if( BufferPlayInfo->SetupFlag == FALSE )
		{
			// �Z�b�g�A�b�v����Ă��Ȃ��v�f��ۑ�
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

		// �g�p����Ă��Ȃ��v�f��ۑ�
		if( NoUseIndex == -1 )
		{
			NoUseIndex = i ;
		}

		if( BufferPlayInfo->Channels      == Buffer->Format.nChannels      &&
			BufferPlayInfo->BitsPerSample == Buffer->Format.wBitsPerSample &&
			BufferPlayInfo->SamplesPerSec == Buffer->Format.nSamplesPerSec )
		{
			// ����������g�p��Ԃɂ��ĕԂ�
			BufferPlayInfo->UseFlag = TRUE ;

			// �o�b�t�@�L���[�C���^�[�t�F�[�X�̃R�[���o�b�N�֐��̐ݒ�
			Result = ( *BufferPlayInfo->PlayerBufferQueueInterface )->RegisterCallback( BufferPlayInfo->PlayerBufferQueueInterface, PlayerBufferQueueCallback, Buffer ) ;
			if( Result != SL_RESULT_SUCCESS )
			{
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\xad\x30\xe5\x30\xfc\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xb3\x30\xfc\x30\xeb\x30\xd0\x30\xc3\x30\xaf\x30\xa2\x95\x70\x65\x6e\x30\x7b\x76\x32\x93\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃o�b�t�@�L���[�C���^�[�t�F�[�X�̃R�[���o�b�N�֐��̓o�^�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
				return -1 ;
			}

			// �N���e�B�J���Z�N�V�����̉��
			CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

			// �Ԃ�
			return i ;
		}
	}

	// �Z�b�g�A�b�v����Ă��Ȃ��v�f���������炻�����g�p����
	if( NoSetupIndex >= 0 )
	{
		NewIndex = NoSetupIndex ;
	}
	else
	// �Đ���񂪍ő吔�ɒB���Ă���ꍇ����
	if( SoundSysData.PF.SoundBufferPlayInfoSetupNum >= BUFFERQUEUE_BUFFER_NUM )
	{
		// �g�p����Ă��Ȃ��v�f������������G���[
		if( NoUseIndex < 0 )
		{
			// �N���e�B�J���Z�N�V�����̉��
			CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

			DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xbb\x30\xc3\x30\xc8\x30\xa2\x30\xc3\x30\xd7\x30\xa8\x30\xe9\x30\xfc\x30\x01\x30\x8d\x51\x1f\x75\xc5\x60\x31\x58\x70\x65\x4c\x30\x00\x67\x27\x59\x70\x65\x20\x00\x25\x00\x64\x00\x20\x00\x6b\x30\x54\x90\x57\x30\x66\x30\x44\x30\x7e\x30\x59\x30\x0a\x00\x00"/*@ L"OpenSL ES �̃T�E���h�o�̓Z�b�g�A�b�v�G���[�A�Đ���񐔂��ő吔 %d �ɒB���Ă��܂�\n" @*/, BUFFERQUEUE_BUFFER_NUM )) ;
			return -1 ;
		}

		// �g�p����Ă��Ȃ��v�f���������ꍇ�͉���������s��
		TerminateSoundBufferPlayInfo( NoUseIndex ) ;

		// �g�p����o�b�t�@�̃A�h���X���Z�b�g
		NewIndex = NoUseIndex ;
	}
	else
	{
		// �ő吔�ɒB���Ă��Ȃ��ꍇ�͔z��̎g�p���ĂȂ��v�f�̃A�h���X���Z�b�g
		NewIndex = SoundSysData.PF.SoundBufferPlayInfoSetupNum ;
	}

SETUP :

	BufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ NewIndex ] ;

	if( NewIndex < 0 || NewIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
	{
		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xbb\x30\xc3\x30\xc8\x30\xa2\x30\xc3\x30\xd7\x30\xa8\x30\xe9\x30\xfc\x30\x01\x30\x4e\x00\x65\x00\x77\x00\x49\x00\x6e\x00\x64\x00\x65\x00\x78\x00\x20\x00\x24\x50\x4c\x30\x0d\x4e\x63\x6b\x0a\x00\x00"/*@ L"OpenSL ES �̃T�E���h�o�̓Z�b�g�A�b�v�G���[�ANewIndex �l���s��\n" @*/ )) ;
		return -1 ;
	}

	// �ő吔�ɒB���Ă��Ȃ��Ă��A�G���[�������������ɒB���Ă�����G���[����
	if( SoundSysData.PF.SoundBufferPlayInfoSetupErrorNum != 0 &&
		SoundSysData.PF.SoundBufferPlayInfoSetupErrorNum <= SoundSysData.PF.SoundBufferPlayInfoSetupNum )
	{
		goto ERR ;
	}

	// �t�H�[�}�b�g��ۑ�
	BufferPlayInfo->Channels      = Buffer->Format.nChannels      ;
	BufferPlayInfo->BitsPerSample = Buffer->Format.wBitsPerSample ;
	BufferPlayInfo->SamplesPerSec = Buffer->Format.nSamplesPerSec ;

	// �o�̓I�u�W�F�N�g�쐬
	Result = ( *SoundSysData.PF.EngineInterface )->CreateOutputMix( SoundSysData.PF.EngineInterface, &BufferPlayInfo->OutputMixObject, 0, NULL, NULL ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 0 ;
		goto ERR ;
	}

	// �o�̓I�u�W�F�N�g�̃��A���C�Y
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

	// �v���C���[�I�u�W�F�N�g�쐬
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

	// �v���C���[�I�u�W�F�N�g�̃��A���C�Y
	Result = ( *BufferPlayInfo->PlayerObject )->Realize( BufferPlayInfo->PlayerObject, SL_BOOLEAN_FALSE ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 3 ;
		goto ERR ;
	}

	// �Đ��C���^�[�t�F�[�X�̎擾
	Result = ( *BufferPlayInfo->PlayerObject )->GetInterface(BufferPlayInfo->PlayerObject, SL_IID_PLAY, &BufferPlayInfo->PlayerPlayInterface ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 4 ;
		goto ERR ;
	}

	// �o�b�t�@�L���[�C���^�[�t�F�[�X�̎擾
	Result = ( *BufferPlayInfo->PlayerObject )->GetInterface(BufferPlayInfo->PlayerObject, SL_IID_BUFFERQUEUE, &BufferPlayInfo->PlayerBufferQueueInterface ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 5 ;
		goto ERR ;
	}

	// ���ʃC���^�[�t�F�[�X�̎擾
	Result = ( *BufferPlayInfo->PlayerObject )->GetInterface(BufferPlayInfo->PlayerObject, SL_IID_VOLUME, &BufferPlayInfo->PlayerVolumeInterface ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 6 ;
		goto ERR ;
	}

	// �o�b�t�@�L���[�C���^�[�t�F�[�X�̃R�[���o�b�N�֐��̐ݒ�
	Result = ( *BufferPlayInfo->PlayerBufferQueueInterface )->RegisterCallback( BufferPlayInfo->PlayerBufferQueueInterface, PlayerBufferQueueCallback, Buffer ) ;
	if( Result != SL_RESULT_SUCCESS )
	{
		ErrorType = 7 ;
		goto ERR ;
	}

	// �Z�b�g�A�b�v������Ԃɂ���
	BufferPlayInfo->SetupFlag = TRUE ;
	SoundSysData.PF.SoundBufferPlayInfoSetupNum ++ ;

	// �g�p��Ԃɂ���
	BufferPlayInfo->UseFlag = TRUE ;

	// �N���e�B�J���Z�N�V�����̉��
	CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

	// ����I��
	return NewIndex ;


	// �G���[����
ERR :

	// �I�u�W�F�N�g�̌�n��
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

	// �Z�b�g�A�b�v����Ă��Ċ��g�p����Ă��Ȃ��v�f����������
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

	// �Z�b�g�A�b�v����Ă��Ċ��g�p����Ă��Ȃ��v�f�������ꍇ�̓G���[
	if( NoUseIndex == -1 )
	{
		// �N���e�B�J���Z�N�V�����̉��
		CriticalSection_Unlock( &SoundSysData.PF.SoundBufferPlayInfosCriticalSection ) ;

		// �O��G���[�����������ۂƃZ�b�g�A�b�v�ł��Ă���v�f�����قȂ�ꍇ�̂݃G���[���O���o��
		if( SoundSysData.PF.SoundBufferPlayInfoSetupErrorNum != 0 &&
			SoundSysData.PF.SoundBufferPlayInfoSetupErrorNum != SoundSysData.PF.SoundBufferPlayInfoSetupNum )
		{
			SoundSysData.PF.SoundBufferPlayInfoSetupErrorNum = SoundSysData.PF.SoundBufferPlayInfoSetupNum ;

			switch( ErrorType )
			{
			case 0 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃T�E���h�o�̓I�u�W�F�N�g�̍쐬�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
				break ;

			case 1 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xea\x30\xa2\x30\xe9\x30\xa4\x30\xba\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃T�E���h�o�̓I�u�W�F�N�g�̃��A���C�Y�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
				break ;

			case 2 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd7\x30\xec\x30\xa4\x30\xe4\x30\xfc\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃v���C���[�I�u�W�F�N�g�̍쐬�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
				break ;

			case 3 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd7\x30\xec\x30\xa4\x30\xe4\x30\xfc\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xea\x30\xa2\x30\xe9\x30\xa4\x30\xba\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃v���C���[�I�u�W�F�N�g�̃��A���C�Y�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
				break ;

			case 4 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\x8d\x51\x1f\x75\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̍Đ��C���^�[�t�F�[�X�̎擾�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
				break ;

			case 5 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\xad\x30\xe5\x30\xfc\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃o�b�t�@�L���[�C���^�[�t�F�[�X�̎擾�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
				break ;

			case 6 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xf3\x97\xcf\x91\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̉��ʃC���^�[�t�F�[�X�̎擾�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
				break ;

			case 7 :
				DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\xad\x30\xe5\x30\xfc\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xb3\x30\xfc\x30\xeb\x30\xd0\x30\xc3\x30\xaf\x30\xa2\x95\x70\x65\x6e\x30\x7b\x76\x32\x93\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃o�b�t�@�L���[�C���^�[�t�F�[�X�̃R�[���o�b�N�֐��̓o�^�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
				break ;
			}

			DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xbb\x30\xc3\x30\xc8\x30\xa2\x30\xc3\x30\xd7\x30\xa8\x30\xe9\x30\xfc\x30\x01\x30\xbb\x30\xc3\x30\xc8\x30\xa2\x30\xc3\x30\xd7\x30\x55\x30\x8c\x30\x66\x30\x44\x30\x66\x30\x14\x4e\x64\x30\x7f\x4f\x28\x75\x55\x30\x8c\x30\x66\x30\x44\x30\x6a\x30\x44\x30\x81\x89\x20\x7d\x4c\x30\x8b\x89\x64\x30\x4b\x30\x8a\x30\x7e\x30\x5b\x30\x93\x30\x67\x30\x57\x30\x5f\x30\x28\x00\x20\x00\x0c\x54\x42\x66\x8d\x51\x1f\x75\x70\x65\x1a\xff\x25\x00\x64\x00\x20\x00\x29\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃T�E���h�o�̓Z�b�g�A�b�v�G���[�A�Z�b�g�A�b�v����Ă��Ċ��g�p����Ă��Ȃ��v�f��������܂���ł���( �����Đ����F%d )\n" @*/, SoundSysData.PF.SoundBufferPlayInfoSetupNum )) ;
		}
		return -1 ;
	}

	// �Z�b�g�A�b�v����Ă��Ċ��g�p���Ă��Ȃ��v�f��������čă`�������W
	TerminateSoundBufferPlayInfo( NoUseIndex ) ;

	// �g�p����C���f�b�N�X�Ƃ��ăZ�b�g
	NewIndex = NoUseIndex ;

	goto SETUP ;
}



// �Đ��̏������s��
static int SoundBuffer_Play_Setup( SOUNDBUFFER *Buffer )
{
	// �Đ��������ł��Ă���ꍇ�͉������Ȃ�
	if( Buffer->PF.PlaySetupComp == TRUE )
	{
		return 0 ;
	}

	Buffer->PF.UseSoundBufferPlayInfoIndex = SetupSoundBufferPlayInfo( Buffer ) ;
	if( Buffer->PF.UseSoundBufferPlayInfoIndex < 0 )
	{
//		DXST_LOGFILE_ADDUTF16LE( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xbb\x30\xc3\x30\xc8\x30\xa2\x30\xc3\x30\xd7\x30\x4c\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"OpenSL ES �̃T�E���h�o�̓Z�b�g�A�b�v�����s���܂���\n" @*/ ) ;
		return -1 ;
	}

	// �Đ���������
	Buffer->PF.PlaySetupComp = TRUE ;

	// ���ʂ̐ݒ�
	SoundBuffer_RefreshVolume_PF( Buffer ) ;

	// ����I��
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
//	// �Đ��������ł��Ă���ꍇ�͉������Ȃ�
//	if( Buffer->PF.PlaySetupComp == TRUE )
//	{
//		return 0 ;
//	}
//
//	// �o�̓I�u�W�F�N�g�쐬
//	Result = ( *SoundSysData.PF.EngineInterface )->CreateOutputMix( SoundSysData.PF.EngineInterface, &Buffer->PF.OutputMixObject, 0, NULL, NULL ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃T�E���h�o�̓I�u�W�F�N�g�̍쐬�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// �o�̓I�u�W�F�N�g�̃��A���C�Y
//	Result = ( *Buffer->PF.OutputMixObject )->Realize( Buffer->PF.OutputMixObject, SL_BOOLEAN_FALSE ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xfa\x51\x9b\x52\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xea\x30\xa2\x30\xe9\x30\xa4\x30\xba\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃T�E���h�o�̓I�u�W�F�N�g�̃��A���C�Y�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
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
//	// �v���C���[�I�u�W�F�N�g�쐬
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
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd7\x30\xec\x30\xa4\x30\xe4\x30\xfc\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\x5c\x4f\x10\x62\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃v���C���[�I�u�W�F�N�g�̍쐬�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// �v���C���[�I�u�W�F�N�g�̃��A���C�Y
//	Result = ( *Buffer->PF.PlayerObject )->Realize( Buffer->PF.PlayerObject, SL_BOOLEAN_FALSE ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd7\x30\xec\x30\xa4\x30\xe4\x30\xfc\x30\xaa\x30\xd6\x30\xb8\x30\xa7\x30\xaf\x30\xc8\x30\x6e\x30\xea\x30\xa2\x30\xe9\x30\xa4\x30\xba\x30\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃v���C���[�I�u�W�F�N�g�̃��A���C�Y�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// �Đ��C���^�[�t�F�[�X�̎擾
//	Result = ( *Buffer->PF.PlayerObject )->GetInterface(Buffer->PF.PlayerObject, SL_IID_PLAY, &Buffer->PF.PlayerPlayInterface ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\x8d\x51\x1f\x75\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̍Đ��C���^�[�t�F�[�X�̎擾�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// �o�b�t�@�L���[�C���^�[�t�F�[�X�̎擾
//	Result = ( *Buffer->PF.PlayerObject )->GetInterface(Buffer->PF.PlayerObject, SL_IID_BUFFERQUEUE, &Buffer->PF.PlayerBufferQueueInterface ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\xad\x30\xe5\x30\xfc\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃o�b�t�@�L���[�C���^�[�t�F�[�X�̎擾�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// ���ʃC���^�[�t�F�[�X�̎擾
//	Result = ( *Buffer->PF.PlayerObject )->GetInterface(Buffer->PF.PlayerObject, SL_IID_VOLUME, &Buffer->PF.PlayerVolumeInterface ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xf3\x97\xcf\x91\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xd6\x53\x97\x5f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̉��ʃC���^�[�t�F�[�X�̎擾�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// �o�b�t�@�L���[�C���^�[�t�F�[�X�̃R�[���o�b�N�֐��̐ݒ�
//	Result = ( *Buffer->PF.PlayerBufferQueueInterface )->RegisterCallback( Buffer->PF.PlayerBufferQueueInterface, PlayerBufferQueueCallback, Buffer ) ;
//	if( Result != SL_RESULT_SUCCESS )
//	{
//		DXST_LOGFILEFMT_ADDUTF16LE(( "\x4f\x00\x70\x00\x65\x00\x6e\x00\x53\x00\x4c\x00\x20\x00\x45\x00\x53\x00\x20\x00\x6e\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\xad\x30\xe5\x30\xfc\x30\xa4\x30\xf3\x30\xbf\x30\xfc\x30\xd5\x30\xa7\x30\xfc\x30\xb9\x30\x6e\x30\xb3\x30\xfc\x30\xeb\x30\xd0\x30\xc3\x30\xaf\x30\xa2\x95\x70\x65\x6e\x30\x7b\x76\x32\x93\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x20\x00\x20\x00\xa8\x30\xe9\x30\xfc\x30\xb3\x30\xfc\x30\xc9\x30\x3a\x00\x30\x00\x78\x00\x25\x00\x30\x00\x38\x00\x78\x00\x0a\x00\x00"/*@ L"OpenSL ES �̃o�b�t�@�L���[�C���^�[�t�F�[�X�̃R�[���o�b�N�֐��̓o�^�Ɏ��s���܂���  �G���[�R�[�h:0x%08x\n" @*/, Result )) ;
//		goto ERR ;
//	}
//
//	// �Đ���������
//	Buffer->PF.PlaySetupComp = TRUE ;
//
//	// ���ʂ̐ݒ�
//	SoundBuffer_RefreshVolume_PF( Buffer ) ;
//
//	// ����I��
//	return 0 ;
//
//	// �G���[����
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

// �Đ��̌�n�����s��
static int SoundBuffer_Play_Terminate( SOUNDBUFFER *Buffer )
{
	// �Đ�����������Ă��Ȃ���Ή��������ɏI��
	if( Buffer->PF.PlaySetupComp == FALSE )
	{
		return 0 ;
	}

	// �T�E���h�o�b�t�@�Đ��p�������g�p���Ă��Ȃ���ԂɕύX
	NotUseSoundBufferPlayInfo( Buffer->PF.UseSoundBufferPlayInfoIndex ) ;
	Buffer->PF.UseSoundBufferPlayInfoIndex = -1 ;

	// �Z�b�g�A�b�v����Ă��Ȃ���ԂɕύX
	Buffer->PF.PlaySetupComp = FALSE ;

	return 0 ;
}

extern int SoundBuffer_Initialize_Timing0_PF( SOUNDBUFFER *Buffer, DWORD Bytes, WAVEFORMATEX *Format, SOUNDBUFFER *Src, int Is3DSound )
{
	int InitializeCriticalSection = FALSE ;

	// �[��������
	_MEMSET( &Buffer->PF, 0, sizeof( Buffer->PF ) ) ;

	// �N���e�B�J���Z�N�V������������
	if( CriticalSection_Initialize( &Buffer->PF.CriticalSection ) < 0 )
	{
		goto ERR ;
	}
	InitializeCriticalSection = TRUE ;

	// �T�E���h�o�b�t�@�p�̃��������m��
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
		DXST_LOGFILEFMT_ADDUTF16LE(( "\xb5\x30\xa6\x30\xf3\x30\xc9\x30\xd0\x30\xc3\x30\xd5\x30\xa1\x30\x28\x75\x6e\x30\xe1\x30\xe2\x30\xea\x30\x6e\x30\xba\x78\xdd\x4f\x6b\x30\x31\x59\x57\x65\x57\x30\x7e\x30\x57\x30\x5f\x30\x0a\x00\x00"/*@ L"�T�E���h�o�b�t�@�p�̃������̊m�ۂɎ��s���܂���\n" @*/ )) ;
		goto ERR ;
	}

	// ����I��
	return 0 ;

	// �G���[����
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
	// �N���e�B�J���Z�N�V�����̍폜
	CriticalSection_Delete( &Buffer->PF.CriticalSection ) ;

	return 0 ;
}

extern int SoundBuffer_CheckEnable_PF( SOUNDBUFFER *Buffer )
{
	return TRUE ;
}

extern int SoundBuffer_Play_PF( SOUNDBUFFER *Buffer, int Loop )
{
	// �N���e�B�J���Z�N�V�����̎擾
	CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;

	// ��~�҂��T�E���h�o�b�t�@���X�g����T�E���h�o�b�t�@���O��
	SoundBuffer_Sub_StopSoundBufferList( Buffer ) ;

	// �Đ�����������Ă��Ȃ�������
	if( Buffer->PF.PlaySetupComp == FALSE )
	{
		// �o�b�t�@�L���[�֌W�̏��̏�����
		Buffer->PF.EnqueueInfoNum = 0 ;
		Buffer->PF.BufferEndPlayEnqueueNum = 0 ;

		// �Đ�����
		if( SoundBuffer_Play_Setup( Buffer ) < 0 )
		{
			Buffer->State = FALSE ;

			// �N���e�B�J���Z�N�V�����̉��
			CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;
			return -1 ;
		}
	}

	Buffer->State = TRUE ;

	// �Đ����������������ꍇ�̂ݏ������s��
	if( Buffer->PF.PlaySetupComp )
	{
		// �o�b�t�@�L���[�ɃT�E���h�f�[�^��ǉ�����
		EnqueueSoundBuffer( Buffer ) ;

		// �Đ���ԂɕύX
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

	// �N���e�B�J���Z�N�V�����̉��
	CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;

	return 0 ;
}

extern int SoundBuffer_Stop_PF(	SOUNDBUFFER *Buffer, int EffectStop )
{
	// �N���e�B�J���Z�N�V�����̎擾
	CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;

	// �Đ�����������Ă���ꍇ�̂ݏ���
	if( Buffer->PF.PlaySetupComp )
	{
		SOUNDBUFFERPLAYINFO *SoundBufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ Buffer->PF.UseSoundBufferPlayInfoIndex ] ;

		if( Buffer->PF.UseSoundBufferPlayInfoIndex < 0 || Buffer->PF.UseSoundBufferPlayInfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
		{
			return -1 ;
		}

	//	( *Buffer->PF.PlayerPlayInterface )->SetPlayState( Buffer->PF.PlayerPlayInterface, SL_PLAYSTATE_PAUSED ) ;
		( *SoundBufferPlayInfo->PlayerPlayInterface )->SetPlayState( SoundBufferPlayInfo->PlayerPlayInterface, SL_PLAYSTATE_STOPPED ) ;

		// �Đ������̌�n��
		SoundBuffer_Play_Terminate( Buffer ) ;

		// �o�b�t�@�L���[�֌W�̏��̏�����
		Buffer->PF.EnqueueInfoNum = 0 ;
		Buffer->PF.BufferEndPlayEnqueueNum = 0 ;
	}

	Buffer->State = FALSE ;

	// ��~�҂��T�E���h�o�b�t�@���X�g����T�E���h�o�b�t�@���O��
	SoundBuffer_Sub_StopSoundBufferList( Buffer ) ;

	// �N���e�B�J���Z�N�V�����̉��
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

	// �_�u���T�C�Y�̏ꍇ�̓X�e���I�f�[�^�ɂ���
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

	// �Đ����������������ꍇ�̂ݏ������s��
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
//	// �N���e�B�J���Z�N�V�����̎擾
//	CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;
//
//	// �Đ����������������ꍇ�̂ݏ������s��
//	if( Buffer->PF.PlaySetupComp )
//	{
//		if( ( *Buffer->PF.PlayerBufferQueueInterface )->GetState( Buffer->PF.PlayerBufferQueueInterface, &BufferQueState ) == SL_RESULT_SUCCESS )
//		{
//			// �Đ��ʒu�̍X�V
//			RefreshSoundBufferCurrentTime( Buffer, BufferQueState, FALSE ) ;
//		}
//	}
//
//	// �N���e�B�J���Z�N�V�����̉��
//	CriticalSection_Unlock( &Buffer->PF.CriticalSection ) ;

	return 2 ;
}

extern int SoundBuffer_SetCurrentPosition_PF( SOUNDBUFFER *Buffer, DWORD NewPos )
{
	int OldState = Buffer->State ;

	// �N���e�B�J���Z�N�V�����̎擾
	CRITICALSECTION_LOCK( &Buffer->PF.CriticalSection ) ;

	if( OldState == TRUE )
	{
		SoundBuffer_Stop( Buffer ) ;
	}

	// �Đ����������������ꍇ�̂ݏ������s��
	if( Buffer->PF.PlaySetupComp )
	{
		SOUNDBUFFERPLAYINFO *SoundBufferPlayInfo = &SoundSysData.PF.SoundBufferPlayInfos[ Buffer->PF.UseSoundBufferPlayInfoIndex ] ;

		if( Buffer->PF.UseSoundBufferPlayInfoIndex < 0 || Buffer->PF.UseSoundBufferPlayInfoIndex >= SOUNDBUFFERPLAYINFO_MAX_NUM )
		{
			return -1 ;
		}

		// �o�b�t�@�L���[�̃T�E���h�f�[�^���N���A����
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

	// �N���e�B�J���Z�N�V�����̉��
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
	// �I��
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





















// �l�h�c�h���t�I�����Ă΂��R�[���o�b�N�֐�
extern int MidiCallBackProcess( void )
{
	return 0 ;
}








#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // DX_NON_SOUND

