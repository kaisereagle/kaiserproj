// -------------------------------------------------------------------------------
// 
// 		�c�w���C�u����		Android�p�V�X�e���v���O�����w�b�_�t�@�C��
// 
// 				Ver 3.18f
// 
// -------------------------------------------------------------------------------

#ifndef __DXSYSTEMANDROID_H__
#define __DXSYSTEMANDROID_H__

// �C���N���[�h ------------------------------------------------------------------
#include "../DxCompileConfig.h"
#include <android/configuration.h>
#include <android/looper.h>
#include <android/native_activity.h>
#include <android/sensor.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <EGL/egl.h>

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// �}�N����` --------------------------------------------------------------------

// �\�t�g���A�N�e�B�u���ǂ������`�F�b�N����
#define CheckActiveState()					\
		if( g_AndroidSys.NonActiveRunFlag == FALSE &&\
			( ( g_AndroidSys.ActivityState != DX_ANDR_CMD_START &&\
			  g_AndroidSys.ActivityState != DX_ANDR_CMD_RESUME ) ||\
			  g_AndroidSys.NativeWindow == NULL ) )\
		{\
			DxActiveWait_Android() ;\
		}

// Looper �̃C�x���gID
#define DX_LOOPER_ID_MAIN							(1)				// ���C���X���b�h���瑗��R�}���h�C�x���g
#define DX_LOOPER_ID_INPUT							(2)				// ���̓C�x���g
#define DX_LOOPER_ID_SENSOR_ACCELEROMETER			(3)				// �����x�Z���T�[�̃C�x���g
#define DX_LOOPER_ID_SENSOR_MAGNETIC_FIELD			(4)				// ���E�Z���T�[�̃C�x���g
#define DX_LOOPER_ID_SENSOR_GYROSCOPE				(5)				// �W���C���X�R�[�v�Z���T�[�̃C�x���g
#define DX_LOOPER_ID_SENSOR_LIGHT					(6)				// �Ɠx�Z���T�[�̃C�x���g
#define DX_LOOPER_ID_SENSOR_PROXIMITY				(7)				// �ߐڃZ���T�[�̃C�x���g
#define DX_LOOPER_ID_SENSOR_PRESSURE				(8)				// �����Z���T�[�̃C�x���g
#define DX_LOOPER_ID_SENSOR_AMBIENT_TEMPERATURE		(9)				// ���x�Z���T�[�̃C�x���g

// �\�t�g���s�p�X���b�h�ɑ���R�}���h
#define DX_ANDR_CMD_START							(0)				// onStart ���Ă΂ꂽ
#define DX_ANDR_CMD_RESUME							(1)				// onResume ���Ă΂ꂽ
#define DX_ANDR_CMD_PAUSE							(2)				// onPause ���Ă΂ꂽ
#define DX_ANDR_CMD_STOP							(3)				// onStop ���Ă΂ꂽ
#define DX_ANDR_CMD_DESTROY							(4)				// onDestroy ���Ă΂ꂽ
#define DX_ANDR_CMD_GAINED_FOCUS					(5)				// onWindowFocusChanged ���Ă΂�A�t�H�[�J�XON�ɂȂ���
#define DX_ANDR_CMD_LOST_FOCUS						(6)				// onWindowFocusChanged ���Ă΂�A�t�H�[�J�XOFF�ɂȂ���
#define DX_ANDR_CMD_WINDOW_CHANGED					(7)				// �E�C���h�E�̕ύX
#define DX_ANDR_CMD_INPUT_CHANGED					(8)				// ���̓C�x���g���󂯎��L���[�̕ύX

// �\���̒�` --------------------------------------------------------------------

struct ANDROID_SENSOR_INFO
{
	const ASensor				*Sensor ;							// �Z���T�[
	ASensorEventQueue			*SensorEventQueue ;					// �Z���T�[�C�x���g�L���[
	ASensorEvent				SensorEvent ;						// �Z���T�[�C�x���g
} ;

struct DXLIB_ANDROID_SYSTEMINFO
{
	volatile int				NativeActivityBufferLength ;		// NativeActivity �A�h���X�i�[�p�o�b�t�@�Ɋi�[�ł���v�f��
	volatile int				NativeActivityNum ;					// NativeActivity �A�h���X�i�[�p�o�b�t�@�Ɋi�[����Ă���v�f�̐�
	ANativeActivity				**NativeActivityBuffer ;			// NativeActivity �A�h���X�i�[�p�o�b�t�@
	ANativeActivity				*NativeActivity ;					// �\�t�g�� Activity
	pthread_mutex_t				NativeActivityMutex ;				// NativeActivity �A�N�Z�X���p�~���[�e�b�N�X
	ALooper						*Looper ;							// �\�t�g�̃C�x���g�����p Looper

	volatile AInputQueue		*InputQueue ;						// ���̓C�x���g���󂯎�邽�߂̃L���[
	volatile AInputQueue		*NewInputQueue ;					// �V�������̓C�x���g���󂯎�邽�߂̃L���[

	volatile ANativeWindow		*NativeWindow ;						// �\�t�g�̃E�C���h�E
	volatile ANativeWindow		*NewNativeWindow ;					// �V�����\�t�g�̃E�C���h�E

	volatile int				ActivityState ;						// ���݂� Activity �̏��( DX_APP_CMD_RESUME, DX_APP_CMD_START, DX_APP_CMD_PAUSE, DX_APP_CMD_STOP �̂����ꂩ )
	volatile int				DestroyRequested ;					// onDestroy �R�}���h���\�t�g���s�p�X���b�h���󂯎�������ǂ���( 1:�󂯎����  0:�󂯎���Ă��Ȃ� )
	volatile int				DestroyRequestedTime ;				// onDestroy �R�}���h��������������
	volatile int				onDestroyEnd ;						// onDestroy �R�}���h�̏������I���������ǂ���( 0:�I�����Ă��Ȃ�  1:�I������ )

	volatile int				MutexLockIndex ;
	pthread_mutex_t				Mutex ;								// ���C���X���b�h�ƃ\�t�g���s�p�X���b�h�ł��݂��Ɏg�p����ϐ��A�N�Z�X�����Ɏg�p����~���[�e�b�N�X
	pthread_cond_t				Cond ;								// ���C���X���b�h�ƃ\�t�g���s�p�X���b�h�ł��݂��̏����̎��s�����҂��ȂǂɎg�p��������ϐ�

	volatile int				NonActiveRunFlag ;					// �A�v�����A�N�e�B�u�ł͂Ȃ��Ă����������s���邩�ǂ����̃t���O

	volatile int				MessageRead ;						// ���C���X���b�h����\�t�g���s�p�X���b�h�ւ̃��b�Z�[�W�󂯎��p�t�@�C���f�B�X�N���v�^
	volatile int				MessageWrite ;						// ���C���X���b�h����\�t�g���s�p�X���b�h�ւ̃��b�Z�[�W�������ݗp�t�@�C���f�B�X�N���v�^

	pthread_t					SoftThread ;						// �\�t�g���s�p�X���b�h
	volatile int				SoftThreadRunning ;					// �\�t�g���s�p�X���b�h�̎��s���J�n���ꂽ���ǂ���( 0:�J�n����Ă��Ȃ�  1:�J�n���ꂽ )
	volatile int				SoftThreadDestroyedStart ;			// �\�t�g���s�p�X���b�h�̏I���������J�n�������ǂ���( 0:�J�n����Ă��Ȃ�  1:�J�n���ꂽ )
	volatile int				SoftThreadDestroyed ;				// �\�t�g���s�p�X���b�h���I���������ǂ���( 0:�I�����Ă��Ȃ�  1:�I������ )
	volatile int				SoftThreadWait ;					// �\�t�g���s�p�X���b�h���҂���Ԃ��ǂ���( 1:�҂����  0:���s��� )

	volatile int				SoundAndMoviePause ;				// �E�C���h�E����A�N�e�B�u�ɂȂ�T�E���h�Ɠ��悪�ꎞ��~���Ă��邩�ǂ���( TRUE:�ꎞ��~���Ă���  FALSE:�ꎞ��~���Ă��Ȃ� )

	volatile void				( *LostFocusCallbackFunction )( void *Data ) ;		// �A�v���̃t�H�[�J�X������ꂽ�ۂɌĂ΂��R�[���o�b�N�֐�
	volatile void				*LostFocusCallbackFunctionData ;					// �A�v���̃t�H�[�J�X������ꂽ�ۂɌĂ΂��R�[���o�b�N�֐��ɓn���|�C���^
	volatile void				( *GainedFocusCallbackFunction )( void *Data ) ;	// �A�v���̃t�H�[�J�X������ꂽ�ۂɌĂ΂��R�[���o�b�N�֐�
	volatile void				*GainedFocusCallbackFunctionData ;					// �A�v���̃t�H�[�J�X������ꂽ�ۂɌĂ΂��R�[���o�b�N�֐��ɓn���|�C���^

	ASensorManager				*SensorManager ;					// �Z���T�[�}�l�[�W���[
	ANDROID_SENSOR_INFO			SensorInfos[ DX_ANDROID_SENSOR_NUM ] ;	// �e�Z���T�[�̏��

	jobject						object_EditText ;					// ��������͂��s�����߂� EditText
	jobject						object_AlertDialog_Builder ;		// ��������̓_�C�A���O�� AlertDialog.Builder
	jobject						object_Dialog ;						// ��������̓_�C�A���O�� Dialog
} ;

// �e�[�u��-----------------------------------------------------------------------

// �������ϐ��錾 --------------------------------------------------------------

extern DXLIB_ANDROID_SYSTEMINFO g_AndroidSys ;
extern int g_AndroidRunFlag ;

// �֐��v���g�^�C�v�錾-----------------------------------------------------------

// �X�N���[�����W���c�w���C�u������ʍ��W�ɕϊ�����
extern int ConvScreenPositionToDxScreenPosition( int ScreenX, int ScreenY, int *DxScreenX, int *DxScreenY ) ;

// �A�N�e�B�u�ɂȂ�܂ŉ������Ȃ�
extern void DxActiveWait_Android( void ) ;

#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // __DXSYSTEMANDROID_H__