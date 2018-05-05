// -------------------------------------------------------------------------------
// 
// 		ＤＸライブラリ		WinAPIプログラムヘッダファイル
// 
// 				Ver 3.18f
// 
// -------------------------------------------------------------------------------

#ifndef __DXWINAPI_H__
#define __DXWINAPI_H__

// インクルード ------------------------------------------------------------------
#include "../DxCompileConfig.h"

#if !defined(CINTERFACE) && defined(__c2__) &&  __clang_major__ == 3 && __clang_minor__ == 8
//To avoid compile error
//C:\Program Files (x86)\Windows Kits\8.1\Include\um\combaseapi.h(229,21): error : unknown type name 'IUnknown'
//          static_cast<IUnknown*>(*pp);    // make sure everyone derives from IUnknown
#define CINTERFACE
#endif
#include <windows.h>

#include <winsock.h>
#include <wtypes.h>
#include "../DxNetwork.h"
#include "DxDirectX.h"

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// マクロ定義 --------------------------------------------------------------------

#define D_HTOUCHINPUT		void *

// コールバック関数定義 ----------------------------------------------------------

typedef VOID ( CALLBACK* D_TIMERPROC )( HWND, UINT, UINT_PTR, DWORD ) ;

#if _MSC_VER > 1200 || defined( DX_GCC_COMPILE_4_9_2 )
typedef void ( CALLBACK D_TIMECALLBACK )( UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2 ) ;
#else
typedef void ( CALLBACK D_TIMECALLBACK )( UINT uTimerID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2 ) ;
#endif
typedef D_TIMECALLBACK FAR *LPD_TIMECALLBACK ;

// 構造体定義 --------------------------------------------------------------------

typedef struct D_tagWCRANGE
{
	WCHAR					wcLow ;
	USHORT					cGlyphs ;
} D_WCRANGE, *D_PWCRANGE,FAR *D_LPWCRANGE ;

typedef struct D_tagGLYPHSET
{
	DWORD					cbThis ;
	DWORD					flAccel ;
	DWORD					cGlyphsSupported ;
	DWORD					cRanges ;
	D_WCRANGE				ranges[ 1 ] ;
} D_GLYPHSET, *D_PGLYPHSET, FAR *D_LPGLYPHSET;

typedef struct D__TOUCHINPUT
{
	LONG					x ;
	LONG					y ;
	HANDLE					hSource ;
	DWORD					dwID ;
	DWORD					dwFlags ;
	DWORD					dwMask ;
	DWORD					dwTime ;
#ifdef _WIN64
	ULONGLONG				dwExtraInfo ;
#else
	DWORD					dwExtraInfo;
#endif
	DWORD					cxContact ;
	DWORD					cyContact ;
} D_TOUCHINPUT, *D_PTOUCHINPUT ;
typedef D_TOUCHINPUT const * D_PCTOUCHINPUT;

#ifndef DX_NON_NETWORK

// WinSock の DLL のポインタや中のAPIのポインタなど
struct WINSOCKFUNCTION
{
	HMODULE					WinSockDLL ;						// WinSockDLL
	int						( WINAPI *WSAGetLastErrorFunc )( void ) ;
	int						( WINAPI *WSAStartupFunc )( WORD wVersionRequested, LPWSADATA lpWSAData ) ;
	int						( WINAPI *WSACleanupFunc )( void ) ;
	int						( WINAPI *WSAAsyncSelectFunc )( SOCKET s, HWND hWnd, unsigned int wMsg, long lEvent ) ;
	int						( WINAPI *getaddrinfoFunc )( const char *nodename, const char *servname, const _addrinfo *hints, _addrinfo **res ) ;
	struct hostent*			( WINAPI *gethostbyaddrFunc )( const char *addr, int len, int type ) ;
	struct hostent*			( WINAPI *gethostbynameFunc )( const char *name ) ;
	int						( WINAPI *gethostnameFunc )( char *name, int namelen ) ;
	u_short					( WINAPI *ntohsFunc )( u_short netshort ) ;
	u_short					( WINAPI *htonsFunc )( u_short hostshort ) ;
	int						( WINAPI *connectFunc )( SOCKET s, const struct sockaddr *name, int namelen ) ;
	SOCKET					( WINAPI *socketFunc )( int af, int type, int protocol ) ;
	int						( WINAPI *sendFunc )( SOCKET s, const char *buf, int len, int flags ) ;
	int						( WINAPI *sendtoFunc )( SOCKET s, const char *buf, int len, int flags, const struct sockaddr *to, int tolen ) ;
	int						( WINAPI *recvfromFunc )( SOCKET s, char *buf, int len, int flags, struct sockaddr *from, int *fromlen ) ;
	SOCKET					( WINAPI *acceptFunc )( SOCKET s, struct sockaddr *addr, int *addrlen ) ;
	int						( WINAPI *closesocketFunc )( SOCKET s ) ;
	int						( WINAPI *shutdownFunc )( SOCKET s, int how ) ;
	int						( WINAPI *listenFunc )( SOCKET s, int backlog ) ;
	int						( WINAPI *bindFunc )( SOCKET s, const struct sockaddr *name, int namelen ) ;
	unsigned long			( WINAPI *inet_addrFunc )( const char *cp ) ;
	int						( WINAPI *recvFunc )( SOCKET s, char *buf, int len, int flags ) ;
	int						( WINAPI *setsockoptFunc )( SOCKET s, int level, int optname, const char *optval, int optlen ) ;
} ;

#endif // DX_NON_NETWORK

// Input Method Manager DLL のポインタや中のAPIのポインタなど
struct IMMFUNCTION
{
	HMODULE					Imm32DLL ;
	HIMC					( WINAPI *ImmGetContextFunc )( HWND hWnd );
	BOOL					( WINAPI *ImmReleaseContextFunc )( HWND hWnd, HIMC hIMC ) ;
	BOOL					( WINAPI *ImmGetOpenStatusFunc )( HIMC hIMC ) ;
	BOOL					( WINAPI *ImmGetConversionStatusFunc )( HIMC hIMC, LPDWORD lpfdwConversion, LPDWORD lpfdwSentence ) ;
	BOOL					( WINAPI *ImmNotifyIMEFunc )( HIMC hIMC, DWORD dwAction, DWORD dwIndex, DWORD dwValue ) ;
	BOOL					( WINAPI *ImmSetOpenStatusFunc )( HIMC hIMC, BOOL fOpen ) ;

	DWORD					( WINAPI *ImmGetCandidateListFunc )( HIMC hIMC, DWORD deIndex, LPCANDIDATELIST lpCandList, DWORD dwBufLen ) ;
	DWORD					( WINAPI *ImmGetCandidateListCountFunc )( HIMC hIMC, LPDWORD lpdwListCount ) ;
	LONG					( WINAPI *ImmGetCompositionStringFunc )( HIMC hIMC, DWORD dwIndex, LPVOID lpBuf, DWORD dwBufLen ) ;
	BOOL					( WINAPI *ImmSetCompositionStringFunc )( HIMC hIMC, DWORD dwIndex, LPCVOID lpComp, DWORD dwCompLen, LPCVOID lpRead, DWORD dwReadLen ) ;
} ;

// Win32 API DLL のポインタや API のポインタなど
struct WIN32APIFUNCTION
{
	HMODULE					WinMMDLL ;
	MMRESULT				( WINAPI *timeSetEventFunc )( UINT uDelay, UINT uResolution, LPD_TIMECALLBACK lpTimeProc, DWORD_PTR dwUser, UINT fuEvent ) ;
	MMRESULT				( WINAPI *timeKillEventFunc )( UINT uTimerID ) ;
	MMRESULT				( WINAPI *timeBeginPeriodFunc )( UINT uPeriod ) ;
	MMRESULT				( WINAPI *timeEndPeriodFunc )( UINT uPeriod ) ;
	DWORD					( WINAPI *timeGetTimeFunc )( VOID ) ;
	MMRESULT				( WINAPI *timeGetDevCapsFunc )( LPTIMECAPS ptc, UINT cbtc ) ;
	MMRESULT				( WINAPI *joyGetPosExFunc )( UINT uJoyID, LPJOYINFOEX pji ) ;

	MMRESULT				( WINAPI *joyGetDevCapsFunc )( UINT uJoyID, LPJOYCAPSW pjc, UINT cbjc ) ;
	MCIERROR				( WINAPI *mciSendCommandFunc )( MCIDEVICEID IDDevice, UINT uMsg, DWORD_PTR fdwCommand, DWORD_PTR dwParam ) ;


	HMODULE					Kernel32DLL ;
	HMODULE					( WINAPI *GetModuleHandleWFunc )( LPCWSTR lpModuleName ) ;
	BOOL					( WINAPI *VerifyVersionInfoWFunc )( LPOSVERSIONINFOEXW lpVersionInformation, DWORD dwTypeMask, DWORDLONG dwlConditionMask ) ;
	ULONGLONG				( WINAPI *VerSetConditionMaskFunc )( ULONGLONG ConditionMask, DWORD TypeMask, BYTE Condition ) ;


	HMODULE					Old32DLL ;
	HRESULT					( WINAPI *CoCreateInstanceFunc )( REFCLSID rclsid, D_IUnknown * pUnkOuter, DWORD dwClsContext, REFIID riid, LPVOID *ppv ) ;
	LPVOID					( WINAPI *CoTaskMemAllocFunc )( SIZE_T cb ) ;
	void					( WINAPI *CoTaskMemFreeFunc )( LPVOID pv ) ;
	HRESULT					( WINAPI *CoInitializeExFunc )( LPVOID pvReserved, DWORD dwCoInit ) ;
	void					( WINAPI *CoFreeUnusedLibrariesFunc )( void ) ;
	void					( WINAPI *CoUninitializeFunc )( void ) ;


	HMODULE					OleAut32DLL ;
	void					( WINAPI *SysFreeStringFunc )( BSTR bstrString ) ;


	HMODULE					Comctl32DLL ;
	void					( WINAPI *InitCommonControlsFunc )( VOID ) ;


	HMODULE					User32DLL ;
	BOOL					( WINAPI *WINNLSEnableIME_Func )( HWND hwnd, BOOL bFlag ) ;	// WINNLSEnableIME APIのアドレス
	BOOL					( WINAPI *UpdateLayeredWindow )( HWND, HDC, POINT*, SIZE*, HDC, POINT*, COLORREF, BLENDFUNCTION*, DWORD ) ;		// UpdateLayeredWindow のＡＰＩポインタ
	HWND					( WINAPI *CreateWindowExWFunc )( DWORD dwExStyle, LPCWSTR lpClassName, LPCWSTR lpWindowName, DWORD dwStyle, int X, int Y, int nWidth, int nHeight, HWND hWndParent, HMENU hMenu, HINSTANCE hInstance, LPVOID lpParam ) ;
	BOOL					( WINAPI *EnumDisplayDevicesWFunc )( LPCWSTR lpDevice, DWORD iDevNum, PDISPLAY_DEVICEW lpDisplayDevice, DWORD dwFlags ) ;
	BOOL					( WINAPI *CloseTouchInputHandleFunc )( D_HTOUCHINPUT hTouchInput ) ;
	BOOL					( WINAPI *GetTouchInputInfoFunc )( D_HTOUCHINPUT hTouchInput, UINT cInputs, D_PTOUCHINPUT pInputs, int cbSize ) ;
	BOOL					( WINAPI *IsTouchWindowFunc )( HWND hWnd, ULONG *pulFlags ) ;
	BOOL					( WINAPI *RegisterTouchWindowFunc )( HWND hWnd, ULONG ulFlags ) ;
	BOOL					( WINAPI *UnregisterTouchWindowFunc )( HWND hWnd ) ;
	BOOL					( WINAPI *ShutdownBlockReasonCreateFunc )( HWND hWnd, LPCWSTR pwszReason ) ;
	BOOL					( WINAPI *ShutdownBlockReasonDestroyFunc )( HWND hWnd ) ;
	UINT_PTR				( WINAPI *SetTimerFunc )( HWND hWnd, UINT_PTR nIDEvent, UINT uElapse, D_TIMERPROC lpTimerFunc ) ;


	HMODULE					GDI32DLL ;
	HANDLE					( WINAPI *AddFontMemResourceExFunc )( LPVOID pbFont, DWORD cbFont, void /* DESIGNVECTOR */ *pdv, DWORD *pcFonts ) ;
	int						( WINAPI *RemoveFontMemResourceExFunc )( HANDLE fh ) ;
	DWORD					( WINAPI *GetFontUnicodeRangesFunc )( HDC hdc, D_LPGLYPHSET lpgs ) ;


	HMODULE					NTDLL ;
	void					( WINAPI *RtlGetVersionFunc )( LPOSVERSIONINFOEXW lpVersionInfomation ) ;


	HMODULE					MFPLATDLL ;
	HRESULT					( WINAPI *MFStartupFunc )( ULONG Version, DWORD dwFlags ) ;
	HRESULT					( WINAPI *MFShutdownFunc )( void ) ;
	HRESULT					( WINAPI *MFCreateMediaTypeFunc )( D_IMFMediaType **ppMFType ) ;
	HRESULT					( WINAPI *MFCreateWaveFormatExFromMFMediaTypeFunc )( D_IMFMediaType *pMFType, WAVEFORMATEX **ppWF, UINT32 *pcbSize, UINT32 Flags ) ;
	HRESULT					( WINAPI *MFCreateAttributesFunc )( D_IMFAttributes** ppMFAttributes, UINT32 cInitialSize ) ;
	HRESULT					( WINAPI *MFCreateAsyncResultFunc )( D_IUnknown * punkObject, D_IMFAsyncCallback * pCallback, D_IUnknown * punkState, D_IMFAsyncResult ** ppAsyncResult ) ;
	HRESULT					( WINAPI *MFInvokeCallbackFunc )( D_IMFAsyncResult * pAsyncResult ) ;


	HMODULE					MFREADWRITEDLL ;
	HRESULT					( WINAPI *MFCreateSourceReaderFromURLFunc )( LPCWSTR pwszURL, D_IMFAttributes *pAttributes, D_IMFSourceReader **ppSourceReader ) ;
	HRESULT					( WINAPI *MFCreateSourceReaderFromByteStreamFunc )( D_IMFByteStream *pByteStream, D_IMFAttributes *pAttributes, D_IMFSourceReader **ppSourceReader ) ;


	HMODULE					PROPSYSDLL ;
	HRESULT					( WINAPI *PropVariantToInt64Func )( const D_PROPVARIANT &propvarIn, LONGLONG *pllRet ) ;
} ;

// WinAPI 情報構造体
struct WINAPIDATA
{
#ifndef DX_NON_NETWORK
	WINSOCKFUNCTION			WinSockFunc ;						// WinSock API 関係のデータ
#endif

#ifndef DX_NON_KEYEX
	IMMFUNCTION				ImmFunc ;							// IMM API 関係のデータ
#endif

	WIN32APIFUNCTION		Win32Func ;							// Win32 API 関係のデータ

	HMODULE					DwmApiDLL ;							// Desktop Window Manager API DLL
	HRESULT					( WINAPI *DF_DwmEnableComposition )( UINT uCompositionAction ) ;	// DwmEnableComposition API のアドレス
	HRESULT					( WINAPI *DwmGetWindowAttributeFunc )( HWND hwnd, DWORD dwAttribute, PVOID pvAttribute, DWORD cbAttribute ) ;
} ;

// 内部大域変数宣言 --------------------------------------------------------------

extern WINAPIDATA WinAPIData ;

// 関数プロトタイプ宣言-----------------------------------------------------------

extern int LoadWinAPI( void ) ;				// WindowsOS の DLL を読み込む
extern int ReleaseWinAPI( void ) ;			// WindowsOS の DLL を解放する

extern HRESULT WinAPI_CoCreateInstance_ASync( REFCLSID rclsid, D_IUnknown * pUnkOuter, DWORD dwClsContext, REFIID riid, LPVOID *ppv, int ASyncThread = FALSE ) ;

#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // __DXWINAPI_H__
