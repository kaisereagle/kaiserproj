//	core.h
//	todo:coreのプラットフォーム別をそのうちテンプレートに変更

#ifdef _MBCS 
#define _ATOM_CLASS  "CPP_CLASS"
#define _WNDCLSS "WIN_CLASS"
#define _WINTITLE "WIN_TITLE"

#endif
#ifdef _UNICODE 
#define _ATOM_CLASS  _T("CPP_CLASS")
#define _WNDCLSS	_T("WIN_CLASS"),
#define _WINTITLE _T("WIN_TITLE"),

#endif
#ifdef _M_AMD64
#define _WND_P GWLP_WNDPROC
#else
#define _WND_P GWL_WNDPROC

#endif
#ifndef __core__
#ifdef _WINDOWS
//#include <Winuser.h>
#include <windows.h>
//#include "windowsx.h"
#ifdef _USE_OPENGL



#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <GL/gl.h>
//#include "GLES2/gl2.h"
#endif
//ウィンドウプロシージャのサブクラス化を行うアトム変数
#define WINDOWS_SUBCLASS \
ATOM func_p; \

#define WINDOWS_STATICPROC \
LRESULT CALLBACK nammu::core::static_proc(HWND in_wnd, UINT in_msg, WPARAM in_wprm, LPARAM in_lprm) \
{ \
	core* p_base = (core*)GetProp(in_wnd, (LPCTSTR)(LONG64)func_p); \
	if (!p_base) \
	{ \
		if ((in_msg == WM_CREATE) || (in_msg == WM_NCCREATE)) \
			p_base = (core*)((LPCREATESTRUCT)in_lprm)->lpCreateParams; \
		if (p_base) \
			p_base->set_v(in_wnd); \
	} \
	if (p_base) \
	{ \
		LRESULT ret = p_base->v_proc(in_wnd, in_msg, in_wprm, in_lprm); \
		if (in_msg == WM_DESTROY) \
			p_base->release(in_wnd); \
		return ret; \
	} \
	return DefWindowProc(in_wnd, in_msg, in_wprm, in_lprm); \
};

extern ATOM func_p;
#endif

#ifdef _WINDOWS
	#define _MAIN() int WINAPI WinMain(HINSTANCE in_inst, HINSTANCE in_hpreinst,LPSTR in_cmd, int in_show)

#elif _CONSOLE
	#define _MAIN() int main() 

#endif

#ifdef _USE_DIRECTX
	#define RANDER_HANDLE HWND
#else
	#define RANDER_HANDLE HDC
#endif 

#ifdef _USE_DIRECTX
#include "d3d9.h"
struct Vertex {
	float x, y, z;
	union {
		unsigned color;
		struct { unsigned char b, g, r, a; };
	};
};

#endif

namespace nammu
{

	template <typename T>
	class rander
	{
	protected:
		T *view;
	public:
		rander() { view = new T; }
		~rander() { delete view; }
#ifdef _USE_OPENGL
		void render_init(HWND in_wnd)
		{
			view->rand_init(in_wnd);
			//view->init();
		}
#else
	#ifdef _WINDOWS
			void render_init(HWND in_wnd) 
			{
				view->rand_init(in_wnd);
			}
	#endif
#endif
		void draw()
		{
			view->draw();
		}

	};
	class _opengl
	{
		float m_rotation;
#ifdef _USE_OPENGL
		// カラーバッファ
		GLuint   buffer_color;
		// 深度＆ステンシルバッファ
		GLuint   buffer_depth_stencil;
		// レンダリング用のContext
		// EAGLContext *renderingContext;
		RANDER_HANDLE dc;
		HWND wnd;
		HGLRC glrc;
	#ifdef __APPLE__
			GLKView * m_glk_view;
	#endif

#endif
	public:
#ifdef _USE_OPENGL
		void rand_init(HWND in_wnd) 
		{
			wnd = in_wnd; 
			dc = GetDC(wnd);
			init();
	
		}
		void draw()
		{

			wglMakeCurrent(dc, glrc);
			glClearColor(0.0f, 0.5f, 1.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			glRectf(-0.5f, -0.5f, 0.5f, 0.5f);

			glFlush();
			SwapBuffers(dc);
			wglMakeCurrent(NULL, NULL);
		}
#endif
		_opengl() {}
		~_opengl() {}
#ifdef _USE_OPENGL
		bool init()
		{		
			
			//  ピクセルフォーマット初期化
			PIXELFORMATDESCRIPTOR pfd =
			{
				sizeof(PIXELFORMATDESCRIPTOR),
				1,
				PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,		//Flags
				PFD_TYPE_RGBA,													//The kind of framebuffer. RGBA or palette.
				32,																//Colordepth of the framebuffer.
				0, 0, 0, 0, 0, 0,
				0,
				0,
				0,
				0, 0, 0, 0,
				24,																//Number of bits for the depthbuffer
				8,																//Number of bits for the stencilbuffer
				0,																//Number of Aux buffers in the framebuffer.
				PFD_MAIN_PLANE,
				0,
				0, 0, 0
			};

			//glEnable(GL_DEPTH_TEST);
			//glFrontFace(GL_CCW);
			//glCullFace(GL_BACK);
			//glEnable(GL_CULL_FACE);
			int format = ChoosePixelFormat(dc, &pfd);
			if (format == 0)
				return false;	// 該当するピクセルフォーマットが無い

								// OpenGLが使うデバイスコンテキストに指定のピクセルフォーマットをセット
			if (!SetPixelFormat(dc, format, &pfd))
				return false;	// DCへフォーマットを設定するのに失敗

								// OpenGL contextを作成
			glrc = wglCreateContext(dc);

			// 作成されたコンテキストがカレント（現在使用中のコンテキスト）か？
			if (!wglMakeCurrent(dc, glrc))
				return false;	// 何か正しくないみたい…

			wglMakeCurrent(dc, glrc);
			//OpenGLの初期設定
			;
			wglMakeCurrent(dc, 0);
			//ReleaseDC(wnd, dc);
			SendMessage(wnd, WM_PAINT, 0, 0);
			return true;
		}
#endif
	};
#ifdef _WINDOWS
	#ifdef _USE_DIRECTX
		class _directx
		{
			HWND hWnd;
			HDC dc;
			LPDIRECT3D9 pD3D;
			LPDIRECT3DDEVICE9 pDev;
			Vertex vtx[4];

		public:
			void rand_init(HWND in_wnd)
			{
				hWnd = in_wnd;
				dc = GetDC(hWnd);
				_init();

			}
			_directx() {}
			~_directx() {}
			bool _init() 
			{
				Vertex v[4] = {
					{ -0.5f, -0.5f, 0.0f, 0xffe0e060 },
					{ -0.5f,  0.5f, 0.0f, 0xffe0e060 },
					{ 0.5f, -0.5f, 0.0f, 0xffe0e060 },
					{ 0.5f,  0.3f, 0.0f, 0xffe0e060 },
				};
				memcpy(vtx, v, sizeof(Vertex) * 4);
				if (!(pD3D = Direct3DCreate9(D3D_SDK_VERSION)))
					return false;

				D3DPRESENT_PARAMETERS d3dpp = { 640, 480, D3DFMT_A8R8G8B8, 0, D3DMULTISAMPLE_NONE, 0, D3DSWAPEFFECT_DISCARD, NULL, TRUE, TRUE, D3DFMT_D24S8, 0, D3DPRESENT_RATE_DEFAULT, D3DPRESENT_INTERVAL_ONE };

				if (FAILED(pD3D->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_HARDWARE_VERTEXPROCESSING, &d3dpp, &pDev)))
					if (FAILED(pD3D->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_SOFTWARE_VERTEXPROCESSING, &d3dpp, &pDev)))
					{
						pD3D->Release();
						return false;
					}

				return true;
			}
			void begin() 
			{
				pDev->BeginScene();
				pDev->Clear(0, 0, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, D3DCOLOR_COLORVALUE(0.0f, 0.5f, 1.0f, 1.0f), 1.0f, 0);
			}
			void swap() 
			{
				pDev->EndScene();
				pDev->Present(0, 0, 0, 0);
			}
			void terminate() 
			{
				if (pDev) pDev->Release();
				if (pD3D) pD3D->Release();
			}
			void render(Vertex *vtx, unsigned vtxNum, unsigned primNum) 
			{
				pDev->SetRenderState(D3DRS_LIGHTING, FALSE);
				pDev->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE);
				pDev->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, primNum, vtx, sizeof(Vertex));
			}
			void draw()
			{
				begin();
				render(vtx, 4, 2);
				swap();
			}
		};
	#endif
#endif
	class core
	{
	protected:
#ifdef _WINDOWS

		WNDPROC old_proc_adr;
		HWND wnd;
		HINSTANCE inst;
		WNDCLASSEX wcex;
		MSG msg;
		//		std::string class_name;文字列をstdクラスを使わないようにする

	public:
		HWND get_wnd() { return wnd; }
		bool set_v(HWND in_wnd)
		{
			func_p = GlobalAddAtom(_ATOM_CLASS);
			if (!SetProp(in_wnd, (LPCTSTR)(LONG64)func_p, (HANDLE)this))
				return false;
			old_proc_adr = (WNDPROC)(LONG64)SetWindowLong(in_wnd, _WND_P, (LONG)(LONG64)static_proc);
			if (!old_proc_adr)
				return false;
			return true;
		}
		bool release(HWND in_wnd)
		{
			if (old_proc_adr)
				SetWindowLong(in_wnd, _WND_P, (LONG)(LONG64)old_proc_adr);
			RemoveProp(in_wnd, (LPCTSTR)(LONG64)func_p);
			GlobalDeleteAtom(func_p);
			return true;
		}
		virtual LRESULT v_proc(HWND in_wnd, UINT in_msg, WPARAM in_wprm, LPARAM in_lprm) 
		{ __log("v_proc\n");  return DefWindowProc(in_wnd, in_msg, in_wprm, in_lprm); };
		static LRESULT CALLBACK static_proc(HWND in_wnd, UINT in_msg, WPARAM in_wprm, LPARAM in_lprm);
		HWND cr_win()//std::string in_title)
		{
			wnd = CreateWindowEx(
#ifdef _MAIDEN
				WS_EX_LAYERED |
#endif
				WS_EX_TOOLWINDOW,
				//				class_name.c_str(),
				//			in_title.c_str(),
				//上記引数にて切替用にする
				_WNDCLSS,
				_WINTITLE,
#ifdef _MAIDEN
				WS_POPUP,
#else
				WS_OVERLAPPEDWINDOW,
#endif

				0, 0, 640, 400, NULL, NULL, inst, this);

			ShowWindow(wnd, SW_SHOW);
			UpdateWindow(wnd);
			return wnd;
		}
		void init(HINSTANCE in_inst) { inst = in_inst; reg_class(); cr_win(); };
		void set_inst(HINSTANCE in_inst) { inst = in_inst; }
		void reg_class()
		{
			old_proc_adr = NULL;
			ZeroMemory(&wcex, sizeof(wcex));
			wcex.cbSize = sizeof(WNDCLASSEX);
			wcex.style = CS_HREDRAW | CS_VREDRAW;
			wcex.lpfnWndProc = (WNDPROC)core::static_proc;
			wcex.cbClsExtra = 0;
			wcex.cbWndExtra = 0;
			wcex.hInstance = inst;
			wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
			wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
			wcex.lpszClassName = _WNDCLSS;//class_name.c_str(); あとでウィンドウすクラスを設定できるように
			RegisterClassEx(&wcex);
		}
		void msg_loop()
		{
			while (GetMessage(&msg, NULL, 0, 0))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
		void render_init()
		{
#ifdef _USE_OPENGL

#endif

		}
#elif ANDROID

#elif __APPLE__

#endif
	public:
		core() 
		{


		}
		~core() { return; }
		void init()
		{
#ifdef _WINDOWS

			rander<_opengl> rand;

#else __APPLE__


#endif
		}
		void update();
		void draw();
		void end();
	};



}

#ifdef _USE_OPENGL
	#define RENDER nammu::rander<nammu::_opengl>
#else
	#define RENDER nammu::rander<nammu::_directx>
#endif

#endif // !__core__
