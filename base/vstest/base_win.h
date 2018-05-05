
#ifdef _WIN32
#define _WIN32_WINNT 0x0500
	#include <windows.h>
#include "windowsx.h"
#include <winuser.h>
#include <string>
class base_win
{
protected:
    WNDPROC old_proc_adr;
	HWND wnd;
	HINSTANCE inst;
	WNDCLASSEX wcex;
	std::string class_name;
public:
	base_win(HINSTANCE in_inst,std::string in_class_name)
	{
		old_proc_adr = NULL;
		inst=in_inst;
		class_name=in_class_name;
		ZeroMemory(&wcex, sizeof(wcex));
		wcex.cbSize = sizeof(WNDCLASSEX);
		wcex.style = CS_HREDRAW | CS_VREDRAW;
		wcex.lpfnWndProc = (WNDPROC)base_win::static_proc;
		wcex.cbClsExtra = 0;
		wcex.cbWndExtra = 0;
		wcex.hInstance = in_inst;
		wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
		wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW+1);
		wcex.lpszClassName = class_name.c_str();
		RegisterClassEx( &wcex );
	};

	HWND cr_win(std::string in_title)
	{
		wnd=CreateWindowEx(WS_EX_LAYERED,//|WS_EX_TOOLWINDOW,
		 class_name.c_str(), in_title.c_str(),WS_POPUP,0,0,640,400,NULL,NULL,inst,this);

		ShowWindow(wnd, SW_SHOW);
		UpdateWindow(wnd);
		return wnd;
	}
	~base_win(void)
	{
		return;
	};
	bool set_v(HWND in_wnd);
    bool release(HWND in_wnd);
	virtual LRESULT v_proc(HWND in_wnd, UINT in_msg, WPARAM in_wprm, LPARAM in_lprm){return DefWindowProc(in_wnd, in_msg, in_wprm, in_lprm);};
    static LRESULT CALLBACK static_proc( HWND in_wnd, UINT in_msg, WPARAM in_wprm, LPARAM in_lprm);
	void init(HWND in_wnd);



};
#endif
