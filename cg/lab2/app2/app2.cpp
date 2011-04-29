// app2.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "app2.h"
#include "task.h"

static int _width = 640;
static int _height = 480;

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);
void start_drawing();
void end_drawing();

int APIENTRY _tWinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPTSTR    lpCmdLine,
                     int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

 	// TODO: Place code here.
	MSG msg;
	HACCEL hAccelTable;

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_APP2, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_APP2));

	// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return (int) msg.wParam;
}



//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
//  COMMENTS:
//
//    This function and its usage are only necessary if you want this code
//    to be compatible with Win32 systems prior to the 'RegisterClassEx'
//    function that was added to Windows 95. It is important to call this function
//    so that the application will get 'well formed' small icons associated
//    with it.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_APP2));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_APP2);
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   HWND hWnd;

   hInst = hInstance; // Store instance handle in our global variable

   hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, _width, _height, NULL, NULL, hInstance, NULL);

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}

static HDC _hdc = NULL;
static HBITMAP _hbitmap = NULL;

static void create_buffer(HWND hwnd)
{
   end_drawing();

   if (_hbitmap)
      DeleteObject(_hbitmap);
   if (_hdc)
      DeleteDC(_hdc);

   RECT r;
   GetClientRect(hwnd, &r);

   _width = r.right - r.left;
   _height = r.bottom - r.top;

   HDC hdc = GetDC(hwnd);
      _hdc = CreateCompatibleDC(hdc );
      _hbitmap = CreateCompatibleBitmap(hdc, _width, _height);
   ReleaseDC(hwnd, hdc);
   HANDLE holdbitmap = (HBITMAP)SelectObject(_hdc, _hbitmap);
   DeleteObject(holdbitmap);

   start_drawing();
}

static RECT r = { 0, 0, 300, 300 };
VOID CALLBACK draw_timer_proc(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime)
{
   //BitBlt(_hdc, 0, 0, _width, _height, NULL, 0, 0, BLACKNESS );

   //r.left += 1;
   //r.right += 1;

   //FillRect(_hdc, &r, (HBRUSH) (COLOR_WINDOW+1));
   InvalidateRect(hwnd, NULL, FALSE);
}

struct render_data
{
   bool is_alive;
   int win_w;
   int win_h;
   HDC dc;
   HANDLE hmutex;

   render_data()
      : hmutex(CreateMutex(NULL, FALSE, NULL))
   {
   }
};
static render_data rd;

DWORD WINAPI thread_fun(LPVOID data)
{
   render_data * rd = (render_data *)data;
   render(rd->win_w, rd->win_h, rd->dc, &rd->is_alive, rd->hmutex);
   return 0;
}

HANDLE hthread = NULL;

static int num_threads = 0;
void start_drawing()
{
   end_drawing();
   rd.win_w = _width;
   rd.win_h = _height;
   rd.is_alive = true;
   rd.dc = _hdc;

   hthread = CreateThread(NULL, 0, thread_fun, (LPVOID)&rd, 0, NULL);
   num_threads++;
}

void end_drawing()
{
   if (hthread)
   {
      rd.is_alive = false;
      WaitForSingleObject(hthread, INFINITE);
      CloseHandle(hthread);
   }
   hthread = NULL;
}


LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	PAINTSTRUCT ps;
	HDC hdc;

	switch (message)
	{
   case WM_CREATE:
      {
         create_buffer(hWnd);
         SetTimer(hWnd, NULL, 50, draw_timer_proc);
      }
      break;
	case WM_COMMAND:
		wmId    = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;
	case WM_PAINT:
      {
		   hdc = BeginPaint(hWnd, &ps);

         WaitForSingleObject(rd.hmutex, INFINITE);
         BOOL res = BitBlt(hdc, 0, 0, _width, _height, _hdc, 0, 0, SRCCOPY);
         ReleaseMutex(rd.hmutex);

		   EndPaint(hWnd, &ps);
      }
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
   case WM_SIZE:
      _width = LOWORD(lParam);
      _height = HIWORD(lParam);
      create_buffer(hWnd);
      break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}
