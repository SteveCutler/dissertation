//------------------------------------------------------------------------------
// <copyright file="DepthBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

//
#include "KTMKinectRecorder.h"

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
{
    CDepthBasics application;
	
    application.Run(hInstance, nCmdShow);
}

/// <summary>
/// Constructor
/// </summary>
CDepthBasics::CDepthBasics() :
    m_pD2DFactory(NULL)//,
//    m_pDrawDepth(NULL)
{
	kinect = new KTMKinectWrapper();
}

/// <summary>
/// Destructor
/// </summary>
CDepthBasics::~CDepthBasics(){
    // clean up Direct2D renderer
    //delete m_pDrawDepth;
   // m_pDrawDepth = NULL;

    // clean up Direct2D
	if(m_pD2DFactory){
		m_pD2DFactory->Release();
		m_pD2DFactory = NULL;
	}
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CDepthBasics::Run(HINSTANCE hInstance, int nCmdShow){
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hInstance     = hInstance;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDD_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"DepthBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        hInstance,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CDepthBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    const int eventCount = 1;
    //HANDLE hEvents[eventCount];

    // Main message loop
    while (WM_QUIT != msg.message)
    {
      //  hEvents[0] = m_hNextDepthFrameEvent;

        // Check to see if we have either a message (by passing in QS_ALLINPUT)
        // Or a Kinect event (hEvents)
        // Update() will check for Kinect events individually, in case more than one are signalled
       // DWORD dwEvent = MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLINPUT);

        // Check if this is an event we're waiting on and not a timeout or message
       // if (WAIT_OBJECT_0 == dwEvent)
       // {
            Update();
       // }

        if (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if ((hWndApp != NULL) && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CDepthBasics::Update(){
	USHORT* depthData = NULL;
	if(NULL != kinect)
		depthData = kinect->nextFrame();
	if(NULL != depthData){
		m_pDrawDepth->DrawDepth(depthData, kinect->getFrameHeight() * kinect->getFrameWidth());
	}
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CDepthBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CDepthBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CDepthBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
			/* Connect Kinect Device */

			//SetStatusMessage(L"Connecting to device...");
			HRESULT hr;

            m_pDrawDepth = new ImageRenderer();
			hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_DEPTH_VIEW), m_pD2DFactory, kinect->getFrameWidth(), kinect->getFrameHeight(), kinect->getFrameWidth() * sizeof(ULONG));
            if (FAILED(hr))
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.");
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND:
            // If it was for the near mode control and a clicked event, change near mode
            if (IDC_CHECK_NEAR_MODE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
                m_bNearMode = !m_bNearMode;

                if(NULL != kinect)
					kinect->setNearMode(m_bNearMode);

				if(m_bNearMode)
					SetStatusMessage(L"Near mode enabled...");
				else
					SetStatusMessage(L"Near mode disabled...");
            }

			if(IDC_BUTTON_KINECT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				SetStatusMessage(L"Connecting to device...");

				if(FAILED(kinect->streamFromKinect()))
					SetStatusMessage(L"Could not connect to device...");
				else
					SetStatusMessage(L"Streaming from Kinect...");
			}

			if(IDC_BUTTON_RECORD_START == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				SetStatusMessage(L"Starting record...");
				if(kinect->setOutFile("output/out.avi", "DIB ")){
					kinect->record(true);
					SetStatusMessage(L"Recording...");
				}else{
					SetStatusMessage(L"Could not open stream for writing...");
				}
			}

			if(IDC_BUTTON_RECORD_STOP == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				SetStatusMessage(L"Recording stopped...");
				kinect->record(false);
				kinect->releaseOutFile();
			}

			if(IDC_BUTTON_FILE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK_NEAR_MODE), FALSE);
				SetStatusMessage(L"Disconnecting Kinect...");
				kinect->disconnectDevice();
				SetStatusMessage(L"Setting up file stream...");
				if(kinect->streamFromFile("output/out.avi"))
					SetStatusMessage(L"Streaming from file...");
				else
					SetStatusMessage(L"Could not stream from file...");
			}
            break;
    }

    return FALSE;
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
void CDepthBasics::SetStatusMessage(WCHAR * szMessage)
{
    SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, (LPARAM)szMessage);
}