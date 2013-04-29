//------------------------------------------------------------------------------
// <copyright file="DepthBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

//
#include "KTMKinectRecorder.hpp"
#include "NotificationInterface.hpp"

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
    KinectRecorder application;
	
    application.Run(hInstance, nCmdShow);
}

/// <summary>
/// Constructor
/// </summary>
KinectRecorder::KinectRecorder() :
    m_pD2DFactory(NULL)
{
	kinect = new KTM::KinectWrapper();
}

/// <summary>
/// Destructor
/// </summary>
KinectRecorder::~KinectRecorder(){
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
int KinectRecorder::Run(HINSTANCE hInstance, int nCmdShow){
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
        (DLGPROC)KinectRecorder::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    const int eventCount = 1;
    //HANDLE hEvents[eventCount];

	KTM::NotificationInterface::setWindowHandle(m_hWnd);
	KTM::NotificationInterface::setMessageBoxControl(IDC_STATUS);

    // Main message loop
    while (WM_QUIT != msg.message)
    {
		Update();

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
void KinectRecorder::Update(){
	try{
	USHORT* depthData = NULL;
	char* RGBData = NULL;

	int width;
	int height;
	
	
	if(NULL != kinect){
		
		kinect->nextFrame(depthData, RGBData);

		
		kinect->getDepthResolution(width, height);
		m_pDrawDepth->DrawDepth(depthData, width * height);

		
		kinect->getRGBResolution(width, height);
		m_pDrawRGB->DrawRGB(RGBData, width * height * OUT_FRAME_CHANNELS);


	}
	}catch(...){
		KTM::NotificationInterface::setMessage(L"Exception thrown in update!");
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
LRESULT CALLBACK KinectRecorder::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    KinectRecorder* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<KinectRecorder*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<KinectRecorder*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
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
LRESULT CALLBACK KinectRecorder::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
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

			HRESULT hr;

            m_pDrawDepth = new ImageRenderer();
			m_pDrawRGB = new ImageRenderer();
			hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_DEPTH_VIEW), m_pD2DFactory, DEPTH_FRAME_WIDTH, DEPTH_FRAME_HEIGHT, DEPTH_FRAME_WIDTH * sizeof(ULONG));
            if (FAILED(hr))
                SetStatusMessage(L"Failed to initialize the Depth draw device.");

			hr = m_pDrawRGB->Initialize(GetDlgItem(m_hWnd, IDC_RGB_VIEW), m_pD2DFactory, RGBA_FRAME_WIDTH, RGBA_FRAME_HEIGHT, RGBA_FRAME_WIDTH * OUT_FRAME_CHANNELS * sizeof(UCHAR));
            if (FAILED(hr))
                SetStatusMessage(L"Failed to initialize the RGB draw device.");

			EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK_NEAR_MODE), FALSE);
			EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), FALSE);
			EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_STOP), FALSE);
			enableResolutionButtons(false);
			CheckDlgButton(m_hWnd,IDC_CHECK_NEAR_MODE, BST_CHECKED);
			CheckDlgButton(m_hWnd,IDC_RECORD_RGB, BST_CHECKED);
			CheckDlgButton(m_hWnd,IDC_RECORD_DEPTH, BST_CHECKED);
			updateResolutionButtons();
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
					CheckDlgButton(m_hWnd,IDC_CHECK_NEAR_MODE, BST_CHECKED);
				else
					CheckDlgButton(m_hWnd,IDC_CHECK_NEAR_MODE, BST_UNCHECKED);
            }

            if (ID_MENU_FILE_OPEN == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK_NEAR_MODE), FALSE);
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), FALSE);
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_STOP), FALSE);
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_FILE), FALSE);
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_KINECT), TRUE);
				enableResolutionButtons(false);
				kinect->releaseOutFile();
				SetStatusMessage(L"Disconnecting Kinect...");
				kinect->disconnectDevice();
				std::string fp = getFilePath();
				SetStatusMessage(L"Setting up file stream...");
				if(kinect->streamFromFile((char*)fp.c_str()) && !fp.empty()){
					SetStatusMessage(L"Streaming from file...");
					resetDepthImageRenderer();
					resetRGBImageRenderer();
					updateResolutionButtons();
				}else{
					EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK_NEAR_MODE), TRUE);
					EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), TRUE);
					EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_STOP), FALSE);
					EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_FILE), TRUE);
					SetStatusMessage(L"Could not stream from file...");
				}
            }

			if (IDC_RGB_RESOLUTION_1280_960 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				changeRGBResolution(NUI_IMAGE_RESOLUTION_1280x960);
            }

			if (IDC_RGB_RESOLUTION_640_480== LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				changeRGBResolution(NUI_IMAGE_RESOLUTION_640x480);
            }

			if (IDC_DEPTH_RESOLUTION_640_480 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				changeDepthResolution(NUI_IMAGE_RESOLUTION_640x480);
            }

			if (IDC_DEPTH_RESOLUTION_320_240 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				changeDepthResolution(NUI_IMAGE_RESOLUTION_320x240);
            }

			if (IDC_DEPTH_RESOLUTION_80_60 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				changeDepthResolution(NUI_IMAGE_RESOLUTION_80x60);
            }

			if (IDC_RECORD_RGB == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				kinect->setRecordRGB(IsDlgButtonChecked(m_hWnd, IDC_RECORD_RGB) == BST_CHECKED);
            }

			if (IDC_RECORD_DEPTH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				kinect->setRecordDepth(IsDlgButtonChecked(m_hWnd, IDC_RECORD_DEPTH) == BST_CHECKED);
            }

			if (IDC_BUTTON_TILT_UP == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				kinect->tiltUp();
            }

			if (IDC_BUTTON_TILT_DOWN == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				kinect->tiltDown();
            }

            if (ID_MENU_FILE_SETRECORDFILE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				std::string fp = getFilePath(L"Save As...");
				if(kinect->setOutFile((char*)fp.c_str())){
					if(kinect->isConnected())
						EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), TRUE);
					else
						EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), FALSE);
					EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_STOP), FALSE);
					SendDlgItemMessageW(m_hWnd, IDC_RECORD_FILE_PATH_TEXT, WM_SETTEXT, 0, (LPARAM)L"File ready to record...");
				}else{
					EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), FALSE);
					EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_STOP), FALSE);
					SetStatusMessage(L"Could not open stream for writing...");
					SendDlgItemMessageW(m_hWnd, IDC_RECORD_FILE_PATH_TEXT, WM_SETTEXT, 0, (LPARAM)L"No file to record to...");
				}
            }

			if(IDC_BUTTON_KINECT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				SetStatusMessage(L"Connecting to device...");
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_KINECT), FALSE);

				if(FAILED(kinect->streamFromKinect())){
					SetStatusMessage(L"Could not connect to device...");
					EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK_NEAR_MODE), FALSE);
					EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), FALSE);
					EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_STOP), FALSE);
					EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_KINECT), TRUE);
					enableResolutionButtons(false);
				}else{
					SetStatusMessage(L"Streaming from Kinect...");
					EnableWindow(GetDlgItem(m_hWnd, IDC_CHECK_NEAR_MODE), TRUE);
					if(kinect->hasOutFile())
						EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), TRUE);
					else
						EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), FALSE);
					EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_STOP), FALSE);
					enableResolutionButtons(true);
				}
			}

			if(IDC_BUTTON_RECORD_START == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				SetStatusMessage(L"Starting record...");
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), FALSE);
				kinect->record(true);
				SetStatusMessage(L"Recording...");
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_STOP), TRUE);
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_FILE), FALSE);
				enableResolutionButtons(false);
			}

			if(IDC_BUTTON_RECORD_STOP == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
				SetStatusMessage(L"Recording stopped...");
				kinect->record(false);
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_START), TRUE);
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RECORD_STOP), FALSE);
				EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_FILE), TRUE);
				enableResolutionButtons(true);
			}
            break;
    }

    return FALSE;
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
void KinectRecorder::SetStatusMessage(WCHAR * szMessage)
{
    //SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, (LPARAM)szMessage);
	KTM::NotificationInterface::setMessage(szMessage);
}

std::string KinectRecorder::getFilePath(wchar_t* dialogTitle){
	OPENFILENAME ofn;
	wchar_t fileName[MAX_PATH];
	char fileNameC[MAX_PATH] = "";
	ZeroMemory(fileName, MAX_PATH * sizeof(wchar_t));
	ZeroMemory(&ofn, sizeof(ofn));

	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = m_hWnd;
	ofn.lpstrFilter = L"All Files (*.*)\0*.*\0";
	ofn.lpstrFile = (LPTSTR)fileName;
	ofn.nMaxFile = MAX_PATH;
	ofn.Flags = OFN_EXPLORER;
	ofn.lpstrDefExt = L"";
	ofn.lpstrTitle = dialogTitle;

	std::string fileNameStr;

	if (!GetOpenFileName(&ofn))
		return "";

	wcstombs(fileNameC, fileName, MAX_PATH);
	fileNameStr = fileNameC;

	return fileNameStr;
}

std::string KinectRecorder::getFilePath(){
	return getFilePath(L"Open...");
}

void KinectRecorder::changeRGBResolution(NUI_IMAGE_RESOLUTION resolutionCode){
	switch(resolutionCode){
	case NUI_IMAGE_RESOLUTION_1280x960:
		SetStatusMessage(L"Changing to 1280 x 960 RGB Resolution");
		break;
	case NUI_IMAGE_RESOLUTION_640x480:
		SetStatusMessage(L"Changing to 640 x 480 RGB Resolution");
		break;
	default:
		SetStatusMessage(L"Not a valid resolution...");
		break;
	}
	
	kinect->setRGBRecordingResolution(resolutionCode);
	resetRGBImageRenderer();
}

void KinectRecorder::changeDepthResolution(NUI_IMAGE_RESOLUTION resolutionCode){
	switch(resolutionCode){
	case NUI_IMAGE_RESOLUTION_640x480:
		SetStatusMessage(L"Changing to 640 x 480 Depth Resolution");
		break;
	case NUI_IMAGE_RESOLUTION_320x240:
		SetStatusMessage(L"Changing to 320 x 240 Depth Resolution");
		break;
	case NUI_IMAGE_RESOLUTION_80x60:
		SetStatusMessage(L"Changing to 80 x 60 Depth Resolution");
		break;
	default:
		SetStatusMessage(L"Not a valid resolution...");
		break;
	}
	
	kinect->setDepthRecordingResolution(resolutionCode);
	resetDepthImageRenderer();
}

void KinectRecorder::resetDepthImageRenderer(){
	int width;
	int height;

	kinect->getDepthResolution(width, height);

	if(NULL != m_pDrawDepth)
		delete m_pDrawDepth;
	m_pDrawDepth = new ImageRenderer();
	HRESULT hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_DEPTH_VIEW), m_pD2DFactory, width, height, width * sizeof(ULONG));
    if (FAILED(hr))
        SetStatusMessage(L"Failed to initialize the Depth draw device.");
}

void KinectRecorder::resetRGBImageRenderer(){
	int width;
	int height;

	kinect->getRGBResolution(width, height);

	if(NULL != m_pDrawRGB){
		delete m_pDrawRGB;
	}
	m_pDrawRGB = new ImageRenderer();
	HRESULT hr = m_pDrawRGB->Initialize(GetDlgItem(m_hWnd, IDC_RGB_VIEW), m_pD2DFactory, width, height, width * OUT_FRAME_CHANNELS * sizeof(UCHAR));
    if (FAILED(hr))
        SetStatusMessage(L"Failed to initialize the RGB draw device.");
}

void KinectRecorder::enableResolutionButtons(bool enabled){
	int e = enabled ? TRUE : FALSE;
	EnableWindow(GetDlgItem(m_hWnd, IDC_RGB_RESOLUTION_1280_960), e);
	EnableWindow(GetDlgItem(m_hWnd, IDC_RGB_RESOLUTION_640_480), e);
	EnableWindow(GetDlgItem(m_hWnd, IDC_DEPTH_RESOLUTION_640_480), e);
	EnableWindow(GetDlgItem(m_hWnd, IDC_DEPTH_RESOLUTION_320_240), e);
	EnableWindow(GetDlgItem(m_hWnd, IDC_DEPTH_RESOLUTION_80_60), e);
	EnableWindow(GetDlgItem(m_hWnd, IDC_RECORD_RGB), e);
	EnableWindow(GetDlgItem(m_hWnd, IDC_RECORD_DEPTH), e);
	EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_TILT_UP), e);
	EnableWindow(GetDlgItem(m_hWnd, IDC_BUTTON_TILT_DOWN), e);
}

void KinectRecorder::updateResolutionButtons(){
	NUI_IMAGE_RESOLUTION res = kinect->getDepthResolutionCode();

	CheckDlgButton(m_hWnd,IDC_DEPTH_RESOLUTION_640_480, BST_UNCHECKED);
	CheckDlgButton(m_hWnd,IDC_DEPTH_RESOLUTION_320_240, BST_UNCHECKED);
	CheckDlgButton(m_hWnd,IDC_DEPTH_RESOLUTION_80_60, BST_UNCHECKED);
	CheckDlgButton(m_hWnd,IDC_RGB_RESOLUTION_1280_960, BST_UNCHECKED);
	CheckDlgButton(m_hWnd,IDC_RGB_RESOLUTION_640_480, BST_UNCHECKED);


	switch(res){
	case NUI_IMAGE_RESOLUTION_640x480:
		CheckDlgButton(m_hWnd,IDC_DEPTH_RESOLUTION_640_480, BST_CHECKED);
		break;
	case NUI_IMAGE_RESOLUTION_320x240:
		CheckDlgButton(m_hWnd,IDC_DEPTH_RESOLUTION_320_240, BST_CHECKED);
		break;
	case NUI_IMAGE_RESOLUTION_80x60:
		CheckDlgButton(m_hWnd,IDC_DEPTH_RESOLUTION_80_60, BST_CHECKED);
		break;
	}

	res = kinect->getRGBResolutionCode();

	switch(res){
	case NUI_IMAGE_RESOLUTION_1280x960:
		CheckDlgButton(m_hWnd,IDC_RGB_RESOLUTION_1280_960, BST_CHECKED);
		break;	
	case NUI_IMAGE_RESOLUTION_640x480:
		CheckDlgButton(m_hWnd,IDC_RGB_RESOLUTION_640_480, BST_CHECKED);
		break;
	}

	CheckDlgButton(m_hWnd,IDC_RECORD_RGB, kinect->getRecordRGB() ? BST_CHECKED : BST_UNCHECKED);
	CheckDlgButton(m_hWnd,IDC_RECORD_DEPTH, kinect->getRecordDepth() ? BST_CHECKED : BST_UNCHECKED);
}