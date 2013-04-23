#ifndef KTM_KINECT_RECORDER_H
#define KTM_KINECT_RECORDER_H

#include <Windows.h>
#include <stdlib.h>
#include "KinectWrapper.hpp"
#include "NuiApi.h"
#include "resource.h"
#include "ImageRenderer.hpp"

//#include <Shlobj.h>

// Direct2D Header Files
#include <d2d1.h>
#pragma comment ( lib, "d2d1.lib" )

class KinectRecorder{
    static const int        cStatusMessageMaxLen = MAX_PATH*2;
	KTM::KinectWrapper*		kinect;

public:
    /// <summary>
    /// Constructor
    /// </summary>
    KinectRecorder();

    /// <summary>
    /// Destructor
    /// </summary>
    ~KinectRecorder();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
    HWND                    m_hWnd;

    // Direct2D
    ImageRenderer*          m_pDrawDepth;
	ImageRenderer*			m_pDrawRGB;
    ID2D1Factory*           m_pD2DFactory;

    /// <summary>
    /// Main processing function
    /// </summary>
    void                    Update();
	bool					m_bNearMode;

    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    void                    SetStatusMessage(WCHAR* szMessage);
	std::string				getFilePath(wchar_t* dialogTitle);
	std::string				getFilePath();
	void					changeDepthResolution(NUI_IMAGE_RESOLUTION resolutionCode);
	void					changeRGBResolution(NUI_IMAGE_RESOLUTION resolutionCode);
	void					enableResolutionButtons(bool enabled);
	void					updateResolutionButtons();
	void					resetDepthImageRenderer();
	void					resetRGBImageRenderer();
};

#endif