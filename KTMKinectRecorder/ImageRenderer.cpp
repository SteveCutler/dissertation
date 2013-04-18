//------------------------------------------------------------------------------
// <copyright file="ImageRenderer.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

//#include "stdafx.h"
#include "ImageRenderer.hpp"

/// <summary>
/// Constructor
/// </summary>
ImageRenderer::ImageRenderer() : 
    m_hWnd(0),
    m_sourceWidth(0),
    m_sourceHeight(0),
    m_sourceStride(0),
    m_pD2DFactory(NULL), 
    m_pRenderTarget(NULL),
    m_pBitmap(0)
{
}

/// <summary>
/// Destructor
/// </summary>
ImageRenderer::~ImageRenderer()
{
    DiscardResources();
	if(m_pD2DFactory){
		m_pD2DFactory->Release();
		m_pD2DFactory = NULL;
	}
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT ImageRenderer::EnsureResources()
{
    HRESULT hr = S_OK;

    if (NULL == m_pRenderTarget)
    {
        D2D1_SIZE_U size = D2D1::SizeU(m_sourceWidth, m_sourceHeight);

        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        // Create a hWnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(m_hWnd, size),
            &m_pRenderTarget
            );

        if ( FAILED(hr) )
        {
            return hr;
        }

        // Create a bitmap that we can copy image data into and then render to the target
        hr = m_pRenderTarget->CreateBitmap(
            size, 
            D2D1::BitmapProperties( D2D1::PixelFormat( DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE) ),
            &m_pBitmap 
            );

        if ( FAILED(hr) )
        {
			if(m_pRenderTarget){
				m_pRenderTarget->Release();
				m_pRenderTarget = NULL;
			}
            return hr;
        }
    }


    return hr;
}

/// <summary>
/// Dispose of Direct2d resources 
/// </summary>
void ImageRenderer::DiscardResources()
{
	if(m_pRenderTarget){
		m_pRenderTarget->Release();
		m_pRenderTarget = NULL;
	}

	if(m_pBitmap){
		m_pBitmap->Release();
		m_pBitmap = NULL;
	}
}

/// <summary>
/// Set the window to draw to as well as the video format
/// Implied bits per pixel is 32
/// </summary>
/// <param name="hWnd">window to draw to</param>
/// <param name="pD2DFactory">already created D2D factory object</param>
/// <param name="sourceWidth">width (in pixels) of image data to be drawn</param>
/// <param name="sourceHeight">height (in pixels) of image data to be drawn</param>
/// <param name="sourceStride">length (in bytes) of a single scanline</param>
/// <returns>indicates success or failure</returns>
HRESULT ImageRenderer::Initialize(HWND hWnd, ID2D1Factory* pD2DFactory, int sourceWidth, int sourceHeight, int sourceStride)
{
    if (NULL == pD2DFactory)
    {
        return E_INVALIDARG;
    }

    m_hWnd = hWnd;

    // One factory for the entire application so save a pointer here
    m_pD2DFactory = pD2DFactory;

    m_pD2DFactory->AddRef();

    // Get the frame size
    m_sourceWidth  = sourceWidth;
    m_sourceHeight = sourceHeight;
    m_sourceStride = sourceStride;

    return S_OK;
}

/// <summary>
/// Draws a 32 bit per pixel image of previously specified width, height, and stride to the associated hwnd
/// </summary>
/// <param name="pImage">image depth data</param>
/// <param name="cbImage">size of image data in bytes</param>
/// <returns>indicates success or failure</returns>
HRESULT ImageRenderer::DrawDepth(USHORT* pImage, unsigned long cbImage){
    // incorrectly sized image data passed in
//    if ( cbImage < ((m_sourceHeight - 1) * m_sourceStride) + (m_sourceWidth * 4) )
//        return E_INVALIDARG;

	char* rgbaData = new char[m_sourceWidth * m_sourceHeight * 4];
	if(NULL != pImage){
		for(unsigned int i = 0; i < cbImage; i++){
			/* Assign depth data to the red blue and green channels, leaving the alpha channel zero */
			unsigned char d = 0;
			if(pImage[i] != 0)
				d = unsigned char(255 - (((float)pImage[i] / 4000.0f) * 255));
			rgbaData[(i * 4)] = d;
			rgbaData[(i * 4) + 1] = d;
			rgbaData[(i * 4) + 2] = d;
		}
	}else{
		ZeroMemory(rgbaData, m_sourceWidth * m_sourceHeight * 4);
	}

    // create the resources for this draw device
    // they will be recreated if previously lost
    HRESULT hr = EnsureResources();

    if ( FAILED(hr) )
    {
        return hr;
    }
    
    // Copy the image that was passed in into the direct2d bitmap
    hr = m_pBitmap->CopyFromMemory(NULL, rgbaData, m_sourceStride);

	delete[] rgbaData;

    if ( FAILED(hr) )
    {
        return hr;
    }
       
    m_pRenderTarget->BeginDraw();

    // Draw the bitmap stretched to the size of the window
    m_pRenderTarget->DrawBitmap(m_pBitmap);
            
    hr = m_pRenderTarget->EndDraw();

    // Device lost, need to recreate the render target
    // We'll dispose it now and retry drawing
    if (hr == D2DERR_RECREATE_TARGET)
    {
        hr = S_OK;
        DiscardResources();
    }

    return hr;
}


HRESULT ImageRenderer::DrawRGB(char* pImage, unsigned long cbImage){
	char* rgbaData = new char[m_sourceWidth * m_sourceHeight * 4];
	if(NULL != pImage){
		for(unsigned int i = 0; i < cbImage;i++){
			rgbaData[i] = pImage[i];
		}
	}else{
		ZeroMemory(rgbaData, m_sourceWidth * m_sourceHeight * 4);
	}

    // create the resources for this draw device
    // they will be recreated if previously lost
    HRESULT hr = EnsureResources();

    if ( FAILED(hr) )
    {
        return hr;
    }
    
    // Copy the image that was passed in into the direct2d bitmap
    hr = m_pBitmap->CopyFromMemory(NULL, rgbaData, m_sourceStride);

	delete[] rgbaData;

    if ( FAILED(hr) )
    {
        return hr;
    }
       
    m_pRenderTarget->BeginDraw();

    // Draw the bitmap stretched to the size of the window
    m_pRenderTarget->DrawBitmap(m_pBitmap);
            
    hr = m_pRenderTarget->EndDraw();

    // Device lost, need to recreate the render target
    // We'll dispose it now and retry drawing
    if (hr == D2DERR_RECREATE_TARGET)
    {
        hr = S_OK;
        DiscardResources();
    }

    return hr;
}