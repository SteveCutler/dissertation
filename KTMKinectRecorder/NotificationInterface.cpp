#include "NotificationInterface.hpp"

HWND KTM::NotificationInterface::windowHandle = NULL;
unsigned int KTM::NotificationInterface::dialogControl = NULL;

void KTM::NotificationInterface::setWindowHandle(HWND handle){
	windowHandle = handle;
}

void KTM::NotificationInterface::setMessageBoxControl(unsigned int control){
	dialogControl = control;
}

void KTM::NotificationInterface::setMessage(wchar_t* msg){
	if(NULL != windowHandle && NULL != dialogControl){
		SendDlgItemMessageW(windowHandle, dialogControl, WM_SETTEXT, 0, (LPARAM)msg);
	}
}