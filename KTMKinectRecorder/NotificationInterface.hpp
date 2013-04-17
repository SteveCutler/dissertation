#pragma once

#include <Windows.h>

namespace KTM{
	class NotificationInterface{
	private:
		static HWND windowHandle;
		static unsigned int dialogControl;
	public:
		static void setWindowHandle(HWND handle);
		static void setMessageBoxControl(unsigned int control);
		static void setMessage(wchar_t* msg);
	};
};