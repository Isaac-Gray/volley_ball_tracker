#pragma once
#include "SapClassBasic.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv/cv.h>
#include <tchar.h>
#include <windows.h>
#include <string>

class grabFrame
{
private:
	SapAcqDevice *AcqDevice;
	SapBuffer *pBuffer;
	SapBuffer *outBuffer;
	SapColorConversion *m_Conv;
	SapView *View;
	SapAcqDeviceToBuf *Xfer;
	
	// Construction
	public:
		static void XferCallback(SapXferCallbackInfo *pInfo);
		BOOL CreateObjects();
		BOOL DestroyObjects();
		BOOL DefineObjects(char* cameraName);
		void onGrab();
		void stopPlease();
};