#pragma once
/*******************************
FlyCapture2 with OpenCV Wrapper

*******************************/


#ifdef _DEBUG
#define FC2_EXT "d.lib"
#else
#define FC2_EXT ".lib"
#endif
#pragma comment(lib, "FlyCapture2" FC2_EXT)
#pragma comment(lib, "FlyCapture2GUI" FC2_EXT)

#ifdef _WIN32
#define WIN32
#endif
#ifdef _WIN64
#define WIN64
#endif

#include <opencv2\opencv.hpp>
#include <FlyCapture2.h>
#include <FlyCapture2GUI.h>


class FlyCap2CVWrapper
{
protected:
	FlyCapture2::Camera flycam;
	FlyCapture2::Error flycamError;
	cv::Mat cvImg;

public:
	FlyCap2CVWrapper();
	~FlyCap2CVWrapper();
	cv::Mat readImage();
	//	Settings
	void autoExposure(bool flag, float absValue);
	void autoWhiteBalance(bool flag, int red, int blue);
	void autoSaturation(bool flag, float absValue);
	void autoShutter(bool flag, float ms);
	void autoGain(bool flag, float dB);
	void autoFrameRate(bool flag, float fps);
	bool checkError();
	void open();
	void close();
};

