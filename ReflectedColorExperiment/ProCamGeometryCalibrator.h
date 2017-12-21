#pragma once

#include <opencv2\opencv.hpp>
#include <FlyCap2CVWrapper.h>

class ProCamGeometryCalibrator
{
private:
	cv::Mat mapCPX, mapCPY;		//	cam(x,y) から見た proj(x,y) を格納（size = camSize）
	cv::Mat mapPCX, mapPCY;		//	proj(x,y) から見た cam(x,y) を格納 (size = projSize)

public:
	bool calibrated = false;

	ProCamGeometryCalibrator();
	~ProCamGeometryCalibrator();

	//	キャリブレーション
	void calibrate(FlyCap2CVWrapper flycap, cv::Rect projArea);
	//	逆マップの作製
	void getInverseProCamMap(cv::Size projectorSize, cv::Mat &invMapX, cv::Mat &invMapY);
};

