#pragma once

#include <opencv2\opencv.hpp>
#include <FlyCap2CVWrapper.h>

class ProCamGeometryCalibrator
{
private:
	cv::Mat mapCPX, mapCPY;		//	cam(x,y) ���猩�� proj(x,y) ���i�[�isize = camSize�j
	cv::Mat mapPCX, mapPCY;		//	proj(x,y) ���猩�� cam(x,y) ���i�[ (size = projSize)

public:
	bool calibrated = false;

	ProCamGeometryCalibrator();
	~ProCamGeometryCalibrator();

	//	�L�����u���[�V����
	void calibrate(FlyCap2CVWrapper flycap, cv::Rect projArea);
	//	�t�}�b�v�̍쐻
	void getInverseProCamMap(cv::Size projectorSize, cv::Mat &invMapX, cv::Mat &invMapY);
};
