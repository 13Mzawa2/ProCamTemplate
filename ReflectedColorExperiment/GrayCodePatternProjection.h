#pragma once

#include <opencv2/opencv.hpp>

class GrayCodePatternProjection
{
public:
	std::vector<cv::Mat> patternsW, patternsH, patternsWN, patternsHN;
	std::vector<cv::Mat> captureW, captureH, captureWN, captureHN;
	cv::Mat patternListW, patternListH;			//	s”rows‚ªpatterns‚Ì—v‘f”‚Æˆê’v
	cv::Mat mapX, mapY, mask;				//	Œ‹‰Ê‚ğ‚±‚±‚É•Û‘¶
	cv::Mat camera2ProjectorMap;
	cv::Size projectorSize, cameraSize;

	GrayCodePatternProjection();
	GrayCodePatternProjection(cv::Size projSize, cv::Size camSize);
	~GrayCodePatternProjection();
	int bin2gray(int bin);
	int gray2bin(int gray);
	void makeGrayCodePatternLists(void);
	void makeGrayCodeImages(void);
	void init(cv::Size projectorSize, cv::Size cameraSize);
	void getMask(int thresh = 20);
	void loadCapPatterns(std::vector<cv::Mat> cap);
	void decodePatterns(void);
	int showMaps(void);
};

