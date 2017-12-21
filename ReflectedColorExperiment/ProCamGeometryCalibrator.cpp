#include "ProCamGeometryCalibrator.h"
#include "GrayCodePatternProjection.h"


ProCamGeometryCalibrator::ProCamGeometryCalibrator()
{
}


ProCamGeometryCalibrator::~ProCamGeometryCalibrator()
{
}

void ProCamGeometryCalibrator::calibrate(FlyCap2CVWrapper flycap, cv::Rect projArea)
{
	//	setup
	cv::Mat camImg = flycap.readImage();
	cv::Mat projImg(projArea.size(), CV_8UC3);
	cv::namedWindow("cv_camera");
	cv::namedWindow("cv_patch");
	cv::namedWindow("cv_projector", cv::WINDOW_FREERATIO);
	cv::moveWindow("cv_projector", projArea.x, projArea.y);
	cv::setWindowProperty("cv_projector", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	//	make gray code patterns
	GrayCodePatternProjection gcp(projArea.size(), camImg.size());
	std::vector<cv::Mat> captures;

	//	projecting...
	for (int i = 0; i < gcp.patternListW.rows; i++) {
		cv::imshow("cv_projector", gcp.patternsW[i]);
		cv::waitKey(200);
		cv::Mat img = flycap.readImage();
		captures.push_back(img);
		cv::imshow("cv_camera", img);
		cv::imshow("cv_projector", gcp.patternsWN[i]);
		cv::waitKey(200);
		img = flycap.readImage();
		captures.push_back(img);
		cv::imshow("cv_camera", img);
		cv::waitKey(10);
	}
	for (int i = 0; i < gcp.patternListH.rows; i++) {
		cv::imshow("cv_projector", gcp.patternsH[i]);
		cv::waitKey(200);
		cv::Mat img = flycap.readImage();
		captures.push_back(img);
		cv::imshow("cv_camera", img);
		cv::imshow("cv_projector", gcp.patternsHN[i]);
		cv::waitKey(200);
		img = flycap.readImage();
		captures.push_back(img);
		cv::imshow("cv_camera", img);
		cv::waitKey(10);
	}

	//	calculate projected gray codes
	gcp.loadCapPatterns(captures);
	gcp.decodePatterns();

	//	result
	gcp.showMaps();
	//	save
	mapCPX = gcp.mapX.clone();
	mapCPY = gcp.mapY.clone();
	calibrated = false;

	//	calculate inverse map
	getInverseProCamMap(projArea.size(), mapPCX, mapPCY);

	//	destroy windows
	cv::destroyWindow("cv_camera");
	cv::destroyWindow("cv_projector");
	cv::destroyWindow("cv_patch");

}

void ProCamGeometryCalibrator::getInverseProCamMap(cv::Size projectorSize, cv::Mat & invMapX, cv::Mat & invMapY)
{
	//	プロジェクタからカメラにアクセスするためのマップを作製
	if (!calibrated) {
		std::cout << "先にキャリブレーションが必要です．" << std::endl;
		return;
	}

	//	恒等写像マップを作成
	cv::Mat mapIXpro = cv::Mat(projectorSize, CV_32FC1);
	cv::Mat mapIYpro = cv::Mat(projectorSize, CV_32FC1);
	for (int i = 0; i < projectorSize.height; i++) {
		for (int j = 0; j < projectorSize.width; j++) {
			mapIXpro.at<float>(i, j) = j;
			mapIYpro.at<float>(i, j) = i;
		}
	}
	invMapX = mapIXpro.clone();
	invMapY = mapIYpro.clone();
	float kpx = 1.1;		//	ずれ補正ゲイン
	float kpy = 1.1;
	int thresh = 1;
	int loopNum = 64;
	cv::Mat maskX = cv::Mat::zeros(projectorSize, CV_8UC1);
	cv::Mat maskY = maskX.clone();
	for (int i = 0; i < loopNum; i++) {
		cv::Mat tempX, tempY;
		//	Pro > Cam
		cv::remap(mapIXpro, tempX, mapCPX, mapCPY, cv::INTER_LINEAR);
		cv::remap(mapIYpro, tempY, mapCPX, mapCPY, cv::INTER_LINEAR);
		//	Cam > Pro
		cv::remap(tempX, tempX, invMapX, invMapY, cv::INTER_LINEAR);
		cv::remap(tempY, tempY, invMapX, invMapY, cv::INTER_LINEAR);
		//	Pro上での正しい位置からのずれ量
		cv::Mat dX = tempX - mapIXpro;
		cv::Mat dY = tempY - mapIYpro;
		//	Proマップにずれ量をフィードバック
		invMapX -= kpx * dX; invMapY -= kpy * dY;
		//	Camマップ範囲外のマスクを作成
		cv::Mat absdX = cv::abs(dX);
		absdX.convertTo(absdX, CV_8UC1);
		cv::threshold(absdX, maskX, thresh, 1, cv::THRESH_BINARY_INV);
		cv::Mat absdY = cv::abs(dY);
		absdY.convertTo(absdY, CV_8UC1);
		cv::threshold(absdY, maskY, thresh, 1, cv::THRESH_BINARY_INV);
	}
	//	必要な部分だけマスク(範囲外は0)
	maskX.convertTo(maskX, CV_32FC1);
	maskY.convertTo(maskY, CV_32FC1);
	invMapX = maskX.mul(invMapX);
	invMapY = maskY.mul(invMapY);
}
