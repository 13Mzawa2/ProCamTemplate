#pragma once
#include <opencv2\opencv.hpp>
#include <array>

class ColorChecker
{
private:
	//	ColorCheckerの仕様
	static const std::array<cv::Vec3d, 6> gray_XYZ;
	static const cv::Vec3d white_Lab;

public:
	//	ColorChecker Classic の番号
	enum {
		PATCH_CC1_DARK_SKIN = 0,
		PATCH_CC2_LIGHT_SKIN = 1,
		PATCH_CC3_BLUE_SKY = 2,
		PATCH_CC4_FOLIAGE = 3,
		PATCH_CC5_BLUE_FLOWER = 4,
		PATCH_CC6_BLUISH_GREEN = 5,
		PATCH_CC7_ORANGE = 6,
		PATCH_CC8_PURPLISH_BLUE = 7,
		PATCH_CC9_MODERATE_RED = 8,
		PATCH_CC10_PURPLE = 9,
		PATCH_CC11_YELLOW_GREEN = 10,
		PATCH_CC12_ORANGE_YELLOW = 11,
		PATCH_CC13_BLUE = 12,
		PATCH_CC14_GREEN = 13,
		PATCH_CC15_RED = 14,
		PATCH_CC16_YELLOW = 15,
		PATCH_CC17_MAGENTA = 16,
		PATCH_CC18_CYAN = 17,
		PATCH_CC19_WHITE = 18,
		PATCH_CC20_GRAY_N80 = 19,
		PATCH_CC21_GRAY_N65 = 20,
		PATCH_CC22_GRAY_N50 = 21,
		PATCH_CC23_GRAY_N35 = 22,
		PATCH_CC24_BLACK = 23,
		PATCH_NUM = 24
	};
	//	ColorChecker画像
	cv::Mat CCImg;
	std::vector<cv::Mat> CCPatches;
	cv::Size CCImgSize = cv::Size(550,360);	//	275mm:180mm
	cv::Size patchSize = cv::Size(60, 60);	//	CCImgのパッチの大きさ
	cv::Point shiftSize = cv::Point(92, 90);	//	CCImgの各パッチのシフト量
	cv::Point CC1_point = cv::Point(12, 18);	//	CC1パッチの位置
	std::vector<cv::Point2f> corners;	//	左上から反時計回りに指定

	ColorChecker();
	~ColorChecker();
	
	//	ColorCheckerの情報取得
	//	カメラ入力からクロッピング
	cv::Mat getCCImage(cv::Mat camImg);
	//	クロッピング後の画像からパッチ群を取得
	void getPatches(cv::Mat ccimg);
	void measurePatches(std::vector<cv::Mat> patchImgs, std::vector<cv::Vec3d> &patchColors);
	cv::Mat drawPatches(std::vector<cv::Mat> patchImgs);
	
	//	キャリブレーション情報の取得
	static void getGrayPatchReflectance(std::vector<cv::Vec3d> &reflectances);
};

