#include "ColorChecker.h"


//	クラス内静的変数
const cv::Vec3d ColorChecker::white_Lab = cv::Vec3d(96.0, -0.06, 0.06);
const std::array<cv::Vec3d, 6> ColorChecker::gray_XYZ = {
	cv::Vec3d(88.24,90.01,106.32),
	cv::Vec3d(57.94,59.1,69.8),
	cv::Vec3d(35.49,36.2,42.76),
	cv::Vec3d(19.38,19.77,23.35),
	cv::Vec3d(8.83,9.0,10.63),
	cv::Vec3d(3.06,3.13,3.69)
};

ColorChecker::ColorChecker()
{
}


ColorChecker::~ColorChecker()
{
}

cv::Mat ColorChecker::getCCImage(cv::Mat camImg)
{
	//	cv::getPerspectiveTransform()はdouble型を受け付けない
	std::vector<cv::Point2f> corners_fit = {
		cv::Point2f(0,0),
		cv::Point2f(0, CCImgSize.height),
		cv::Point2f(CCImgSize.width, CCImgSize.height),
		cv::Point2f(CCImgSize.width, 0)
	};
	cv::Mat perspectiveMat = cv::getPerspectiveTransform(corners, corners_fit);
	cv::Mat dst;
	cv::warpPerspective(camImg, dst, perspectiveMat, CCImgSize, cv::INTER_LINEAR);
	dst.copyTo(CCImg);

	return dst;
}

void ColorChecker::getPatches(cv::Mat ccimg)
{
	CCPatches.clear();
	for (auto i = 0; i < 4; i++) {
		for (auto j = 0; j < 6; j++) {
			cv::Point p = CC1_point + cv::Point(j*shiftSize.x, i*shiftSize.y);
			cv::Rect cc_roi(p, patchSize);
			CCPatches.push_back(ccimg(cc_roi).clone());
		}
	}
}

void ColorChecker::measurePatches(std::vector<cv::Mat> patchImgs, std::vector<cv::Vec3d>& patchColors)
{
	patchColors.clear();
	for (auto patch: patchImgs) {
		cv::Mat p, mean;
		patch.convertTo(p, CV_64FC3);
		cv::reduce(p, mean, 0, cv::REDUCE_AVG);
		cv::reduce(mean, mean, 1, cv::REDUCE_AVG);
		patchColors.push_back(mean.at<cv::Vec3d>(0));
	}
}

cv::Mat ColorChecker::drawPatches(std::vector<cv::Mat> patchImgs)
{
	cv::Mat patches(patchSize.height * 4, patchSize.width * 6, CV_8UC3);
	std::vector<cv::Vec3d> patchcolors;
	measurePatches(patchImgs, patchcolors);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 6; j++) {
			cv::Point p(j*patchSize.width, i*patchSize.height);
			cv::Rect roi(p, patchSize);
			patchImgs[i * 6 + j].copyTo(patches(roi));
			auto c = patchcolors[i * 6 + j];
			for (int k = 0; k < 3; k++) {
				cv::String s = cv::format("%.1lf", c[k]);
				cv::putText(patches, s, p + cv::Point(0, 10*(k+1)),
					cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
			}
		}
	}

	return patches;
}

void ColorChecker::getGrayPatchReflectance(std::vector<cv::Vec3d>& reflectances)
{
	//	ColorChecker仕様測定時の光源を求める
	//	L*a*b*色空間では(100,0,0)が光源
	cv::Vec3d illuminantC_XYZ = {
		pow(white_Lab[1] / 500 + (white_Lab[0] + 16) / 116, -3) * gray_XYZ[0][0],
		pow(116 / (white_Lab[0] + 16), 3)*gray_XYZ[0][1],
		pow((white_Lab[0] + 16) / 116 - white_Lab[2] / 200, -3) * gray_XYZ[0][2]
	};

	//	各グレーパッチのXYZ反射率を求め，平均した値を反射率とする
	reflectances.clear();
	for (auto g: gray_XYZ) {
		//	平均反射率
		double r_mean =(
			g[0] / illuminantC_XYZ[0] + 
			g[1] / illuminantC_XYZ[1] + 
			g[2] / illuminantC_XYZ[2]) / 3.0;
		cv::Vec3d r = cv::Vec3d(r_mean, r_mean, r_mean);
		reflectances.push_back(r);
	}
}
