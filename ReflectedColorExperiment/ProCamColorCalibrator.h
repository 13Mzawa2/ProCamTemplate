#pragma once
#include <opencv2\opencv.hpp>
#include <FlyCap2CVWrapper.h>
#include "ColorChecker.h"

//----------------------------------------
//	モデル式
//	入力方程式: P_i = (I_Pi / 255)^gamma_Pi
//	反射方程式: C = R(MP + C_0)
//	           C = RK(MP + C_0)  発色の空間変化も加味した場合はこちら
//	出力方程式: I_ci = 255 * max(0, C_i - C_thi)^(1/gamma_Ci)
//	OpenCVの仕様に準じているのでRGBの順番はすべてBGR順

class ProCamColorCalibrator
{
private:
	//	モデル式のパラメータ
	//	[0 - 2] gamma_P
	//	[3 - 5] gamma_C
	//	[6 - 8] C_0
	//	[9 - 11] C_th
	//	[12 - 20] M
	double params[21];
	
	//	反射率推定のパラメータ
	std::vector<cv::Mat> matPCA;
	cv::Mat matJD, matJDinv;
	cv::Mat whiteGain;		//	撮像の空間的な色の違いを表すゲイン

	//	最小化ソルバ用
	//	パラメータの解釈
	static cv::Vec3d gammaPro(const double *x) { return cv::Vec3d(x[0], x[1], x[2]); };
	static cv::Vec3d gammaCam(const double *x) { return cv::Vec3d(x[3], x[4], x[5]); };
	static cv::Vec3d C_0(const double *x) { return cv::Vec3d(x[6], x[7], x[8]); };
	static cv::Vec3d C_th(const double *x) { return cv::Vec3d(x[9], x[10], x[11]); };
	static cv::Mat colorConvertMat(const double *x)
	{
		cv::Mat m = (cv::Mat_<double>(3, 3) <<
			x[12], x[13], x[14],
			x[15], x[16], x[17],
			x[18], x[19], x[20]);
		return m;
	};

	//	モデル式
	static cv::Vec3d linearizePro(const double *x, cv::Vec3d Ip);
	static cv::Vec3d gammaPro(const double *x, cv::Vec3d P);
	static cv::Vec3d reflectProCam(const double *x, cv::Vec3d P, cv::Mat R);
	static cv::Vec3d reflectProCam(const double *x, cv::Vec3d P, cv::Vec3d r);	//	対角性を仮定したもの
	static cv::Vec3d linearizeCam(const double *x, cv::Vec3d Ic);
	static cv::Vec3d gammaCam(const double *x, cv::Vec3d C);

	//	最小化する目的関数
	class CostFunction : public cv::MinProblemSolver::Function 
	{
	public:
		//	計測したパッチ色(BGR, [0.0, 255.0])
		//	patchColors[light: 0-(n>3)][patchNum: 0-23]
		std::vector<std::vector<cv::Vec3d>> patchColors;
		//	投影色
		//	patchColorsの要素順と対応
		std::vector<cv::Vec3d> projectColors;
		//	目的関数の定義
		double calc(const double* x) const;
		//	パラメータベクトルの次元数
		virtual int getDims() const { return 21; }	
	};


public:
	bool model_calibrated = false;		//	モデル式パラメータが確定されたときにtrue
	bool reflectance_calibrated = false;
	bool whitegain_calibrated = false;

	ProCamColorCalibrator();
	~ProCamColorCalibrator();

	void paramInit();

	//	パラメータの解釈
	cv::Vec3d gammaPro() { return gammaPro(params); };
	cv::Vec3d gammaCam() { return gammaCam(params); };
	cv::Vec3d C_0() { return C_0(params); };
	cv::Vec3d C_th() { return C_th(params); };
	cv::Mat colorConvertMat() { return colorConvertMat(params); };
	std::vector<cv::Mat> getMatPCA() { return matPCA; }
	cv::Mat getMatJD() { return matJD; }
	cv::Mat getWhiteImg() { return whiteGain; }

	//	モデル式
	cv::Vec3d linearizePro(cv::Vec3d Ip) { return linearizePro(params, Ip); };
	cv::Vec3d gammaPro(cv::Vec3d P) { return gammaPro(params, P); };
	cv::Vec3d reflectProCam(cv::Vec3d P, cv::Mat R) { return reflectProCam(params, P, R); };
	cv::Vec3d reflectProCam(cv::Vec3d P, cv::Vec3d r) { return reflectProCam(params, P, r); };
	cv::Vec3d linearizeCam(cv::Vec3d Ic) { return linearizeCam(params, Ic); };
	cv::Vec3d gammaCam(cv::Vec3d C) { return gammaCam(params, C); };

	//	ProCamカラーキャリブレーション
	//
	//	OpenCVのウィンドウを生成して行う
	void calibrate(FlyCap2CVWrapper &flycap, cv::Rect projArea, ColorChecker cc);
	void calibrateWhite(FlyCap2CVWrapper &flycap, cv::Rect projArea, ColorChecker cc);
	//	滑降シンプレックス法で最適化
	double fit(std::vector<std::vector<cv::Vec3d>> _patchColors, std::vector<cv::Vec3d> _lightColors);
	//	パラメータの表示
	void showParams();
	cv::Mat drawWhiteGain();
	//	パラメータベクトルのロード
	void load(cv::String path);
	void save(cv::String path);

	//	反射率行列に関する計算（モデル式パラメータを求めた後に利用可能）
	//
	//	各カラーパッチの反射率行列の最小二乗解を得る
	void getReflectanceMatrices(std::vector<std::vector<cv::Vec3d>> _patchColors, std::vector<cv::Vec3d> _lightColors, std::vector<cv::Mat> &_refMats);
	//	同時対角化
	void jointDiagonalization(std::vector<cv::Mat> _reflectances, cv::Mat &_matJD);
	//	主成分行列
	void principalMatrices(std::vector<cv::Mat> _reflectances, std::vector<cv::Mat> &_matPC);

	//	モデル式による反射色推定
	//	@param
	//		camColor: 現在のカメラ撮像の色(BGR, 0-255)
	//		projColor: 現在投影中のプロジェクタ投影像の色(BGR, 0-255)
	//		light: 反射色を推定したいプロジェクタ投影像の色(BGR, 0-255) default = (255,255,255)
	//	@return　光源lightの下での推定反射色
	//
	//	反射率計算
	cv::Mat reflectanceDiag(cv::Vec3d C, cv::Vec3d Cp);
	cv::Mat reflectancePCA(cv::Vec3d C, cv::Vec3d Cp);
	cv::Mat reflectanceJD(cv::Vec3d C, cv::Vec3d Cp);

	//	反射率行列の対角性を仮定したモデル
	cv::Vec3d estimateColorDiag(cv::Vec3d camColor, cv::Vec3d projColor, cv::Vec3d light = cv::Vec3d(255., 255., 255.));
	//	主成分行列を利用したモデル
	cv::Vec3d estimateColorPCA(cv::Vec3d camColor, cv::Vec3d projColor, cv::Vec3d light = cv::Vec3d(255., 255., 255.));
	//	同時対角化を利用したモデル
	cv::Vec3d estimateColorJD(cv::Vec3d camColor, cv::Vec3d projColor, cv::Vec3d light = cv::Vec3d(255., 255., 255.));

	//	↑を画像に対して行う　projImgはremap後
	cv::Mat estimateColorDiag(cv::Mat camImg, cv::Mat projImg);
	cv::Mat estimateColorPCA(cv::Mat camImg, cv::Mat projImg);
	cv::Mat estimateColorJD(cv::Mat camImg, cv::Mat projImg);

	//	Proの配光特性を考慮した場合
	cv::Mat estimateColor2Diag(cv::Mat camImg, cv::Mat projImg);
	cv::Mat estimateColor2PCA(cv::Mat camImg, cv::Mat projImg);
	cv::Mat estimateColor2JD(cv::Mat camImg, cv::Mat projImg);

	//	カメラから見たプロジェクタ像を算出
	cv::Mat calcProjColorFromCam(cv::Mat projImg);

	//	テスト用
	void estimate(FlyCap2CVWrapper &flycap, cv::Rect projArea, cv::Mat projImg, cv::Mat projImgCam, int estMode = 0);
	void testEstimation(FlyCap2CVWrapper &flycap, cv::Rect projArea, cv::Mat projImg, cv::Mat projImgCam);
	cv::Mat distance(cv::Mat img1, cv::Mat img2);
	cv::Mat psudoColordDist(cv::Mat distImg, double vmin = 0, double vmax = 1);
	void testEstimationPoints(FlyCap2CVWrapper &flycap, cv::Rect projArea, std::vector<cv::Point> points, cv::Size blockSize);

	//	キャリブレーション結果の書き出し
	//
	//	csvファイルに書き出し
	//	|投影色BGR|撮影色BGR(1-24)|推定色BGR(diag,1-24)|推定色BGR(pca,1-24)|推定色BGR(jd,1-24)|
	void writeCSV(cv::String path, std::vector<std::vector<cv::Vec3d>> _camColors, std::vector<cv::Vec3d> _projColors);
};

