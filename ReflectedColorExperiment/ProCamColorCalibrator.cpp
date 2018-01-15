#include "ProCamColorCalibrator.h"
#include <future>		//	非同期マルチスレッド処理
#include <iostream>		//	csvファイル入出力
#include <fstream>

ProCamColorCalibrator::ProCamColorCalibrator()
{
	paramInit();
}


ProCamColorCalibrator::~ProCamColorCalibrator()
{
}

void ProCamColorCalibrator::paramInit()
{
	for (int i = 0; i < 21; i++) {
		params[i] = 0;
	}
	model_calibrated = false;
	reflectance_calibrated = false;
	matPCA.clear();
}

void ProCamColorCalibrator::calibrate(FlyCap2CVWrapper &flycap, cv::Rect projArea, ColorChecker cc)
{
	////	OpenCVの仕様確認
	//cv::Mat m0 = (cv::Mat_<double>(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
	//std::cout << m0 << std::endl;
	//auto m0_diag = cv::Mat::diag(m0.diag(0));
	//std::cout << m0_diag << std::endl;
	//auto m1 = m0 - m0_diag;
	//std::cout << m1 << std::endl;

	//	setup
	cv::Mat camImg;
	cv::Mat projImg(projArea.size(), CV_8UC3);
	cv::namedWindow("cv_camera");
	cv::namedWindow("cv_patch");
	cv::namedWindow("cv_projector", cv::WINDOW_FREERATIO);
	cv::moveWindow("cv_projector", projArea.x, projArea.y);
	cv::setWindowProperty("cv_projector", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	//	5x5x5段階で撮影
	std::vector<double> projInput = { 0., 63., 127., 191., 255. };
	std::vector<cv::Vec3d> projColors;
	std::vector<std::vector<cv::Vec3d>> patchColors;
	std::vector<std::vector<cv::Mat>> patchImgs;
	for (auto b : projInput) {
		for (auto g : projInput) {
			for (auto r : projInput) {
				//	投影
				//	描画命令から実際に描画されるまでは数フレーム遅れるため，
				//	正しい描画のために複数回描画命令を出す
				projImg = cv::Scalar(b, g, r);
				for(int i=0;i<3;i++){
					cv::imshow("cv_projector", projImg);
					cv::waitKey(30);
				}
				//	撮影
				camImg = flycap.readImage();
				//	パッチ色を取得
				std::vector<cv::Vec3d> patch;
				cc.getCCImage(camImg);
				cc.getPatches(cc.CCImg);
				cc.measurePatches(cc.CCPatches, patch);
				//	描画
				cv::imshow("cv_camera", camImg);
				cv::imshow("cv_patch", cc.drawPatches(cc.CCPatches));
				cv::waitKey(15);
				//	記録
				projColors.push_back(cv::Vec3d(b, g, r));
				patchColors.push_back(patch);
				patchImgs.push_back(cc.CCPatches);
			}
		}
	}
	//	パラメータ計算開始
	std::thread t1([&] { fit(patchColors, projColors); });

	
	// 撮影パッチの再確認(for debugging)
	for (auto i = 0; i < patchImgs.size(); i++) {
		cv::imshow("cv_patch", cc.drawPatches(patchImgs[i]));
		cv::setWindowTitle("cv_patch",
			cv::format("pro = (%.0lf, %.0lf, %.0lf)", 
				projColors[i][0], projColors[i][1], projColors[i][2]));
		auto c = cv::waitKey(300);
		if (c == ' ') cv::waitKey();	//	一時停止
		if (c == 27) break;		//	中断
	}

	//	生成したウィンドウを閉じる
	cv::destroyWindow("cv_camera");
	cv::destroyWindow("cv_projector");
	cv::destroyWindow("cv_patch");

	t1.join();	//	スレッドt1の処理が終わるまで待機

	//	反射率行列の計算（パラメータ計算後に行なわれる）
	std::vector<cv::Mat> reflectances;
	cv::Mat matJD;
	std::vector<cv::Mat> matPC;
	getReflectanceMatrices(patchColors, projColors, reflectances);
	jointDiagonalization(reflectances, matJD);
	principalMatrices(reflectances, matPC);
	
	reflectance_calibrated = true;

	//	キャリブレーションのためのデータと結果の保存
	writeCSV("./data/calib.csv", patchColors, projColors);
}

void ProCamColorCalibrator::calibrateWhite(FlyCap2CVWrapper & flycap, cv::Rect projArea, ColorChecker cc)
{
	//	setup
	cv::Mat camImg;
	cv::Mat projImg(projArea.size(), CV_8UC3, cv::Scalar::all(255));
	cv::namedWindow("cv_camera");
	cv::namedWindow("cv_projector", cv::WINDOW_FREERATIO);
	cv::moveWindow("cv_projector", projArea.x, projArea.y);
	cv::setWindowProperty("cv_projector", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	//	白色光を投影&撮影
	cv::Mat imgW;
	cv::imshow("cv_projector", projImg);
	cv::waitKey(800);
	imgW = flycap.readImage();

	//	ColorChecker No.19-24の領域で反射率を求める
	//	3色を投影＆撮影
	std::vector<cv::Mat> imgBGR;
	cv::imshow("cv_projector", cv::Mat(projArea.size(), CV_8UC3, cv::Scalar(255, 0, 0)));
	cv::waitKey(200);
	imgBGR.push_back(flycap.readImage());
	cv::imshow("cv_projector", cv::Mat(projArea.size(), CV_8UC3, cv::Scalar(0, 255, 0)));
	cv::waitKey(200);
	imgBGR.push_back(flycap.readImage());
	cv::imshow("cv_projector", cv::Mat(projArea.size(), CV_8UC3, cv::Scalar(0, 0, 255)));
	cv::waitKey(200);
	imgBGR.push_back(flycap.readImage());
	//	ColorCheckerの領域を抽出
	std::vector<cv::Vec3d> patchColors;
	for (auto img : imgBGR) {
		std::vector<cv::Vec3d> patch;
		cc.getCCImage(img);
		cc.getPatches(cc.CCImg);
		cc.measurePatches(cc.CCPatches, patch);
		for (int i = cc.PATCH_CC19_WHITE; i < cc.PATCH_NUM; i++) {
			patchColors.push_back(patch[i]);
		}
	}
	//	精確な反射率行列を求める
	cv::Mat lightMat(3, patchColors.size(), CV_64FC1),
		patchMat(3, patchColors.size(), CV_64FC1),
		R, Rt;
	for (int i = 0; i < patchColors.size(); i++) {
		auto C = linearizeCam(patchColors[i]);
		cv::Vec3b Ip = (i < 6) ? cv::Vec3b(255, 0, 0) 
			: (i < 12) ? cv::Vec3b(0, 255, 0) 
			: cv::Vec3b(0, 0, 255);
		auto Cp = reflectProCam(linearizePro(Ip), cv::Vec3d(1,1,1));
		lightMat.at<cv::Vec3d>(i) = Cp;
		patchMat.at<cv::Vec3d>(i) = C;
	}
	//	線形最小二乗法でRを求める
	//	patches = R * lights -> patches_t = lights_t * R_t
	cv::solve(lightMat.t(), patchMat.t(), Rt, cv::DECOMP_SVD);
	R = Rt.t();

	//	カメラ内の反射率を一定として，白色ゲインを求める
	whiteGain = cv::Mat(imgW.size(), CV_64FC3);
	whiteGain.forEach<cv::Vec3d>([&](cv::Vec3d &k, const int *pos)->void {
		auto Ic = (cv::Vec3d)imgW.at<cv::Vec3b>(pos[0], pos[1]);
		auto C = linearizeCam(Ic);
		auto Cw = reflectProCam(cv::Vec3d(1., 1., 1.), cv::Vec3d(1., 1., 1.));
		auto RinvC = (cv::Vec3d)cv::Mat(R.inv()*cv::Mat(C));
		for (int i = 0; i < 3; i++) {
			k[i] = RinvC[i] / Cw[i];
		}
	});

	whitegain_calibrated = true;

	//	白色ゲインの表示
	cv::imshow("white gain", drawWhiteGain());
	cv::waitKey();

	//	生成したウィンドウを閉じる
	cv::destroyWindow("cv_camera");
	cv::destroyWindow("cv_projector");
	cv::destroyWindow("white gain");
}

double ProCamColorCalibrator::fit(std::vector<std::vector<cv::Vec3d>> _patchColors, std::vector<cv::Vec3d> _lightColors)
{
	if (_patchColors.empty() || _lightColors.empty()) return -1;
	if (_patchColors.size() != _lightColors.size()) return -1;
	
	//	パラメータベクトル
	cv::Mat param = (cv::Mat_<double>(1, 21) <<
		2., 2., 2.,			//	プロジェクタガンマ
		1., 1., 1.,			//	カメラガンマ
		0.01, 0.01, 0.01,	//	環境光
		0.01, 0.01, 0.01,	//	カメラ感度
		0.6, 0.0, 0.0,		//	ProCam色変換行列
		0.0, 0.6, 0.0,
		0.0, 0.0, 0.6);
	//	初期移動量
	cv::Mat step = (cv::Mat_<double>(1, 21) <<
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5);

	//	目的関数の設定
	auto ptr_func(new CostFunction());
	ptr_func->patchColors = _patchColors;
	ptr_func->projectColors = _lightColors;
	//	ソルバの設定
	auto solver = cv::DownhillSolver::create();
	solver->setFunction(ptr_func);
	solver->setInitStep(step);
	//	最小化実行
	double res = solver->minimize(param);

	//	パラメータベクトルの保存
	param.forEach<double>([&](double &p, const int pos[2])->void {
		params[pos[1]] = p;
	});

	//	最適化結果の表示
	showParams();
	model_calibrated = true;

	return 0.0;
}

void ProCamColorCalibrator::showParams()
{
	std::cout
		<< "\n"
		<< "******** Color Calibration Result *********\n"
		<< "projector gamma = " << gammaPro() << std::endl
		<< "camera gamma = " << gammaCam() << std::endl
		<< "C_0 = " << C_0() << std::endl
		<< "C_th = " << C_th() << std::endl
		<< "M = \n" << colorConvertMat() << std::endl;
}

cv::Mat ProCamColorCalibrator::drawWhiteGain()
{
	if (!whitegain_calibrated) return cv::Mat();

	cv::Mat m;
	whiteGain.convertTo(m, CV_8UC3, 0.9 * 255);
	return m;
}

void ProCamColorCalibrator::load(cv::String path)
{
	cv::Mat p;
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (fs.isOpened()) {
		fs["param_vec"] >> p;

		//	cv::Mat -> double[]
		p.forEach<double>([&](double &x, const int pos[2])->void {
			params[pos[1]] = x;
		});
		model_calibrated = true;

		matPCA.clear();
		fs["matJD"] >> matJD;
		matJDinv = matJD.inv();

		cv::FileNode fn(fs["matPCA"]);
		cv::Mat m;
		fn["matPCA0"] >> m; matPCA.push_back(m);
		fn["matPCA1"] >> m; matPCA.push_back(m);
		fn["matPCA2"] >> m; matPCA.push_back(m);

		reflectance_calibrated = true;
	}
	else {
		std::cout << "ファイルパスを確認してください: " << path << std::endl;
	}
}

void ProCamColorCalibrator::save(cv::String path)
{
	if (model_calibrated) {
		//	double[] -> cv::Mat
		cv::Mat p(1, 21, CV_64FC1);
		p.forEach<double>([&](double &x, const int pos[2]) -> void {
			x = params[pos[1]];
		});

		cv::FileStorage fs(path, cv::FileStorage::WRITE);
		if (fs.isOpened()) {
			fs << "param_vec" << p;
			if (reflectance_calibrated) {
				fs << "matPCA" << "{"
					<< "matPCA0" << matPCA[0]
					<< "matPCA1" << matPCA[1]
					<< "matPCA2" << matPCA[2] << "}";
				fs << "matJD" << matJD;
			}
			std::cout << "saved at " << path << std::endl;
		}
	}
}

void ProCamColorCalibrator::getReflectanceMatrices(std::vector<std::vector<cv::Vec3d>> _patchColors, std::vector<cv::Vec3d> _lightColors, std::vector<cv::Mat>& _refMats)
{
	if (_patchColors.empty() || _lightColors.empty() || !model_calibrated) return;
	
	_refMats.clear();
	//	パッチ番号毎に計算
	for (int patchNum = 0; patchNum < _patchColors[0].size(); patchNum++) {
		//	cv::Mat型に代入
		cv::Mat lightColors(3, _lightColors.size(), CV_64FC1), 
			patchColors(3, _lightColors.size(), CV_64FC1),
			R, Rt;
		for (int lightNum = 0; lightNum < _lightColors.size(); lightNum++) {
			//	線形化
			auto patchLinear = linearizeCam(_patchColors[lightNum][patchNum]);
			auto lightLinear = linearizePro(_lightColors[lightNum]);
			//	カメラ色空間に変更
			cv::Vec3d Cp = reflectProCam(lightLinear, cv::Vec3d(1, 1, 1));
			for (int j = 0; j < 3; j++) {
				lightColors.at<double>(j, lightNum) = Cp[j];
				patchColors.at<double>(j, lightNum) = patchLinear[j];
			}
		}
		//	線形最小二乗法でRを求める
		//	patches = R * lights -> patches_t = lights_t * R_t
		cv::solve(lightColors.t(), patchColors.t(), Rt, cv::DECOMP_SVD);
		R = Rt.t();
		_refMats.push_back(R);
	}
	std::cout << "R0 = \n" << _refMats[3] << std::endl;
	std::cout << "R1 = \n" << _refMats[4] << std::endl;
	std::cout << "R2 = \n" << _refMats[5] << std::endl;
}

void ProCamColorCalibrator::jointDiagonalization(std::vector<cv::Mat> _reflectances, cv::Mat & _matJD)
{
	if (_reflectances.empty()) return;

	//	同時対角化行列Tの初期値
	//	R0 = u*w*vt = P^-1 D P
	auto R0 = _reflectances[0];
	cv::SVD svd(R0, cv::SVD::FULL_UV);
	auto D0 = svd.w, T0 = svd.vt;

	//	評価関数の勾配 nabla Jを求める関数
	auto nablaJ = [](cv::Mat _T, std::vector<cv::Mat> Ms) {
		cv::Mat M_sum = cv::Mat::zeros(3, 3, CV_64FC1);
		cv::Mat _Tinv = _T.inv();
		for (auto M : Ms) {
			auto M1 = (_Tinv*M*_T).t();
			auto M2 = _Tinv*M*_T - cv::Mat::diag((_Tinv*M*_T).diag(0));
			M_sum += M1*M2 - M2*M1;
		}
		auto dJ = 2.0*_T*M_sum;
		return dJ;
	};

	//	最小化
	auto Ti = T0.clone();
	auto T_new = cv::Mat::eye(3, 3, CV_64FC1);
	for (int i = 0; i < 1000; i++) {
		auto T_old = Ti;
		T_new = T_old - 0.01 / cv::norm(nablaJ(T_old, _reflectances)) * nablaJ(T_old, _reflectances);
		Ti = T_new;
	}
	_matJD = Ti.clone();
	matJD = _matJD.clone();
	matJDinv = matJD.inv();

	//	同時対角化できているか確認
	std::cout << "T = " << matJD << std::endl;
	std::cout << "T^-1 R0 T = \n" << matJD.inv() *_reflectances[3] * matJD << std::endl;
	std::cout << "T^-1 R1 T = \n" << matJD.inv() *_reflectances[4] * matJD << std::endl;
	std::cout << "T^-1 R2 T = \n" << matJD.inv() *_reflectances[5] * matJD << std::endl;
}

void ProCamColorCalibrator::principalMatrices(std::vector<cv::Mat> _reflectances, std::vector<cv::Mat>& _matPC)
{
	//	反射率行列を行ベクトルに変形
	if (_reflectances.size() < 3) return;

	cv::Mat rvecs(_reflectances.size(), 9, CV_64FC1);
	for (int i = 0; i < rvecs.rows; i++) {
		for (int j = 0; j < 9; j++) {
			rvecs.at<double>(i, j) = _reflectances[i].at<double>(j);
		}
	}
	//	SVD
	cv::SVD svd(rvecs, cv::SVD::FULL_UV);
	std::cout << "SVD Result" << std::endl;
	std::cout << "Singular Value = \n" << svd.w << std::endl;
	//std::cout << "components = \n" << svd.vt << std::endl;

	//	上位3主成分を取り出す
	_matPC.clear();
	std::cout << "components = \n";
	for (int i = 0; i < 3; i++) {
		_matPC.push_back(svd.vt.row(i).reshape(1, 3));
		std::cout << _matPC.back() << std::endl;
	}
	matPCA = _matPC;
}

cv::Mat ProCamColorCalibrator::reflectanceDiag(cv::Vec3d C, cv::Vec3d Cp)
{
	cv::Vec3d R_diag;
	for (int i = 0; i < 3; i++) {
		R_diag[i] = C[i] / Cp[i];
		R_diag[i] = MAX(0, MIN(1, R_diag[i]));
	}
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
	for (int i = 0; i < 3; i++) {
		R.at<double>(i, i) = R_diag[i];
	}
	return R;
}

cv::Mat ProCamColorCalibrator::reflectancePCA(cv::Vec3d C, cv::Vec3d Cp)
{
	cv::Mat U(3, 3, CV_64FC1);
	for (int i = 0; i < 3; i++) {
		for (int k = 0; k < 3; k++) {
			cv::Vec3d r_ik = (cv::Vec3d)cv::Mat(matPCA[k].row(i));
			double u_ik = r_ik[0] * Cp[0] + r_ik[1] * Cp[1] + r_ik[2] * Cp[2];
			U.at<double>(i, k) = u_ik;
		}
	}
	cv::Mat alpha_dst;
	cv::solve(U, C, alpha_dst);
	auto alpha = (cv::Vec3d)alpha_dst;
	cv::Mat R = alpha[0] * matPCA[0] + alpha[1] * matPCA[1] + alpha[2] * matPCA[2];

	return R;
}

cv::Mat ProCamColorCalibrator::reflectanceJD(cv::Vec3d C, cv::Vec3d Cp)
{
	cv::Vec3d C_d = cv::Vec3d(cv::Mat(matJDinv * cv::Mat(C)));
	cv::Vec3d Cp_d = cv::Vec3d(cv::Mat(matJDinv * cv::Mat(Cp)));

	cv::Vec3d D(C_d[0] / Cp_d[0], C_d[1] / Cp_d[1], C_d[2] / Cp_d[2]);
	D = cv::Vec3d(MAX(0, MIN(1, D[0])), MAX(0, MIN(1, D[1])), MAX(0, MIN(1, D[2])));
	cv::Mat R = matJD * cv::Mat::diag(cv::Mat(D)) * matJDinv;

	return R;
}

cv::Vec3d ProCamColorCalibrator::estimateColorDiag(cv::Vec3d camColor, cv::Vec3d projColor, cv::Vec3d light)
{
	if (!model_calibrated) return cv::Vec3d(0, 0, 0);

	auto C = linearizeCam(camColor);
	auto Cp = reflectProCam(linearizePro(projColor), cv::Vec3d(1.0, 1.0, 1.0));
	cv::Vec3d R_diag;
	for (int i = 0; i < 3; i++) {
		R_diag[i] = C[i] / Cp[i];
		R_diag[i] = MAX(0, MIN(1, R_diag[i]));
	}
	auto C_est = reflectProCam(linearizePro(light), R_diag);
	auto Ic = gammaCam(C_est);

	return Ic;
}

cv::Vec3d ProCamColorCalibrator::estimateColorPCA(cv::Vec3d camColor, cv::Vec3d projColor, cv::Vec3d light)
{
	if (!reflectance_calibrated || !model_calibrated) return cv::Vec3d(0, 0, 0);

	auto C = linearizeCam(camColor);
	auto Cp = reflectProCam(linearizePro(projColor), cv::Vec3d(1.0, 1.0, 1.0));

	cv::Mat R = reflectancePCA(C, Cp);

	auto C_est = reflectProCam(linearizePro(light), R);
	auto Ic = gammaCam(C_est);

	return Ic;
}

cv::Vec3d ProCamColorCalibrator::estimateColorJD(cv::Vec3d camColor, cv::Vec3d projColor, cv::Vec3d light)
{
	if (!reflectance_calibrated || !model_calibrated) return cv::Vec3d(0, 0, 0);

	auto C = linearizeCam(camColor);
	auto Cp = reflectProCam(linearizePro(projColor), cv::Vec3d(1.0, 1.0, 1.0));

	cv::Mat R = reflectanceJD(C, Cp);

	auto C_est = reflectProCam(linearizePro(light), R);
	auto Ic = gammaCam(C_est);

	return Ic;
}

cv::Mat ProCamColorCalibrator::estimateColorDiag(cv::Mat camImg, cv::Mat projImg)
{
	cv::Mat removed = camImg.clone();
	removed.forEach<cv::Vec3b>([&](cv::Vec3b &c, const int pos[]) -> void {
		cv::Vec3d cam = (cv::Vec3d)camImg.at<cv::Vec3b>(pos[0], pos[1]);
		cv::Vec3d proj = (cv::Vec3d)projImg.at<cv::Vec3b>(pos[0], pos[1]);
		auto rem = estimateColorDiag(cam, proj);
		c = cv::Vec3b(rem);
	});
	return removed;
}

cv::Mat ProCamColorCalibrator::estimateColorPCA(cv::Mat camImg, cv::Mat projImg)
{
	cv::Mat removed = camImg.clone();
	removed.forEach<cv::Vec3b>([&](cv::Vec3b &c, const int pos[]) -> void {
		cv::Vec3d cam = (cv::Vec3d)camImg.at<cv::Vec3b>(pos[0], pos[1]);
		cv::Vec3d proj = (cv::Vec3d)projImg.at<cv::Vec3b>(pos[0], pos[1]);
		auto rem = estimateColorPCA(cam, proj);
		c = cv::Vec3b(rem);
	});
	return removed;
}

cv::Mat ProCamColorCalibrator::estimateColorJD(cv::Mat camImg, cv::Mat projImg)
{
	cv::Mat removed = camImg.clone();
	removed.forEach<cv::Vec3b>([&](cv::Vec3b &c, const int pos[]) -> void {
		cv::Vec3d cam = (cv::Vec3d)camImg.at<cv::Vec3b>(pos[0], pos[1]);
		cv::Vec3d proj = (cv::Vec3d)projImg.at<cv::Vec3b>(pos[0], pos[1]);
		auto rem = estimateColorJD(cam, proj);
		c = cv::Vec3b(rem);
	});
	return removed;
}

cv::Mat ProCamColorCalibrator::estimateColor2Diag(cv::Mat camImg, cv::Mat projImg)
{
	if (!whitegain_calibrated) return estimateColorDiag(camImg, projImg);

	cv::Mat removed = camImg.clone();
	removed.forEach<cv::Vec3b>([&](cv::Vec3b &c, const int pos[]) -> void {
		cv::Vec3d cam = (cv::Vec3d)camImg.at<cv::Vec3b>(pos[0], pos[1]);
		cv::Vec3d proj = (cv::Vec3d)projImg.at<cv::Vec3b>(pos[0], pos[1]);
		cv::Vec3d wgain = (cv::Vec3d)whiteGain.at<cv::Vec3d>(pos[0], pos[1]);

		auto C = linearizeCam(cam);
		auto Cp = reflectProCam(linearizePro(proj), cv::Vec3d(1.0, 1.0, 1.0));
		//	配光特性を考慮
		Cp = cv::Vec3d(wgain[0] * Cp[0], wgain[1] * Cp[1], wgain[2] * Cp[2]);
		cv::Vec3d R_diag;
		for (int i = 0; i < 3; i++) {
			R_diag[i] = C[i] / Cp[i];
			R_diag[i] = MAX(0, MIN(1, R_diag[i]));
		}
		auto C_est = reflectProCam(cv::Vec3d(1,1,1), R_diag);
		auto Ic = gammaCam(C_est);

		c = cv::Vec3b(Ic);
	});

	return removed;
}

cv::Mat ProCamColorCalibrator::estimateColor2PCA(cv::Mat camImg, cv::Mat projImg)
{
	if (!whitegain_calibrated) return estimateColorPCA(camImg, projImg);

	cv::Mat removed = camImg.clone();
	removed.forEach<cv::Vec3b>([&](cv::Vec3b &c, const int pos[]) -> void {
		cv::Vec3d cam = (cv::Vec3d)camImg.at<cv::Vec3b>(pos[0], pos[1]);
		cv::Vec3d proj = (cv::Vec3d)projImg.at<cv::Vec3b>(pos[0], pos[1]);
		cv::Vec3d wgain = (cv::Vec3d)whiteGain.at<cv::Vec3d>(pos[0], pos[1]);

		auto C = linearizeCam(cam);
		auto Cp = reflectProCam(linearizePro(proj), cv::Vec3d(1.0, 1.0, 1.0));
		//	配光特性を考慮
		Cp = cv::Vec3d(wgain[0] * Cp[0], wgain[1] * Cp[1], wgain[2] * Cp[2]);

		cv::Mat R = reflectancePCA(C, Cp);
		auto C_est = reflectProCam(cv::Vec3d(1, 1, 1), R);
		auto Ic = gammaCam(C_est);

		c = cv::Vec3b(Ic);
	});

	return removed;
}

cv::Mat ProCamColorCalibrator::estimateColor2JD(cv::Mat camImg, cv::Mat projImg)
{
	if (!whitegain_calibrated) return estimateColorJD(camImg, projImg);

	cv::Mat removed = camImg.clone();
	removed.forEach<cv::Vec3b>([&](cv::Vec3b &c, const int pos[]) -> void {
		cv::Vec3d cam = (cv::Vec3d)camImg.at<cv::Vec3b>(pos[0], pos[1]);
		cv::Vec3d proj = (cv::Vec3d)projImg.at<cv::Vec3b>(pos[0], pos[1]);
		cv::Vec3d wgain = (cv::Vec3d)whiteGain.at<cv::Vec3d>(pos[0], pos[1]);

		auto C = linearizeCam(cam);
		auto Cp = reflectProCam(linearizePro(proj), cv::Vec3d(1.0, 1.0, 1.0));
		//	配光特性を考慮
		Cp = cv::Vec3d(wgain[0] * Cp[0], wgain[1] * Cp[1], wgain[2] * Cp[2]);

		cv::Mat R = reflectanceJD(C, Cp);
		auto C_est = reflectProCam(cv::Vec3d(1, 1, 1), R);
		auto Ic = gammaCam(C_est);

		c = cv::Vec3b(Ic);
	});

	return removed;
}

void ProCamColorCalibrator::estimate(FlyCap2CVWrapper & flycap, cv::Rect projArea, cv::Mat projImg, cv::Mat projImgCam, int estMode)
{
	//	setup
	cv::Mat camImg;
	cv::namedWindow("cv_camera");
	cv::namedWindow("cv_projector", cv::WINDOW_FREERATIO);
	cv::moveWindow("cv_projector", projArea.x, projArea.y);
	cv::setWindowProperty("cv_projector", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	//	loop
	while (1) {
		camImg = flycap.readImage();
		cv::Mat remImg;
		switch (estMode) {
		case 1:
			remImg = estimateColorPCA(camImg, projImgCam);
			break;
		case 2:
			remImg = estimateColorJD(camImg, projImgCam);
			break;
		default:
			remImg = estimateColorDiag(camImg, projImgCam);
			break;
		}
		cv::imshow("cv_camera", remImg);
		cv::imshow("cv_projector", projImg);
		auto c = cv::waitKey(10);
		if (c == ' ') break;
		if (c == 's') {
			cv::imwrite("./data/cam.png", camImg);
			cv::imwrite("./data/rem.png", remImg);
		}
	}

	//	生成したウィンドウを閉じる
	cv::destroyWindow("cv_camera");
	cv::destroyWindow("cv_projector");
}

void ProCamColorCalibrator::testEstimation(FlyCap2CVWrapper & flycap, cv::Rect projArea, cv::Mat projImg, cv::Mat projImgCam)
{
	//	setup
	cv::Mat camImg;
	cv::namedWindow("cv_camera");
	cv::namedWindow("cv_projector", cv::WINDOW_FREERATIO);
	cv::moveWindow("cv_projector", projArea.x, projArea.y);
	cv::setWindowProperty("cv_projector", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	cv::Mat whitelight(projArea.size(), CV_8UC3, cv::Scalar::all(255));

	while (1) {
		camImg = flycap.readImage();
		cv::imshow("cv_projector", projImg);
		cv::imshow("cv_camera", camImg);
		auto c = cv::waitKey(10);
		if (c == 27) break;
		if (c == 'c') {
			//	白色光下画像の撮影
			cv::imshow("cv_projector", whitelight);
			cv::waitKey(200);
			cv::Mat whiteImg = flycap.readImage();
			cv::imshow("cv_camera", whiteImg);
			cv::waitKey(10);
			//	3個のモデル式で推定
			auto imgDiag = estimateColor2Diag(camImg, projImgCam);
			auto imgPCA = estimateColor2PCA(camImg, projImgCam);
			auto imgJD = estimateColor2JD(camImg, projImgCam);
			//	白色光下画像との誤差算出
			cv::Mat diffDiag, diffPCA, diffJD;
			diffDiag = psudoColordDist(distance(whiteImg, imgDiag), 0, 50);
			diffPCA = psudoColordDist(distance(whiteImg, imgPCA), 0, 50);
			diffJD = psudoColordDist(distance(whiteImg, imgJD), 0, 50);

			//	推定結果の表示
			cv::imshow("diag", imgDiag);
			cv::imshow("PCA", imgPCA);
			cv::imshow("JD", imgJD);

			cv::imshow("diff diag", diffDiag);
			cv::imshow("diff PCA", diffPCA);
			cv::imshow("diff JD", diffJD);

			cv::waitKey();

			cv::destroyWindow("diag");
			cv::destroyWindow("PCA");
			cv::destroyWindow("JD");
			cv::destroyWindow("diff diag");
			cv::destroyWindow("diff PCA");
			cv::destroyWindow("diff JD");

			cv::imwrite("./data/est_diag.png", imgDiag);
			cv::imwrite("./data/est_pca.png", imgPCA);
			cv::imwrite("./data/est_jd.png", imgJD);
			cv::imwrite("./data/diff_diag.png", diffDiag);
			cv::imwrite("./data/diff_pca.png", diffPCA);
			cv::imwrite("./data/diff_jd.png", diffJD);

			cv::imwrite("./data/original.png", camImg);
			cv::imwrite("./data/gt.png", whiteImg);
			cv::imwrite("./data/proj.png", projImgCam);
		}
	}

	//	生成したウィンドウを閉じる
	cv::destroyWindow("cv_camera");
	cv::destroyWindow("cv_projector");
}

//	img1とimg2のユークリッド距離を求める
cv::Mat ProCamColorCalibrator::distance(cv::Mat img1, cv::Mat img2)
{
	cv::Mat img1f, img2f;
	img1.convertTo(img1f, CV_64FC3);
	img2.convertTo(img2f, CV_64FC3);
	cv::Mat diff(img1.size(), CV_64FC1);
	diff.forEach<double>([&](double &c, const int *pos)->void{
		auto c1 = img1f.at<cv::Vec3d>(pos[0], pos[1]);
		auto c2 = img2f.at<cv::Vec3d>(pos[0], pos[1]);
		auto cd = c1 - c2;
		c = sqrt(cd.dot(cd));
	});

	return diff;
}

cv::Mat ProCamColorCalibrator::psudoColordDist(cv::Mat distImg, double vmin, double vmax)
{
	cv::Mat img_hsv(distImg.size(), CV_8UC3, cv::Scalar::all(255));
	img_hsv.forEach<cv::Vec3b>([&](cv::Vec3b &c, const int *pos)->void{
		auto v = distImg.at<double>(pos[0], pos[1]);
		v = MAX(vmin, MIN(vmax, v));		//	clipping
		c[0] = (unsigned char)(120 - 120 * (v - vmin) / (vmax - vmin));		//	hue: 0 - 240degにマッピング
	});
	cv::Mat img;
	cv::cvtColor(img_hsv, img, cv::COLOR_HSV2BGR);
	return img;
}

void ProCamColorCalibrator::writeCSV(cv::String path, std::vector<std::vector<cv::Vec3d>> _camColors, std::vector<cv::Vec3d> _projColors)
{
	std::ofstream ofs(path);
	if (ofs.is_open()) {
		//	タイトル行
		ofs << "lightB,lightG,lightR,";
		for (int i = 0; i < ColorChecker::PATCH_NUM; i++) {
			ofs << "CC" + std::to_string(i) + "_CamB,";
			ofs << "CC" + std::to_string(i) + "_CamG,";
			ofs << "CC" + std::to_string(i) + "_CamR,";
			ofs << "CC" + std::to_string(i) + "_Est_diagB,";
			ofs << "CC" + std::to_string(i) + "_Est_diagG,";
			ofs << "CC" + std::to_string(i) + "_Est_diagR,";
			ofs << "CC" + std::to_string(i) + "_Est_pcaB,";
			ofs << "CC" + std::to_string(i) + "_Est_pcaG,";
			ofs << "CC" + std::to_string(i) + "_Est_pcaR,";
			ofs << "CC" + std::to_string(i) + "_Est_jdB,";
			ofs << "CC" + std::to_string(i) + "_Est_jdG,";
			ofs << "CC" + std::to_string(i) + "_Est_jdR,";
		}
		ofs << std::endl;

		//	データ
		for (int lightNum = 0; lightNum < _projColors.size(); lightNum++) {
			auto Ip = _projColors[lightNum];
			ofs << Ip[0] << "," << Ip[1] << "," << Ip[2] << ",";
			for (int patchNum = 0; patchNum < ColorChecker::PATCH_NUM; patchNum++) {
				auto Ic = _camColors[lightNum][patchNum];
				ofs << Ic[0] << "," << Ic[1] << "," << Ic[2] << ",";
				auto Ic_diag = estimateColorDiag(Ic, Ip);
				ofs << Ic_diag[0] << "," << Ic_diag[1] << "," << Ic_diag[2] << ",";
				auto Ic_pca = estimateColorPCA(Ic, Ip);
				ofs << Ic_pca[0] << "," << Ic_pca[1] << "," << Ic_pca[2] << ",";
				auto Ic_jd = estimateColorJD(Ic, Ip);
				ofs << Ic_jd[0] << "," << Ic_jd[1] << "," << Ic_jd[2] << ",";
			}
			ofs << std::endl;
		}
	}
}

cv::Vec3d ProCamColorCalibrator::linearizePro(const double * x, cv::Vec3d Ip)
{
	auto g = gammaPro(x);
	cv::Vec3d P;
	for (int i = 0; i < 3;i++) {
		P[i] = pow(Ip[i] / 255.0, g[i]);
	}
	return P;
}

cv::Vec3d ProCamColorCalibrator::gammaPro(const double * x, cv::Vec3d P)
{
	auto g = gammaPro(x);
	cv::Vec3d Ip;
	for (int i = 0; i < 3; i++) {
		auto p = MIN(1.0, MAX(0.0, P[i]));
		Ip[i] = 255.0 * pow(p, 1.0/g[i]);
	}
	return Ip;
}

cv::Vec3d ProCamColorCalibrator::reflectProCam(const double * x, cv::Vec3d P, cv::Mat R)
{
	auto M = colorConvertMat(x);
	auto C0 = C_0(x);
	auto Cp = (cv::Vec3d)cv::Mat(M*cv::Mat(P) + cv::Mat(C0));
	auto C = (cv::Vec3d)cv::Mat(R * cv::Mat(Cp));
	return C;
}

cv::Vec3d ProCamColorCalibrator::reflectProCam(const double * x, cv::Vec3d P, cv::Vec3d r)
{
	auto M = colorConvertMat(x);
	auto C0 = C_0(x);
	auto Cp = (cv::Vec3d)cv::Mat(M*cv::Mat(P) + cv::Mat(C0));
	auto C = r.mul(Cp);
	return C;
}

cv::Vec3d ProCamColorCalibrator::linearizeCam(const double * x, cv::Vec3d Ic)
{
	auto g = gammaCam(x);
	auto Cth = C_th(x);
	cv::Vec3d C;
	for (int i = 0; i < 3; i++) {
		C[i] += pow(Ic[i] / 255.0, g[i]) + Cth[i];
	}
	return C;
}

cv::Vec3d ProCamColorCalibrator::gammaCam(const double * x, cv::Vec3d C)
{
	auto g = gammaCam(x);
	auto Cth = C_th(x);
	cv::Vec3d Ic;
	for (int i = 0; i < 3; i++) {
		auto c = MIN(1.0, MAX(0.0, C[i] - Cth[i]));
		Ic[i] = 255.0 * pow(c, 1.0 / g[i]);
	}
	return Ic;
}

double ProCamColorCalibrator::CostFunction::calc(const double * x) const
{
	//	全投影色に対してNo.19 - 24（灰色）のパッチ色の予測誤差を最小化
	std::vector<cv::Vec3d> error;
	std::vector<cv::Vec3d> ref;		//	フラットな分光反射率を持つNo.19-24のXYZ反射率の平均値
	ColorChecker::getGrayPatchReflectance(ref);
	for (int lightNum = 0; lightNum < projectColors.size(); lightNum++) {
		auto projColor = projectColors[lightNum];
		//	Pro線形化
		auto P = linearizePro(x, projColor);
		//	各パッチ色での反射
		for (int patchNum = ColorChecker::PATCH_CC19_WHITE; patchNum < ColorChecker::PATCH_NUM; patchNum++) {
			auto i = patchNum - ColorChecker::PATCH_CC19_WHITE;	//	反射率用インデックス
			auto C_est = reflectProCam(x, P, ref[i]);
			//	反射光を撮影
			auto Ic_est = gammaCam(x, C_est);
			//	実測値との誤差を記録
			auto Ic_msr = patchColors[lightNum][patchNum];
			error.push_back(Ic_est - Ic_msr);
		}
	}
	//	誤差二乗和を計算
	double cost = 0.0;
	for (auto e : error) {
		cost += e.dot(e);
	}
	return cost;
}
