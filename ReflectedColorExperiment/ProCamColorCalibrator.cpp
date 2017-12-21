#include "ProCamColorCalibrator.h"
#include <future>		//	�񓯊��}���`�X���b�h����
#include <iostream>		//	csv�t�@�C�����o��
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
	////	OpenCV�̎d�l�m�F
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

	//	5x5x5�i�K�ŎB�e
	std::vector<double> projInput = { 0., 63., 127., 191., 255. };
	std::vector<cv::Vec3d> projColors;
	std::vector<std::vector<cv::Vec3d>> patchColors;
	std::vector<std::vector<cv::Mat>> patchImgs;
	for (auto b : projInput) {
		for (auto g : projInput) {
			for (auto r : projInput) {
				//	���e
				//	�`�施�߂�����ۂɕ`�悳���܂ł͐��t���[���x��邽�߁C
				//	�������`��̂��߂ɕ�����`�施�߂��o��
				projImg = cv::Scalar(b, g, r);
				for(int i=0;i<3;i++){
					cv::imshow("cv_projector", projImg);
					cv::waitKey(30);
				}
				//	�B�e
				camImg = flycap.readImage();
				//	�p�b�`�F���擾
				std::vector<cv::Vec3d> patch;
				cc.getCCImage(camImg);
				cc.getPatches(cc.CCImg);
				cc.measurePatches(cc.CCPatches, patch);
				//	�`��
				cv::imshow("cv_camera", camImg);
				cv::imshow("cv_patch", cc.drawPatches(cc.CCPatches));
				cv::waitKey(15);
				//	�L�^
				projColors.push_back(cv::Vec3d(b, g, r));
				patchColors.push_back(patch);
				patchImgs.push_back(cc.CCPatches);
			}
		}
	}
	//	�p�����[�^�v�Z�J�n
	std::thread t1([&] { fit(patchColors, projColors); });

	
	// �B�e�p�b�`�̍Ċm�F(for debugging)
	for (auto i = 0; i < patchImgs.size(); i++) {
		cv::imshow("cv_patch", cc.drawPatches(patchImgs[i]));
		cv::setWindowTitle("cv_patch",
			cv::format("pro = (%.0lf, %.0lf, %.0lf)", 
				projColors[i][0], projColors[i][1], projColors[i][2]));
		auto c = cv::waitKey(300);
		if (c == ' ') cv::waitKey();	//	�ꎞ��~
		if (c == 27) break;		//	���f
	}

	//	���������E�B���h�E�����
	cv::destroyWindow("cv_camera");
	cv::destroyWindow("cv_projector");
	cv::destroyWindow("cv_patch");

	t1.join();	//	�X���b�ht1�̏������I���܂őҋ@

	//	���˗��s��̌v�Z�i�p�����[�^�v�Z��ɍs�Ȃ���j
	std::vector<cv::Mat> reflectances;
	cv::Mat matJD;
	std::vector<cv::Mat> matPC;
	getReflectanceMatrices(patchColors, projColors, reflectances);
	jointDiagonalization(reflectances, matJD);
	principalMatrices(reflectances, matPC);
	
	reflectance_calibrated = true;

	//	�L�����u���[�V�����̂��߂̃f�[�^�ƌ��ʂ̕ۑ�
	writeCSV("./data/calib.csv", patchColors, projColors);
}

double ProCamColorCalibrator::fit(std::vector<std::vector<cv::Vec3d>> _patchColors, std::vector<cv::Vec3d> _lightColors)
{
	if (_patchColors.empty() || _lightColors.empty()) return -1;
	if (_patchColors.size() != _lightColors.size()) return -1;
	
	//	�p�����[�^�x�N�g��
	cv::Mat param = (cv::Mat_<double>(1, 21) <<
		2., 2., 2.,			//	�v���W�F�N�^�K���}
		1., 1., 1.,			//	�J�����K���}
		0.01, 0.01, 0.01,	//	����
		0.01, 0.01, 0.01,	//	�J�������x
		0.6, 0.0, 0.0,		//	ProCam�F�ϊ��s��
		0.0, 0.6, 0.0,
		0.0, 0.0, 0.6);
	//	�����ړ���
	cv::Mat step = (cv::Mat_<double>(1, 21) <<
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5);

	//	�ړI�֐��̐ݒ�
	auto ptr_func(new CostFunction());
	ptr_func->patchColors = _patchColors;
	ptr_func->projectColors = _lightColors;
	//	�\���o�̐ݒ�
	auto solver = cv::DownhillSolver::create();
	solver->setFunction(ptr_func);
	solver->setInitStep(step);
	//	�ŏ������s
	double res = solver->minimize(param);

	//	�p�����[�^�x�N�g���̕ۑ�
	param.forEach<double>([&](double &p, const int pos[2])->void {
		params[pos[1]] = p;
	});

	//	�œK�����ʂ̕\��
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
		std::cout << "�t�@�C���p�X���m�F���Ă�������: " << path << std::endl;
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
	//	�p�b�`�ԍ����Ɍv�Z
	for (int patchNum = 0; patchNum < _patchColors[0].size(); patchNum++) {
		//	cv::Mat�^�ɑ��
		cv::Mat lightColors(3, _lightColors.size(), CV_64FC1), 
			patchColors(3, _lightColors.size(), CV_64FC1),
			R, Rt;
		for (int lightNum = 0; lightNum < _lightColors.size(); lightNum++) {
			//	���`��
			auto patchLinear = linearizeCam(_patchColors[lightNum][patchNum]);
			auto lightLinear = linearizePro(_lightColors[lightNum]);
			//	�J�����F��ԂɕύX
			cv::Vec3d Cp = reflectProCam(lightLinear, cv::Vec3d(1, 1, 1));
			for (int j = 0; j < 3; j++) {
				lightColors.at<double>(j, lightNum) = Cp[j];
				patchColors.at<double>(j, lightNum) = patchLinear[j];
			}
		}
		//	���`�ŏ����@��R�����߂�
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

	//	�����Ίp���s��T�̏����l
	//	R0 = u*w*vt = P^-1 D P
	auto R0 = _reflectances[0];
	cv::SVD svd(R0, cv::SVD::FULL_UV);
	auto D0 = svd.w, T0 = svd.vt;

	//	�]���֐��̌��z nabla J�����߂�֐�
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

	//	�ŏ���
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

	//	�����Ίp���ł��Ă��邩�m�F
	std::cout << "T = " << matJD << std::endl;
	std::cout << "T^-1 R0 T = \n" << matJD.inv() *_reflectances[3] * matJD << std::endl;
	std::cout << "T^-1 R1 T = \n" << matJD.inv() *_reflectances[4] * matJD << std::endl;
	std::cout << "T^-1 R2 T = \n" << matJD.inv() *_reflectances[5] * matJD << std::endl;
}

void ProCamColorCalibrator::principalMatrices(std::vector<cv::Mat> _reflectances, std::vector<cv::Mat>& _matPC)
{
	//	���˗��s����s�x�N�g���ɕό`
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

	//	���3�听�������o��
	_matPC.clear();
	std::cout << "components = \n";
	for (int i = 0; i < 3; i++) {
		_matPC.push_back(svd.vt.row(i).reshape(1, 3));
		std::cout << _matPC.back() << std::endl;
	}
	matPCA = _matPC;
}

cv::Vec3d ProCamColorCalibrator::estimateColorDiag(cv::Vec3d camColor, cv::Vec3d projColor, cv::Vec3d light)
{
	if (!model_calibrated) return cv::Vec3d(0, 0, 0);

	auto C = linearizeCam(camColor);
	auto Cp = reflectProCam(linearizePro(projColor), cv::Vec3d(1.0, 1.0, 1.0));
	cv::Vec3d R_diag;
	for (int i = 0; i < 3; i++) {
		R_diag[i] = C[i] / MAX(Cp[i], 0.001);
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

	cv::Mat R_pca, U(3,3,CV_64FC1);
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

	auto C_est = reflectProCam(linearizePro(light), R);
	auto Ic = gammaCam(C_est);

	return Ic;
}

cv::Vec3d ProCamColorCalibrator::estimateColorJD(cv::Vec3d camColor, cv::Vec3d projColor, cv::Vec3d light)
{
	if (!reflectance_calibrated || !model_calibrated) return cv::Vec3d(0, 0, 0);

	auto C = linearizeCam(camColor);
	auto Cp = reflectProCam(linearizePro(projColor), cv::Vec3d(1.0, 1.0, 1.0));

	cv::Vec3d C_d = cv::Vec3d(cv::Mat(matJDinv * cv::Mat(C)));
	cv::Vec3d Cp_d = cv::Vec3d(cv::Mat(matJDinv * cv::Mat(Cp)));

	cv::Vec3d D(C_d[0] / Cp_d[0], C_d[1] / Cp_d[1], C_d[2] / Cp_d[2]);
	cv::Mat R = matJD * cv::Mat::diag(cv::Mat(D)) * matJDinv;

	auto C_est = reflectProCam(linearizePro(light), R);
	auto Ic = gammaCam(C_est);

	return Ic;
}

void ProCamColorCalibrator::writeCSV(cv::String path, std::vector<std::vector<cv::Vec3d>> _camColors, std::vector<cv::Vec3d> _projColors)
{
	std::ofstream ofs(path);
	if (ofs.is_open()) {
		//	�^�C�g���s
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

		//	�f�[�^
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
	//	�S���e�F�ɑ΂���No.19 - 24�i�D�F�j�̃p�b�`�F�̗\���덷���ŏ���
	std::vector<cv::Vec3d> error;
	std::vector<cv::Vec3d> ref;		//	�t���b�g�ȕ������˗�������No.19-24��XYZ���˗��̕��ϒl
	ColorChecker::getGrayPatchReflectance(ref);
	for (int lightNum = 0; lightNum < projectColors.size(); lightNum++) {
		auto projColor = projectColors[lightNum];
		//	Pro���`��
		auto P = linearizePro(x, projColor);
		//	�e�p�b�`�F�ł̔���
		for (int patchNum = ColorChecker::PATCH_CC19_WHITE; patchNum < ColorChecker::PATCH_NUM; patchNum++) {
			auto i = patchNum - ColorChecker::PATCH_CC19_WHITE;	//	���˗��p�C���f�b�N�X
			auto C_est = reflectProCam(x, P, ref[i]);
			//	���ˌ����B�e
			auto Ic_est = gammaCam(x, C_est);
			//	�����l�Ƃ̌덷���L�^
			auto Ic_msr = patchColors[lightNum][patchNum];
			error.push_back(Ic_est - Ic_msr);
		}
	}
	//	�덷���a���v�Z
	double cost = 0.0;
	for (auto e : error) {
		cost += e.dot(e);
	}
	return cost;
}
