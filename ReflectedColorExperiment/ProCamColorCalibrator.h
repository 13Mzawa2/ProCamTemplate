#pragma once
#include <opencv2\opencv.hpp>
#include <FlyCap2CVWrapper.h>
#include "ColorChecker.h"

//----------------------------------------
//	���f����
//	���͕�����: P_i = (I_Pi / 255)^gamma_Pi
//	���˕�����: C = R(MP + C_0)
//	�o�͕�����: I_ci = 255 * max(0, C_i - C_thi)^(1/gamma_Ci)
//	OpenCV�̎d�l�ɏ����Ă���̂�RGB�̏��Ԃ͂��ׂ�BGR��

class ProCamColorCalibrator
{
private:
	//	[0 - 2] gamma_P
	//	[3 - 5] gamma_C
	//	[6 - 8] C_0
	//	[9 - 11] C_th
	//	[12 - 20] M
	double params[21];


	//	�ŏ����\���o�p
	//	�p�����[�^�̉���
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

	//	���f����
	static cv::Vec3d linearizePro(const double *x, cv::Vec3d Ip);
	static cv::Vec3d gammaPro(const double *x, cv::Vec3d P);
	static cv::Vec3d reflectProCam(const double *x, cv::Vec3d P, cv::Mat R);
	static cv::Vec3d reflectProCam(const double *x, cv::Vec3d P, cv::Vec3d r);	//	�Ίp�������肵������
	static cv::Vec3d linearizeCam(const double *x, cv::Vec3d Ic);
	static cv::Vec3d gammaCam(const double *x, cv::Vec3d C);

	//	�ŏ�������ړI�֐�
	class CostFunction : public cv::MinProblemSolver::Function 
	{
	public:
		//	�v�������p�b�`�F(BGR, [0.0, 255.0])
		//	patchColors[light: 0-(n>3)][patchNum: 0-23]
		std::vector<std::vector<cv::Vec3d>> patchColors;
		//	���e�F
		//	patchColors�̗v�f���ƑΉ�
		std::vector<cv::Vec3d> projectColors;
		//	�ړI�֐��̒�`
		double calc(const double* x) const;
		//	�p�����[�^�x�N�g���̎�����
		virtual int getDims() const { return 21; }	
	};


public:
	bool calibrated = false;


	ProCamColorCalibrator();
	~ProCamColorCalibrator();

	void paramInit();

	//	�p�����[�^�̉���
	cv::Vec3d gammaPro() { return gammaPro(params); };
	cv::Vec3d gammaCam() { return gammaCam(params); };
	cv::Vec3d C_0() { return C_0(params); };
	cv::Vec3d C_th() { return C_th(params); };
	cv::Mat colorConvertMat() { return colorConvertMat(params); };

	//	���f����
	cv::Vec3d linearizePro(cv::Vec3d Ip) { return linearizePro(params, Ip); };
	cv::Vec3d gammaPro(cv::Vec3d P) { return gammaPro(params, P); };
	cv::Vec3d reflectProCam(cv::Vec3d P, cv::Mat R) { return reflectProCam(params, P, R); };
	cv::Vec3d reflectProCam(cv::Vec3d P, cv::Vec3d r) { return reflectProCam(params, P, r); };
	cv::Vec3d linearizeCam(cv::Vec3d Ic) { return linearizeCam(params, Ic); };
	cv::Vec3d gammaCam(cv::Vec3d C) { return gammaCam(params, C); };

	//	ProCam�J���[�L�����u���[�V����
	//
	//	OpenCV�̃E�B���h�E�𐶐����čs��
	void calibrate(FlyCap2CVWrapper &flycap, cv::Rect projArea, ColorChecker cc);
	//	���~�V���v���b�N�X�@�ōœK��
	double fit(std::vector<std::vector<cv::Vec3d>> _patchColors, std::vector<cv::Vec3d> _lightColors);
	//	�p�����[�^�̕\��
	void showParams();

	//	���˗��s��Ɋւ���v�Z
	//
	//	�e�J���[�p�b�`�̔��˗��s��̍ŏ������𓾂�
	void getReflectanceMatrices(std::vector<std::vector<cv::Vec3d>> _patchColors, std::vector<cv::Vec3d> _lightColors, std::vector<cv::Mat> &_refMats);
	//	�����Ίp��
	void jointDiagonalization(std::vector<cv::Mat> _reflectances, cv::Mat &matJD);
};
