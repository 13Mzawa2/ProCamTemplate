//	���ːF����C���e�F�t�B�[�h�o�b�N�����ɒS���N���X

#pragma once

#include <OpenGLHeader.h>
#include <opencv2\opencv.hpp>
#include <Shader.h>
#include "ProCamColorCalibrator.h"
#include "ProCamGeometryCalibrator.h"
#include "GLTexture.h"
#include "FrameBufferObject.h"

class ProCamFeedback
{
private:
	//	�E�B���h�E
	GLFWwindow *camWindow, *projWindow;
	cv::Size cameraSize, projectorSize;
	//	OpenGL�e�N�X�`��
	GLTexture
		cameraTex,
		projectorTex,
		removedTex,
		targetTex,
		mapCPXTex,
		mapCPYTex,
		mapPCXTex,
		mapPCYTex;
	GLuint
		cameraSamplerID_c, cameraSamplerID_p,
		projectorSamplerID_c, projectorSamplerID_p;
	//	�I�t�X�N���[�������_�����O�p�t���[���o�b�t�@
	FrameBufferObject fbo;
	//	�`��p���_���
	GLuint vertexArrayObj, vertexBufferObj;
	int vertexNum;
	//	�V�F�[�_�[
	Shader camShader, projShader;
	//	�L�����u���[�V�����f�[�^
	cv::Mat mapCPX, mapCPY, mapPCX, mapPCY;
	cv::Vec3d gamma_p, gamma_c, c_0, c_th;
	cv::Mat colorConvMat;
	cv::Mat matJD;
	std::vector<cv::Mat> matPCA;

	void setupCam();
	void setupPro();
	void setupVertices();
	void setupUniformConstants();

public:
	
	ProCamFeedback();
	~ProCamFeedback();

	void init(
		GLFWwindow *controlWin, GLFWwindow *projector, 
		ProCamColorCalibrator cc, ProCamGeometryCalibrator gc,
		const char *vertexDir, const char *camFragDir, const char *projFragDir);

	//	���݂̃J�����摜�C�v���W�F�N�^�摜���甒�F�����̉摜�𐄒肷��
	//	mode : 0 = default (diag), 1 = pca, 2 = jd
	void remove(cv::Mat cameraImg, cv::Mat projectorImg, cv::Mat &removedImg, int mode = 0);
	//	�ڕW�摜�����Ƃɓ��e�����쐬����
	//	backlight: ���ʂ̍Œ�l
	void generateProjectorLight(cv::Mat targetImg, float backlight = 0.f);

};

