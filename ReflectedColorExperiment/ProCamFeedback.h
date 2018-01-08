//	反射色推定，投影色フィードバックを一手に担うクラス

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
	//	ウィンドウ
	GLFWwindow *camWindow, *projWindow;
	cv::Size cameraSize, projectorSize;
	//	OpenGLテクスチャ
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
	//	オフスクリーンレンダリング用フレームバッファ
	FrameBufferObject fbo;
	//	描画用頂点情報
	GLuint vertexArrayObj, vertexBufferObj;
	int vertexNum;
	//	シェーダー
	Shader camShader, projShader;
	//	キャリブレーションデータ
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

	//	現在のカメラ画像，プロジェクタ画像から白色光下の画像を推定する
	//	mode : 0 = default (diag), 1 = pca, 2 = jd
	void remove(cv::Mat cameraImg, cv::Mat projectorImg, cv::Mat &removedImg, int mode = 0);
	//	目標画像をもとに投影像を作成する
	//	backlight: 光量の最低値
	void generateProjectorLight(cv::Mat targetImg, float backlight = 0.f);

};

