#include "ProCamFeedback.h"



void ProCamFeedback::setupCam()
{
	glfwMakeContextCurrent(camWindow);

	//	テクスチャ関連オブジェクト生成
	cameraTex.create(GLTexture::TextureType::TYPE_8UC3, 0, cameraSize);
	projectorTex.create(GLTexture::TextureType::TYPE_8UC3, 1, projectorSize);
	mapCPXTex.create(GLTexture::TextureType::TYPE_32F, 2, cameraSize);
	mapCPYTex.create(GLTexture::TextureType::TYPE_32F, 3, cameraSize);

	//	初期化時に固定テクスチャを転送
	mapCPXTex.uploadPBO(mapCPX);
	mapCPYTex.uploadPBO(mapCPY);
	
	//	Uniform変数のIDを取得
	cameraSamplerID_c = cameraTex.getSampler(camShader.program, "camImgSampler");
	projectorSamplerID_c = projectorTex.getSampler(camShader.program, "projImgSampler");
	mapCPXTex.getSampler(camShader.program, "mapXSampler");
	mapCPYTex.getSampler(camShader.program, "mapYSampler");

	//	フレームバッファ生成
	fbo.create(cameraSize.width, cameraSize.height);
}

void ProCamFeedback::setupPro()
{
	glfwMakeContextCurrent(projWindow);

	removedTex.create(GLTexture::TextureType::TYPE_8UC3, 4, cameraSize);
	mapPCXTex.create(GLTexture::TextureType::TYPE_32F, 5, projectorSize);
	mapPCYTex.create(GLTexture::TextureType::TYPE_32F, 6, projectorSize);

	mapPCXTex.uploadPBO(mapCPX);
	mapPCYTex.uploadPBO(mapCPY);

	cameraSamplerID_p = cameraTex.getSampler(projShader.program, "camImgSampler");
	projectorSamplerID_p = projectorTex.getSampler(projShader.program, "projImgSampler");
	removedTex.getSampler(projShader.program, "remImgSampler");
	mapPCXTex.getSampler(projShader.program, "mapXSampler");
	mapPCYTex.getSampler(projShader.program, "mapYSampler");
}

void ProCamFeedback::setupVertices()
{
	//	描画範囲の頂点設定
	// 頂点配列オブジェクト(バッファの集合)を生成
	glGenVertexArrays(1, &vertexArrayObj);
	glBindVertexArray(vertexArrayObj);
	// 頂点バッファオブジェクト
	glGenBuffers(1, &vertexBufferObj);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObj);
	// [-1, 1] の正方形
	static const GLfloat position[][2] =
	{
		{ -1.0f, -1.0f },
		{ 1.0f, -1.0f },
		{ 1.0f, 1.0f },
		{ -1.0f, 1.0f }
	};
	vertexNum = sizeof(position) / sizeof(position[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(position), position, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);
}

void ProCamFeedback::setUniformVariables()
{
	//	テクスチャ
	cameraTex.setUniformSampler();
}

ProCamFeedback::ProCamFeedback()
{
}


ProCamFeedback::~ProCamFeedback()
{
}

void ProCamFeedback::init(GLFWwindow * controlWin, GLFWwindow * projector, ProCamColorCalibrator cc, ProCamGeometryCalibrator gc, const char * vertexDir, const char * camFragDir, const char * projFragDir)
{
	camWindow = controlWin;
	projWindow = projector;

	//	キャリブレーションデータのコピー
	mapCPX = gc.mapCPX.clone();
	mapCPY = gc.mapCPY.clone();
	mapPCX = gc.mapPCX.clone();
	mapPCY = gc.mapPCY.clone();
	gamma_p = cc.gammaPro();
	gamma_c = cc.gammaCam();
	c_0 = cc.C_0();
	c_th = cc.C_th();
	colorConvMat = cc.colorConvertMat();
	matJD = cc.getMatJD();
	matPCA = cc.getMatPCA();
	cameraSize = mapCPX.size();
	projectorSize = mapPCX.size();

	//	シェーダープログラムのコンパイル
	camShader.initGLSL(vertexDir, camFragDir);
	projShader.initGLSL(vertexDir, projFragDir);

	//	各シェーダに対してセットアップ
	setupCam();
	setupPro();
	setupVertices();
}
