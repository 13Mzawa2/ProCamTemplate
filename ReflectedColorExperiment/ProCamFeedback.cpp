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
	//mapCPXTex.uploadPBO(mapCPX);
	//mapCPYTex.uploadPBO(mapCPY);
	mapCPXTex.upload(mapCPX);
	mapCPYTex.upload(mapCPY);
	
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
	targetTex.create(GLTexture::TextureType::TYPE_8UC3, 5, cameraSize);
	mapPCXTex.create(GLTexture::TextureType::TYPE_32F, 6, projectorSize);
	mapPCYTex.create(GLTexture::TextureType::TYPE_32F, 7, projectorSize);

	//mapPCXTex.uploadPBO(mapCPX);
	//mapPCYTex.uploadPBO(mapCPY);
	mapPCXTex.upload(mapCPX);
	mapPCYTex.upload(mapCPY);

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

//	キャリブレーションデータ等の定数をアップロード
//	camShader
void ProCamFeedback::setUniformConstantsCam()
{
	//	定数
	glUniform3fv(glGetUniformLocation(camShader.program, "gamma_p"), 1, cv::Vec3f(gamma_p).val);
	glUniform3fv(glGetUniformLocation(camShader.program, "gamma_c"), 1, cv::Vec3f(gamma_c).val);
	glUniform3fv(glGetUniformLocation(camShader.program, "c_0"), 1, cv::Vec3f(c_0).val);
	glUniform3fv(glGetUniformLocation(camShader.program, "c_th"), 1, cv::Vec3f(c_th).val);
	glUniformMatrix3fv(glGetUniformLocation(camShader.program, "cmatPC"), 1, true, (float*)colorConvMat.data);
	glUniformMatrix3fv(glGetUniformLocation(camShader.program, "matJD"), 1, true, (float*)matJD.data);
	cv::Mat matJDinv = matJD.inv();
	glUniformMatrix3fv(glGetUniformLocation(camShader.program, "matJDinv"), 1, true, (float*)matJDinv.data);
	glUniformMatrix3fv(glGetUniformLocation(camShader.program, "matPCA0"), 1, true, (float*)matPCA[0].data);
	glUniformMatrix3fv(glGetUniformLocation(camShader.program, "matPCA1"), 1, true, (float*)matPCA[1].data);
	glUniformMatrix3fv(glGetUniformLocation(camShader.program, "matPCA2"), 1, true, (float*)matPCA[2].data);
	glUniform1i(glGetUniformLocation(camShader.program, "estmodel"), est_model);
	glUniform2i(glGetUniformLocation(camShader.program, "camSize"), cameraSize.width, cameraSize.height);
	glUniform2i(glGetUniformLocation(camShader.program, "projSize"), projectorSize.width, projectorSize.height);
	//	サンプラーをテクスチャに関連付け
	cameraTex.setUniformSampler(cameraSamplerID_c);
	projectorTex.setUniformSampler(projectorSamplerID_c);
	mapCPXTex.setUniformSampler();
	mapCPYTex.setUniformSampler();
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
	cameraSize = gc.mapCPX.size();
	projectorSize = gc.mapPCX.size();
	mapCPX = gc.mapCPX.clone();
	mapCPY = gc.mapCPY.clone();
	mapPCX = gc.mapPCX.clone();
	mapPCY = gc.mapPCY.clone();
	gamma_p = cc.gammaPro();
	gamma_c = cc.gammaCam();
	c_0 = cc.C_0();
	c_th = cc.C_th();
	cc.colorConvertMat().convertTo(colorConvMat, CV_32F);
	cc.getMatJD().convertTo(matJD, CV_32F);
	matPCA.clear();
	for (int i = 0; i < 3; i++) {
		cv::Mat m;
		cc.getMatPCA()[i].convertTo(m, CV_32F);
		matPCA.push_back(m);
	}

	//	シェーダープログラムのコンパイル
	camShader.initGLSL(vertexDir, camFragDir);
	projShader.initGLSL(vertexDir, projFragDir);

	//	各シェーダに対してセットアップ
	setupCam();
	setupPro();
	setupVertices();

	initialised = true;
}

void ProCamFeedback::remove(cv::Mat cameraImg, cv::Mat projectorImg, cv::Mat & removedImg, int removeMode)
{
	glfwMakeContextCurrent(camWindow);
	////	FBOを有効化（これ以降はオフスクリーンレンダリング）
	//fbo.enable();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	camShader.enable();
	//	画像のアップロード
	//cameraTex.uploadPBO(cameraImg);
	//projectorTex.uploadPBO(projectorImg);
	cameraTex.upload(cameraImg);
	projectorTex.upload(projectorImg);
	//	その他変数のアップロード
	est_model = removeMode;
	setUniformConstantsCam();

	//	描画(1e-5ms)
	glBindVertexArray(vertexArrayObj);
	glDrawArrays(GL_TRIANGLE_FAN, 0, vertexNum);
	glBindVertexArray(0);
	//fbo.disable();
	camShader.disable();
	glBindTexture(GL_TEXTURE_RECTANGLE, 0);

	//removedTex.downloadPBO(fbo.id);
	//removedTex.readPixelsPBO(removedImg);
	//removedTex.readPixelsPBO(fbo.id, removedImg);
	removedTex.readPixels(removedImg);
}
