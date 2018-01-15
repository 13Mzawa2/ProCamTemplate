#include "ProCamFeedback.h"



void ProCamFeedback::setupCam()
{
	glfwMakeContextCurrent(camWindow);

	//	�e�N�X�`���֘A�I�u�W�F�N�g����
	cameraTex.create(GLTexture::TextureType::TYPE_8UC3, 0, cameraSize);
	projectorTex.create(GLTexture::TextureType::TYPE_8UC3, 1, projectorSize);
	mapCPXTex.create(GLTexture::TextureType::TYPE_32F, 2, cameraSize);
	mapCPYTex.create(GLTexture::TextureType::TYPE_32F, 3, cameraSize);

	//	���������ɌŒ�e�N�X�`����]��
	//mapCPXTex.uploadPBO(mapCPX);
	//mapCPYTex.uploadPBO(mapCPY);
	mapCPXTex.upload(mapCPX);
	mapCPYTex.upload(mapCPY);
	
	//	Uniform�ϐ���ID���擾
	cameraSamplerID_c = cameraTex.getSampler(camShader.program, "camImgSampler");
	projectorSamplerID_c = projectorTex.getSampler(camShader.program, "projImgSampler");
	mapCPXTex.getSampler(camShader.program, "mapXSampler");
	mapCPYTex.getSampler(camShader.program, "mapYSampler");

	//	�t���[���o�b�t�@����
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
	//	�`��͈͂̒��_�ݒ�
	// ���_�z��I�u�W�F�N�g(�o�b�t�@�̏W��)�𐶐�
	glGenVertexArrays(1, &vertexArrayObj);
	glBindVertexArray(vertexArrayObj);
	// ���_�o�b�t�@�I�u�W�F�N�g
	glGenBuffers(1, &vertexBufferObj);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObj);
	// [-1, 1] �̐����`
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

//	�L�����u���[�V�����f�[�^���̒萔���A�b�v���[�h
//	camShader
void ProCamFeedback::setUniformConstantsCam()
{
	//	�萔
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
	//	�T���v���[���e�N�X�`���Ɋ֘A�t��
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

	//	�L�����u���[�V�����f�[�^�̃R�s�[
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

	//	�V�F�[�_�[�v���O�����̃R���p�C��
	camShader.initGLSL(vertexDir, camFragDir);
	projShader.initGLSL(vertexDir, projFragDir);

	//	�e�V�F�[�_�ɑ΂��ăZ�b�g�A�b�v
	setupCam();
	setupPro();
	setupVertices();

	initialised = true;
}

void ProCamFeedback::remove(cv::Mat cameraImg, cv::Mat projectorImg, cv::Mat & removedImg, int removeMode)
{
	glfwMakeContextCurrent(camWindow);
	////	FBO��L�����i����ȍ~�̓I�t�X�N���[�������_�����O�j
	//fbo.enable();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	camShader.enable();
	//	�摜�̃A�b�v���[�h
	//cameraTex.uploadPBO(cameraImg);
	//projectorTex.uploadPBO(projectorImg);
	cameraTex.upload(cameraImg);
	projectorTex.upload(projectorImg);
	//	���̑��ϐ��̃A�b�v���[�h
	est_model = removeMode;
	setUniformConstantsCam();

	//	�`��(1e-5ms)
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
