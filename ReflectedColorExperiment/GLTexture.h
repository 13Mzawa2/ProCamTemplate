//======================================
//	OpenGL�̃e�N�X�`���o�b�t�@�֘A
//	�V�F�[�_�֘A�̊ȗ����̂��߂̃N���X�Ȃ̂ŕ`��@�\�͖���
//	cv::Mat��`�悵�����Ȃ�GLImage���g���ׂ�
//======================================

#pragma once
#include <OpenGLHeader.h>
#include <opencv2\opencv.hpp>

class GLTexture
{
public:
	enum TextureType {
		TYPE_8UC3 = 0,
		TYPE_32F = 1
	};
	GLuint textureID;
	GLuint samplerID;
	GLint unitNum;
	TextureType textureType;
	cv::Size size;
	
	//	Pixel Buffer Object
	//		�s�N�Z���f�[�^�̂����̍������Ɏg�p
	//		Direct Memory Access (DMA)��p����GPU�ƃf�[�^������肷�邽�߁CCPU�T�C�N���ɉe�����ꂸ�ɍ����]���ł���
	//	PACK: Frame Buffer -> PBO  glReadPixels()�Ȃ�
	//	UNPACK: PBO -> Frame Buffer  glDrawPixels(), glTexSubImage2D()�Ȃ�
	GLuint pbo;

	GLTexture() {};
	~GLTexture() {};

	//	OpenGL�p�e�N�X�`���̐���
	//	@param:
	//		type: ��`�����^
	//		texUnitNum: �e�N�X�`�����j�b�g�ԍ�
	//		textureSize:�@��������e�N�X�`���̃T�C�Y
	void create(TextureType type, GLuint texUnitNum, cv::Size textureSize)
	{
		//	�e�N�X�`���I�u�W�F�N�g�̐����iID�̍쐬�j
		glGenTextures(1, &textureID);
		//	�ݒ肷��e�N�X�`���̎w��
		glBindTexture(GL_TEXTURE_RECTANGLE, textureID);

		//	�e�N�X�`���̈�̊m��
		GLint internalformat, format, bittype;
		getType(type, internalformat, format, bittype);
		glTexImage2D(GL_TEXTURE_RECTANGLE, 0, internalformat,
			textureSize.width, textureSize.height, 0, format, bittype, NULL);

		//	�e�N�X�`���̊g��k�����@�̐ݒ�F���`���
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		//	�e�N�Z���͈͊O�ɃA�N�Z�X�����ꍇ�̐ݒ�
		//		GL_CLAMP_TO_BORDER: ���E�F�œh��Ԃ�
		GLfloat border[] = {0,0,0,0 };
		glTexParameterfv(GL_TEXTURE_RECTANGLE, GL_TEXTURE_BORDER_COLOR, border);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

		//	PBO�̍쐬
		createPBO(type, textureSize);

		//	�ݒ�̕ۑ�
		textureType = type;
		unitNum = texUnitNum;
		size = textureSize;
	}

	//	PBO�̐���
	void createPBO(TextureType type, cv::Size textureSize)
	{
		glGenBuffers(1, &pbo);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);

		int buffersize = getBufferSize(type, textureSize);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, buffersize, NULL, GL_STREAM_READ);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}

	//	�摜��GPU�̃e�N�X�`�����j�b�g�֓]��
	void upload(cv::Mat img)
	{
		//	�e�N�X�`�����j�b�g�̎w��
		glActiveTexture(GL_TEXTURE0 + unitNum);
		glBindTexture(GL_TEXTURE_RECTANGLE, textureID);
		//	�e�N�X�`���̓]��
		GLint internalformat, format, bittype;
		getType(textureType, internalformat, format, bittype);
		glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, img.cols, img.rows, format, bittype, img.data);
		glActiveTexture(GL_TEXTURE0);
	}

	//	PBO��p�����e�N�X�`������
	//	�摜��GPU�̃e�N�X�`�����j�b�g��PBO����ē]��
	void uploadPBO(cv::Mat img)
	{
		//	�e�N�X�`�����j�b�g�̎w��
		glActiveTexture(GL_TEXTURE0 + unitNum);
		glBindTexture(GL_TEXTURE_RECTANGLE, textureID);

		//	�f�[�^�T�C�Y�̌v�Z
		GLint internalformat, format, bittype;
		getType(textureType, internalformat, format, bittype);
		int buffersize = getBufferSize(textureType, img.size());

		//	PBO�̎w��
		//	GPU��TextureObject�ւ̓]���Ȃ̂�UNPACK
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
		//	CPU����PBO�ւ̉摜�A�b�v���[�h
		//	GPU���������I����܂őҋ@��Ԃɓ���̂�����邽�߁CNULL�|�C���^�ŌĂ񂾌��
		//	glMapBuffer()���ĂԂ��ƂŁC�����I�Ƀ������̈���m�ۂ�����
		glBufferData(GL_PIXEL_UNPACK_BUFFER, buffersize, 0, GL_STREAM_DRAW);
		//	CPU�̈��PBO���������ݐ�p�œW�J
		GLubyte* pboptr = (GLubyte*)glMapBufferARB(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
		if (pboptr) {
			//	�摜�f�[�^��PBO�փR�s�[
			std::memcpy(pboptr, img.data, buffersize);
			//	�}�b�v�̉���
			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
		}

		//	PBO����TextureObject�֓]��
		glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, img.cols, img.rows, format, bittype, 0);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		glActiveTexture(GL_TEXTURE0);
	}

	//	�ʂ̏������ݍς�PBO��n���ē]��
	void uploadPBO(GLuint _pbo)
	{
		//	�e�N�X�`�����j�b�g�̎w��
		glActiveTexture(GL_TEXTURE0 + unitNum);
		glBindTexture(GL_TEXTURE_RECTANGLE, textureID);

		//	�f�[�^�T�C�Y�̌v�Z
		GLint internalformat, format, bittype;
		getType(textureType, internalformat, format, bittype);
		int buffersize = getBufferSize(textureType, size);

		//	PBO�̎w��
		//	GPU��TextureObject�ւ̓]���Ȃ̂�UNPACK
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, _pbo);
		//	PBO����TextureObject�֓]��
		glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, size.width, size.height, format, bittype, 0);
		
		//	��A�N�e�B�u��
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		glActiveTexture(GL_TEXTURE0);
	}

	//	�C�ӂ�FrameBufferObject����PBO�փR�s�[
	//	PBO���烁�����R�s�[�͕�
	void downloadPBO(GLuint buffID)
	{
		glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, buffID);
		//	PBO�ւ̏�������
		GLint internalformat, format, bittype;
		getType(textureType, internalformat, format, bittype);
		glReadPixels(0, 0, size.width, size.height, format, bittype, 0);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	}

	//	�������ݍς�PBO����CPU�̈�ɃR�s�[
	void readPixelsPBO(cv::Mat &img)
	{
		img = cv::Mat(size, CV_8UC3);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo);
		GLubyte *pboptr = (GLubyte*)glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
		if (pboptr) {
			int buffersize = getBufferSize(textureType, size);
			std::memcpy(img.data, pboptr, buffersize);
			glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
		}
		glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	}

	//	�V�F�[�_�v���O��������e�N�X�`���T���v���[��ID���擾
	GLuint getSampler(GLuint program, const GLchar *name)
	{
		samplerID = glGetUniformLocation(program, name);
		return samplerID;
	}

	//	�V�F�[�_��Uniform�ϐ��ɃT���v���[�ƃe�N�X�`�����j�b�g��o�^
	void setUniformSampler()
	{
		glUniform1i(samplerID, unitNum);
	}
	void setUniformsampler(GLuint _samplerID)
	{
		glUniform1i(_samplerID, unitNum);
	}

private:
	//	TextureType�̖|��
	void getType(TextureType type, GLint &internalformat, GLint &format, GLint &bittype)
	{
		switch (type) {
		case TYPE_32F:
			internalformat = GL_R32F;
			format = GL_RED;
			bittype = GL_FLOAT;
			break;
		default:	//	TYPE_8UC3
			internalformat = GL_RGB;
			format = GL_BGR;
			bittype = GL_UNSIGNED_BYTE;
		}
	}

	//	�o�b�t�@�T�C�Y�̌v�Z
	int getBufferSize(TextureType type, cv::Size size)
	{
		int buffersize;
		switch (type) {
		case TYPE_32F:
			buffersize = size.width*size.height * sizeof(float);
			break;
		default:	// TYPE_8UC3
			buffersize = size.width*size.height * sizeof(unsigned char);
		}
	}
	

};