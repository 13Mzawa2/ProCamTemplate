/********************************************************
OpenGL Image with OpenCV
GLFW��OpenCV��cv::Mat��w�i�`�悷�邽�߂̃N���X

How to Use:
1. ���C�����[�v�ɓ���O��GLImage�𐶐�
2. �`�悵����GLFWwindow��^����GLImage��������
3. ���C�����[�v���Ŏ��̗l�ɏ���(ex. mainWindow�̔w�i��frameImg��`��)

//	Change Current Window
glfwMakeContextCurrent(mainWindow);
//	Clear Buffer Bits
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	Draw Image
glImg.draw(frameImg);		//	<- Only render to back buffer
//	Clear Depth Bits (so you can overwride CG on frameImg)
glClear(GL_DEPTH_BUFFER_BIT);
//	Draw your CG
//	End Draw
glfwSwapBuffers(mainWindow);

Change 20180506:
�EPBO���_�u���o�b�t�@�����O�ɂ����i����Ȃɑ���ry�j

Change 20171228:
�EPBO�ɂ�鍂������}��i����Ȃɑ����Ȃ�Ȃ������j

Change 20170202:
�Eflip���V�F�[�_���Ŏ��s����悤�ɕύX

Change 20160119:
�E�R���X�g���N�^�ŏ������ł���悤�ɂ���
�E�R�����g�啝�ǉ�
�EGLSL���C�����C�������ĊO���t�@�C����s�v�ɂ���


*********************************************************/

#pragma once

#include "OpenGLHeader.h"
#include "Shader.h"
#include <opencv2\opencv.hpp>

class GLImage
{
private:
	GLFWwindow *imgWindow;
	GLuint vao;		//	���_�z��I�u�W�F�N�g
	GLuint vbo;		//	���_�o�b�t�@�I�u�W�F�N�g
	GLuint image;	//	�e�N�X�`���I�u�W�F�N�g
	GLuint imageLoc;//	�I�u�W�F�N�g�̏ꏊ
	Shader s;		//	�V�F�[�_

	//	Pixel Buffer Object
	//		�s�N�Z���f�[�^�̂����̍������Ɏg�p
	//		Direct Memory Access (DMA)��p����GPU�ƃf�[�^������肷�邽�߁CCPU�T�C�N���ɉe�����ꂸ�ɍ����]���ł���
	//	PACK: Frame Buffer -> PBO  glReadPixels()�Ȃ�
	//	UNPACK: PBO -> Frame Buffer  glDrawPixels(), glTexSubImage2D()�Ȃ�
	GLuint pbo[2];
	int pbo_buffersize;
	int index;
	int nextindex;

	//	�o�[�e�b�N�X�V�F�[�_
	const char *vertexSource =
		"#version 330 core \n"
		"layout(location = 0) in vec4 pv;\n"
		"void main(void)\n" 
		"{\n" 
		"	gl_Position = pv;\n" 
		"}\n";
	//	�t���O�����g�V�F�[�_
	const char *fragmentSource =
		"#version 330 core \n"
		"uniform sampler2DRect image;\n"
		"uniform vec2 resolution;\n"
		"layout(location = 0) out vec4 fc;\n"
		"void main(void)\n"
		"{\n"
		"	fc = texture(image, vec2(gl_FragCoord.x, resolution.y-gl_FragCoord.y));\n"
		"}\n";
	int vertices;

public:
	GLImage()
	{
	}
	GLImage(GLFWwindow *window)
	{
		init(window);
	}
	void init(GLFWwindow *window)
	{
		int w, h;
		glfwMakeContextCurrent(window);
		glfwGetWindowSize(window, &w, &h);
		imgWindow = window;

		// ���_�z��I�u�W�F�N�g
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);

		// ���_�o�b�t�@�I�u�W�F�N�g
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);

		// [-1, 1] �̐����`
		static const GLfloat position[][2] =
		{
			{ -1.0f, -1.0f },
			{ 1.0f, -1.0f },
			{ 1.0f, 1.0f },
			{ -1.0f, 1.0f }
		};
		vertices = sizeof(position) / sizeof (position[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(position), position, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);

		//	�e�N�X�`��
		glGenTextures(1, &image);
		glBindTexture(GL_TEXTURE_RECTANGLE, image);
		glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGB, w, h, 0, GL_BGR, GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

		// PBO�̐���
		// PBO�Ń_�u���o�b�t�@�����O���邽��2�쐬����
		pbo_buffersize = w*h * 3 * sizeof(GLubyte);
		glGenBuffers(2, pbo);
		// 1��
		index = 0;
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo[index]);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, pbo_buffersize, NULL, GL_STREAM_READ);
		// 2��
		nextindex = 1;
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo[nextindex]);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, pbo_buffersize, NULL, GL_STREAM_READ);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

		//	�V�F�[�_�̃��[�h
		s.initInlineGLSL(vertexSource, fragmentSource);
		imageLoc = glGetUniformLocation(s.program, "image");
	}
	void draw(cv::Mat frame)
	{
		glfwMakeContextCurrent(imgWindow);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		// index�̍X�V
		index = (index + 1) % 2;
		nextindex = (index + 1) % 2;

		// �������ݍς݂�PBO���e�N�X�`���̈�ɓ]��
		// PBO�w���Ԃ�glTexSubImage2D()���f�[�^�|�C���^��n�����Ɏ��s�����PBO����]�������
		glBindTexture(GL_TEXTURE_RECTANGLE, image);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo[index]);
		glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, 0);
		
		//	����PBO�Ƀf�[�^����������
		//	GPU��TextureObject�ւ̓]���Ȃ̂�UNPACK
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo[nextindex]);
		//	CPU����PBO�ւ̉摜�A�b�v���[�h
		//	GPU���������I����܂őҋ@��Ԃɓ���̂�����邽�߁CNULL�|�C���^�ŌĂ񂾌��
		//	glMapBuffer()���ĂԂ��ƂŁC�����I�Ƀ������̈���m�ۂ�����
		glBufferData(GL_PIXEL_UNPACK_BUFFER, pbo_buffersize, 0, GL_STREAM_DRAW);
		//	CPU�̈��PBO���������ݐ�p�œW�J
		GLubyte* pboptr = (GLubyte*)glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
		if (pboptr) {
			//	�摜�f�[�^��PBO�փR�s�[
			std::memcpy(pboptr, frame.data, pbo_buffersize);
			//	�}�b�v�̉���
			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
		}

		//	PBO�̎w�������
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

		// �����_�����O�J�n
		// �V�F�[�_�v���O�����̎g�p�J�n
		s.enable();

		// uniform �T���v���̎w��
		glUniform1i(imageLoc, 0);

		// �𑜓x�̎w��
		glUniform2f(glGetUniformLocation(s.program, "resolution"), frame.cols, frame.rows);

		// �e�N�X�`�����j�b�g�ƃe�N�X�`���̎w��
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_RECTANGLE, image);

		// �`��Ɏg�����_�z��I�u�W�F�N�g�̎w��
		glBindVertexArray(vao);

		// �}�`�̕`��
		glDrawArrays(GL_TRIANGLE_FAN, 0, vertices);

		// ���_�z��I�u�W�F�N�g�̎w�����
		glBindVertexArray(0);

		// �V�F�[�_�v���O�����̎g�p�I��
		s.disable();
	}
};