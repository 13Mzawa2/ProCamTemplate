#pragma once

#include <OpenGLHeader.h>

class FrameBufferObject
{
public:
	GLuint id;
	GLuint w, h;
	GLuint colorBuffer;		//	�t���[���o�b�t�@�Ƃ��ď������܂��J���[�e�N�X�`��

	FrameBufferObject()
	{
	}

	~FrameBufferObject()
	{
		//glDeleteFramebuffers(1, &id);
	}

	void create(int width, int height)
	{
		//	�J���[�o�b�t�@�p�̃e�N�X�`����p��
		glGenTextures(1, &colorBuffer);
		glBindTexture(GL_TEXTURE_RECTANGLE, colorBuffer);
		glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, 0);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_RECTANGLE, 0);
		w = width; h = height;

		//	�t���[���o�b�t�@���쐬���C�J���[�e�N�X�`�����֘A�t����
		glGenFramebuffers(1, &id);
		glBindFramebuffer(GL_FRAMEBUFFER, id);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE, colorBuffer, 0);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	void enable()
	{
		glBindFramebuffer(GL_FRAMEBUFFER, id);
	}

	void disable()
	{
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
};

