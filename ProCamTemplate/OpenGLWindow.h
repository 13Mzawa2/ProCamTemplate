//--------------------------------------------
//	GLFW�ɂ���č쐬����E�B���h�E�̊��N���X
//	��{�I�Ɏ���E�B���h�E�͂�����p�����č쐬����
//--------------------------------------------

#pragma once

#include <OpenGLHeader.h>
#include <GLImage.h>

class OpenGLWindow {
protected:

public:
	//	�����o�ϐ�
	GLFWwindow *window;
	GLImage glImg;
	glm::uvec2 winSize;
	glm::uvec2 winPos;
	int monitorID;

	//	�R���X�g���N�^
	OpenGLWindow() {};
	OpenGLWindow(glm::uvec2 size, glm::uvec2 pos) :
		winSize(size), winPos(pos) {};
	//	�f�X�g���N�^
	~OpenGLWindow() { close(); };

	//	�E�B���h�E����
	//	GLFW�̏�������CGLEW�������O�ɍs��
	int create(const char* title, glm::uvec2 size, glm::uvec2 pos, bool border_frame = true)
	{
		//	�����o�ϐ��֑��
		winSize = size;
		winPos = pos;

		//	GLFW��window�ݒ�
		glfwWindowHint(GLFW_SAMPLES, 4);				//	4x �A���`�G�C���A�X
		glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);		//	���T�C�Y�s��
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);	//	OpenGL 4.3�𗘗p
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);	//	OpenGL�̃R�A�@�\�̂ݎg�p
		glfwWindowHint(GLFW_DECORATED, border_frame);

		window = glfwCreateWindow(
			winSize[0], winSize[1],
			title, NULL, NULL
		);
		if (window == NULL) {
			std::cerr << title << "�̐����Ɏ��s���܂����D" << std::endl;
			glfwTerminate();
			return EXIT_FAILURE;
		}
		glfwSetWindowPos(window, pos[0], pos[1]);

		return EXIT_SUCCESS;
	}

	//	������
	//	glImage�̏���������GLEW���K�v�ɂȂ�̂ŁCGLEW��������ɍs��
	void baseInit(void)
	{
		glfwMakeContextCurrent(window);

		glfwSwapInterval(0);
		glClearColor(1.0, 1.0, 1.0, 1.0);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glEnable(GL_LESS);			//	�`�悳��Ȃ��ʂ̃����_�����O�ȗ�

		//	glImage�̏�����
		glImg.init(window);
	}
	//	�������������J�X�^���������ꍇ�͂��̊֐���override���ׂ�
	void init(void)
	{
		baseInit();
	}

	//	�`�揈���i��̓I�ȏ�����override�ŏ����j
	void draw()
	{
		glfwMakeContextCurrent(window);
	}
	//	�`���Ԃ���ʂɔ��f(swapbuffer)
	void swapBuffers()
	{
		glfwSwapBuffers(window);
	}

	//	����Ƃ��̏���
	void close()
	{
	}
};