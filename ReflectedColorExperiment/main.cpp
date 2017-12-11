#include "main.h"

int main(void)
{
	setup();
	while (mainLoop() != EXIT_FAILURE) {};
	release();

	return EXIT_SUCCESS;
}

void setup()
{
	controlWindow.cameraInit();
	glm::uvec2 camsz(controlWindow.cameraSize.width, controlWindow.cameraSize.height);
	
	//	GLFW�̏�����
	if (glfwInit() != GL_TRUE) {
		std::cerr << "GLFW�̏������Ɏ��s���܂����D" << std::endl;
		exit(EXIT_FAILURE);
	}
	//	�e�E�B���h�E�̏�����
	controlWindow.create("Main Window", camsz, glm::uvec2(100, 100));
	controlWindow.projectorWindow.create("Projector Window", glm::uvec2(1024, 768), glm::uvec2(1920, 0), false);
	glfwMakeContextCurrent(controlWindow.window);

	//	GLEW�̏������iglfwMakeContextCurrent()�ďo����łȂ��Ǝ��s����j
	glewExperimental = true;		//	�R�A�v���t�@�C���̏ꍇ�K�v
	if (glewInit() != GLEW_OK) {
		std::cerr << "GLEW�̏������Ɏ��s���܂����D" << std::endl;
		exit(EXIT_FAILURE);
	}
	//	GLEW��������̏�����
	controlWindow.init();
	controlWindow.projectorWindow.init();
}

int mainLoop()
{
	controlWindow.update();
	controlWindow.projectorWindow.update();

	controlWindow.draw();
	controlWindow.drawGUI();
	controlWindow.projectorWindow.draw();

	controlWindow.swapBuffers();
	controlWindow.projectorWindow.swapBuffers();

	//	Quit Program
	if (glfwGetKey(controlWindow.window, GLFW_KEY_ESCAPE) == GLFW_PRESS		//	Esc�L�[
		|| glfwGetKey(controlWindow.projectorWindow.window, GLFW_KEY_ESCAPE) == GLFW_PRESS
		|| glfwWindowShouldClose(controlWindow.window)) {			//	�E�B���h�E�̕���{�^��
		return EXIT_FAILURE;
	}
	glfwPollEvents();

	return EXIT_SUCCESS;
}

void release()
{
	controlWindow.close();
	controlWindow.projectorWindow.close();
}