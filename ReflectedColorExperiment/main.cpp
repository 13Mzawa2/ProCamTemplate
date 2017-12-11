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
	
	//	GLFWの初期化
	if (glfwInit() != GL_TRUE) {
		std::cerr << "GLFWの初期化に失敗しました．" << std::endl;
		exit(EXIT_FAILURE);
	}
	//	各ウィンドウの初期化
	controlWindow.create("Main Window", camsz, glm::uvec2(100, 100));
	controlWindow.projectorWindow.create("Projector Window", glm::uvec2(1024, 768), glm::uvec2(1920, 0), false);
	glfwMakeContextCurrent(controlWindow.window);

	//	GLEWの初期化（glfwMakeContextCurrent()呼出し後でないと失敗する）
	glewExperimental = true;		//	コアプロファイルの場合必要
	if (glewInit() != GLEW_OK) {
		std::cerr << "GLEWの初期化に失敗しました．" << std::endl;
		exit(EXIT_FAILURE);
	}
	//	GLEW初期化後の初期化
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
	if (glfwGetKey(controlWindow.window, GLFW_KEY_ESCAPE) == GLFW_PRESS		//	Escキー
		|| glfwGetKey(controlWindow.projectorWindow.window, GLFW_KEY_ESCAPE) == GLFW_PRESS
		|| glfwWindowShouldClose(controlWindow.window)) {			//	ウィンドウの閉じるボタン
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