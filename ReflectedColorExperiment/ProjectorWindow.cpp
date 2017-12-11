#include "ProjectorWindow.h"

void ProjectorWindow::init(void)
{
	baseInit();
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
}

void ProjectorWindow::update(void)
{
}

void ProjectorWindow::draw(void)
{
	glfwMakeContextCurrent(window);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
