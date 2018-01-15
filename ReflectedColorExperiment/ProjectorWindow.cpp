#include "ProjectorWindow.h"


void ProjectorWindow::init(void)
{
	baseInit();
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	projImg = cv::Mat(projSize, CV_8UC3, cv::Scalar::all(255));
}

void ProjectorWindow::update(void)
{
}

void ProjectorWindow::draw(void)
{
	glfwMakeContextCurrent(window);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glImg.draw(projImg);
}
