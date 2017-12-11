#pragma once

#include "OpenGLWindow.h"

class ProjectorWindow : public OpenGLWindow
{
public:
	cv::Size projSize = cv::Size(1024, 768);
	void init(void);
	void update(void);
	void draw(void);
};