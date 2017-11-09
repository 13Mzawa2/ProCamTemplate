#pragma once

#include "OpenGLWindow.h"
#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <FlyCap2CVWrapper.h>

class ControlWindow : public OpenGLWindow
{
public:
	FlyCap2CVWrapper flycap;
	cv::Size cameraSize;
	bool show_test_window = false;

	void cameraInit(void);
	void init(void);
	void draw(void);
	void drawGUI(void);
	void close(void);
};