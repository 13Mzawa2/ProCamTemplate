#pragma once

#include "OpenGLWindow.h"
#include "ProjectorWindow.h"
#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <FlyCap2CVWrapper.h>
#include "ColorChecker.h"
#include "ProCamColorCalibrator.h"

class ControlWindow : public OpenGLWindow
{
public:
	//===========================================
	//	Interfaces
	//===========================================
	FlyCap2CVWrapper flycap;
	cv::Mat cameraImg;
	cv::Mat drawImg;
	cv::Size cameraSize;
	ProjectorWindow projectorWindow;
	ColorChecker cc;
	ProCamColorCalibrator calibrator;

	//===========================================
	//	Flags
	//===========================================
	bool show_test_window = false;
	bool show_crop_popup = false;
	bool clopping_mode = false;

	//===========================================
	//	Import file paths
	//===========================================
	static const std::string colorProfilePath;

	void cameraInit(void);
	void init(void);
	void update(void);
	void draw(void);
	void drawGUI(void);
	void close(void);
};

