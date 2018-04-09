#pragma once

#include "OpenGLWindow.h"
#include "ProjectorWindow.h"
#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <FlyCap2CVWrapper.h>
#include "ColorChecker.h"
#include "ProCamColorCalibrator.h"
#include "ProCamGeometryCalibrator.h"
#include "ProCamFeedback.h"

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
	ProCamGeometryCalibrator geocalib;
	ProCamFeedback fb;

	//===========================================
	//	Flags
	//===========================================
	bool show_test_window = false;
	bool show_calib_popup = false;
	bool show_fb_popup = false;
	bool clopping_mode = false;
	bool check_mode = false;
	bool procam_mode = false;
	int removemode = 0;

	//===========================================
	//	Global Variables
	//===========================================
	std::vector<cv::Point> clickPoints;
	int clickSize = 1;

	//===========================================
	//	Import file paths
	//===========================================
	static const std::string colorProfilePath;
	static const std::string testTexturePath;
	static const std::string vertexShaderDir;
	static const std::string removerShaderDir;
	static const std::string generatorShaderDir;

	void cameraInit(void);
	void init(void);
	void update(void);
	void draw(void);
	void drawGUI(void);
	void close(void);
};