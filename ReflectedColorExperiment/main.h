#pragma once

#pragma region Disable Warning C4996
//
// Disable Warning C4996
//
#ifndef _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES
#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
#endif
#ifndef _CRT_SECURE_CPP_OVERLOAD_SECURE_NAMES
#define _CRT_SECURE_CPP_OVERLOAD_SECURE_NAMES 1
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
#endif
#pragma endregion


//===========================================
//	Includes
//===========================================
//	Extra Libraries
#include <OpenGLHeader.h>
#include <OpenCVHeader.h>
//	Original Libraries
#include "ControlWindow.h"

//===========================================
//	import file path
//===========================================



//-----------------------------------------------------
//	GLFW User Interface
//-----------------------------------------------------
ControlWindow controlWindow;


//-----------------------------------------------------
//	Function Prototypes
//-----------------------------------------------------
void setup();
int mainLoop();
void release();