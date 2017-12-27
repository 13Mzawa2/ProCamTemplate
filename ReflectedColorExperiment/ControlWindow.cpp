#include "ControlWindow.h"

const std::string ControlWindow::colorProfilePath = "./data/colorProfile.xml";

void ControlWindow::cameraInit(void)
{
	//	�J�����ݒ�
	flycap.open();
	cameraSize = flycap.readImage().size();
}

void ControlWindow::init(void)
{
	baseInit();
	ImGui_ImplGlfwGL3_Init(window, true);	//	imgui�`��E�B���h�E�̐ݒ�

	glfwSetInputMode(window, GLFW_STICKY_MOUSE_BUTTONS, GL_TRUE);
}

void ControlWindow::update(void)
{
	cameraImg = flycap.readImage();
	drawImg = cameraImg.clone();

	if (clopping_mode) {
		//	���ݒn���擾
		cv::Vec2d p;
		glfwGetCursorPos(this->window, &p[0], &p[1]);
		//	�{�^���𗣂����u��
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
			if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE) {
				//	���ݒn��ۑ�
				cc.corners.push_back((cv::Vec2f)p);
				//	4�_����ꂽ�ꍇ
				if (cc.corners.size() >= 4) {
					cc.getCCImage(cameraImg);
					cc.getPatches(cc.CCImg);
					cv::Mat patchImg = cc.drawPatches(cc.CCPatches);
					cv::imshow("ColorChecker Image", patchImg);
					clopping_mode = false;
				}
			}
		}
		//	�`��
		if (!cc.corners.empty()) {
			for (int i = 0; i < cc.corners.size(); i++) {
				cv::circle(drawImg, cc.corners[i], 5, cv::Scalar(0, 255, 0));
			}
			cv::line(drawImg, cc.corners.back(), cv::Point(p[0], p[1]), cv::Scalar(0, 255, 0));
		}
	}
}

void ControlWindow::draw(void)
{
	glfwMakeContextCurrent(window);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//	�L���v�`���摜�`��
	glImg.draw(drawImg);
}

void ControlWindow::drawGUI(void)
{
	glfwMakeContextCurrent(window);

	//	�V�K�t���[���`��J�n
	ImGuiIO imgui_io = ImGui::GetIO();
	ImGui_ImplGlfwGL3_NewFrame();
	ImGui::SetNextWindowSize(ImVec2(400, 250), ImGuiSetCond_FirstUseEver);
	ImGui::Begin("Control GUI");
	ImGui::Text("Application average %.3f ms / frame (%.1f FPS)", 
		imgui_io.DeltaTime, imgui_io.Framerate);
	if (ImGui::Button("Calibration")) show_calib_popup ^= true;
	ImGui::NewLine();
	if (ImGui::Button("ImGui Demo Window")) show_test_window ^= true;
	ImGui::End();

	//	�L�����u���[�V�����pGUI
	if (show_calib_popup) {
		ImGui::SetNextWindowSize(ImVec2(100, 100), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("Calibration GUI");
		//	ColorChecker�ɂ��J���[�L�����u���[�V����
		if (ImGui::CollapsingHeader("Color")) {
			ImGui::Text(
				"Please right click at the corner of ColorChecker (+)\n"
				"1: upper-left (Brown)\n"
				"2: lower-left (White)\n"
				"3: lower-right (Black)\n"
				"4: upper-right (Lime)");
			if (ImGui::Button("Start")) {
				//	ColorChecker�̃R�[�i�[�ʒu�̏�����
				cc.corners.clear();
				clopping_mode = true;
			}
			if (clopping_mode) {
				ImGui::SameLine();
				ImGui::Text("Now Clopping...");
			}
			//	�N���b�s���O�I����̃f�[�^�\��
			if (!clopping_mode && !cc.CCPatches.empty()) {
				if (ImGui::Button("Show Patches")) {
					cv::imshow("ColorChecker Image", cc.drawPatches(cc.CCPatches));
				}
			}
			//	�L�����u���[�V����
			if (ImGui::CollapsingHeader("ProCam Color Calibration")) {
				//	�f�[�^�̓ǂݍ��݁E��������
				if (ImGui::Button("Load")) {
					calibrator.load(colorProfilePath);
					calibrator.showParams();
				}
				ImGui::SameLine();
				if (ImGui::Button("Save") && calibrator.model_calibrated) {
					calibrator.save(colorProfilePath);
				}
				//	ProCam�L�����u���[�V����(�N���b�s���O��ɑI���\)
				if (!clopping_mode && !cc.corners.empty()) {
					if (ImGui::Button("Start Calibration")) {
						projectorWindow.hide();
						cv::Point pos(projectorWindow.winPos[0], projectorWindow.winPos[1]);
						calibrator.calibrate(flycap, cv::Rect(pos, projectorWindow.projSize), cc);
						projectorWindow.show();
					}
				}
			}
		}
		if (ImGui::CollapsingHeader("Geometry")) {
			ImGui::Text(
			"ProCam geometry calibration by Gray-Code Patterns\n"
			);
			if (ImGui::Button("Start Calibration")) {
				projectorWindow.hide();
				cv::Point pos(projectorWindow.winPos[0], projectorWindow.winPos[1]);
				geocalib.calibrate(flycap, cv::Rect(pos, projectorWindow.projSize));
				projectorWindow.show();
			}
			if (ImGui::Button("Show Maps")) {
				geocalib.showColoredMaps();
			}
		}
		ImGui::End();
	}


	//	ImGui Test Window (Demo) �Q�Ɨp
	if (show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}

	// Rendering
	int display_w, display_h;
	glfwGetFramebufferSize(window, &display_w, &display_h);
	glViewport(0, 0, display_w, display_h);
	ImGui::Render();
}

void ControlWindow::close(void)
{
	flycap.close();
}
