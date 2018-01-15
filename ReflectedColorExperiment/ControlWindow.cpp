#include "ControlWindow.h"


const std::string ControlWindow::colorProfilePath = "./data/colorProfile.xml";
const std::string ControlWindow::testTexturePath = "./data/xyColor.png";
const std::string ControlWindow::vertexShaderDir = "./VertexShader.glsl";
const std::string ControlWindow::removerShaderDir = "./Frag_Remover.glsl";
const std::string ControlWindow::generatorShaderDir = "./Frag_Generator.glsl";

void ControlWindow::cameraInit(void)
{
	//	カメラ設定
	flycap.open();
	cameraSize = flycap.readImage().size();
}

void ControlWindow::init(void)
{
	baseInit();
	ImGui_ImplGlfwGL3_Init(window, true);	//	imgui描画ウィンドウの設定

	glfwSetInputMode(window, GLFW_STICKY_MOUSE_BUTTONS, GL_TRUE);
}

void ControlWindow::update(void)
{
	cameraImg = flycap.readImage();
	drawImg = cameraImg.clone();

	if (procam_mode) {
		fb.remove(cameraImg, projectorWindow.projImg, drawImg, removemode);
	}

	if (clopping_mode) {
		//	現在地を取得
		cv::Vec2d p;
		glfwGetCursorPos(this->window, &p[0], &p[1]);
		//	ボタンを離した瞬間
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
			if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE) {
				//	現在地を保存
				cc.corners.push_back((cv::Vec2f)p);
				//	4点得られた場合
				if (cc.corners.size() >= 4) {
					cc.getCCImage(cameraImg);
					cc.getPatches(cc.CCImg);
					cv::Mat patchImg = cc.drawPatches(cc.CCPatches);
					cv::imshow("ColorChecker Image", patchImg);
					clopping_mode = false;
				}
			}
		}
		//	描画
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

	//	キャプチャ画像描画
	glImg.draw(drawImg);
}

void ControlWindow::drawGUI(void)
{
	glfwMakeContextCurrent(window);

	//	新規フレーム描画開始
	ImGuiIO imgui_io = ImGui::GetIO();
	ImGui_ImplGlfwGL3_NewFrame();
	ImGui::SetNextWindowSize(ImVec2(400, 250), ImGuiSetCond_FirstUseEver);
	ImGui::Begin("Control GUI");
	ImGui::Text("Application average %.3f ms / frame (%.1f FPS)", 
		imgui_io.DeltaTime, imgui_io.Framerate);
	if (ImGui::Button("Capture")) {
		cv::imwrite("./data/capture.png", drawImg);
	}
	if (ImGui::Button("Calibration")) show_calib_popup ^= true;
	if (ImGui::Button("ProCam System")) show_fb_popup ^= true;
	ImGui::NewLine();
	if (ImGui::Button("ImGui Demo Window")) show_test_window ^= true;
	ImGui::End();

	//	キャリブレーション用GUI
	if (show_calib_popup) {
		ImGui::SetNextWindowSize(ImVec2(100, 100), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("Calibration GUI");
		//	ColorCheckerによるカラーキャリブレーション
		if (ImGui::CollapsingHeader("Color")) {
			ImGui::Text(
				"Please right click at the corner of ColorChecker (+)\n"
				"1: upper-left (Brown)\n"
				"2: lower-left (White)\n"
				"3: lower-right (Black)\n"
				"4: upper-right (Lime)");
			if (ImGui::Button("Start")) {
				//	ColorCheckerのコーナー位置の初期化
				cc.corners.clear();
				clopping_mode = true;
			}
			if (clopping_mode) {
				ImGui::SameLine();
				ImGui::Text("Now Clopping...");
			}
			//	クロッピング終了後のデータ表示
			if (!clopping_mode && !cc.CCPatches.empty()) {
				if (ImGui::Button("Show Patches")) {
					cv::imshow("ColorChecker Image", cc.drawPatches(cc.CCPatches));
				}
			}
			//	キャリブレーション
			if (ImGui::CollapsingHeader("ProCam Color Calibration")) {
				//	データの読み込み・書き込み
				if (ImGui::Button("Load")) {
					calibrator.load(colorProfilePath);
					calibrator.showParams();
				}
				ImGui::SameLine();
				if (ImGui::Button("Save") && calibrator.model_calibrated) {
					calibrator.save(colorProfilePath);
				}
				//	ProCamキャリブレーション(クロッピング後に選択可能)
				if (!clopping_mode && !cc.corners.empty()) {
					if (ImGui::Button("Start Calibration")) {
						projectorWindow.hide();
						cv::Point pos(projectorWindow.winPos[0], projectorWindow.winPos[1]);
						calibrator.calibrate(flycap, cv::Rect(pos, projectorWindow.projSize), cc);
						projectorWindow.show();
					}
					if (ImGui::Button("Get White Gain")) {
						projectorWindow.hide();
						cv::Point pos(projectorWindow.winPos[0], projectorWindow.winPos[1]);
						calibrator.calibrateWhite(flycap, cv::Rect(pos, projectorWindow.projSize), cc);
						projectorWindow.show();
					}
				}
			}
		}
		if (ImGui::CollapsingHeader("Geometry")) {
			ImGui::Text(
			"ProCam geometry calibration by Gray-Code Patterns\n"
			);
			if (ImGui::Button("Start Pattern Projection")) {
				projectorWindow.hide();
				cv::Point pos(projectorWindow.winPos[0], projectorWindow.winPos[1]);
				geocalib.calibrate(flycap, cv::Rect(pos, projectorWindow.projSize));
				projectorWindow.show();
			}
			if (ImGui::Button("Show Maps")) {
				geocalib.showColoredMaps();
			}
		}
		if (ImGui::CollapsingHeader("Test")) {
			ImGui::Text(
				"Estimation Test on CPU\n"
			);
			ImGui::RadioButton("Diag", &removemode, 0);
			ImGui::RadioButton("PCA", &removemode, 1);
			ImGui::RadioButton("JD", &removemode, 2);
			//	動画像で確認
			if (ImGui::Button("Remove") && geocalib.calibrated && calibrator.reflectance_calibrated) {
				projectorWindow.hide();
				cv::Point pos(projectorWindow.winPos[0], projectorWindow.winPos[1]);
				//	テクスチャ画像読み込み
				auto texture = cv::imread(testTexturePath);
				int backlight = 40;
				texture.forEach<cv::Vec3b>([&](cv::Vec3b &c, const int* pos)->void {
					c[0] = MAX(backlight, c[0]);
					c[1] = MAX(backlight, c[1]);
					c[2] = MAX(backlight, c[2]);
				});
				projectorWindow.projImg = texture;
				cv::resize(projectorWindow.projImg, projectorWindow.projImg, projectorWindow.projSize);
				cv::Mat projImgCam;
				cv::remap(projectorWindow.projImg, projImgCam, geocalib.mapCPX, geocalib.mapCPY, cv::INTER_LINEAR);
				
				calibrator.estimate(flycap, cv::Rect(pos, projectorWindow.projSize),
					projectorWindow.projImg, projImgCam, removemode);
				
				projectorWindow.show();
				projectorWindow.projImg = cv::Mat(projectorWindow.projSize, CV_8UC3, cv::Scalar::all(255));
			}
			//	解析
			if (ImGui::Button("Test remover") && geocalib.calibrated && calibrator.reflectance_calibrated) {
				projectorWindow.hide();
				cv::Point pos(projectorWindow.winPos[0], projectorWindow.winPos[1]);
				//	テクスチャ画像読み込み
				auto texture = cv::imread(testTexturePath);
				int backlight = 100;
				texture.forEach<cv::Vec3b>([&](cv::Vec3b &c, const int* pos)->void {
					c[0] = MAX(backlight, c[0]);
					c[1] = MAX(backlight, c[1]);
					c[2] = MAX(backlight, c[2]);
				});
				projectorWindow.projImg = texture;
				cv::resize(projectorWindow.projImg, projectorWindow.projImg, projectorWindow.projSize);
				cv::Mat projImgCam;
				cv::remap(projectorWindow.projImg, projImgCam, geocalib.mapCPX, geocalib.mapCPY, cv::INTER_LINEAR);

				calibrator.testEstimation(flycap, cv::Rect(pos, projectorWindow.projSize),
					projectorWindow.projImg, projImgCam);

				projectorWindow.show();
				projectorWindow.projImg = cv::Mat(projectorWindow.projSize, CV_8UC3, cv::Scalar::all(255));
			}
		}
		ImGui::End();
	}

	//	システム評価用
	if (show_fb_popup) {
		ImGui::SetNextWindowSize(ImVec2(100, 100), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("ProCam System GUI");
		if (ImGui::Button("Initialize")) {
			fb.init(window, projectorWindow.window,
				calibrator, geocalib,
				vertexShaderDir.c_str(), removerShaderDir.c_str(), generatorShaderDir.c_str());
		}
		if (fb.initialised) {
			ImGui::RadioButton("Diag", &removemode, 0);
			ImGui::RadioButton("PCA", &removemode, 1);
			ImGui::RadioButton("JD", &removemode, 2);
			if (ImGui::Button("launch")) {
				auto img = cv::imread(testTexturePath);
				cv::resize(img, projectorWindow.projImg, projectorWindow.projSize);
				procam_mode = true;
			}
		}
		ImGui::End();
	}

	//	ImGui Test Window (Demo) 参照用
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
