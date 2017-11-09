#include "ControlWindow.h"

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

}

void ControlWindow::draw(void)
{
	cv::Mat cameraImg = flycap.readImage();

	glfwMakeContextCurrent(window);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//	�L���v�`���摜�`��
	glImg.draw(cameraImg);
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
	ImGui::NewLine();
	if (ImGui::Button("ImGui Demo Window")) show_test_window ^= true;
	ImGui::End();

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
