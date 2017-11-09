//--------------------------------------------
//	GLFWによって作成するウィンドウの基底クラス
//	基本的に自作ウィンドウはこれを継承して作成する
//--------------------------------------------

#pragma once

#include <OpenGLHeader.h>
#include <GLImage.h>

class OpenGLWindow {
protected:

public:
	//	メンバ変数
	GLFWwindow *window;
	GLImage glImg;
	glm::uvec2 winSize;
	glm::uvec2 winPos;
	int monitorID;

	//	コンストラクタ
	OpenGLWindow() {};
	OpenGLWindow(glm::uvec2 size, glm::uvec2 pos) :
		winSize(size), winPos(pos) {};
	//	デストラクタ
	~OpenGLWindow() { close(); };

	//	ウィンドウ生成
	//	GLFWの初期化後，GLEW初期化前に行う
	int create(const char* title, glm::uvec2 size, glm::uvec2 pos, bool border_frame = true)
	{
		//	メンバ変数へ代入
		winSize = size;
		winPos = pos;

		//	GLFWのwindow設定
		glfwWindowHint(GLFW_SAMPLES, 4);				//	4x アンチエイリアス
		glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);		//	リサイズ不可
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);	//	OpenGL 4.3を利用
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);	//	OpenGLのコア機能のみ使用
		glfwWindowHint(GLFW_DECORATED, border_frame);

		window = glfwCreateWindow(
			winSize[0], winSize[1],
			title, NULL, NULL
		);
		if (window == NULL) {
			std::cerr << title << "の生成に失敗しました．" << std::endl;
			glfwTerminate();
			return EXIT_FAILURE;
		}
		glfwSetWindowPos(window, pos[0], pos[1]);

		return EXIT_SUCCESS;
	}

	//	初期化
	//	glImageの初期化時にGLEWが必要になるので，GLEW初期化後に行う
	void baseInit(void)
	{
		glfwMakeContextCurrent(window);

		glfwSwapInterval(0);
		glClearColor(1.0, 1.0, 1.0, 1.0);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glEnable(GL_LESS);			//	描画されない面のレンダリング省略

		//	glImageの初期化
		glImg.init(window);
	}
	//	初期化処理をカスタムしたい場合はこの関数をoverrideすべし
	void init(void)
	{
		baseInit();
	}

	//	描画処理（具体的な処理はoverrideで書く）
	void draw()
	{
		glfwMakeContextCurrent(window);
	}
	//	描画状態を画面に反映(swapbuffer)
	void swapBuffers()
	{
		glfwSwapBuffers(window);
	}

	//	閉じるときの処理
	void close()
	{
	}
};