/********************************************************
OpenGL Image with OpenCV
GLFWでOpenCVのcv::Matを背景描画するためのクラス

How to Use:
1. メインループに入る前にGLImageを生成
2. 描画したいGLFWwindowを与えてGLImageを初期化
3. メインループ内で次の様に書く(ex. mainWindowの背景にframeImgを描画)

//	Change Current Window
glfwMakeContextCurrent(mainWindow);
//	Clear Buffer Bits
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	Draw Image
glImg.draw(frameImg);		//	<- Only render to back buffer
//	Clear Depth Bits (so you can overwride CG on frameImg)
glClear(GL_DEPTH_BUFFER_BIT);
//	Draw your CG
//	End Draw
glfwSwapBuffers(mainWindow);

Change 20180506:
・PBOをダブルバッファリングにした（そんなに早くry）

Change 20171228:
・PBOによる高速化を図る（そんなに早くならなかった）

Change 20170202:
・flipをシェーダ内で実行するように変更

Change 20160119:
・コンストラクタで初期化できるようにした
・コメント大幅追加
・GLSLをインライン化して外部ファイルを不要にした


*********************************************************/

#pragma once

#include "OpenGLHeader.h"
#include "Shader.h"
#include <opencv2\opencv.hpp>

class GLImage
{
private:
	GLFWwindow *imgWindow;
	GLuint vao;		//	頂点配列オブジェクト
	GLuint vbo;		//	頂点バッファオブジェクト
	GLuint image;	//	テクスチャオブジェクト
	GLuint imageLoc;//	オブジェクトの場所
	Shader s;		//	シェーダ

	//	Pixel Buffer Object
	//		ピクセルデータのやり取りの高速化に使用
	//		Direct Memory Access (DMA)を用いてGPUとデータをやり取りするため，CPUサイクルに影響されずに高速転送できる
	//	PACK: Frame Buffer -> PBO  glReadPixels()など
	//	UNPACK: PBO -> Frame Buffer  glDrawPixels(), glTexSubImage2D()など
	GLuint pbo[2];
	int pbo_buffersize;
	int index;
	int nextindex;

	//	バーテックスシェーダ
	const char *vertexSource =
		"#version 330 core \n"
		"layout(location = 0) in vec4 pv;\n"
		"void main(void)\n" 
		"{\n" 
		"	gl_Position = pv;\n" 
		"}\n";
	//	フラグメントシェーダ
	const char *fragmentSource =
		"#version 330 core \n"
		"uniform sampler2DRect image;\n"
		"uniform vec2 resolution;\n"
		"layout(location = 0) out vec4 fc;\n"
		"void main(void)\n"
		"{\n"
		"	fc = texture(image, vec2(gl_FragCoord.x, resolution.y-gl_FragCoord.y));\n"
		"}\n";
	int vertices;

public:
	GLImage()
	{
	}
	GLImage(GLFWwindow *window)
	{
		init(window);
	}
	void init(GLFWwindow *window)
	{
		int w, h;
		glfwMakeContextCurrent(window);
		glfwGetWindowSize(window, &w, &h);
		imgWindow = window;

		// 頂点配列オブジェクト
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);

		// 頂点バッファオブジェクト
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);

		// [-1, 1] の正方形
		static const GLfloat position[][2] =
		{
			{ -1.0f, -1.0f },
			{ 1.0f, -1.0f },
			{ 1.0f, 1.0f },
			{ -1.0f, 1.0f }
		};
		vertices = sizeof(position) / sizeof (position[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(position), position, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);

		//	テクスチャ
		glGenTextures(1, &image);
		glBindTexture(GL_TEXTURE_RECTANGLE, image);
		glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGB, w, h, 0, GL_BGR, GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

		// PBOの生成
		// PBOでダブルバッファリングするため2個作成する
		pbo_buffersize = w*h * 3 * sizeof(GLubyte);
		glGenBuffers(2, pbo);
		// 1個目
		index = 0;
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo[index]);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, pbo_buffersize, NULL, GL_STREAM_READ);
		// 2個目
		nextindex = 1;
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo[nextindex]);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, pbo_buffersize, NULL, GL_STREAM_READ);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

		//	シェーダのロード
		s.initInlineGLSL(vertexSource, fragmentSource);
		imageLoc = glGetUniformLocation(s.program, "image");
	}
	void draw(cv::Mat frame)
	{
		glfwMakeContextCurrent(imgWindow);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		// indexの更新
		index = (index + 1) % 2;
		nextindex = (index + 1) % 2;

		// 書き込み済みのPBOをテクスチャ領域に転送
		// PBO指定状態でglTexSubImage2D()をデータポインタを渡さずに実行するとPBOから転送される
		glBindTexture(GL_TEXTURE_RECTANGLE, image);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo[index]);
		glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, 0);
		
		//	次のPBOにデータを書き込み
		//	GPUのTextureObjectへの転送なのでUNPACK
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo[nextindex]);
		//	CPUからPBOへの画像アップロード
		//	GPUが処理を終えるまで待機状態に入るのを避けるため，NULLポインタで呼んだ後に
		//	glMapBuffer()を呼ぶことで，強制的にメモリ領域を確保させる
		glBufferData(GL_PIXEL_UNPACK_BUFFER, pbo_buffersize, 0, GL_STREAM_DRAW);
		//	CPU領域にPBOを書き込み専用で展開
		GLubyte* pboptr = (GLubyte*)glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
		if (pboptr) {
			//	画像データをPBOへコピー
			std::memcpy(pboptr, frame.data, pbo_buffersize);
			//	マップの解除
			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
		}

		//	PBOの指定を解除
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

		// レンダリング開始
		// シェーダプログラムの使用開始
		s.enable();

		// uniform サンプラの指定
		glUniform1i(imageLoc, 0);

		// 解像度の指定
		glUniform2f(glGetUniformLocation(s.program, "resolution"), frame.cols, frame.rows);

		// テクスチャユニットとテクスチャの指定
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_RECTANGLE, image);

		// 描画に使う頂点配列オブジェクトの指定
		glBindVertexArray(vao);

		// 図形の描画
		glDrawArrays(GL_TRIANGLE_FAN, 0, vertices);

		// 頂点配列オブジェクトの指定解除
		glBindVertexArray(0);

		// シェーダプログラムの使用終了
		s.disable();
	}
};