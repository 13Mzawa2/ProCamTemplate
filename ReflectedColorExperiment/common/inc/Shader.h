/********************************************************
Shader Class
GLSLシェーダを管理するライブラリ
参考：
Modern OpenGL Tutorial	http://www.opengl-tutorial.org/
床井研究室	http://marina.sys.wakayama-u.ac.jp/~tokoi/?date=20090827

Change 20160119:
シェーダのソースコード文字列を直接読む仕様を追加
*********************************************************/

#pragma once

#include "OpenGLHeader.h"

class Shader
{
protected:
	GLuint vertexShader, fragmentShader;		//	シェーダオブジェクト
	const char *vertexFileName, *fragmentFileName;		//	シェーダファイル名
	void readShaderCompile(GLuint shader, const char *file);	//	.shaderのコンパイル
	void readInlineShaderCompile(GLuint shader, const char *source);	//	シェーダのインラインソースをコンパイル
	void link(GLuint prog);		//	コンパイルしたshaderをリンクする
public:
	GLuint program;			//	シェーダプログラム
	Shader();				//	コンストラクタ
	~Shader();				//	デストラクタ
	Shader &operator=(Shader &_s)	//	コピーコンストラクタ
	{
		initGLSL(_s.vertexFileName, _s.fragmentFileName);
		return *this;
	}
	//	初期化
	//	フラグメントシェーダーの有無で分ける
	void initGLSL(const char *vertexFile);
	void initGLSL(const char *vertexFile, const char *fragmentFile);
	void initInlineGLSL(const char *vertexSource, const char *fragmentSource);
	//	有効化
	void enable(){ glUseProgram(program); }
	void disable(){ glUseProgram(0); }

};

