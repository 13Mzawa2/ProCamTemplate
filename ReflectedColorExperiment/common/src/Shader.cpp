/********************************************************
Shader Class
GLSLシェーダを管理するライブラリ
参考：
Modern OpenGL Tutorial	http://www.opengl-tutorial.org/
床井研究室	http://marina.sys.wakayama-u.ac.jp/~tokoi/?date=20090827

Change 20160119:
シェーダのソースコード文字列を直接読む仕様を追加
*********************************************************/

#include "Shader.h"

Shader::Shader()
{
}

Shader::~Shader()
{
}

void Shader::readShaderCompile(GLuint shader, const char *file)
{
	FILE *fp;
	char *buf;
	GLsizei size, len;
	GLint compiled;
	// ファイルを開く
	fopen_s(&fp, file, "rb");
	if (!fp) printf("ファイルを開くことができません %s\n", file);
	//ファイルの末尾に移動し現在位置を得る
	fseek(fp, 0, SEEK_END);
	size = ftell(fp);//ファイルサイズを取得
	// ファイルサイズのメモリを確保
	buf = (GLchar *)malloc(size);
	if (buf == NULL){
		printf("メモリが確保できませんでした \n");
	}
	// ファイルを先頭から読み込む
	fseek(fp, 0, SEEK_SET);
	fread(buf, 1, size, fp);
	//シェーダオブジェクトにプログラムをセット
	glShaderSource(shader, 1, (const GLchar **)&buf, &size);
	//シェーダ読み込み領域の解放
	free(buf);
	fclose(fp);
	// シェーダのコンパイル
	glCompileShader(shader);
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
	if (compiled == GL_FALSE){
		printf("コンパイルできませんでした!!: %s \n ", file);
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &size);
		if (size > 0){
			buf = (char *)malloc(size);
			glGetShaderInfoLog(shader, size, &len, buf);
			printf(buf);
			free(buf);
		}
	}
}

void Shader::readInlineShaderCompile(GLuint shader, const char *source)
{
	char *infolog;
	GLsizei size, infolen;
	GLint compiled;
	//printf(source);
	size = strlen(source);
	//	シェーダオブジェクトにプログラムをセット
	glShaderSource(shader, 1, (const GLchar**)&source, &size);

	//	シェーダのコンパイル
	glCompileShader(shader);
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
	if (compiled == GL_FALSE){
		printf("コンパイルできませんでした!!: InlineShader \n ");
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &size);
		if (size > 0){
			infolog = (char *)malloc(size);
			glGetShaderInfoLog(shader, size, &infolen, infolog);
			printf(infolog);
			free(infolog);
		}
	}
}

void Shader::link(GLuint prog)
{
	GLsizei size, len;
	GLint linked;
	char *infoLog;
	glLinkProgram(prog);
	glGetProgramiv(prog, GL_LINK_STATUS, &linked);
	if (linked == GL_FALSE){
		printf("リンクできませんでした!! \n");
		glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &size);
		if (size > 0){
			infoLog = (char *)malloc(size);
			glGetProgramInfoLog(prog, size, &len, infoLog);
			printf(infoLog);
			free(infoLog);
		}
	}
}

void Shader::initGLSL(const char *vertexFile)
{
	// GPU,OpenGL情報
	printf("VENDOR= %s \n", glGetString(GL_VENDOR));
	printf("GPU= %s \n", glGetString(GL_RENDERER));
	printf("OpenGL= %s \n", glGetString(GL_VERSION));
	printf("GLSL= %s \n", glGetString(GL_SHADING_LANGUAGE_VERSION));
	//シェーダーオブジェクトの作成
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	//シェーダーの読み込みとコンパイル
	readShaderCompile(vertexShader, vertexFile);
	// シェーダプログラムの作成
	program = glCreateProgram();
	// シェーダオブジェクトをシェーダプログラムに関連付ける
	glAttachShader(program, vertexShader);
	// シェーダオブジェクトの削除
	glDeleteShader(vertexShader);
	// シェーダプログラムのリンク
	link(program);
}

void Shader::initGLSL(const char *vertexFile, const char *fragmentFile)
{
	//glewの初期化
	GLenum err = glewInit();
	if (err != GLEW_OK){
		printf("Error: %s\n", glewGetErrorString(err));
	}
	memcpy(&vertexFileName, &vertexFile, sizeof(vertexFile) / sizeof(char));
	memcpy(&fragmentFileName, &fragmentFile, sizeof(fragmentFile) / sizeof(char));
	// GPU,OpenGL情報
	printf("VENDOR= %s \n", glGetString(GL_VENDOR));
	printf("GPU= %s \n", glGetString(GL_RENDERER));
	printf("OpenGL= %s \n", glGetString(GL_VERSION));
	printf("GLSL= %s \n", glGetString(GL_SHADING_LANGUAGE_VERSION));
	//シェーダーオブジェクトの作成
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	//シェーダーの読み込みとコンパイル
	readShaderCompile(vertexShader, vertexFile);
	readShaderCompile(fragmentShader, fragmentFile);
	// プログラムオブジェクトの作成
	program = glCreateProgram();
	// シェーダオブジェクトをシェーダプログラムに関連付ける
	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);
	// シェーダオブジェクトの削除
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	// シェーダプログラムのリンク
	link(program);
}

void Shader::initInlineGLSL(const char *vertexSource, const char *fragmentSource)
{
	//glewの初期化
	GLenum err = glewInit();
	if (err != GLEW_OK){
		printf("Error: %s\n", glewGetErrorString(err));
	}
	//	擬似的にシェーダファイル名を作成
	char vertexFile[] = "InlineVertex", fragmentFile[] = "InlineFragment";
	memcpy(&vertexFileName, &vertexFile, sizeof(vertexFile) / sizeof(char));
	memcpy(&fragmentFileName, &fragmentFile, sizeof(fragmentFile) / sizeof(char));
	// GPU,OpenGL情報
	printf("VENDOR= %s \n", glGetString(GL_VENDOR));
	printf("GPU= %s \n", glGetString(GL_RENDERER));
	printf("OpenGL= %s \n", glGetString(GL_VERSION));
	printf("GLSL= %s \n", glGetString(GL_SHADING_LANGUAGE_VERSION));
	//シェーダーオブジェクトの作成
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	//シェーダーの読み込みとコンパイル
	readInlineShaderCompile(vertexShader, vertexSource);
	readInlineShaderCompile(fragmentShader, fragmentSource);
	// プログラムオブジェクトの作成
	program = glCreateProgram();
	// シェーダオブジェクトをシェーダプログラムに関連付ける
	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);
	// シェーダオブジェクトの削除
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	// シェーダプログラムのリンク
	link(program);
}

