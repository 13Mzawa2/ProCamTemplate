//======================================
//	OpenGLのテクスチャバッファ関連
//	シェーダ関連の簡略化のためのクラスなので描画機能は無し
//	cv::Matを描画したいならGLImageを使うべし
//======================================

#pragma once
#include <OpenGLHeader.h>
#include <opencv2\opencv.hpp>

class GLTexture
{
public:
	enum TextureType {
		TYPE_8UC3 = 0,
		TYPE_32F = 1
	};
	GLuint textureID;
	GLuint samplerID;
	GLint unitNum;
	TextureType textureType;
	cv::Size size;
	
	//	Pixel Buffer Object
	//		ピクセルデータのやり取りの高速化に使用
	//		Direct Memory Access (DMA)を用いてGPUとデータをやり取りするため，CPUサイクルに影響されずに高速転送できる
	//	PACK: Frame Buffer -> PBO  glReadPixels()など
	//	UNPACK: PBO -> Frame Buffer  glDrawPixels(), glTexSubImage2D()など
	GLuint pbo;

	GLTexture() {};
	~GLTexture() {};

	//	OpenGL用テクスチャの生成
	//	@param:
	//		type: 定義した型
	//		texUnitNum: テクスチャユニット番号
	//		textureSize:　生成するテクスチャのサイズ
	void create(TextureType type, GLuint texUnitNum, cv::Size textureSize)
	{
		//	テクスチャオブジェクトの生成（IDの作成）
		glGenTextures(1, &textureID);
		//	設定するテクスチャの指定
		glBindTexture(GL_TEXTURE_RECTANGLE, textureID);

		//	テクスチャ領域の確保
		GLint internalformat, format, bittype;
		getType(type, internalformat, format, bittype);
		glTexImage2D(GL_TEXTURE_RECTANGLE, 0, internalformat,
			textureSize.width, textureSize.height, 0, format, bittype, NULL);

		//	テクスチャの拡大縮小方法の設定：線形補間
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		//	テクセル範囲外にアクセスした場合の設定
		//		GL_CLAMP_TO_BORDER: 境界色で塗りつぶす
		GLfloat border[] = {0,0,0,0 };
		glTexParameterfv(GL_TEXTURE_RECTANGLE, GL_TEXTURE_BORDER_COLOR, border);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

		//	PBOの作成
		createPBO(type, textureSize);

		//	設定の保存
		textureType = type;
		unitNum = texUnitNum;
		size = textureSize;
	}

	//	PBOの生成
	void createPBO(TextureType type, cv::Size textureSize)
	{
		glGenBuffers(1, &pbo);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);

		int buffersize = getBufferSize(type, textureSize);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, buffersize, NULL, GL_STREAM_READ);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}

	//	画像をGPUのテクスチャユニットへ転送
	void upload(cv::Mat img)
	{
		//	テクスチャユニットの指定
		glActiveTexture(GL_TEXTURE0 + unitNum);
		glBindTexture(GL_TEXTURE_RECTANGLE, textureID);
		//	テクスチャの転送
		GLint internalformat, format, bittype;
		getType(textureType, internalformat, format, bittype);
		glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, img.cols, img.rows, format, bittype, img.data);
		glActiveTexture(GL_TEXTURE0);
	}

	//	PBOを用いたテクスチャ操作
	//	画像をGPUのテクスチャユニットへPBOを介して転送
	void uploadPBO(cv::Mat img)
	{
		//	テクスチャユニットの指定
		glActiveTexture(GL_TEXTURE0 + unitNum);
		glBindTexture(GL_TEXTURE_RECTANGLE, textureID);

		//	データサイズの計算
		GLint internalformat, format, bittype;
		getType(textureType, internalformat, format, bittype);
		int buffersize = getBufferSize(textureType, img.size());

		//	PBOの指定
		//	GPUのTextureObjectへの転送なのでUNPACK
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
		//	CPUからPBOへの画像アップロード
		//	GPUが処理を終えるまで待機状態に入るのを避けるため，NULLポインタで呼んだ後に
		//	glMapBuffer()を呼ぶことで，強制的にメモリ領域を確保させる
		glBufferData(GL_PIXEL_UNPACK_BUFFER, buffersize, 0, GL_STREAM_DRAW);
		//	CPU領域にPBOを書き込み専用で展開
		GLubyte* pboptr = (GLubyte*)glMapBufferARB(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
		if (pboptr) {
			//	画像データをPBOへコピー
			std::memcpy(pboptr, img.data, buffersize);
			//	マップの解除
			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
		}

		//	PBOからTextureObjectへ転送
		glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, img.cols, img.rows, format, bittype, 0);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		glActiveTexture(GL_TEXTURE0);
	}

	//	別の書き込み済みPBOを渡して転送
	void uploadPBO(GLuint _pbo)
	{
		//	テクスチャユニットの指定
		glActiveTexture(GL_TEXTURE0 + unitNum);
		glBindTexture(GL_TEXTURE_RECTANGLE, textureID);

		//	データサイズの計算
		GLint internalformat, format, bittype;
		getType(textureType, internalformat, format, bittype);
		int buffersize = getBufferSize(textureType, size);

		//	PBOの指定
		//	GPUのTextureObjectへの転送なのでUNPACK
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, _pbo);
		//	PBOからTextureObjectへ転送
		glTexSubImage2D(GL_TEXTURE_RECTANGLE, 0, 0, 0, size.width, size.height, format, bittype, 0);
		
		//	非アクティブ化
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		glActiveTexture(GL_TEXTURE0);
	}

	//	任意のFrameBufferObjectからPBOへコピー
	//	PBOからメモリコピーは別
	void downloadPBO(GLuint buffID)
	{
		glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, buffID);
		//	PBOへの書き込み
		GLint internalformat, format, bittype;
		getType(textureType, internalformat, format, bittype);
		glReadPixels(0, 0, size.width, size.height, format, bittype, 0);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	}

	//	書き込み済みPBOからCPU領域にコピー
	void readPixelsPBO(cv::Mat &img)
	{
		img = cv::Mat(size, CV_8UC3);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo);
		GLubyte *pboptr = (GLubyte*)glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
		if (pboptr) {
			int buffersize = getBufferSize(textureType, size);
			std::memcpy(img.data, pboptr, buffersize);
			glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
		}
		glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	}

	//	シェーダプログラムからテクスチャサンプラーのIDを取得
	GLuint getSampler(GLuint program, const GLchar *name)
	{
		samplerID = glGetUniformLocation(program, name);
		return samplerID;
	}

	//	シェーダのUniform変数にサンプラーとテクスチャユニットを登録
	void setUniformSampler()
	{
		glUniform1i(samplerID, unitNum);
	}
	void setUniformsampler(GLuint _samplerID)
	{
		glUniform1i(_samplerID, unitNum);
	}

private:
	//	TextureTypeの翻訳
	void getType(TextureType type, GLint &internalformat, GLint &format, GLint &bittype)
	{
		switch (type) {
		case TYPE_32F:
			internalformat = GL_R32F;
			format = GL_RED;
			bittype = GL_FLOAT;
			break;
		default:	//	TYPE_8UC3
			internalformat = GL_RGB;
			format = GL_BGR;
			bittype = GL_UNSIGNED_BYTE;
		}
	}

	//	バッファサイズの計算
	int getBufferSize(TextureType type, cv::Size size)
	{
		int buffersize;
		switch (type) {
		case TYPE_32F:
			buffersize = size.width*size.height * sizeof(float);
			break;
		default:	// TYPE_8UC3
			buffersize = size.width*size.height * sizeof(unsigned char);
		}
	}
	

};