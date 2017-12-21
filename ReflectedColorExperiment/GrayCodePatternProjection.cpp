#include "GrayCodePatternProjection.h"

using namespace cv;
using namespace std;

GrayCodePatternProjection::GrayCodePatternProjection()
{
}

GrayCodePatternProjection::GrayCodePatternProjection(Size projSize, Size camSize)
{
	init(projSize, camSize);
}

//	����������
//	����ꂽ�v���W�F�N�^�E�J�����̉�p����ɃO���C�R�[�h�p�^�[���𐶐�����
void GrayCodePatternProjection::init(cv::Size projSize, cv::Size camSize)
{
	projectorSize = projSize; cameraSize = camSize;
	makeGrayCodePatternLists();
	makeGrayCodeImages();
}

GrayCodePatternProjection::~GrayCodePatternProjection()
{
}

//	Convert Binary Code to Gray Code
//	Example:
//	bin = 0x4c = 0b01001100
//
//	  0b01001100
//	^ 0b00100110 (= bin >> 1)
//	-------------------------
//	  0b01101010 = 0x6a = gray
int GrayCodePatternProjection::bin2gray(int bin)
{
	return bin ^ (bin >> 1);
}

//	Convert Gray Code to Binary Code
//	Example:
//	gray = 0x6a = 0b01101010
//	
//	bin = gray       = 0b01101010
//	mask = gray >> 1 = 0b00110101 (begin)
//	bin = bin ^ mask = 0b01011111
//	mask = mask >> 1 = 0b00011010
//	bin = bin ^ mask = 0b01000101
//	mask = mask >> 1 = 0b00001101
//	bin = bin ^ mask = 0b01001000
//	mask = mask >> 1 = 0b00000110
//	bin = bin ^ mask = 0b01001110
//	mask = mask >> 1 = 0b00000011
//	bin = bin ^ mask = 0b01001101
//	mask = mask >> 1 = 0b00000001
//	bin = bin ^ mask = 0b01001100 = 0x4c = bin
//	mask = mask >> 1 = 0b00000000 (end)
//	bin = bin ^ mask = 0b01001100
int GrayCodePatternProjection::gray2bin(int gray)
{
	int bin = gray, mask = gray >> 1;
	for (; mask != 0; mask = mask >> 1)
		bin = bin ^ mask;
	return bin;
}

//	�O���C�R�[�h�p�^�[����\���s��̍쐬
//	�s��̊e�s���O���C�R�[�h�p�^�[����\��
//
//	Output = {patternListW, patternListH}
//	Example:
//	projSize = [16, 6]
//	
//	patternListW =
//		[0000000011111111] -> pattern 1
//		[0000111111110000] -> pattern 2
//		[0011110000111100] -> pattern 3
//		[0110011001100110] -> pattern 4
//
//	patternListH = 
//		[000011] -> pattern 1
//		[001111] -> pattern 2
//		[011001] -> pattern 3
void GrayCodePatternProjection::makeGrayCodePatternLists(void)
{
	//	�ő�r�b�g��l�̃J�E���g
	int lw = 0, lh = 0;		//	�v���W�F�N�^�摜�T�C�Y���ꂼ��̍ő�r�b�g��
	for (int x = projectorSize.width - 1; x > 0; x = x >> 1) lw++;
	for (int y = projectorSize.height - 1; y > 0; y = y >> 1) lh++;
	//	0 ~ projSize.width (heigt) �̃O���C�R�[�h���r�b�g�����čs��Ɋi�[
	//	�s�����o���΃p�^�[���ɂȂ�悤�ɂ���
	patternListW = Mat(Size(projectorSize.width, lw), CV_8UC1);
	patternListH = Mat(Size(projectorSize.height, lh), CV_8UC1);
	for (int i = 0; i < projectorSize.width; i++) {
		for (int j = lw; j > 0; j--) {			//	�ŏ��bit���璲�ׂĂ���
												//	�O���C�R�[�h�� i �� j bit��(�i�X�������Ȃ�)��1�ł����1���C�����łȂ����0����f�l�ɓ����
			patternListW.at<uchar>(lw - j, i) = (bin2gray(i) & (1 << (j - 1))) && 1;
		}
	}
	for (int i = 0; i < projectorSize.height; i++) {
		for (int j = lh; j > 0; j--) {			//	�ŏ��bit���璲�ׂĂ���
												//	�O���C�R�[�h�� i �� j bit�ڂ�1�ł����1���C�����łȂ����0������
			patternListH.at<uchar>(lh - j, i) = (bin2gray(i) & (1 << (j - 1))) && 1;
		}
	}

}

//	�p�^�[�����X�g����O���C�R�[�h�p�^�[���𐶐�
//	isVertical: true = �c�����Cfalse = ������
//	1�Ԗڂ��ŏ�ʃr�b�g
void GrayCodePatternProjection::makeGrayCodeImages(void)
{
	//	�p�^�[���z���������
	patternsW.clear(); patternsWN.clear();
	patternsH.clear(); patternsHN.clear();

	for (int i = 0; i < patternListW.rows; i++)
	{
		Mat pattern(projectorSize, CV_8UC1);
		resize(255 * patternListW.row(i), pattern, projectorSize, INTER_NEAREST);
		patternsW.push_back(pattern);
		patternsWN.push_back(~pattern);		//	�f�R�[�h���艻�̂��߂̃l�K
	}
	for (int i = 0; i < patternListH.rows; i++)
	{
		Mat pattern(projectorSize, CV_8UC1);
		resize(255 * patternListH.row(i).t(), pattern, projectorSize, INTER_NEAREST);
		patternsH.push_back(pattern);
		patternsHN.push_back(~pattern);		//	�f�R�[�h���艻�̂��߂̃l�K
	}

}

//	�}�X�N�摜���쐬����
void GrayCodePatternProjection::getMask(int thresh)
{
}

//	�B�e�����p�^�[����ǂݍ���
//	cap�� w[0], wn[0], w[1], wn[1], ..., wn[wrows], h[0], hn[0], ..., hn[hrows]�@�̏�
void GrayCodePatternProjection::loadCapPatterns(vector<Mat> cap)
{
	//	�p�^�[���z��̏�����
	captureW.clear(); captureWN.clear();
	captureH.clear(); captureHN.clear();
	//	���e�摜�̓ǂݍ���
	for (int i = 0; i < patternListW.rows; i++)
	{
		int it = 2 * i;
		if (cap[2 * i].channels() != 1)
			cvtColor(cap[it], cap[it], CV_BGR2GRAY);
		captureW.push_back(cap[it]);
		cv::imwrite("images/graycodeW" + to_string(i) + ".jpg", cap[it]);

		if (cap[it + 1].channels() != 1)
			cvtColor(cap[it + 1], cap[it + 1], CV_BGR2GRAY);
		captureWN.push_back(cap[it + 1]);
		cv::imwrite("images/graycodeWN" + to_string(i) + ".jpg", cap[it + 1]);
	}
	for (int i = 0; i < patternListH.rows; i++)
	{
		int it = 2 * patternListW.rows + 2 * i;
		if (cap[it].channels() != 1)
			cvtColor(cap[it], cap[it], CV_BGR2GRAY);
		captureH.push_back(cap[it]);
		cv::imwrite("images/graycodeH" + to_string(i) + ".jpg", cap[it]);

		if (cap[it + 1].channels() != 1)
			cvtColor(cap[it + 1], cap[it + 1], CV_BGR2GRAY);
		captureHN.push_back(cap[it + 1]);
		cv::imwrite("images/graycodeHN" + to_string(i) + ".jpg", cap[it + 1]);
	}
}

//	�ǂݍ��񂾃p�^�[������ǂ��ă}�b�v��
//	unsigned int16�ō��W�l�����
void GrayCodePatternProjection::decodePatterns()
{
	Mat xtemp(cameraSize, CV_32FC1);
	Mat ytemp(cameraSize, CV_32FC1);
	int lw = patternListW.rows, lh = patternListH.rows;
	//cout << "patterns: x=" << lw << ", y=" << lh << endl;
	for (int i = 0; i < xtemp.rows; i++)
	{
		for (int j = 0; j < xtemp.cols; j++)
		{	//	��f���Ɏ��s
			//	x����
			int xgray = 0;
			for (int k = lw; k > 0; k--)
			{	//	k bit�ڂ̉�f�̋P�x���l�K���傫�����1�C�����łȂ����0��Ή�����r�b�g�ŗ��Ă�
				int bit = (captureW[lw - k].at<uchar>(i, j) - captureWN[lw - k].at<uchar>(i, j)) > 0 ? 1 : 0;
				bit = bit << (k - 1);
				xgray = xgray | bit;
			}
			int xbin = gray2bin(xgray);
			//	y����
			int ygray = 0;
			for (int k = lh; k > 0; k--)
			{	//	k bit�ڂ̉�f�̋P�x���l�K���傫�����1�C�����łȂ����0��Ή�����r�b�g�ŗ��Ă�
				int bit = (captureH[lh - k].at<uchar>(i, j) - captureHN[lh - k].at<uchar>(i, j)) > 0 ? 1 : 0;
				bit = bit << (k - 1);
				ygray = ygray | bit;
			}
			int ybin = gray2bin(ygray);
			// temp�փf�R�[�h�������W�l����
			xtemp.at<float>(i, j) = (float)xbin;
			ytemp.at<float>(i, j) = (float)ybin;
		}
	}
	mapX = xtemp.clone();
	mapY = ytemp.clone();
}

int GrayCodePatternProjection::showMaps()
{
	Mat x, y;
	mapX.convertTo(x, CV_8UC1, 255.0 / (projectorSize.width - 1));
	mapY.convertTo(y, CV_8UC1, 255.0 / (projectorSize.height - 1));
	cv::imshow("x.png", x);
	cv::imshow("y.png", y);
	cv::imwrite("images/x.png", x);
	cv::imwrite("images/y.png", y);
	int c = cv::waitKey(0);
	cv::destroyWindow("x.png");
	cv::destroyWindow("y.png");
	return c;
}
