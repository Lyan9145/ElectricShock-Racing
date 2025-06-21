#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file preprocess.cpp
 * @author Leo
 * @brief 图像预处理：RGB转灰度图，图像二值化
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"

using namespace cv;
using namespace std;

/**
**[1] 读取视频
**[2] 图像二值化
*/
class Preprocess
{
public:
	/**
	 * @brief 图像矫正参数初始化
	 *
	 */
	Preprocess()
	{
		// 读取xml中的相机标定参数
		cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参矩阵
		distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));	// 相机的畸变矩阵
		FileStorage file;
		if (file.open("../res/calibration/valid/calibration.xml", FileStorage::READ)) // 读取本地保存的标定文件
		{
			file["cameraMatrix"] >> cameraMatrix;
			file["distCoeffs"] >> distCoeffs;
			cout << "相机矫正参数初始化成功!" << endl;
			enable = true;
		}
		else
		{
			cout << "打开相机矫正参数失败!!!" << endl;
			enable = false;
		}
	};

	/**
     * @brief 图像尺寸标准化：确保输入图像为标准尺寸320x240
     *
     * @param frame 输入原始帧
     * @return Mat  标准化尺寸后的图像
     */
    Mat resizeImage(Mat &frame)
    {
        Mat resizedFrame;
        
        // 如果图像尺寸已经是标准尺寸，直接返回
        if (frame.cols == COLSIMAGE && frame.rows == ROWSIMAGE)
        {
            return frame;
        }
        
        // 将图像缩放到标准尺寸320x240
		resizedFrame = Mat(ROWSIMAGE, COLSIMAGE, frame.type());
		
		float scaleX = (float)frame.cols / COLSIMAGE;
		float scaleY = (float)frame.rows / ROWSIMAGE;
		
		if (frame.channels() == 3) {
			for (int y = 0; y < ROWSIMAGE; ++y) {
			uchar* dst_row = resizedFrame.ptr<uchar>(y);
			float src_y = y * scaleY;
			int y0 = (int)src_y;
			int y1 = min(y0 + 1, frame.rows - 1);
			float dy = src_y - y0;
			
			for (int x = 0; x < COLSIMAGE; ++x) {
				float src_x = x * scaleX;
				int x0 = (int)src_x;
				int x1 = min(x0 + 1, frame.cols - 1);
				float dx = src_x - x0;
				
				uchar* p00 = frame.ptr<uchar>(y0) + x0 * 3;
				uchar* p01 = frame.ptr<uchar>(y0) + x1 * 3;
				uchar* p10 = frame.ptr<uchar>(y1) + x0 * 3;
				uchar* p11 = frame.ptr<uchar>(y1) + x1 * 3;
				
				for (int c = 0; c < 3; ++c) {
				float val = p00[c] * (1 - dx) * (1 - dy) + 
					   p01[c] * dx * (1 - dy) + 
					   p10[c] * (1 - dx) * dy + 
					   p11[c] * dx * dy;
				dst_row[x * 3 + c] = (uchar)val;
				}
			}
			}
		} else {
			for (int y = 0; y < ROWSIMAGE; ++y) {
			uchar* dst_row = resizedFrame.ptr<uchar>(y);
			float src_y = y * scaleY;
			int y0 = (int)src_y;
			int y1 = min(y0 + 1, frame.rows - 1);
			float dy = src_y - y0;
			
			for (int x = 0; x < COLSIMAGE; ++x) {
				float src_x = x * scaleX;
				int x0 = (int)src_x;
				int x1 = min(x0 + 1, frame.cols - 1);
				float dx = src_x - x0;
				
				uchar p00 = frame.at<uchar>(y0, x0);
				uchar p01 = frame.at<uchar>(y0, x1);
				uchar p10 = frame.at<uchar>(y1, x0);
				uchar p11 = frame.at<uchar>(y1, x1);
				
				float val = p00 * (1 - dx) * (1 - dy) + 
					   p01 * dx * (1 - dy) + 
					   p10 * (1 - dx) * dy + 
					   p11 * dx * dy;
				dst_row[x] = (uchar)val;
			}
			}
		}
        
        return resizedFrame;
    }

	/**
	 * @brief 图像二值化
	 *
	 * @param frame	输入原始帧
	 * @return Mat	二值化图像
	 */
	Mat binaryzation(Mat &frame)
	{
		Mat imageGray, imageBinary;

		cvtColor(frame, imageGray, COLOR_BGR2GRAY); // RGB转灰度图

		threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU); // OTSU二值化方法

		return imageBinary;
	}

	/**
	 * @brief 矫正图像
	 *
	 * @param imagesPath 图像路径
	 */
	Mat correction(Mat &image)
	{
		if (enable)
		{
			Size sizeImage; // 图像的尺寸
			sizeImage.width = image.cols;
			sizeImage.height = image.rows;

			Mat mapx = Mat(sizeImage, CV_32FC1);	// 经过矫正后的X坐标重映射参数
			Mat mapy = Mat(sizeImage, CV_32FC1);	// 经过矫正后的Y坐标重映射参数
			Mat rotMatrix = Mat::eye(3, 3, CV_32F); // 内参矩阵与畸变矩阵之间的旋转矩阵

			// 采用initUndistortRectifyMap+remap进行图像矫正
			initUndistortRectifyMap(cameraMatrix, distCoeffs, rotMatrix, cameraMatrix, sizeImage, CV_32FC1, mapx, mapy);
			Mat imageCorrect = image.clone();
			remap(image, imageCorrect, mapx, mapy, INTER_LINEAR);

			// 采用undistort进行图像矫正
			//  undistort(image, imageCorrect, cameraMatrix, distCoeffs);

			return imageCorrect;
		}
		else
		{
			return image;
		}
	}

private:
	bool enable = false; // 图像矫正使能：初始化完成
	Mat cameraMatrix;	 // 摄像机内参矩阵
	Mat distCoeffs;		 // 相机的畸变矩阵
};
