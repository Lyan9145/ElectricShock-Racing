#pragma once
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "common.hpp"

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
    Preprocess();

    Mat resizeImage(Mat &frame);

    Mat binaryzation(Mat &frame);

    Mat correction(Mat &image);

private:
    bool enable = false; // 图像矫正使能：初始化完成
    Mat cameraMatrix;    // 摄像机内参矩阵
    Mat distCoeffs;      // 相机的畸变矩阵
    // Scaled camera matrix for 320x240 input images.
    const cv::Mat PRECOMPUTED_MTX_LOW_RES = (cv::Mat_<double>(3, 3) << 186.5517098331297, 0.0, 167.08490140903737, 0.0, 185.7452458436099, 123.59342696925486, 0.0, 0.0, 1.0);

    // Distortion coefficients (resolution-independent).
    const cv::Mat PRECOMPUTED_DIST = (cv::Mat_<double>(1, 5) << -0.3692941216884664, 0.14635301101349377, -0.0012768867661858236, 0.0008188111919998465, -0.027400821365335705);

    // Optimal new camera matrix for undistortion at 320x240.
    const cv::Mat PRECOMPUTED_NEW_MTX_LOW_RES = (cv::Mat_<double>(3, 3) << 99.59942626953125, 0.0, 170.52866866446857, 0.0, 105.8468017578125, 125.79756609980541, 0.0, 0.0, 1.0);

    // Region of Interest (ROI) for cropping black borders at 320x240.
    const cv::Rect PRECOMPUTED_ROI_LOW_RES(81, 55, 171, 137);
};
