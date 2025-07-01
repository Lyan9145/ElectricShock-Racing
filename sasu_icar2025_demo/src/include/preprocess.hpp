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
    // Scaled camera matrix for 640x480 input images.
    const cv::Mat PRECOMPUTED_MTX_LOW_RES = (cv::Mat_<double>(3, 3) << 373.1034196662594, 0.0, 334.16980281807474, 0.0, 371.4904916872198, 247.18685393850973, 0.0, 0.0, 1.0);

    // Distortion coefficients (resolution-independent).
    const cv::Mat PRECOMPUTED_DIST = (cv::Mat_<double>(1, 5) << -0.3692941216884664, 0.14635301101349377, -0.0012768867661858236, 0.0008188111919998465, -0.027400821365335705);

    // Optimal new camera matrix for undistortion at 640x480.       
    const cv::Mat PRECOMPUTED_NEW_MTX_LOW_RES = (cv::Mat_<double>(3, 3) << 199.51107788085938, 0.0, 341.59191241490043, 0.0, 212.136474609375, 252.12147881343844, 0.0, 0.0, 1.0);

    // Region of Interest (ROI) for cropping black borders at 640x480.
    const cv::Rect PRECOMPUTED_ROI_LOW_RES = (cv::Rect(163, 111, 342, 274));
};
