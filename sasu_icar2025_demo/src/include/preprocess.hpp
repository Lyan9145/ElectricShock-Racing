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
    cv::Mat mtx = (cv::Mat_<double>(3, 3) <<
        596.96547147, 0.0,   534.67168451,
        0.0,   594.3847867, 395.4989663,
        0.0,   0.0,   1.0);

    cv::Mat dist = (cv::Mat_<double>(1, 5) << -0.36929412, 0.14635301, -0.00127689,  0.00081881, -0.02740082);
};
