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
};
