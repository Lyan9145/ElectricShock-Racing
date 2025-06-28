#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file tracking.cpp
 * @author your name (you@domain.com)
 * @brief 赛道线识别：提取赛道左右边缘数据（包括岔路信息等）
 * @version 0.1
 * @date 2022-02-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "common.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Tracking
{
public:
    vector<POINT> pointsEdgeLeft;     // 赛道左边缘点集
    vector<POINT> pointsEdgeRight;    // 赛道右边缘点集
    vector<POINT> widthBlock;         // 色块宽度=终-起（每行）
    vector<POINT> spurroad;           // 保存岔路信息
    double stdevLeft;                 // 边缘斜率方差（左）
    double stdevRight;                // 边缘斜率方差（右）
    int validRowsLeft = 0;            // 边缘有效行数（左）
    int validRowsRight = 0;           // 边缘有效行数（右）
    POINT garageEnable = POINT(0, 0); // 车库识别标志：（x=1/0，y=row)
    uint16_t rowCutUp = 10;           // 图像顶部切行
    uint16_t rowCutBottom = 10;       // 图像底部切行

    void trackRecognition(bool isResearch, uint16_t rowStart);
    void trackRecognition(Mat &imageBinary);
    void drawImage(Mat &trackImage);
    double stdevEdgeCal(vector<POINT> &v_edge, int img_height);

private:
    Mat imagePath; // 赛道搜索图像
    /**
     * @brief 赛道识别输入图像类型
     *
     */
    enum ImageType
    {
        Binary = 0, // 二值化
        Rgb,        // RGB
    };

    ImageType imageType = ImageType::Binary; // 赛道识别输入图像类型：二值化图像

    void slopeCal(vector<POINT> &edge, int index);
    void validRowsCal(void);
    int getMiddleValue(vector<int> vec);
};
