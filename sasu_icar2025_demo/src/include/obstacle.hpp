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
 * @file obstacle.cpp
 * @author Leo
 * @brief 障碍区AI识别与路径规划
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "detection.hpp"
// #include "tracking.hpp"
#include "imgprocess.hpp"

using namespace std;
using namespace cv;

/**
 * @brief 障碍区AI识别与路径规划类
 *
 */
class Obstacle
{

public:
    enum state
    {
        StateNone = 0, // 无障碍
        EnterObstacle, // 进入障碍区
        InObstacle,    // 在障碍区
        ExitObstacle,  // 离开障碍区
    };
    state current_state = state::StateNone; // 当前状态

    enum ObstaclePos
    {
        ObstaclePosNone = 0, // 无障碍
        Left,                // 左侧障碍
        Right,               // 右侧障碍
    };
    ObstaclePos flag_obstacle_pos = ObstaclePos::ObstaclePosNone;

    enum ObstacleType
    {
        ObstacleTypeNone = 0, // 无障碍
        Cone,                 // 锥桶
        Block,                // 大黑块
        Pedestrian,           // 行人
    };
    ObstacleType flag_obstacle_type = ObstacleType::ObstacleTypeNone;

    int obstacle_counter = 0; // 障碍计数器(里程计)

    bool process(vector<PredictResult> &predict, bool is_straight0, bool is_straight1);
    int run(vector<PredictResult> &predict, float rpts0s[ROWSIMAGE][2], float rpts1s[ROWSIMAGE][2]);
    float getTrackOffset();
    void drawImage(Mat &img);

private:
    ImageProcess _imgprocess; // 图像处理类
    bool enable = false;      // 场景检测使能标志
    PredictResult resultObs;  // 避障目标锥桶

    float pointLeft[2], pointRight[2];           // 避障目标锥桶透视变换后点
    float pointLeftTrans[2], pointRightTrans[2]; // 避障目标锥桶透视变换后点

    float track_offset = 0.0f; // 赛道偏移量


    // void curtailTracking(Tracking &track, bool left);
};
