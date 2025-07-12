#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2025; SaiShu.HC.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file crosswalk.cpp
 * @author HC
 * @brief 停车区AI识别与路径规划
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "common.hpp"
#include "detection.hpp"
#include "uart.hpp"

using namespace std;
using namespace cv;

/**
 * @brief 停车区AI识别与路径规划类
 *
 */
class StopArea
{
public:

  enum State
  {
      Startup = 0,
      Firstdet, // 首次检测斑马线
      Firstpass, // 首次通过终点线
      Flyinglap, // 飞行圈
      Seconddet, // 第二次检测斑马线
      Secondpass, // 第二次通过终点线
      Stop
    };
    State state = State::Startup; // 场景状态
    bool process(vector<PredictResult> predict);
    void run(vector<PredictResult> predict, UartStatus &status);
    void drawUI(Mat &img);

    
    
private:
    bool park = false;      // 停车标志
    float startDistance = 0.0f; // 起始距离
    float passDistance = 0.4f; // 通过距离

    int counter = 0; // 计数器
    bool detected = false; // 是否检测到斑马线
    std::chrono::high_resolution_clock::time_point lapstartTime; // 场景开始时间
    std::chrono::high_resolution_clock::time_point lapendTime; // 场景结束时间


};
