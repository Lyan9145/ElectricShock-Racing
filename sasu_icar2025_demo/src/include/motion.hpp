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
 * @file motion.hpp
 * @author Leo
 * @brief 运动控制器：PD姿态控制||速度控制
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */
class ControlCenter; // <--- 添加这行前向声明

#include "common.hpp"
#include "json.hpp"
// #include "controlcenter.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;

/**
 * @brief 运动控制器
 *
 */
class Motion
{
private:
  int countShift = 0; // 变速计数器

public:
  Motion();

  struct Params
  {
    float speedLow = 0.8;    float speedHigh = 0.8;    float speedBridge = 0.6;    float speedCatering = 0.6;    float speedLayby = 0.6;    float speedObstacle = 0.6;    float speedParking = 0.6;    float speedRing = 0.6;    float speedDown = 0.5;    float runP1 = 0.9;    float runP2 = 0.018;    float runP3 = 0.0;    float turnP = 3.5;    float turnD = 3.5;    bool debug = false;    bool saveImg = false;    uint16_t rowCutUp = 10;    uint16_t rowCutBottom = 10;    bool bridge = true;    bool catering = true;    bool layby = true;    bool obstacle = true;    bool parking = true;    bool ring = true;    bool cross = true;    bool stop = true;
    float score = 0.5;    string model = "../res/model/yolov3_mobilenet_v1";    string video = "../res/samples/demo.mp4";  };

  Params params;                   // 读取控制参数
  uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
  float speed = 0.3;               // 发送给电机的速度

  void poseCtrl(int controlCenter);
  void speedCtrl(bool enable, bool slowDown, ControlCenter control);
};
