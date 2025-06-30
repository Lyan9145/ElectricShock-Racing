#pragma once

class ControlCenter; 

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

// ======================= 新增代码开始 =======================
// 为 Motion::Params 结构体提供 from_json 函数
// 这个函数告诉 nlohmann::json 如何从一个 json 对象填充 Params 结构体
// 注意：这个函数应该定义在 Motion 类的外部，以便 ADL (Argument-Dependent Lookup) 能够找到它。
inline void from_json(const nlohmann::json& j, Motion::Params& p) {
    j.at("speedLow").get_to(p.speedLow);
    j.at("speedHigh").get_to(p.speedHigh);
    j.at("speedBridge").get_to(p.speedBridge);
    j.at("speedCatering").get_to(p.speedCatering);
    j.at("speedLayby").get_to(p.speedLayby);
    j.at("speedObstacle").get_to(p.speedObstacle);
    j.at("speedParking").get_to(p.speedParking);
    j.at("speedRing").get_to(p.speedRing);
    j.at("speedDown").get_to(p.speedDown);
    j.at("runP1").get_to(p.runP1);
    j.at("runP2").get_to(p.runP2);
    j.at("runP3").get_to(p.runP3);
    j.at("turnP").get_to(p.turnP);
    j.at("turnD").get_to(p.turnD);
    j.at("debug").get_to(p.debug);
    j.at("saveImg").get_to(p.saveImg);
    j.at("rowCutUp").get_to(p.rowCutUp);
    j.at("rowCutBottom").get_to(p.rowCutBottom);
    j.at("bridge").get_to(p.bridge);
    j.at("catering").get_to(p.catering);
    j.at("layby").get_to(p.layby);
    j.at("obstacle").get_to(p.obstacle);
    j.at("parking").get_to(p.parking);
    j.at("ring").get_to(p.ring);
    j.at("cross").get_to(p.cross);
    j.at("stop").get_to(p.stop);
    j.at("score").get_to(p.score);
    j.at("model").get_to(p.model);
    j.at("video").get_to(p.video);
}