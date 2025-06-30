#include "../include/common.hpp"
#include "../include/json.hpp"
#include "../include/motion.hpp" // Include the header with the class definition
#include "../include/controlcenter.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;


/**
 * @brief 初始化：加载配置文件
 *
 */
// Add Motion:: before the constructor name
Motion::Motion()
{
  string jsonPath = "../src/config/config.json";
  std::ifstream config_is(jsonPath);
  if (!config_is.good())
  {
    std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
    exit(-1);
  }

  nlohmann::json js_value;
  config_is >> js_value;

  try
  {
    params = js_value.get<Params>();
  }
  catch (const nlohmann::detail::exception &e)
  {
    std::cerr << "Json Params Parse failed :" << e.what() << '\n';
    exit(-1);
  }

  speed = params.speedLow;
  cout << "--- runP1:" << params.runP1 << " | runP2:" << params.runP2
       << " | runP3:" << params.runP3 << endl;
  cout << "--- turnP:" << params.turnP << " | turnD:" << params.turnD << endl;
  cout << "--- speedLow:" << params.speedLow
       << "m/s  |  speedHigh:" << params.speedHigh << "m/s" << endl;
};

// Remove the Params struct definition from here

// Remove member variable definitions from here

/**
 * @brief 姿态PD控制器
 *
 * @param controlCenter 智能车控制中心
 */
// Add Motion:: before the method name
void Motion::poseCtrl(int controlCenter)
{
  float error = controlCenter - COLSIMAGE / 2; // 图像控制中心转换偏差
  static int errorLast = 0;                    // 记录前一次的偏差
  if (abs(error - errorLast) > COLSIMAGE / 10)
  {
    error = error > errorLast ? errorLast + COLSIMAGE / 10
                              : errorLast - COLSIMAGE / 10;
  }

  // params.turnP = abs(error) * params.runP2 + params.runP1;
  int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
  errorLast = error;

  servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
  // 限幅
  if (servoPwm > PWMSERVOMAX)
  {
    servoPwm = PWMSERVOMAX;
    cout << "Hit Max!" << endl; // 调试输出
  }
  else if (servoPwm < PWMSERVOMIN)
  {
    servoPwm = PWMSERVOMIN;
    cout << "Hit Min!" << endl; // 调试输出
  }
  // cout << "Servo PWM:" << servoPwm << endl; // 调试输出
}

/**
 * @brief 变加速控制
 *
 * @param enable 加速使能
 * @param control
 */
// Add Motion:: before the method name
void Motion::speedCtrl(bool enable, bool slowDown, ControlCenter control)
{
  // 控制率
  uint8_t controlLow = 0;   // 速度控制下限
  uint8_t controlMid = 5;   // 控制率
  uint8_t controlHigh = 10; // 速度控制上限

  if (slowDown)
  {
    countShift = controlLow;
    speed = params.speedDown;
  }
  else if (enable) // 加速使能
  {
    if (control.centerEdge.size() < 10)
    {
      speed = params.speedLow;
      countShift = controlLow;
      return;
    }
    if (control.centerEdge[control.centerEdge.size() - 1].x > ROWSIMAGE / 2)
    {
      speed = params.speedLow;
      countShift = controlLow;
      return;
    }
    if (abs(control.sigmaCenter) < 100.0)
    {
      countShift++;
      if (countShift > controlHigh)
        countShift = controlHigh;
    }
    else
    {
      countShift--;
      if (countShift < controlLow)
        countShift = controlLow;
    }

    if (countShift > controlMid)
      speed = params.speedHigh;
    else
      speed = params.speedLow;
  }
  else
  {
    countShift = controlLow;
    speed = params.speedLow;
  }
}