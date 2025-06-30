#pragma once

/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2025; SaiShu.Lcc.; HC; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file common.hpp
 * @author HC
 * @brief 通用方法类
 * @version 0.1
 * @date 2025-02-28
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "json.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署

using namespace std;
using namespace cv;

#define COLSIMAGE 320      // 图像的列数
#define ROWSIMAGE 240      // 图像的行数
#define COLSIMAGE_CAM 1024 // 图像的列数
#define ROWSIMAGE_CAM 768  // 图像的行数
#define COLSIMAGEIPM 320   // IPM图像的列数
#define ROWSIMAGEIPM 400   // IPM图像的行数
#define PWMSERVOMAX 6000   // 舵机PWM最大值（左）
#define PWMSERVOMID 5000   // 舵机PWM中值
#define PWMSERVOMIN 4000   // 舵机PWM最小值（右）

#define LABEL_BATTERY 0    // AI标签：充电站
#define LABEL_BLOCK 1      // AI标签：障碍物
#define LABEL_BRIDGE 2     // AI标签：坡道
#define LABEL_BURGER 3     // AI标签：汉堡
#define LABEL_CAR 4        // AI标签：道具车
#define LABEL_COMPANY 5    // AI标签：公司
#define LABEL_CONE 6       // AI标签：锥桶
#define LABEL_CROSSWALK 7  // AI标签：斑马线
#define LABEL_PEDESTRIAN 8 // AI标签：行人
#define LABEL_SCHOOL 9     // AI标签：学校

/**
 * @brief 场景类型（路况）
 *
 */
enum Scene
{
    NormalScene = 0, // 基础赛道
    CrossScene,      // 十字道路
    RingScene,       // 环岛道路
    BridgeScene,     // 坡道区
    ObstacleScene,   // 障碍区
    CateringScene,   // 快餐店
    LaybyScene,      // 临时停车区
    ParkingScene,    // 停车区
    StopScene        // 停车（结束）
};

string sceneToString(Scene scene);

/**
 * @brief 构建二维坐标
 *
 */
struct POINT
{
    int x = 0;
    int y = 0;
    float slope = 0.0f;

    POINT() {};
    POINT(int x, int y) : x(x), y(y) {};
};

/**
 * @brief 存储图像至本地
 *
 * @param image 需要存储的图像
 */
void savePicture(Mat &image);

//--------------------------------------------------[公共方法]----------------------------------------------------
/**
 * @brief int集合平均值计算
 *
 * @param arr 输入数据集合
 * @return double
 */
double average(vector<int> vec);

/**
 * @brief int集合数据方差计算
 *
 * @param vec Int集合
 * @return double
 */
double sigma(vector<int> vec);

/**
 * @brief 赛道点集的方差计算
 *
 * @param vec
 * @return double
 */
double sigma(vector<POINT> vec);

/**
 * @brief 阶乘计算
 *
 * @param x
 * @return int
 */
int factorial(int x);

/**
 * @brief 贝塞尔曲线
 *
 * @param dt
 * @param input
 * @return vector<POINT>
 */
vector<POINT> Bezier(double dt, vector<POINT> input);

std::string formatDoble2String(double val, int fixed);

/**
 * @brief 点到直线的距离计算
 *
 * @param a 直线的起点
 * @param b 直线的终点
 * @param p 目标点
 * @return double
 */
double distanceForPoint2Line(POINT a, POINT b, POINT p);

/**
 * @brief 两点之间的距离
 *
 * @param a
 * @param b
 * @return double
 */
double distanceForPoints(POINT a, POINT b);

/**
 * @brief UI综合图像绘制
 *
 */
class Display
{
private:
    bool enable = false;   // 显示窗口使能
    int sizeWindow = 1;    // 窗口数量
    cv::Mat imgShow;       // 窗口图像
    bool realShow = false; // 实时更新画面
public:
    int index = 0;      // 图像序号
    int indexLast = -1; // 图像序号
    int frameMax = 0;   // 视频总帧数
    bool save = false;  // 图像存储

    /**
     * @brief 显示窗口初始化
     *
     * @param size 窗口数量(1~7)
     */
    void init(const int size);

    /**
     * @brief 设置新窗口属性
     *
     * @param index 窗口序号
     * @param name 窗口名称
     * @param img 显示图像
     */
    void setNewWindow(int index, string name, Mat img);

    /**
     * @brief 融合后的图像显示
     *
     */
    void show(void);
};