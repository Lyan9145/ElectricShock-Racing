#include "../include/common.hpp" // 公共类方法文件
#include "json.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署

// 位置式 PID 角度外环
float pid_realize_a(float actual, float set, float _p, float _d)
{
    static float last_error = 0.0f;
    static float last_out_d = 0.0f;
    // static float last_actual = 0.0f;
    // static float derivative = 0.0f;

    /* 当前误差 */
    float error = set - actual;

    /* 微分先行 */
    /*
    float temp = 0.618f * _d + _p;
    float c3 = _d / temp;
    float c2 = (_d + _p) / temp;
    float c1 = 0.618f * c3;
    derivative = c1 * derivative + c2 * actual - c3 * last_actual;
    */

    /* 不完全微分 */
    float out_d = _d * 0.8f * (error - last_error) + 0.2f * last_out_d;
    // float out_d = 0.8f * derivative + 0.2f * last_out_d;

    /* 实际输出 */
    float output = _p * error + out_d;

    /* 更新参数 */
    last_error = error;
    last_out_d = out_d;
    // last_actual = actual;

    return output;
}

// 位置式 PID 角速度内环
int pid_realize_o(int actual, int set, float _p, float _d)
{
    static int last_error = 0;

    /* 当前误差 */
    int error = set - actual;

    /* 实际输出 */
    int output = (int)(_p * error + _d * (error - last_error) + 0.5f);

    /* 更新参数 */
    last_error = error;

    return output;
}

int clip(int x, int low, int up)
{
    return x > up ? up : x < low ? low
                                 : x;
}

/* 窗口绘制信息 */
void MAT_INFO(cv::Mat &mat, std::string string_buf, cv::Point point, double size)
{
    cv::putText(mat, (std::string)string_buf, point, cv::FONT_HERSHEY_SIMPLEX,
                size, cv::Scalar(0, 0, 255));
}

float filter(float value)
{
    static float filter_buf[3] = {0};

    filter_buf[2] = filter_buf[1];
    filter_buf[1] = filter_buf[0];
    filter_buf[0] = value;

    return (filter_buf[2] + filter_buf[1] + filter_buf[0]) / 3.0f;
}

/**
 * @brief Get the Scene object
 *
 * @param scene
 * @return string
 */
string sceneToString(Scene scene)
{
    switch (scene)
    {
    case Scene::NormalScene:
        return "Normal";
    case Scene::CrossScene:
        return "Crossroad";
    case Scene::RingScene:
        return "Ring";
    case Scene::BridgeScene:
        return "Bridge";
    case Scene::ObstacleScene:
        return "Obstacle";
    case Scene::CateringScene:
        return "Catering";
    case Scene::LaybyScene:
        return "Layby";
    case Scene::ParkingScene:
        return "Parking";
    case Scene::StopScene:
        return "Stop";
    default:
        return "Unknown";
    }
}

/**
 * @brief 存储图像至本地
 *
 * @param image 需要存储的图像
 */
void savePicture(Mat &image)
{
    // 存图
    string name = ".jpg";
    static int counter = 0;
    counter++;
    string imgPath = "../res/samples/train/";
    name = imgPath + to_string(counter) + ".jpg";
    imwrite(name, image);
}

//--------------------------------------------------[公共方法]----------------------------------------------------
/**
 * @brief int集合平均值计算
 *
 * @param arr 输入数据集合
 * @return double
 */
double average(vector<int> vec)
{
    if (vec.size() < 1)
        return -1;

    double sum = 0;
    for (size_t i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }

    return (double)sum / vec.size();
}

/**
 * @brief int集合数据方差计算
 *
 * @param vec Int集合
 * @return double
 */
double sigma(vector<int> vec)
{
    if (vec.size() < 1)
        return 0;

    double aver = average(vec); // 集合平均值
    double sigma = 0;
    for (size_t i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i] - aver) * (vec[i] - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}
double sigma(float pts[][2], int num)
{
    if (num < 1)
        return 0;

    double sum = 0;
    for (int i = 0; i < num; i++)
        sum += pts[i][0];

    double aver = (double)sum / num;
    double sigma = 0;

    for (int i = 0; i < num; i++)
        sigma += (pts[i][0] - aver) * (pts[i][0] - aver);
    sigma /= (double)num;

    return sigma;
}

/**
 * @brief 赛道点集的方差计算
 *
 * @param vec
 * @return double
 */
double sigma(vector<POINT> vec)
{
    if (vec.size() < 1)
        return 0;

    double sum = 0;
    for (size_t i = 0; i < vec.size(); i++)
    {
        sum += vec[i].y;
    }
    double aver = (double)sum / vec.size(); // 集合平均值

    double sigma = 0;
    for (size_t i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i].y - aver) * (vec[i].y - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}

/**
 * @brief 阶乘计算
 *
 * @param x
 * @return int
 */
int factorial(int x)
{
    int f = 1;
    for (int i = 1; i <= x; i++)
    {
        f *= i;
    }
    return f;
}

/**
 * @brief 贝塞尔曲线
 *
 * @param dt
 * @param input
 * @return vector<POINT>
 */
vector<POINT> Bezier(double dt, vector<POINT> input)
{
    vector<POINT> output;

    double t = 0;
    while (t <= 1)
    {
        POINT p;
        double sumX = 0.0;
        double sumY = 0.0;
        int i = 0;
        int n = input.size() - 1;
        while (i <= n)
        {
            double k =
                factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) * pow(1 - t, n - i);
            sumX += k * input[i].x;
            sumY += k * input[i].y;
            i++;
        }
        p.x = sumX;
        p.y = sumY;
        output.push_back(p);
        t += dt;
    }
    return output;
}

std::string formatDoble2String(double val, int fixed)
{
    auto str = std::to_string(val);
    return str.substr(0, str.find(".") + fixed + 1);
}

/**
 * @brief 点到直线的距离计算
 *
 * @param a 直线的起点
 * @param b 直线的终点
 * @param p 目标点
 * @return double
 */
double distanceForPoint2Line(POINT a, POINT b, POINT p)
{
    double ab_distance =
        sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    double ap_distance =
        sqrt((a.x - p.x) * (a.x - p.x) + (a.y - p.y) * (a.y - p.y));
    double bp_distance =
        sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y));

    double half = (ab_distance + ap_distance + bp_distance) / 2;
    double area = sqrt(half * (half - ab_distance) * (half - ap_distance) * (half - bp_distance));

    return (2 * area / ab_distance);
}

/**
 * @brief 两点之间的距离
 *
 * @param a
 * @param b
 * @return double
 */
double distanceForPoints(POINT a, POINT b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/**
 * @brief UI综合图像绘制
 *
 */

void Display::init(const int size)
{
    if (size <= 0 || size > 7)
        return;

    cv::namedWindow("ICAR", WINDOW_NORMAL);     // 图像名称
    cv::resizeWindow("ICAR", 480 * 2, 320 * 2); // 分辨率

    imgShow = cv::Mat::zeros(ROWSIMAGE * 2, COLSIMAGE * 2, CV_8UC3);
    enable = true;
    sizeWindow = size;
}

/**
 * @brief 设置新窗口属性
 *
 * @param index 窗口序号
 * @param name 窗口名称
 * @param img 显示图像
 */
void Display::setNewWindow(int index, string name, Mat img)
{
    // 数据溢出保护
    if (!enable || index <= 0 || index > sizeWindow)
        return;

    if (img.cols <= 0 || img.rows <= 0)
        return;

    Mat imgDraw = img.clone();

    if (imgDraw.type() == CV_8UC1) // 非RGB类型的图像
        cvtColor(imgDraw, imgDraw, cv::COLOR_GRAY2BGR);

    // 图像缩放
    if (imgDraw.cols != COLSIMAGE || imgDraw.rows != ROWSIMAGE)
    {
        float fx = COLSIMAGE / imgDraw.cols;
        float fy = ROWSIMAGE / imgDraw.rows;
        if (fx <= fy)
            resize(imgDraw, imgDraw, Size(COLSIMAGE, ROWSIMAGE), fx, fx);
        else
            resize(imgDraw, imgDraw, Size(COLSIMAGE, ROWSIMAGE), fy, fy);
    }

    // 限制图片标题长度
    string text = "[" + to_string(index) + "] ";
    if (name.length() > 15)
        text = text + name.substr(0, 15);
    else
        text = text + name;

    putText(imgDraw, text, Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 0), 0.5);

    if (index <= 2)
    {
        Rect placeImg = Rect(COLSIMAGE * (index - 1), 0, COLSIMAGE, ROWSIMAGE);
        imgDraw.copyTo(imgShow(placeImg));
    }

    else
    {
        Rect placeImg = Rect(COLSIMAGE * (index - 3), ROWSIMAGE, COLSIMAGE, ROWSIMAGE);
        imgDraw.copyTo(imgShow(placeImg));
    }

    if (save)
        savePicture(img); // 保存图像
}

/**
 * @brief 融合后的图像显示
 *
 */
void Display::show(void)
{
    if (enable)
    {
        putText(imgShow, "Frame:" + to_string(index), Point(COLSIMAGE / 2 - 50, ROWSIMAGE * 2 - 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
        imshow("ICAR", imgShow);

        char key = waitKey(1);
        if (key != -1)
        {
            if (key == 32) // 空格
                realShow = !realShow;
        }
        if (realShow)
        {
            index++;
            if (index < 0)
                index = 0;
            if (index > frameMax)
                index = frameMax;
        }
    }
}
