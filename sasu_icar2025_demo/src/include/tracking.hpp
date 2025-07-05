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
#include "imgprocess.hpp"

#define ROAD_WIDTH (0.40)     // 赛道宽度 (0.4)
#define BLOCK_SIZE (7)        // 自适应阈值的block大小 (7)
#define CLIP_VALUE (2)        // 自适应阈值的阈值裁减量 (2)
#define LINE_BLUR_KERNEL (7)  // 边线三角滤波核的大小 (7)
#define PIXEL_PER_METER (242) // 俯视图中, 每个像素对应的长度 (102)
#define SAMPLE_DIST (0.02)    // 边线等距采样的间距 (0.02)
#define ANGLE_DIST (0.2)      // 计算边线转角时, 三个计算点的距离 (0.2)

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
    void findline_lefthand_adaptive(cv::Mat img, int block_size, int clip_value,
                                    int x, int y, int pts[][2], int *num);
    void findline_righthand_adaptive(cv::Mat img, int block_size, int clip_value,
                                     int x, int y, int pts[][2], int *num);
    void trackRecognition_new(Mat &imageBinary, Mat &result_img);
    void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel);
    void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2,
                                   float dist);
    void local_angle_points(float pts_in[][2], int num, float angle_out[],
                                      int dist);
    void nms_angle(float angle_in[], int num, float angle_out[], int kernel);
    void track_leftline(float pts_in[][2], int num, float pts_out[][2],
                                  int approx_num, float dist);
    void track_rightline(float pts_in[][2], int num, float pts_out[][2],
                                   int approx_num, float dist);
    float fit_line(float pts[][2], int num, int cut_h);

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

private:
    bool _is_result = true; // 是否生成处理后的图像

    float cx = COLSIMAGE / 2.0f; // 车轮对应点 (纯跟踪起始点)
    float cy = ROWSIMAGE * 0.99f;

    int aim_idx__far = 0; // 远预锚点位置
    int aim_idx_near = 0; // 近预锚点位置

    float mea_0 = 0.0f; // 左直线拟合平均绝对误差
    float mea_1 = 0.0f; // 右直线拟合平均绝对误差

    std::vector<POINT> bezier_line; // 中线贝塞尔曲线拟合

    float aim_angle_p_k;
    float aim_angle_p;
    float aim_angle_d;

    float speed_diff = 0.f; // 减速率

    const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    const int dir_frontleft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};
    int track_row_begin = 190; // 巡线纵坐标起点
    int track_col_begin = 120; // 巡线横坐标起点

public:
    int begin_x_l; // 巡线横坐标起始点 左
    int begin_x_r; // 巡线横坐标起始点 右
    int begin_y_t; // 巡线纵坐标起始点

    float aim_speed = 0.f;       // 速度量
    float aim_speed_shift = 0.f; // 变速计数器
    float aim_angle = 0.0f;      // 偏差量
    float aim_angle_last = 0.0f; // 偏差量 上一帧
    float aim_sigma = 0.0f;      // 偏差方差
    float aim_distance_f = 0.0f; // 远锚点
    float aim_distance_n = 0.0f; // 近锚点

    int element_begin_id = 0;   // 特殊元素中线起始点
    int element_over_route = 0; // 元素结束路程
    bool element_over = false;  // 元素结束标志

    int threshold = 120;

public:
    std::vector<POINT> edge_det;       // AI元素检测边缘点集
    std::vector<POINT> edge_left;      // 赛道左边缘点集   注意: 此点集 x 与 y 位置相反 !!!
    std::vector<POINT> edge_right;     // 赛道右边缘点集
    std::vector<POINT> last_edge_left; // 记录上一场边缘点集 (丢失边)
    std::vector<POINT> last_edge_right;

    /* *********************************************************************** */
    /* ******************************* 角点数据 ******************************* */
    /* *********************************************************************** */

private:
    // L 角点置信度 (角度)
    float Lpt0_found_conf, Lpt1_found_conf;
    // L 角点二次判断
    bool Lpt0_found_last = false, Lpt1_found_last = false;

public:
    // L 角点
    int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
    bool Lpt0_found, Lpt1_found;
    // 长直道
    bool is_straight0, is_straight1;
    // 弯道 左 右 强制
    bool is_curve0, is_curve1;
    /* *********************************************************************** */
    /* ******************************* 边线数据 ******************************* */
    /* *********************************************************************** */

public:
    // 原图左右中线数据定义
    int ipts0[ROWSIMAGE][2]; // 左: 0
    int ipts1[ROWSIMAGE][2]; // 右: 1
    int iptsc[ROWSIMAGE];    // 中: c
    int ipts0_num, ipts1_num, rptsc_num;
    // 透视变换后左右中线  滤波: b
    float rpts0b[ROWSIMAGE][2];
    float rpts1b[ROWSIMAGE][2];
    float rptscb[ROWSIMAGE][2];
    // 透视变换后左右中线  等距采样: s
    float rpts0s[ROWSIMAGE][2];
    float rpts1s[ROWSIMAGE][2];
    float rptscs[ROWSIMAGE][2];
    int rpts0s_num, rpts1s_num, rptscs_num;
    // 左右边线局部角度变化率: a
    float rpts0a[ROWSIMAGE];
    float rpts1a[ROWSIMAGE];
    // 左右边线局部角度变化率非极大抑制: an
    float rpts0an[ROWSIMAGE];
    float rpts1an[ROWSIMAGE];
    // 左右边线偏移中线: c
    float rptsc0[ROWSIMAGE][2];
    float rptsc1[ROWSIMAGE][2];
    int rptsc0_num, rptsc1_num;

private:
    // 中线
    float (*rpts)[2];
    int rpts_num;
    // 归一化中线
    float rptsn[ROWSIMAGE][2];
    int rptsn_num;

    // 透视变换临时变量
    float trans[2];
    enum TrackState
    {
        TRACK_LEFT,
        TRACK_MIDDLE,
        TRACK_RIGHT
    };

    std::vector<std::string> element_string = {
        "STANDARD",
        "GARAGE",
        "RAMP",
        "RESCUE",
        "CIRCLE",
        "CROSS",
        "DANGER"};
    enum ElementState
    {
        STANDARD,
        GARAGE,
        RAMP,
        RESCUE,
        CIRCLE,
        CROSS,
        DANGER
    };
    TrackState track_state = TrackState::TRACK_MIDDLE;
    ElementState elem_state = ElementState::GARAGE;

    cv::Mat bin_img;

    bool flag_elem_over = true;
    int elem_over_cnt = 0;
};
