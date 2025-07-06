#pragma once

#include "common.hpp"
#include <opencv2/opencv.hpp>
#include "imgprocess.hpp"

class Cross{
private:
    ImageProcess _imgprocess;
public:
    // 变量定义
    int cross_route = 0;  // 十字编码器积分
    int not_have_line = 0;  // 十字丢线计次

    // 十字远线 L 角点
    bool far_Lpt0_found, far_Lpt1_found;
    int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;

    // 十字远线
    int far_ipts0[ROWSIMAGE][2];
    int far_ipts1[ROWSIMAGE][2];
    int far_ipts0_num, far_ipts1_num;

    float far_rpts0[ROWSIMAGE][2];
    float far_rpts1[ROWSIMAGE][2];

    float far_rpts0b[ROWSIMAGE][2];
    float far_rpts1b[ROWSIMAGE][2];

    float far_rpts0s[ROWSIMAGE][2];
    float far_rpts1s[ROWSIMAGE][2];
    int far_rpts0s_num, far_rpts1s_num;

    float far_rpts0a[ROWSIMAGE];
    float far_rpts1a[ROWSIMAGE];

    float far_rpts0an[ROWSIMAGE];
    float far_rpts1an[ROWSIMAGE];

    //cross_farline
    int far_x1 = 50;
    int far_x2 = COLSIMAGE - 50;

    int far_y1 = 0;
    int far_y2 = 0;

    int yy1 = BEGIN_Y;
    int yy2 = BEGIN_Y;

    int last_Corner1_x;
    int last_Corner2_x;
    int last_Corner1_y;
    int last_Corner2_y;
    bool last_Corner1_found = false;
    bool last_Corner2_found = false;
    
    
    enum flag_cross_e {
        CROSS_NONE = 0,  // 非十字模式
        CROSS_BEGIN,     // 找到左右两个 L 角点
        CROSS_IN,        // 两个 L 角点很近, 即进入十字内部(此时切换远线控制)
    };
    flag_cross_e flag_cross;

    void check_cross(bool Lpt0_found, bool Lpt1_found,
        int rpts1s_num, int rpts0s_num,
        bool is_curve0, bool is_curve1);
    int run_cross(bool &Lpt0_found, bool &Lpt1_found,
        int &rpts1s_num, int &rpts0s_num, int &ipts0_num, int &rptsc0_num,
        int &Lpt0_rpts0s_id, int &ipts1_num, int &rptsc1_num, int &Lpt1_rpts1s_id,
        cv::Mat & mat_bin, float rpts0s[ROWSIMAGE][2], float rpts1s[ROWSIMAGE][2]);
    void cross_farline(cv::Mat & mat_bin,
        bool Lpt0_found, bool Lpt1_found, float rpts0s[ROWSIMAGE][2], int Lpt0_rpts0s_id,
        float rpts1s[ROWSIMAGE][2], int Lpt1_rpts1s_id);


    // 点集三角滤波
    void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel);
    // 点集等距采样  使走过的采样前折线段的距离为`dist`
    void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2,
                        float dist);
    // 点集局部角度变化率
    void local_angle_points(float pts_in[][2], int num, float angle_out[],
                            int dist);

    // 角度变化率非极大抑制
    void nms_angle(float angle_in[], int num, float angle_out[], int kernel);

    // 左手迷宫巡线
    void findline_lefthand_adaptive(cv::Mat img, int block_size, int clip_value,
                                int x, int y, int pts[][2], int *num);

    // 右手迷宫巡线
    void findline_righthand_adaptive(cv::Mat img, int block_size, int clip_value,
                                 int x, int y, int pts[][2], int *num);

};

