#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include "common.hpp"


class Circle{
public:
    // 变量定义
    int circle_route = 0;  // 环岛编码器积分
    int none_left_line = 0, none_right_line = 0;
    int have_left_line = 0, have_right_line = 0;

    enum flag_circle_e {
    CIRCLE_NONE = 0,  // 非环岛模式
    CIRCLE_LEFT_BEGIN,
    CIRCLE_RIGHT_BEGIN,  // 环岛开始, 识别到单侧 L 角点另一侧长直道。
    CIRCLE_LEFT_IN,
    CIRCLE_RIGHT_IN,  // 环岛进入, 即走到一侧直道, 一侧环岛的位置。
    CIRCLE_LEFT_OUT,
    CIRCLE_RIGHT_OUT,  // 准备出环岛, 即识别到出环处的 L 角点。
    CIRCLE_LEFT_RUNNING,
    CIRCLE_RIGHT_RUNNING,  // 环岛内部。
    };
    flag_circle_e flag_circle;

    void check_circle(bool Lpt0_found, bool Lpt1_found, bool is_straight1, bool is_straight0);
    void run_circle(int &rpts0s_num, int &rpts1s_num, bool &Lpt1_found,
        int &ipts1_num, int &rptsc1_num, int &Lpt1_rpts1s_id, 
        bool &Lpt0_found, int &ipts0_num, int &rptsc0_num, int &Lpt0_rpts0s_id);
    string getCircleState() {
        switch (flag_circle) {
            case CIRCLE_NONE: return "CIRCLE_NONE";
            case CIRCLE_LEFT_BEGIN: return "CIRCLE_LEFT_BEGIN";
            case CIRCLE_RIGHT_BEGIN: return "CIRCLE_RIGHT_BEGIN";
            case CIRCLE_LEFT_IN: return "CIRCLE_LEFT_IN";
            case CIRCLE_RIGHT_IN: return "CIRCLE_RIGHT_IN";
            case CIRCLE_LEFT_OUT: return "CIRCLE_LEFT_OUT";
            case CIRCLE_RIGHT_OUT: return "CIRCLE_RIGHT_OUT";
            case CIRCLE_LEFT_RUNNING: return "CIRCLE_LEFT_RUNNING";
            case CIRCLE_RIGHT_RUNNING: return "CIRCLE_RIGHT_RUNNING";
        }
        return "UNKNOWN_CIRCLE_STATE";
    }
};

