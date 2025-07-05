#include "../../include/circle.hpp"

void Circle::check_circle(bool Lpt0_found, bool Lpt1_found, bool is_straight1, bool is_straight0) {
    // 非环岛, 单边 L 角点, 单边长直道
    if (flag_circle == CIRCLE_NONE &&
        (Lpt0_found && !Lpt1_found && is_straight1)) {
        none_left_line = 0;
        have_left_line = 0;
        circle_route = 0;
        flag_circle = CIRCLE_LEFT_BEGIN;
    }

    if (flag_circle == CIRCLE_NONE &&
        (!Lpt0_found && Lpt1_found && is_straight0)) {
        none_right_line = 0;
        have_right_line = 0;
        circle_route = 0;
        flag_circle = CIRCLE_RIGHT_BEGIN;
    }
    
}

/* ********************************************************************* */

void Circle::run_circle(int &rpts0s_num, int &rpts1s_num, bool &Lpt1_found,
    int &ipts1_num, int &rptsc1_num, int &Lpt1_rpts1s_id, 
    bool &Lpt0_found, int &ipts0_num, int &rptsc0_num, int &Lpt0_rpts0s_id){
    // 左环开始, 寻外直道右线
    if (flag_circle == CIRCLE_LEFT_BEGIN) {
        // 先丢左线后有线
        if (rpts0s_num < 0.2 / SAMPLE_DIST)
            none_left_line++;
        if (rpts0s_num > 0.8 / SAMPLE_DIST && none_left_line > 1) {
            have_left_line++;
            if (have_left_line > 0) {
                none_left_line = 0;
                have_left_line = 0;
                circle_route = 0;
                flag_circle = CIRCLE_LEFT_IN;
            }
        }
        // 入环失败
        if (circle_route > 100 &&
            flag_circle != CIRCLE_LEFT_IN) {
            none_left_line = 0;
            have_left_line = 0;
            flag_circle = CIRCLE_NONE;
        }
    }
    // 入左环, 寻内圆左线
    if (flag_circle == CIRCLE_LEFT_IN) {
        // 先丢右线后有线
        if (rpts1s_num < 0.2 / SAMPLE_DIST)
            none_right_line++;
        if (rpts1s_num > 0.8 / SAMPLE_DIST && none_right_line > 0) {
            if (circle_route > 20)
                have_right_line++;
            if (have_right_line > 0) {
                none_right_line = 0;
                have_right_line = 0;
                flag_circle = CIRCLE_LEFT_RUNNING;
            }
        }
    }
    // 正常巡线, 寻外圆右线
    if (flag_circle == CIRCLE_LEFT_RUNNING) {
        // 外环存在拐点, 可再加拐点距离判据 (左L点)
        if (Lpt1_found)
            ipts1_num = rpts1s_num = rptsc1_num = Lpt1_rpts1s_id;
        if (Lpt1_found && Lpt1_rpts1s_id < 0.6 / SAMPLE_DIST) {
            flag_circle = CIRCLE_LEFT_OUT;
        }
    }
    // 出环, 寻内圆
    if (flag_circle == CIRCLE_LEFT_OUT) {
        if (Lpt1_found)
            ipts1_num = rpts1s_num = rptsc1_num = Lpt1_rpts1s_id;

        // 先丢右线后有线
        if (rpts1s_num < 0.4 / SAMPLE_DIST)
            none_right_line++;
        if (rpts1s_num > 1.0 / SAMPLE_DIST && none_right_line > 0) {
            if (have_right_line > 0) {
                none_right_line = 0;
                have_right_line = 0;
                circle_route = 0;
                flag_circle = CIRCLE_NONE;
            } else {
                have_right_line++;
            }
        }
    }

    /* ***************************************************************** */

    // 右环开始, 寻外直道左线
    if (flag_circle == CIRCLE_RIGHT_BEGIN) {
        // 先丢右线后有线
        if (rpts1s_num < 0.2 / SAMPLE_DIST)
            none_right_line++;
        if (rpts1s_num > 0.8 / SAMPLE_DIST && none_right_line > 1) {
            have_right_line++;
            if (have_right_line > 1) {
                none_right_line = 0;
                have_right_line = 0;
                circle_route = 0;
                flag_circle = CIRCLE_RIGHT_IN;
            }
        }
        // 入环失败
        if (circle_route > 100 &&
            flag_circle != CIRCLE_RIGHT_IN) {
            none_right_line = 0;
            have_right_line = 0;
            circle_route = 0;
            flag_circle = CIRCLE_NONE;
        }
    }
    // 入右环, 寻内圆右线
    if (flag_circle == CIRCLE_RIGHT_IN) {
        // 先丢左线后有线
        if (rpts0s_num < 0.2 / SAMPLE_DIST)
            none_left_line++;
        if (rpts0s_num > 0.8 / SAMPLE_DIST && none_left_line > 0) {
            if (circle_route > 20)
                have_left_line++;
            if (have_left_line > 0) {
                none_left_line = 0;
                have_left_line = 0;
                flag_circle = CIRCLE_RIGHT_RUNNING;
            }
        }
    }
    // 正常巡线, 寻外圆左线
    if (flag_circle == CIRCLE_RIGHT_RUNNING) {
        // 外环存在拐点, 可再加拐点距离判据 (左L点)
        if (Lpt0_found)
            ipts0_num = rpts0s_num = rptsc0_num = Lpt0_rpts0s_id;
        if (Lpt0_found && Lpt0_rpts0s_id < 0.6 / SAMPLE_DIST) {
            flag_circle = CIRCLE_RIGHT_OUT;
        }
    }
    // 出环, 寻内圆
    if (flag_circle == CIRCLE_RIGHT_OUT) {
        if (Lpt0_found)
            ipts0_num = rpts0s_num = rptsc0_num = Lpt0_rpts0s_id;

        // 先丢左线后有线
        if (rpts0s_num < 0.4 / SAMPLE_DIST)
            none_left_line++;
        if (rpts0s_num > 1.0 / SAMPLE_DIST && none_left_line > 0) {
            if (have_left_line > 0) {
                none_left_line = 0;
                have_left_line = 0;
                circle_route = 0;
                flag_circle = CIRCLE_NONE;
            } else {
                have_left_line++;
            }
        }
    }
}
