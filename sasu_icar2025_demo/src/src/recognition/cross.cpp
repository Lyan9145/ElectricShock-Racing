#include "../../include/cross.hpp"

#define AT_IMAGE(img, x, y) (img.at<uint8_t>(y, x))
#define AT_MIN(a, b) (((a) < (b)) ? (a) : (b))

void Cross::check_cross(bool Lpt0_found, bool Lpt1_found,
    int rpts1s_num, int rpts0s_num, 
    bool is_curve0, bool is_curve1){
     if (flag_cross == CROSS_NONE &&
        ((Lpt0_found && Lpt1_found) ||
        //  (Lpt0_found && is_curve1) ||
        //  (Lpt1_found && is_curve0) || 
         (Lpt0_found && rpts1s_num < 0.35 / SAMPLE_DIST) ||
         (Lpt1_found && rpts0s_num < 0.35 / SAMPLE_DIST))) {
        flag_cross = CROSS_BEGIN;
        cross_route = 0;
    }
}

/* ********************************************************************* */

int Cross::run_cross(bool &Lpt0_found, bool &Lpt1_found,
    int &rpts1s_num, int &rpts0s_num, int &ipts0_num, int &rptsc0_num,
    int &Lpt0_rpts0s_id, int &ipts1_num, int &rptsc1_num, int &Lpt1_rpts1s_id, 
    cv::Mat & mat_bin, float rpts0s[ROWSIMAGE][2], float rpts1s[ROWSIMAGE][2]){
    // 检测到十字, 先按照近线走
    if (flag_cross == CROSS_BEGIN) {
        if (Lpt0_found) {
            ipts0_num = rpts0s_num = rptsc0_num = Lpt0_rpts0s_id;
        }
        if (Lpt1_found) {
            ipts1_num = rpts1s_num = rptsc1_num = Lpt1_rpts1s_id;
        }

        // 近角点过少, 进入远线控制
        if ((!Lpt0_found && !Lpt1_found) ||
            (Lpt0_found && Lpt0_rpts0s_id < 0.20 / SAMPLE_DIST) ||
            (Lpt1_found && Lpt1_rpts1s_id < 0.20 / SAMPLE_DIST) ||
            (rpts0s_num < 0.20 / SAMPLE_DIST && rpts1s_num < 0.20 / SAMPLE_DIST)) {
            cross_route = 0;
            flag_cross = CROSS_IN;
        }
        printf("> Cross Begin: Lpt0_found=%d, Lpt1_found=%d, rpts0s_num=%.2f, rpts1s_num=%.2f, lpt0_rpts0s_id=%d, Lpt1_rpts1s_id=%d\n",
               Lpt0_found, Lpt1_found, rpts0s_num, rpts1s_num,
               Lpt0_rpts0s_id, Lpt1_rpts1s_id);
    }

    /* ***************************************************************** */
    // 返回寻哪个中线 0:左；1:右；2:默认；
    int ret_track_state = 2;

    // 远线控制进十字
    if (flag_cross == CROSS_IN) {
        // 寻远线, 算法与近线相同
        cross_farline(mat_bin, Lpt0_found, Lpt1_found, rpts0s, Lpt0_rpts0s_id, rpts1s, Lpt1_rpts1s_id);

        if (far_Lpt0_found && far_Lpt1_found && far_rpts0s[far_rpts0s_num - 1][0] > COLSIMAGE / 2)
            ret_track_state = 0;
        else if (far_Lpt0_found && far_Lpt1_found && far_rpts1s[far_rpts1s_num - 1][0] < COLSIMAGE / 2)
            ret_track_state = 1;
        else if (far_rpts0s_num > far_rpts1s_num)
            ret_track_state = 0;
        else if (far_rpts0s_num < far_rpts1s_num)
            ret_track_state = 1;
        else
            ret_track_state = 0;

        // 正常巡线先丢线后有线
        if (rpts0s_num < 0.2 / SAMPLE_DIST &&
            rpts1s_num < 0.2 / SAMPLE_DIST) {
            not_have_line++;
        }
        if ((rpts0s_num > 1.0 / SAMPLE_DIST &&
             rpts1s_num > 1.0 / SAMPLE_DIST &&
             !Lpt0_found && !Lpt1_found &&
             not_have_line > 0) ||
            (cross_route >= 120)) {
            cross_route = 0;
            not_have_line = 0;
            flag_cross = CROSS_NONE;
        }
    }
    return ret_track_state;
}

/* ********************************************************************* */

void Cross::cross_farline(cv::Mat & mat_bin,
    bool Lpt0_found, bool Lpt1_found, float rpts0s[ROWSIMAGE][2], int Lpt0_rpts0s_id,
    float rpts1s[ROWSIMAGE][2], int Lpt1_rpts1s_id){
    if (Lpt0_found && cross_route < 150) {
        float trans[2]; // 透视变换后的坐标
        _imgprocess.mapPerspective(rpts0s[Lpt0_rpts0s_id][0],
                        rpts0s[Lpt0_rpts0s_id][1], trans, 1);
        yy1 = trans[1] - 5;
        far_x1 = trans[0];
    }
    if (Lpt1_found && cross_route < 100) {
        float trans[2];
        _imgprocess.mapPerspective(rpts1s[Lpt1_rpts1s_id][0],
                        rpts1s[Lpt1_rpts1s_id][1], trans, 1);
        yy2 = trans[1] - 5;
        far_x2 = trans[0];
    }

    /* ***************************************************************** */

    bool white_found = false;
    bool black_found = false;
    far_ipts0_num = sizeof(far_ipts0) / sizeof(far_ipts0[0]); // 远线点集
    for (; yy1 >= 60; yy1--) { // 从下往上找
        if (AT_IMAGE(mat_bin, far_x1, yy1) >= 127) // 找到白色点
            white_found = true;

        if (AT_IMAGE(mat_bin, far_x1, yy1) < 127 && 
            white_found) { // 找到黑色点
            // 找到黑色点后, 检查上方是否有足够的黑色点
            int black = 0;
            for (int i = 1; i <= 15; i++) // 检查上方30个像素点
                if (yy1 - i >= 0) //
                    black += AT_IMAGE(mat_bin, far_x1, yy1 - i);
            if (black > 200) { // 如果上方黑色点太多, 认为是噪声
                white_found = false;
                continue;
            }
            black_found = true;
            far_y1 = yy1;
            break;
        }
    }
    if (AT_IMAGE(mat_bin, far_x1, far_y1 + 1) >= 127 &&
        black_found) // 如果下方是白色点, 认为是远线
        findline_lefthand_adaptive(mat_bin, BLOCK_SIZE, CLIP_VALUE, far_x1,
                                   far_y1 + 1, far_ipts0, &far_ipts0_num);
    else
        far_ipts0_num = 0;

    /* ***************************************************************** */

    white_found = false;
    black_found = false;
    far_ipts1_num = sizeof(far_ipts1) / sizeof(far_ipts1[0]);
    for (; yy2 >= 60; yy2--) {
        if (AT_IMAGE(mat_bin, far_x2, yy2) >= 127)
            white_found = true;

        if (AT_IMAGE(mat_bin, far_x2, yy2) < 127 &&
            white_found) {
            int black = 0;
            for (int i = 1; i <= 30; i++)
                if (yy2 - i >= 0)
                    black += AT_IMAGE(mat_bin, far_x2, yy2 - i);
            if (black > 200) {
                white_found = false;
                continue;
            }
            black_found = true;
            far_y2 = yy2;
            break;
        }
    }
    if (AT_IMAGE(mat_bin, far_x2, far_y2 + 1) >= 127 &&
        black_found)
        findline_righthand_adaptive(mat_bin, BLOCK_SIZE, CLIP_VALUE, far_x2,
                                    far_y2 + 1, far_ipts1, &far_ipts1_num);
    else
        far_ipts1_num = 0;

    /* ***************************************************************** */

    // 原图边线 -> 透视边线
    for (int i = 0; i < far_ipts0_num; i++)
        _imgprocess.mapPerspective(far_ipts0[i][0], far_ipts0[i][1], far_rpts0[i], 0);
    for (int i = 0; i < far_ipts1_num; i++)
        _imgprocess.mapPerspective(far_ipts1[i][0], far_ipts1[i][1], far_rpts1[i], 0);

    // 边线滤波
    blur_points(far_rpts0, far_ipts0_num, far_rpts0b,
                (int)round(LINE_BLUR_KERNEL));
    blur_points(far_rpts1, far_ipts1_num, far_rpts1b,
                (int)round(LINE_BLUR_KERNEL));

    // 边线等距采样
    far_rpts0s_num = sizeof(far_rpts0s) / sizeof(far_rpts0s[0]);
    resample_points(far_rpts0b, far_ipts0_num, far_rpts0s, &far_rpts0s_num,
                    SAMPLE_DIST * PIXEL_PER_METER);
    far_rpts1s_num = sizeof(far_rpts1s) / sizeof(far_rpts1s[0]);
    resample_points(far_rpts1b, far_ipts1_num, far_rpts1s, &far_rpts1s_num,
                    SAMPLE_DIST * PIXEL_PER_METER);

    // 边线局部角度变化率
    local_angle_points(far_rpts0s, far_rpts0s_num, far_rpts0a,
                       (int)round(ANGLE_DIST / SAMPLE_DIST));
    local_angle_points(far_rpts1s, far_rpts1s_num, far_rpts1a,
                       (int)round(ANGLE_DIST / SAMPLE_DIST));

    // 角度变化率非极大抑制
    nms_angle(far_rpts0a, far_rpts0s_num, far_rpts0an,
              (int)round(ANGLE_DIST / SAMPLE_DIST) * 2 + 1);
    nms_angle(far_rpts1a, far_rpts1s_num, far_rpts1an,
              (int)round(ANGLE_DIST / SAMPLE_DIST) * 2 + 1);

    // 找远线上的 L 角点
    far_Lpt0_found = far_Lpt1_found = false;
    for (int i = 0; i < AT_MIN(far_rpts0s_num, 80); i++) {
        if (far_rpts0an[i] == 0)
            continue;
        int im1 =
            clip(i - (int)round(ANGLE_DIST / SAMPLE_DIST), 0, far_rpts0s_num - 1);
        int ip1 =
            clip(i + (int)round(ANGLE_DIST / SAMPLE_DIST), 0, far_rpts0s_num - 1);
        float conf = fabs(far_rpts0a[i]) -
                     (fabs(far_rpts0a[im1]) + fabs(far_rpts0a[ip1])) / 2;

        if (30. / 180. * PI < conf && conf < 150. / 180. * PI && i < 100) {
            far_Lpt0_rpts0s_id = i;
            far_Lpt0_found = true;
            break;
        }
    }
    for (int i = 0; i < AT_MIN(far_rpts1s_num, 80); i++) {
        if (far_rpts1an[i] == 0)
            continue;
        int im1 =
            clip(i - (int)round(ANGLE_DIST / SAMPLE_DIST), 0, far_rpts1s_num - 1);
        int ip1 =
            clip(i + (int)round(ANGLE_DIST / SAMPLE_DIST), 0, far_rpts1s_num - 1);
        float conf = fabs(far_rpts1a[i]) -
                     (fabs(far_rpts1a[im1]) + fabs(far_rpts1a[ip1])) / 2;

        if (30. / 180. * PI < conf && conf < 150. / 180. * PI && i < 100) {
            far_Lpt1_rpts1s_id = i;
            far_Lpt1_found = true;
            break;
        }
    }

}



// 点集三角滤波
void Cross::blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) {
    assert(kernel % 2 == 1);

    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) {
            pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}


// 点集等距采样  使走过的采样前折线段的距离为`dist`
void Cross::resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2,
                     float dist) {
    int remain = 0, len = 0;
    for (int i = 0; i < num1 - 1 && len < *num2; i++) {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i + 1][0] - x0;
        float dy = pts_in[i + 1][1] - y0;
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;

        while (remain < dn && len < *num2) {
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;

            len++;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;
}

// 点集局部角度变化率
void Cross::local_angle_points(float pts_in[][2], int num, float angle_out[],
                        int dist) {
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

// 角度变化率非极大抑制
void Cross::nms_angle(float angle_in[], int num, float angle_out[], int kernel) {
    assert(kernel % 2 == 1);

    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
                angle_out[i] = 0;
                break;
            }
        }
    }
}

/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

// 左手迷宫巡线
void Cross::findline_lefthand_adaptive(cv::Mat img, int block_size, int clip_value,
                                int x, int y, int pts[][2], int *num) {
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < *num) && (half <= x) && (x <= img.cols - half - 1) &&
           (half <= y) && (y <= img.rows - half - 1) && (turn < 4)) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += img.at<uint8_t>(y + dy, x + dx);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value =
            img.at<uint8_t>(y + dir_front[dir][1], x + dir_front[dir][0]);
        int frontleft_value =
            img.at<uint8_t>(y + dir_frontleft[dir][1], x + dir_frontleft[dir][0]);
        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}

// 右手迷宫巡线
void Cross::findline_righthand_adaptive(cv::Mat img, int block_size, int clip_value,
                                 int x, int y, int pts[][2], int *num) {
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < *num) && (half <= x) && (x <= img.cols - half - 1) &&
           (half <= y) && (y <= img.rows - half - 1) && (turn < 4)) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += img.at<uint8_t>(y + dy, x + dx);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value =
            img.at<uint8_t>(y + dir_front[dir][1], x + dir_front[dir][0]);
        int frontright_value =
            img.at<uint8_t>(y + dir_frontright[dir][1], x + dir_frontright[dir][0]);
        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}
