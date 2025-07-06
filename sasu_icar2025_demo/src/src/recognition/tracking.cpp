#include "../../include/tracking.hpp"

using namespace cv;
using namespace std;

void Tracking::trackRecognition(bool isResearch, uint16_t rowStart)
{
    bool flagStartBlock = true;                    // 搜索到色块起始行的标志（行）
    int counterSearchRows = pointsEdgeLeft.size(); // 搜索行计数
    int startBlock[30];                            // 色块起点（行）
    int endBlock[30];                              // 色块终点（行）
    int counterBlock = 0;                          // 色块计数器（行）
    POINT pointSpurroad;                           // 岔路坐标
    bool spurroadEnable = false;

    if (rowCutUp > ROWSIMAGE / 4)
        rowCutUp = ROWSIMAGE / 4;
    if (rowCutBottom > ROWSIMAGE / 4)
        rowCutBottom = ROWSIMAGE / 4;

    if (!isResearch)
    {
        pointsEdgeLeft.clear();              // 初始化边缘结果
        pointsEdgeRight.clear();             // 初始化边缘结果
        widthBlock.clear();                  // 初始化色块数据
        spurroad.clear();                    // 岔路信息
        validRowsLeft = 0;                   // 边缘有效行数（左）
        validRowsRight = 0;                  // 边缘有效行数（右）
        flagStartBlock = true;               // 搜索到色块起始行的标志（行）
        garageEnable = POINT(0, 0);          // 车库识别标志初始化
        rowStart = ROWSIMAGE - rowCutBottom; // 默认底部起始行
    }
    else
    {
        if (pointsEdgeLeft.size() > rowStart)
            pointsEdgeLeft.resize(rowStart);
        if (pointsEdgeRight.size() > rowStart)
            pointsEdgeRight.resize(rowStart);
        if (widthBlock.size() > rowStart)
        {
            widthBlock.resize(rowStart);
            if (rowStart > 1)
                rowStart = widthBlock[rowStart - 1].x - 2;
        }

        flagStartBlock = false; // 搜索到色块起始行的标志（行）
    }


    //  开始识别赛道左右边缘
    for (int row = rowStart; row > rowCutUp; row--) // 有效行：10~220
    {
        counterBlock = 0; // 色块计数器清空
        // 搜索色（block）块信息
        if (imageType == ImageType::Rgb) // 输入RGB图像
        {
            if (imagePath.at<Vec3b>(row, 1)[2] > 0)
            {
                startBlock[counterBlock] = 0;
            }
            for (int col = 1; col < COLSIMAGE; col++) // 搜索出每行的所有色块
            {
                if (imagePath.at<Vec3b>(row, col)[2] > 0 &&
                    imagePath.at<Vec3b>(row, col - 1)[2] == 0)
                {
                    startBlock[counterBlock] = col;
                }
                else
                {
                    if (imagePath.at<Vec3b>(row, col)[2] == 0 &&
                        imagePath.at<Vec3b>(row, col - 1)[2] > 0)
                    {
                        endBlock[counterBlock++] = col;
                        if (counterBlock >= end(endBlock) - begin(endBlock))
                            break;
                    }
                }
            }
            if (imagePath.at<Vec3b>(row, COLSIMAGE - 1)[2] > 0)
            {
                if (counterBlock < end(endBlock) - begin(endBlock) - 1)
                    endBlock[counterBlock++] = COLSIMAGE - 1;
            }
        }
        if (imageType == ImageType::Binary) // 输入二值化图像

        {
            if (imagePath.at<uchar>(row, 1) > 127)
            {
                startBlock[counterBlock] = 0;
            }
            for (int col = 1; col < COLSIMAGE; col++) // 搜索出每行的所有色块
            {
                if (imagePath.at<uchar>(row, col) > 127 &&
                    imagePath.at<uchar>(row, col - 1) <= 127)
                {
                    startBlock[counterBlock] = col;
                }
                else
                {
                    if (imagePath.at<uchar>(row, col) <= 127 &&
                        imagePath.at<uchar>(row, col - 1) > 127)
                    {
                        endBlock[counterBlock++] = col;
                        if (counterBlock >= end(endBlock) - begin(endBlock))
                            break;
                    }
                }
            }
            if (imagePath.at<uchar>(row, COLSIMAGE - 1) > 127)
            {
                if (counterBlock < end(endBlock) - begin(endBlock) - 1)
                    endBlock[counterBlock++] = COLSIMAGE - 1;
            }
        }

        // 搜索起始行
        int widthBlocks = endBlock[0] - startBlock[0]; // 色块宽度临时变量
        int indexWidestBlock = 0;                      // 最宽色块的序号
        if (flagStartBlock)                            // 起始行做特殊处理
        {
            if (row < ROWSIMAGE / 3)
                return;
            if (counterBlock == 0)
            {
                continue;
            }
            for (int i = 1; i < counterBlock; i++) // 搜索最宽色块
            {
                int tmp_width = endBlock[i] - startBlock[i];
                if (tmp_width > widthBlocks)
                {
                    widthBlocks = tmp_width;
                    indexWidestBlock = i;
                }
            }

            int limitWidthBlock = COLSIMAGE * 0.7; // 首行色块宽度限制（不能太小）
            if (row < ROWSIMAGE * 0.6)
            {
                limitWidthBlock = COLSIMAGE * 0.4;
            }
            if (widthBlocks > limitWidthBlock) // 满足首行宽度要求
            {
                flagStartBlock = false;
                POINT pointTmp(row, startBlock[indexWidestBlock]);
                pointsEdgeLeft.push_back(pointTmp);
                pointTmp.y = endBlock[indexWidestBlock];
                pointsEdgeRight.push_back(pointTmp);
                widthBlock.emplace_back(row, endBlock[indexWidestBlock] -
                                                 startBlock[indexWidestBlock]);
                counterSearchRows++;
            }
            spurroadEnable = false;
        }
        else // 其它行色块坐标处理
        {
            if (counterBlock == 0)
            {
                break;
            }

            //-------------------------------------------------<车库标识识别>-------------------------------------------------------------
            if (counterBlock > 5 && !garageEnable.x)
            {
                int widthThis = 0;        // 色块的宽度
                vector<int> widthGarage;  // 当前行色块宽度集合
                vector<int> centerGarage; // 当前行色块质心集合
                vector<int> indexGarage;  // 当前行有效色块的序号

                for (int i = 0; i < counterBlock; i++)
                {
                    widthThis = endBlock[i] - startBlock[i];        // 色块的宽度
                    int center = (endBlock[i] + startBlock[i]) / 2; // 色块的质心
                    if (widthThis > 5 && widthThis < 50)            // 过滤无效色块区域：噪点
                    {
                        centerGarage.push_back(center);
                        widthGarage.push_back(widthThis);
                    }
                }

                int widthMiddle = getMiddleValue(widthGarage); // 斑马线色块宽度中值

                for (size_t i = 0; i < widthGarage.size(); i++)
                {
                    if (abs(widthGarage[i] - widthMiddle) < widthMiddle / 3)
                    {
                        indexGarage.push_back(i);
                    }
                }
                if (indexGarage.size() >= 4) // 验证有效斑马线色块个数
                {
                    vector<int> distance;
                    for (size_t i = 1; i < indexGarage.size(); i++) // 质心间距的方差校验
                    {
                        distance.push_back(widthGarage[indexGarage[i]] -
                                           widthGarage[indexGarage[i - 1]]);
                    }
                    double var = sigma(distance);
                    if (var < 5.0) // 经验参数
                    {
                        garageEnable.x = 1;                      // 车库标志使能
                        garageEnable.y = pointsEdgeRight.size(); // 斑马线行序号
                    }
                }
            }
            //------------------------------------------------------------------------------------------------------------------------

            vector<int> indexBlocks;               // 色块序号（行）
            for (int i = 0; i < counterBlock; i++) // 上下行色块的连通性判断
            {
                int g_cover =
                    min(endBlock[i], pointsEdgeRight[pointsEdgeRight.size() - 1].y) -
                    max(startBlock[i], pointsEdgeLeft[pointsEdgeLeft.size() - 1].y);
                if (g_cover >= 0)
                {
                    indexBlocks.push_back(i);
                }
            }

            if (indexBlocks.size() ==
                0) // 如果没有发现联通色块，则图像搜索完成，结束任务
            {
                break;
            }
            else if (indexBlocks.size() ==
                     1) // 只存在单个色块，正常情况，提取边缘信息
            {
                if (endBlock[indexBlocks[0]] - startBlock[indexBlocks[0]] <
                    COLSIMAGE / 10)
                {
                    continue;
                }
                pointsEdgeLeft.emplace_back(row, startBlock[indexBlocks[0]]);
                pointsEdgeRight.emplace_back(row, endBlock[indexBlocks[0]]);
                slopeCal(pointsEdgeLeft, pointsEdgeLeft.size() - 1); // 边缘斜率计算
                slopeCal(pointsEdgeRight, pointsEdgeRight.size() - 1);
                widthBlock.emplace_back(row, endBlock[indexBlocks[0]] -
                                                 startBlock[indexBlocks[0]]);
                spurroadEnable = false;
            }
            else if (indexBlocks.size() >
                     1) // 存在多个色块，则需要择优处理：选取与上一行最近的色块
            {
                int centerLast = COLSIMAGE / 2;
                if (pointsEdgeRight.size() > 0 && pointsEdgeLeft.size() > 0)
                    centerLast = (pointsEdgeRight[pointsEdgeRight.size() - 1].y +
                                  pointsEdgeLeft[pointsEdgeLeft.size() - 1].y) /
                                 2; // 上一行色块的中心点横坐标
                int centerThis =
                    (startBlock[indexBlocks[0]] + endBlock[indexBlocks[0]]) /
                    2; // 当前行色块的中心点横坐标
                int differBlocks =
                    abs(centerThis - centerLast); // 上下行色块的中心距离
                int indexGoalBlock = 0;           // 目标色块的编号
                int startBlockNear =
                    startBlock[indexBlocks[0]]; // 搜索与上一行最近的色块起点
                int endBlockNear =
                    endBlock[indexBlocks[0]]; // 搜索与上一行最近的色块终点

                for (size_t i = 1; i < indexBlocks.size();
                     i++) // 搜索与上一行最近的色块编号
                {
                    centerThis =
                        (startBlock[indexBlocks[i]] + endBlock[indexBlocks[i]]) / 2;
                    if (abs(centerThis - centerLast) < differBlocks)
                    {
                        differBlocks = abs(centerThis - centerLast);
                        indexGoalBlock = i;
                    }
                    // 搜索与上一行最近的边缘起点和终点
                    if (abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y -
                            startBlock[indexBlocks[i]]) <
                        abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y -
                            startBlockNear))
                    {
                        startBlockNear = startBlock[indexBlocks[i]];
                    }
                    if (abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y -
                            endBlock[indexBlocks[i]]) <
                        abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y -
                            endBlockNear))
                    {
                        endBlockNear = endBlock[indexBlocks[i]];
                    }
                }

                // 检索最佳的起点与终点
                if (abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y -
                        startBlock[indexBlocks[indexGoalBlock]]) <
                    abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y -
                        startBlockNear))
                {
                    startBlockNear = startBlock[indexBlocks[indexGoalBlock]];
                }
                if (abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y -
                        endBlock[indexBlocks[indexGoalBlock]]) <
                    abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y -
                        endBlockNear))
                {
                    endBlockNear = endBlock[indexBlocks[indexGoalBlock]];
                }

                if (endBlockNear - startBlockNear < COLSIMAGE / 10)
                {
                    continue;
                }
                POINT tmp_point(row, startBlockNear);
                pointsEdgeLeft.push_back(tmp_point);
                tmp_point.y = endBlockNear;
                pointsEdgeRight.push_back(tmp_point);
                widthBlock.emplace_back(row, endBlockNear - startBlockNear);
                slopeCal(pointsEdgeLeft, pointsEdgeLeft.size() - 1);
                slopeCal(pointsEdgeRight, pointsEdgeRight.size() - 1);
                counterSearchRows++;

                //-------------------------------<岔路信息提取>----------------------------------------
                pointSpurroad.x = row;
                pointSpurroad.y = endBlock[indexBlocks[0]];
                if (!spurroadEnable)
                {
                    spurroad.push_back(pointSpurroad);
                    spurroadEnable = true;
                }
                //------------------------------------------------------------------------------------
            }

            stdevLeft = stdevEdgeCal(pointsEdgeLeft, ROWSIMAGE); // 计算边缘方差
            stdevRight = stdevEdgeCal(pointsEdgeRight, ROWSIMAGE);

            validRowsCal(); // 有效行计算
        }
    }
}

void Tracking::trackRecognition_new(Mat &imageBinary, Mat &result_img, TaskData &src, std::vector<PredictResult> &predict_result)
{
    imagePath = imageBinary;
    
    // begin_y_t = track_row_begin;
    // if (elem_state != Scene::ParkingScene && 
    // (!(is_curve0 && track_state == TrackState::TRACK_LEFT) && 
    // !(is_curve1 && track_state == TrackState::TRACK_RIGHT))) {
    //     int black_sum = 0; // 黑色像素点计数
    //     for (int i = 100; i < COLSIMAGE - 100; i++) { // 遍历行
    //         if ((imagePath.at<uint8_t>(begin_y_t, i)) < threshold) // 黑色像素点
    //             black_sum++;
    //     }
    //     if (black_sum > 10) { // 如果黑色像素点大于10个，说明是直道
    //         begin_y_t = track_col_begin - 50; // 将起始行上移50行
    //         begin_x_l = begin_x_l + 50 > track_row_begin - 1 ? track_row_begin - 1 : begin_x_l + 50; // 左边线起始点, 上移50行
    //         begin_x_r = begin_x_r - 50 < 0 ? 0 : begin_x_r - 50; // 右边线起始点, 上移50行
    //     }
    // }
    // cout << "> begin_y_t: " << begin_y_t << endl;
    // cout << "> begin_x_l: " << begin_x_l << endl;
    // cout << "> begin_x_r: " << begin_x_r << endl;

    // 原图找左边线 -------------------------------------------------------
    {
        int x1 = begin_x_l, y1 = begin_y_t;
        // 向左寻找白点
        if (imagePath.at<uint8_t>(y1, x1) < threshold)
            for (x1--; x1 > 0; x1--)
                if (imagePath.at<uint8_t>(y1, x1) >= threshold)
                    break;
        // 向左寻找黑点
        for (; x1 > 0; x1--)
            if (imagePath.at<uint8_t>(y1, x1 - 1) < threshold)
                break;
        // 向上寻找黑点
        if (x1 < BLOCK_SIZE / 2)
        {
            x1 = BLOCK_SIZE / 2;
            for (; y1 > ROWSIMAGE * 2 / 3; y1--)
                if (imagePath.at<uint8_t>(y1 - 1, x1) < threshold)
                    break;
        }
        if (imagePath.at<uint8_t>(y1, x1) >= threshold)
        {
            ipts0_num = y1 + (ROWSIMAGE - track_row_begin);
            findline_lefthand_adaptive(imagePath, BLOCK_SIZE, CLIP_VALUE,
                                       x1, y1, ipts0, &ipts0_num);
            begin_x_l = x1 + 50 > COLSIMAGE - 1 ? COLSIMAGE - 1 : x1 + 50;
        }
        else
        {
            ipts0_num = 0;
            begin_x_l = threshold;
        }
    }
    // 原图找右边线 -------------------------------------------------------
    {
        int x2 = begin_x_r, y2 = begin_y_t;
        // 向右寻找白点
        if (imagePath.at<uint8_t>(y2, x2) < threshold)
            for (x2++; x2 < COLSIMAGE - 1; x2++)
                if (imagePath.at<uint8_t>(y2, x2) >= threshold)
                    break;
        // 向右寻找黑点
        for (; x2 < COLSIMAGE - 1; x2++)
            if (imagePath.at<uint8_t>(y2, x2 + 1) < threshold)
                break;
        // 向上寻找黑点
        if (x2 > COLSIMAGE - BLOCK_SIZE / 2 - 1)
        {
            x2 = COLSIMAGE - BLOCK_SIZE / 2 - 1;
            for (; y2 > ROWSIMAGE * 2 / 3; y2--)
                if (imagePath.at<uint8_t>(y2 - 1, x2) < threshold)
                    break;
        }
        if (imagePath.at<uint8_t>(y2, x2) >= threshold)
        {
            ipts1_num = y2 + (ROWSIMAGE - track_row_begin);
            findline_righthand_adaptive(imagePath, BLOCK_SIZE, CLIP_VALUE,
                                        x2, y2, ipts1, &ipts1_num);
            begin_x_r = x2 - 50 < 0 ? 0 : x2 - 50;
        }
        else
        {
            ipts1_num = 0;
            begin_x_r = COLSIMAGE - track_col_begin;
        }
    }
    // 变换后左右中线
    float rpts0[ROWSIMAGE][2] = {0};
    float rpts1[ROWSIMAGE][2] = {0};
    float rptsc[ROWSIMAGE][2] = {0};
    // 原图边线转换数据定义
    int iptsc0[ROWSIMAGE] = {0};
    int iptsc1[ROWSIMAGE] = {0};

    // 原图边线 -> 透视边线 左
    rptsc_num = 0;
    edge_left.clear();

    for (int i = 0; i < ipts0_num; i++, rptsc_num++)
    {
        if (ipts0[i][1] < 10)
            break;

        _imgprocess.mapPerspective(ipts0[i][0], ipts0[i][1], rpts0[i], 0);
        iptsc0[ipts0[i][1]] = ipts0[i][0]; 

        // 注意: x 与 y 位置相反 !!!
        POINT pointTmp(ipts0[i][1], ipts0[i][0]);
        edge_left.push_back(pointTmp);
    }
    ipts0_num = rptsc_num;
    // 原图边线 -> 透视边线 右
    rptsc_num = 0;
    edge_right.clear();
    for (int i = 0; i < ipts1_num; i++, rptsc_num++)
    {
        if (ipts1[i][1] < 10)
            break;

        _imgprocess.mapPerspective(ipts1[i][0], ipts1[i][1], rpts1[i], 0);
        iptsc1[ipts1[i][1]] = ipts1[i][0];

        // 注意: x 与 y 位置相反 !!!
        POINT pointTmp(ipts1[i][1], ipts1[i][0]);
        edge_right.push_back(pointTmp);
    };
    ipts1_num = rptsc_num;

    // 中线获取 图像顶部10个像素丢弃
    rptsc_num = 0;
    for (int ccy = ROWSIMAGE - 1; ccy >= 10; ccy--)
    {
        iptsc[ccy] = iptsc0[ccy] + iptsc1[ccy];

        if (iptsc[ccy] != 0) // 中线存在
        {
            if (iptsc1[ccy] == 0) // 右边线不存在
                iptsc[ccy] = (int)((COLSIMAGE - iptsc[ccy]) / 2 + iptsc[ccy]); // 右边线不存在，取左边线的中点
            else
                iptsc[ccy] = (int)(iptsc[ccy] / 2); // 中线为左右边线的中点

            // 原图中线 -> 透视中线
            _imgprocess.mapPerspective(iptsc[ccy], ccy, rptsc[rptsc_num++], 0);
        }
    }

        // 滤波
    blur_points(rpts0, ipts0_num, rpts0b, (int)round(LINE_BLUR_KERNEL));
    blur_points(rpts1, ipts1_num, rpts1b, (int)round(LINE_BLUR_KERNEL));
    blur_points(rptsc, rptsc_num, rptscb, (int)round(LINE_BLUR_KERNEL));

    // 边线等距采样
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, ipts0_num, rpts0s, &rpts0s_num, SAMPLE_DIST * PIXEL_PER_METER);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, ipts1_num, rpts1s, &rpts1s_num, SAMPLE_DIST * PIXEL_PER_METER);
    rptscs_num = sizeof(rptscs) / sizeof(rptscs[0]);
    resample_points(rptscb, rptsc_num, rptscs, &rptscs_num, SAMPLE_DIST * PIXEL_PER_METER);

    // 边线局部角度变化率
    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int)round(ANGLE_DIST / SAMPLE_DIST));
    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int)round(ANGLE_DIST / SAMPLE_DIST));

    // 角度变化率非极大抑制
    nms_angle(rpts0a, rpts0s_num, rpts0an, (int)round(ANGLE_DIST / SAMPLE_DIST) * 2 + 1);
    nms_angle(rpts1a, rpts1s_num, rpts1an, (int)round(ANGLE_DIST / SAMPLE_DIST) * 2 + 1);

    // 左右中线跟踪
    track_leftline(rpts0s, rpts0s_num, rptsc0,
                   (int)round(ANGLE_DIST / SAMPLE_DIST),
                   PIXEL_PER_METER * track_offset);
    rptsc0_num = rpts0s_num;
    track_rightline(rpts1s, rpts1s_num, rptsc1,
                    (int)round(ANGLE_DIST / SAMPLE_DIST),
                    PIXEL_PER_METER * track_offset);
    rptsc1_num = rpts1s_num;

    
    // 可视化：将透视左右边线和中线画在单独一张图上
    if (0) {
        cv::Mat perspective_lines = cv::Mat::zeros(result_img.size(), CV_8UC3);

        // 左边线（青色）
        for (int a = 0; a < rpts0s_num; a++) {
            int x = static_cast<int>(rpts0s[a][0]);
            int y = static_cast<int>(rpts0s[a][1]);
            if (y >= 0 && y < perspective_lines.rows && x >= 0 && x < perspective_lines.cols) {
                perspective_lines.at<cv::Vec3b>(y, x) = cv::Vec3b(238, 238, 0);
            }
        }
        // 右边线（黄色）
        for (int a = 0; a < rpts1s_num; a++) {
            int x = static_cast<int>(rpts1s[a][0]);
            int y = static_cast<int>(rpts1s[a][1]);
            if (y >= 0 && y < perspective_lines.rows && x >= 0 && x < perspective_lines.cols) {
                perspective_lines.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 238, 238);
            }
        }
        // 中线（黑色）
        for (int a = 0; a < rptscs_num; a++) {
            int x = static_cast<int>(rptscs[a][0]);
            int y = static_cast<int>(rptscs[a][1]);
            if (y >= 0 && y < perspective_lines.rows && x >= 0 && x < perspective_lines.cols) {
                perspective_lines.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            }
        }

        // 可选：显示或保存
        cv::imshow("Perspective Lines", perspective_lines);
        cv::waitKey(1); // 等待按键

        // cv::imwrite("perspective_lines.png", perspective_lines);
    }
    // 透视中线 -> 原图中线
    for (int i = 0; i < rptsc0_num; i++)
        _imgprocess.mapPerspective(rptsc0[i][0], rptsc0[i][1], rptsc0[i], 1);
    for (int i = 0; i < rptsc1_num; i++)
        _imgprocess.mapPerspective(rptsc1[i][0], rptsc1[i][1], rptsc1[i], 1);
    for (int i = 0; i < rptscs_num; i++)
        _imgprocess.mapPerspective(rptscs[i][0], rptscs[i][1], rptscs[i], 1);

    /* ***************************************************************** */
    /* *************************** 弯直道检测 *************************** */
    /* ***************************************************************** */

    // 标志位重置
    is_curve0 = is_curve1 = is_straight0 = is_straight1 = false;
    mea_0 = mea_1 = 10.0f;
    // 左线直线拟合
    if (rpts0s_num > 10) {
        mea_0 = fit_line(rpts0s, rpts0s_num, 60);
        if (mea_0 < 2.5f && rpts0s_num > 60)
            is_straight0 = true;
        else
            is_curve0 = true;
    }
    // 右线直线拟合
    if (rpts1s_num > 10) {
        mea_1 = fit_line(rpts1s, rpts1s_num, 60);
        if (mea_1 < 2.5f && rpts1s_num > 60)
            is_straight1 = true;
        else
            is_curve1 = true;
    }
    /* ***************************************************************** */
    /* **************************** 角点检测 **************************** */
    /* ***************************************************************** */

    // 角点重置
    Lpt0_found = Lpt1_found = false;
    // 左线角点
    for (int i = 0; i < rpts0s_num; i++) {
        if (rpts0an[i] == 0)
            continue;
        int im1 = clip(i - (int)round(ANGLE_DIST / SAMPLE_DIST), 0, rpts0s_num - 1);
        int ip1 = clip(i + (int)round(ANGLE_DIST / SAMPLE_DIST), 0, rpts0s_num - 1);
        float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;
        // L 角点阈值 0.6981317 < x < 2.44346
        if (Lpt0_found == false && 40. / 180. * PI < conf &&
            conf < 140. / 180. * PI && i < 0.8 / SAMPLE_DIST) {
            Lpt0_found_conf = conf;
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        if (Lpt0_found)
            break;
    }
    // 右线角点
    for (int i = 0; i < rpts1s_num; i++) {
        if (rpts1an[i] == 0)
            continue;
        int im1 = clip(i - (int)round(ANGLE_DIST / SAMPLE_DIST), 0, rpts1s_num - 1);
        int ip1 = clip(i + (int)round(ANGLE_DIST / SAMPLE_DIST), 0, rpts1s_num - 1);
        float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
        // L 角点阈值
        if (Lpt1_found == false && 40. / 180. * PI < conf &&
            conf < 140. / 180. * PI && i < 0.8 / SAMPLE_DIST) {
            Lpt1_found_conf = conf;
            Lpt1_rpts1s_id = i;
            Lpt1_found = true;
        }
        if (Lpt1_found)
            break;
    }


    // L 点二次检查, 车库模式不检查, 依据两角点距离及角点后张开特性
    if (elem_state != Scene::ParkingScene) {
        if (Lpt0_found && Lpt1_found) {
            float dx = rpts0s[Lpt0_rpts0s_id][0] - rpts1s[Lpt1_rpts1s_id][0];
            float dy = rpts0s[Lpt0_rpts0s_id][1] - rpts1s[Lpt1_rpts1s_id][1];
            float dn = sqrtf(dx * dx + dy * dy);
            if (fabs(dn - 0.45 * PIXEL_PER_METER) < 0.15 * PIXEL_PER_METER) {
                float dwx = rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] -
                            rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0];
                float dwy = rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][1] -
                            rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][1];
                float dwn = sqrtf(dwx * dwx + dwy * dwy);
                if (!(dwn > 0.7 * PIXEL_PER_METER &&
                      rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] <
                          rpts0s[Lpt0_rpts0s_id][0] &&
                      rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0] >
                          rpts1s[Lpt1_rpts1s_id][0])) {
                    Lpt0_found = Lpt1_found = false;
                }
            } else
                Lpt0_found = Lpt1_found = false;
        }
    }

    // 角点二次确认 左
    if (Lpt0_found) {
        if (!Lpt0_found_last) {
            Lpt0_found_last = true;
            Lpt0_found = false;
        } else if (_is_result) {
            _imgprocess.mapPerspective(rpts0s[Lpt0_rpts0s_id][0], rpts0s[Lpt0_rpts0s_id][1],
                            trans, 1);
            MAT_INFO(result_img, std::string("L_%.3f", Lpt0_found_conf), cv::Point(trans[0] + 5, trans[1]), 0.3);
        }
    } else
        Lpt0_found_last = false;
    // 角点二次确认 右
    if (Lpt1_found) {
        if (!Lpt1_found_last) {
            Lpt1_found_last = true;
            Lpt1_found = false;
        } else if (_is_result) {
            _imgprocess.mapPerspective(rpts1s[Lpt1_rpts1s_id][0], rpts1s[Lpt1_rpts1s_id][1],
                            trans, 1);
            MAT_INFO(result_img, std::string("L_%.3f", Lpt1_found_conf), cv::Point(trans[0] + 5, trans[1]), 0.3);
        }
    } else Lpt1_found_last = false;

    /* ***************************************************************** */
    /* **************************** 选定中线 **************************** */
    /* *********************** 单侧线少, 切换巡线方向 ********************** */
    // TODO: 适配
    track_state = TrackState::TRACK_MIDDLE; // 默认中线巡线
    if (is_straight0 && is_straight1)
        track_state = TrackState::TRACK_MIDDLE;
    else if (is_straight0)
        track_state = TrackState::TRACK_LEFT;
    else if (is_straight1)
        track_state = TrackState::TRACK_RIGHT;
    else if (is_curve0 && is_curve1 && rptscs[rptscs_num - 1][0] > COLSIMAGE / 2)
        track_state = TrackState::TRACK_LEFT;
    else if (is_curve0 && is_curve1 && rptscs[rptscs_num - 1][0] < COLSIMAGE / 2)
        track_state = TrackState::TRACK_RIGHT;
    else if (rpts0s_num == 0 && rpts1s_num != 0)
        track_state = TrackState::TRACK_RIGHT;
    else if (rpts0s_num != 0 && rpts1s_num == 0)
        track_state = TrackState::TRACK_LEFT;
    else if (rpts0s_num < rpts1s_num / 2)
        track_state = TrackState::TRACK_RIGHT;
    else if (rpts0s_num / 2 > rpts1s_num)
        track_state = TrackState::TRACK_LEFT;
    else if (rpts0s_num < 10 && rpts0s_num < rpts1s_num)
        track_state = TrackState::TRACK_RIGHT;
    else if (rpts1s_num < 10 && rpts0s_num > rpts1s_num)
        track_state = TrackState::TRACK_LEFT;
    else
        track_state = TrackState::TRACK_MIDDLE;

    /* ***************************************************************** */
    /* **************************** 元素检测 **************************** */
    /* ***************************************************************** */
    // TODO: 元素检测需要适配

    if (!flag_elem_over) {
        ++elem_over_cnt;
    }
    if (elem_over_cnt >= 80) {
        flag_elem_over = true;
        elem_over_cnt = 0;
    }

    // 正常巡线下查找特殊元素
    if (elem_state == Scene::NormalScene && flag_elem_over) {
        // auto signs = find_sign.findSigns(src_img, _config);
        // find_sign.sign_classify(signs);

        // 十字
        if (elem_state == Scene::NormalScene && flag_elem_over) {
            cross.check_cross(Lpt0_found, Lpt1_found, rpts1s_num, rpts0s_num, is_curve0, is_curve1);
            if (cross.flag_cross != Cross::flag_cross_e::CROSS_NONE) {
                elem_state = Scene::CrossScene;
            } else {
                elem_state = Scene::NormalScene;
            }
        }

        // 圆环
        if (elem_state == Scene::NormalScene && flag_elem_over) {
            circle.check_circle(Lpt0_found, Lpt1_found, is_straight1, is_straight0);
            if (circle.flag_circle != Circle::flag_circle_e::CIRCLE_NONE) {
                elem_state = Scene::RingScene;
            } else {
                elem_state = Scene::NormalScene;
            }
        }

        // 障碍
        if (elem_state == Scene::NormalScene && flag_elem_over) {
            if (obstacle.process(predict_result, is_straight0, is_straight1)) {
                elem_state = Scene::ObstacleScene;
            } else {
                elem_state = Scene::NormalScene;
            }
        }
    }

    // 行车线处理 TODO：进行适配
    // if (elem_state == Scene::GARAGE) {
    //     garage.run_garage(predict_result);
    //     track_state = TrackState::TRACK_MIDDLE;
    //     if (garage.flag_garage == Garage::flag_garage_e::GARAGE_NONE) {
    //         elem_state = Scene::NormalScene;
    //         flag_elem_over = false;
    //     }
    // } else if (elem_state == Scene::BridgeScene) {
    //     ramp.run_ramp();
    //     if (ramp.flag_ramp == Ramp::flag_ramp_e::RAMP_NONE) {
    //         elem_state = Scene::NormalScene;
    //         flag_elem_over = false;
    //     } else {
    //         track_state = TrackState::TRACK_MIDDLE;
    //     }

    if (elem_state == Scene::RingScene) {
        circle.run_circle(rpts0s_num, rpts1s_num, Lpt1_found, 
            ipts1_num, rptsc1_num, Lpt1_rpts1s_id, Lpt0_found, 
            ipts0_num, rptsc0_num, Lpt0_rpts0s_id);
        ++circle.circle_route;
        std::cout << "Circle State: " << circle.getCircleState() << std::endl;
        switch (circle.flag_circle) {
            case Circle::flag_circle_e::CIRCLE_LEFT_BEGIN:
                track_state = TrackState::TRACK_RIGHT;
                break;
            case Circle::flag_circle_e::CIRCLE_LEFT_IN:
                track_state = TrackState::TRACK_LEFT;
                break;
            case Circle::flag_circle_e::CIRCLE_LEFT_RUNNING:
                track_state = TrackState::TRACK_RIGHT;
		break;
            case Circle::flag_circle_e::CIRCLE_LEFT_OUT:
                track_state = TrackState::TRACK_LEFT;
                break;
            case Circle::flag_circle_e::CIRCLE_RIGHT_BEGIN:
                track_state = TrackState::TRACK_LEFT;
                break;
            case Circle::flag_circle_e::CIRCLE_RIGHT_IN:
                track_state = TrackState::TRACK_RIGHT;
                break;
            case Circle::flag_circle_e::CIRCLE_RIGHT_RUNNING:
                track_state = TrackState::TRACK_LEFT;
                break;
            case Circle::flag_circle_e::CIRCLE_RIGHT_OUT:
                track_state = TrackState::TRACK_RIGHT;
                break;
            default:
                track_state = TrackState::TRACK_MIDDLE;
                break;
        }
        if (circle.flag_circle == Circle::flag_circle_e::CIRCLE_NONE) {
            elem_state = Scene::NormalScene;
            circle.circle_route = 0;
            flag_elem_over = false;
        }
    } else if (elem_state == Scene::CrossScene) {
        int ret_state = cross.run_cross(Lpt0_found, Lpt1_found, rpts1s_num,
            rpts0s_num, ipts0_num, rptsc0_num, Lpt0_rpts0s_id, 
            ipts1_num, rptsc1_num, Lpt0_rpts0s_id, imagePath,
            rpts0s, rpts1s);
        ++cross.cross_route;
        std::cout << "Cross State: " << cross.flag_cross << std::endl;
        switch (ret_state) {
            case 0:
                track_state = TrackState::TRACK_LEFT;
                break;
            case 1:
                track_state = TrackState::TRACK_RIGHT;
                break;
            default:
                track_state = TrackState::TRACK_MIDDLE;
                break;
        }
        if (cross.flag_cross == Cross::flag_cross_e::CROSS_NONE) {
            elem_state = Scene::NormalScene;
            cross.cross_route = 0;
            flag_elem_over = false;
        }
    } else if (elem_state == Scene::ObstacleScene) {
        obstacle.run(predict, is_straight0, is_straight1);
        track_offset = obstacle.getTrackOffset();
        switch (obstacle.flag_obstacle_pos)  // 根据障碍物位置调整巡线状态
        {
        case Obstacle::ObstaclePos::Left:
            track_state = TrackState::TRACK_LEFT;
            break;
        case Obstacle::ObstaclePos::Right:
            track_state = TrackState::TRACK_RIGHT;
            break;
        default:
            break;
        }
        if (obstacle.current_state == Obstacle::state::StateNone) {
            elem_state = Scene::NormalScene;
            flag_elem_over = false;
            track_offset = ROAD_WIDTH / 2.0f;
        }
    } else {
        track_offset = ROAD_WIDTH / 2.0f;
        elem_state = Scene::NormalScene;
    }

    /* ***************************************************************** */
    /* **************************** 中线处理 **************************** */
    /* ***************************************************************** */
    // 十字根据远线
    // else if (flag_cross == CROSS_IN) {
    if (cross.flag_cross == Cross::flag_cross_e::CROSS_IN) {
        if (track_state == TrackState::TRACK_LEFT) {
            track_leftline(cross.far_rpts0s + cross.far_Lpt0_rpts0s_id,
                           cross.far_rpts0s_num - cross.far_Lpt0_rpts0s_id, rpts,
                           (int)round(ANGLE_DIST / SAMPLE_DIST),
                           PIXEL_PER_METER * track_offset);
            rpts_num = cross.far_rpts0s_num - cross.far_Lpt0_rpts0s_id;
        } else {
            track_rightline(cross.far_rpts1s + cross.far_Lpt1_rpts1s_id,
                            cross.far_rpts1s_num - cross.far_Lpt1_rpts1s_id, rpts,
                            (int)round(ANGLE_DIST / SAMPLE_DIST),
                            PIXEL_PER_METER * track_offset);
            rpts_num = cross.far_rpts1s_num - cross.far_Lpt1_rpts1s_id;
        }
        // 透视 -> 原图
        for (int i = 0; i < rpts_num; i++)
            _imgprocess.mapPerspective(rpts[i][0], rpts[i][1], rpts[i], 1);

    } else { // 正常巡线
        if (track_state == TrackState::TRACK_LEFT) {
            rpts = rptsc0;
            rpts_num = rptsc0_num;
        } else if (track_state == TrackState::TRACK_RIGHT) {
            rpts = rptsc1;
            rpts_num = rptsc1_num;
        } else {
            rpts = rptscs;
            rpts_num = rptscs_num;
        }
    }




    /* ***************************************************************** */
    /* **************************** 偏差计算 **************************** */
    /* ***************************************************************** */

    aim_distance_f = 0.8f;
    aim_distance_n = 0.4f;
    aim_angle_p_k = 0.01f;
    aim_angle_p = motion.params.turnP;
    aim_angle_d = motion.params.turnD;

    // 找最近点(起始点中线归一化)
    float min_dist = 1e10;
    int begin_id = -1;
    bool flag_rpts = false;
    for (int i = 0; i < rpts_num; i++) {
        float dx = rpts[i][0] - cx;
        float dy = rpts[i][1] - cy;
        float dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            begin_id = i;
        }
    }
    begin_id = begin_id + element_begin_id >= rpts_num ? begin_id
                                                       : begin_id + element_begin_id;

    // 特殊模式下, 不找最近点 (由于边线会绕一圈回来, 导致最近点为边线最后一个点, 从而中线无法正常生成)
    if (cross.flag_cross == Cross::flag_cross_e::CROSS_IN) {
        begin_id = 0;
    }

    // 中线有点, 同时最近点不是最后几个点
    if (begin_id >= 0 && rpts_num - begin_id >= 3) {
        // 找到中线
        flag_rpts = true;

        // 归一化中线
        rpts[begin_id][0] = cx;
        rpts[begin_id][1] = cy;
        rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
        resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num,
                        SAMPLE_DIST * PIXEL_PER_METER);

        aim_idx__far = clip(round(aim_distance_f / SAMPLE_DIST), 0, rptsn_num - 1);
        aim_idx_near = clip(round(aim_distance_n / SAMPLE_DIST), 0, rptsn_num - 1);

        std::vector<POINT> v_center(4);  // 三阶贝塞尔曲线
        v_center[0] = {(int)cx, (int)cy};
        v_center[1] = {(int)rptsn[aim_idx_near][0], (int)(ROWSIMAGE * (1 - aim_distance_n))};
        v_center[2] = {(int)rptsn[(int)((aim_idx__far + aim_idx_near) / 2)][0],
                       (int)(ROWSIMAGE * (1 - (aim_distance_f + aim_distance_n) / 2))};
        v_center[3] = {(int)rptsn[aim_idx__far][0], (int)(ROWSIMAGE * (1 - aim_distance_f))};
        
        
        bezier_line = Bezier(0.03, v_center);

        // 计算远锚点偏差值
        float dx = bezier_line[bezier_line.size() - 1].x - cx;  // rptsn[aim_idx__far][0] - cx;
        float dy = cy - bezier_line[bezier_line.size() - 1].y;  // cy - rptsn[aim_idx__far][1];
        float error_far = (-atan2f(dx, dy) * 180 / PI);
        assert(!isnan(error_far));

        // 计算近锚点偏差值
        float dx_near = bezier_line[bezier_line.size() / 2].x - cx;  // rptsn[aim_idx_near][0] - cx;
        float dy_near = cy - bezier_line[bezier_line.size() / 2].y;  // cy - rptsn[aim_idx_near][1];
        float error_near = (-atan2f(dx_near, dy_near) * 180 / PI);
        assert(!isnan(error_near));

        aim_angle = error_far * 0.9 + error_near * 0.1;

        aim_sigma = sigma(rptsn + aim_idx_near, aim_idx__far - aim_idx_near);
        // aim_sigma = sigma(v_center);

    } else {
        // 中线点过少
        flag_rpts = false;
        aim_angle = aim_angle_last;
        aim_sigma = 100.0f;

        // 环岛内部丢线
        if (circle.flag_circle > 2)
            aim_angle = aim_angle_last * 1.39f;

        // 十字丢线
        if (cross.flag_cross != Cross::flag_cross_e::CROSS_NONE)
            aim_angle = aim_angle_last;

        // 斑马线丢线
        // if (garage.flag_garage >= Garage::flag_garage_e::GARAGE_PASS)
        //     aim_angle = aim_angle_last;
        // else if (garage.flag_garage >= Garage::flag_garage_e::GARAGE_IN)
        //     aim_angle = aim_angle_last * 1.35f;
    }


    /* ***************************************************************** */
    /* **************************** 速度判定 **************************** */
    /* ***************************************************************** */

    // if (garage.flag_garage == Garage::flag_garage_e::GARAGE_OUT) {
    //     aim_speed = _config.speed_garage_out;
    // }  // 出库
    // else if (garage.flag_garage == Garage::flag_garage_e::GARAGE_IN) {
    //     aim_speed = _config.speed_garage_in;
    // }  // 入库
    // else if (ramp.flag_ramp == Ramp::flag_ramp_e::RAMP_UP) {
    //     aim_speed = _config.speed_ramp_up;
    // }  // 坡道 上
    // else if (ramp.flag_ramp == Ramp::flag_ramp_e::RAMP_DOWN) {
    //     aim_speed = _config.speed_ramp_down;
    // }  // 坡道 下
    // else if (rescue.flag_rescue != Rescue::RESCUE_NONE) {
    //     if (rescue.flag_rescue == Rescue::RESCUE_DETECTION_LEFT ||
    //     rescue.flag_rescue == Rescue::RESCUE_DETECTION_RIGHT) {
    //         aim_speed = _config.speed_rescue_detection;
    //     } else if (rescue.flag_rescue == Rescue::RESCUE_IN_LEFT ||
    //     rescue.flag_rescue == Rescue::RESCUE_IN_RIGHT) {
    //         aim_speed = _config.speed_rescue_in;
    //     } else if (rescue.flag_rescue == Rescue::RESCUE_IN_LEFT ||
    //     rescue.flag_rescue == Rescue::RESCUE_IN_RIGHT) {
    //         aim_speed = _config.speed_rescue_in;
    //     } else if (rescue.flag_rescue == Rescue::RESCUE_STOP_LEFT ||
    //     rescue.flag_rescue == Rescue::RESCUE_STOP_RIGHT) {
    //         aim_speed = _config.speed_rescue_stop;
    //     } else if (rescue.flag_rescue == Rescue::RESCUE_OUT_LEFT ||
    //     rescue.flag_rescue == Rescue::RESCUE_OUT_RIGHT) {
    //         aim_speed = _config.speed_rescue_out;
    //     } else if (rescue.flag_rescue == Rescue::RESCUE_ADJUST_LEFT ||
    //     rescue.flag_rescue == Rescue::RESCUE_ADJUST_RIGHT) {
    //         aim_speed = _config.speed_rescue_adjust;
    //     } else {
    //         aim_speed = _config.speed_base;
    //     }
    // } // 救援区
    // else if ((circle.flag_circle > 4 && circle.flag_circle < 7) || 
    // (circle.flag_circle > 0 && circle.flag_circle < 3)) {
    //     aim_speed = _config.speed_base;
    // }  // 出入环岛
    // else if (circle.flag_circle > 6 || circle.flag_circle == 3 || circle.flag_circle == 4) {
    //     aim_speed = _config.speed_circle;
    // }  // 环岛内部加速
    // else if (cross.flag_cross != Cross::flag_cross_e::CROSS_NONE) {
    //     aim_speed = _config.speed_cross;
    // }  // 十字速度
    // else if (danger.flag_danger != Danger::flag_danger_e::DANGER_NONE) {
    // 	aim_speed = _config.speed_danger;
    // }
    // else {
    //     // 根据偏差方差加减速
    //     if (abs(aim_sigma) < 10.0) {
    //         aim_speed_shift += 10.f;
    //     } else {
    //         aim_speed_shift -= speed_diff;
    //     }

    //     // 速度限幅
    //     aim_speed_shift = aim_speed_shift > _config.speed_up ? _config.speed_up : aim_speed_shift < _config.speed_base ? _config.speed_base : aim_speed_shift;
    //     aim_speed = aim_speed_shift;
    // }

    // // 停车
    // if (garage.flag_garage == Garage::flag_garage_e::GARAGE_STOP) {
    //     aim_speed = -0.0;
    // }

    /* ***************************************************************** */
    /* **************************** 运行控制 **************************** */
    /* ***************************************************************** */

    // 偏差限幅
    aim_angle = aim_angle > 500.0f ? 500.0f : aim_angle < -500.0f ? -500.0f
                                                                  : aim_angle;

    // 偏差滤波
    float aim_angle_filter = filter(aim_angle);
    aim_angle_last = aim_angle_filter;

    // 动态 P 项, 出入库禁止
    // if ( elem_state != Scene::GARAGE &&
    //     ((is_curve0 && track_state == TRACK_LEFT) || (is_curve1 && track_state == TRACK_RIGHT))) {
    //     aim_angle_p += fabs(aim_angle_filter) * aim_angle_p_k;
    //     aim_angle_p = aim_angle_p > _config.steering_p * 3.0f ? _config.steering_p * 3.0f
    //                                                            : aim_angle_p;
    // }

    // 计算舵机 PID
    int aim_angle_pwm = 0;
    aim_angle_pwm = (int)(pid_realize_a(aim_angle_filter, 0.0f, aim_angle_p, aim_angle_d) + 0.5f);
    // cout << "> aim_angle: " << aim_angle_pwm << endl;
    aim_angle_pwm = 5000 + clip(aim_angle_pwm, -1000, 1000);
    src.steering_pwm = aim_angle_pwm;


    // 绘图
    if (_is_result) {
        // 模式
        cv::putText(result_img, sceneToString(elem_state), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        // 巡线状态
        cv::putText(result_img, trackStateToString(track_state), cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

        // 十字 ----------------------------------------------------
        if (cross.flag_cross == Cross::flag_cross_e::CROSS_IN) {
            // 角点
            if (cross.far_Lpt0_found) {  // 绿色
                _imgprocess.mapPerspective(cross.far_rpts0s[cross.far_Lpt0_rpts0s_id][0], 
                    cross.far_rpts0s[cross.far_Lpt0_rpts0s_id][1], trans, 1);
                cv::circle(result_img, cv::Point2f(trans[0], trans[1]), 8, cv::Scalar(0, 255, 0), 2, 8);
            }
            if (cross.far_Lpt1_found) {  // 绿色
                _imgprocess.mapPerspective(cross.far_rpts1s[cross.far_Lpt1_rpts1s_id][0], 
                    cross.far_rpts1s[cross.far_Lpt1_rpts1s_id][1], trans, 1);
                cv::circle(result_img, cv::Point2f(trans[0], trans[1]), 8, cv::Scalar(0, 255, 0), 2, 8);
            }

            // 远线
            for (int a = 0; a < cross.far_rpts0s_num; a++) {  // 边线等距采样 左 十字远线
                _imgprocess.mapPerspective(cross.far_rpts0s[a][0], cross.far_rpts0s[a][1], trans, 1);
                if ((int)trans[1] >= 0 && (int)trans[1] < result_img.rows &&
                    (int)trans[0] >= 0 && (int)trans[0] < result_img.cols) {
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[0] = 0;
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[1] = 238;
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[2] = 238;
                }
            }
            for (int a = 0; a < cross.far_rpts1s_num; a++) {  // 边线等距采样 右 十字远线
                _imgprocess.mapPerspective(cross.far_rpts1s[a][0], cross.far_rpts1s[a][1], trans, 1);
                if ((int)trans[1] >= 0 && (int)trans[1] < result_img.rows &&
                    (int)trans[0] >= 0 && (int)trans[0] < result_img.cols) {
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[0] = 238;
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[1] = 238;
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[2] = 0;
                }
            }
        }
        // 正常巡线 --------------------------------------------
        else
        {
            // 角点
            if (Lpt0_found) {  // 绿色
                _imgprocess.mapPerspective(rpts0s[Lpt0_rpts0s_id][0], rpts0s[Lpt0_rpts0s_id][1], trans, 1);
                cv::circle(result_img, cv::Point2f(trans[0], trans[1]), 8, cv::Scalar(0, 255, 0), 2, 8);
            }
            if (Lpt1_found) {  // 绿色
                _imgprocess.mapPerspective(rpts1s[Lpt1_rpts1s_id][0], rpts1s[Lpt1_rpts1s_id][1], trans, 1);
                cv::circle(result_img, cv::Point2f(trans[0], trans[1]), 8, cv::Scalar(0, 255, 0), 2, 8);
            }

            // 边线
            for (int a = 0; a < rpts0s_num; a++) {  // 边线等距采样 左
                _imgprocess.mapPerspective(rpts0s[a][0], rpts0s[a][1], trans, 1);
                if ((int)trans[1] >= 0 && (int)trans[1] < result_img.rows &&
                    (int)trans[0] >= 0 && (int)trans[0] < result_img.cols) {
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[0] = 0;
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[1] = 238;
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[2] = 238;
                }
            }
            for (int a = 0; a < rpts1s_num; a++) {  // 边线等距采样 右
                _imgprocess.mapPerspective(rpts1s[a][0], rpts1s[a][1], trans, 1);
                if ((int)trans[1] >= 0 && (int)trans[1] < result_img.rows &&
                    (int)trans[0] >= 0 && (int)trans[0] < result_img.cols) {
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[0] = 238;
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[1] = 238;
                    result_img.at<cv::Vec3b>((int)trans[1], (int)trans[0])[2] = 0;
                }
            }
        }

        // 中线
        for (int a = 0; a < rptscs_num; a++) {  // 原图中线
            if ((int)rptscs[a][1] >= 0 && (int)rptscs[a][1] < result_img.rows &&
                (int)rptscs[a][0] >= 0 && (int)rptscs[a][0] < result_img.cols) {
                result_img.at<cv::Vec3b>((int)rptscs[a][1], (int)rptscs[a][0])[0] = 0;
                result_img.at<cv::Vec3b>((int)rptscs[a][1], (int)rptscs[a][0])[1] = 0;
                result_img.at<cv::Vec3b>((int)rptscs[a][1], (int)rptscs[a][0])[2] = 0;
            }
        }
        for (int a = 0; a < rptsc0_num; a++) {  // 左中线
            if ((int)rptsc0[a][1] >= 0 && (int)rptsc0[a][1] < result_img.rows &&
                (int)rptsc0[a][0] >= 0 && (int)rptsc0[a][0] < result_img.cols) {
                result_img.at<cv::Vec3b>((int)rptsc0[a][1], (int)rptsc0[a][0])[0] = 0;
                result_img.at<cv::Vec3b>((int)rptsc0[a][1], (int)rptsc0[a][0])[1] = 238;
                result_img.at<cv::Vec3b>((int)rptsc0[a][1], (int)rptsc0[a][0])[2] = 238;
            }
        }
        for (int a = 0; a < rptsc1_num; a++) {  // 右中线
            if ((int)rptsc1[a][1] >= 0 && (int)rptsc1[a][1] < result_img.rows &&
                (int)rptsc1[a][0] >= 0 && (int)rptsc1[a][0] < result_img.cols) {
                result_img.at<cv::Vec3b>((int)rptsc1[a][1], (int)rptsc1[a][0])[0] = 238;
                result_img.at<cv::Vec3b>((int)rptsc1[a][1], (int)rptsc1[a][0])[1] = 238;
                result_img.at<cv::Vec3b>((int)rptsc1[a][1], (int)rptsc1[a][0])[2] = 0;
            }
        }

        if (flag_rpts) {
            // 贝塞尔曲线
            for (auto p : bezier_line)
                cv::circle(result_img, Point(p.x, p.y), 1, Scalar(0, 0, 255), 5);

            // 归一化中线
            for (int a = 0; a < rptsn_num; a++) {
                if ((int)rptsn[a][1] >= 0 && (int)rptsn[a][1] < result_img.rows &&
                    (int)rptsn[a][0] >= 0 && (int)rptsn[a][0] < result_img.cols) {
                    result_img.at<cv::Vec3b>((int)rptsn[a][1], (int)rptsn[a][0])[0] = 0;
                    result_img.at<cv::Vec3b>((int)rptsn[a][1], (int)rptsn[a][0])[1] = 0;
                    result_img.at<cv::Vec3b>((int)rptsn[a][1], (int)rptsn[a][0])[2] = 255;
                }
            }

            // 预瞄点
            cv::circle(result_img, cv::Point2f(rptsn[aim_idx__far][0], rptsn[aim_idx__far][1]),
                       10, cv::Scalar(0, 0, 255), 2, 8);
            cv::circle(result_img, cv::Point2f(rptsn[aim_idx_near][0], rptsn[aim_idx_near][1]),
                       10, cv::Scalar(0, 0, 255), 2, 8);
        }
    }
}

/**
 * @brief 赛道线识别
 *
 * @param imageBinary 赛道识别基准图像
 */
void Tracking::trackRecognition(Mat &imageBinary)
{
    imagePath = imageBinary;
    trackRecognition(false, 0);
}

/**
 * @brief 显示赛道线识别结果
 *
 * @param trackImage 需要叠加显示的图像
 */
void Tracking::drawImage(Mat &trackImage)
{
    for (size_t i = 0; i < edge_left.size(); i++)
    {
        cv::circle(trackImage, cv::Point(edge_left[i].y, edge_left[i].x), 1,
               cv::Scalar(0, 255, 0), -1); // 绿色点
    }
    for (size_t i = 0; i < edge_right.size(); i++)
    {
        cv::circle(trackImage, cv::Point(edge_right[i].y, edge_right[i].x), 1,
               cv::Scalar(0, 255, 255), -1); // 黄色点
    }

    for (size_t i = 0; i < spurroad.size(); i++)
    {
        cv::circle(trackImage, cv::Point(spurroad[i].y, spurroad[i].x), 3,
               cv::Scalar(0, 0, 255), -1); // 红色点
    }

    putText(trackImage, to_string(validRowsRight) + " " + to_string(stdevRight),
            Point(COLSIMAGE - 100, ROWSIMAGE - 50), FONT_HERSHEY_TRIPLEX, 0.3,
            Scalar(0, 0, 255), 1, CV_AA);
    putText(trackImage, to_string(validRowsLeft) + " " + to_string(stdevLeft),
            Point(20, ROWSIMAGE - 50), FONT_HERSHEY_TRIPLEX, 0.3,
            Scalar(0, 0, 255), 1, CV_AA);
}

/**
 * @brief 边缘斜率计算
 *
 * @param v_edge
 * @param img_height
 * @return double
 */
double Tracking::stdevEdgeCal(vector<POINT> &v_edge, int img_height)
{
    if (v_edge.size() < static_cast<size_t>(img_height / 4))
    {
        return 1000;
    }
    vector<int> v_slope;
    int step = 10; // v_edge.size()/10;
    for (size_t i = step; i < v_edge.size(); i += step)
    {
        if (v_edge[i].x - v_edge[i - step].x)
            v_slope.push_back((v_edge[i].y - v_edge[i - step].y) * 100 /
                              (v_edge[i].x - v_edge[i - step].x));
    }
    if (v_slope.size() > 1)
    {
        double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
        double mean = sum / v_slope.size(); // 均值
        double accum = 0.0;
        for_each(begin(v_slope), end(v_slope),
                 [&](const double d)
                 { accum += (d - mean) * (d - mean); });

        return sqrt(accum / (v_slope.size() - 1)); // 方差
    }
    else
        return 0;
}

void Tracking::slopeCal(vector<POINT> &edge, int index)
{
    if (index <= 4)
    {
        return;
    }
    float temp_slop1 = 0.0, temp_slop2 = 0.0;
    if (edge[index].x - edge[index - 2].x != 0)
    {
        temp_slop1 = (float)(edge[index].y - edge[index - 2].y) * 1.0f /
                     ((edge[index].x - edge[index - 2].x) * 1.0f);
    }
    else
    {
        temp_slop1 = edge[index].y > edge[index - 2].y ? 255 : -255;
    }
    if (edge[index].x - edge[index - 4].x != 0)
    {
        temp_slop2 = (float)(edge[index].y - edge[index - 4].y) * 1.0f /
                     ((edge[index].x - edge[index - 4].x) * 1.0f);
    }
    else
    {
        edge[index].slope = edge[index].y > edge[index - 4].y ? 255 : -255;
    }
    if (abs(temp_slop1) != 255 && abs(temp_slop2) != 255)
    {
        edge[index].slope = (temp_slop1 + temp_slop2) * 1.0 / 2;
    }
    else if (abs(temp_slop1) != 255)
    {
        edge[index].slope = temp_slop1;
    }
    else
    {
        edge[index].slope = temp_slop2;
    }
}

/**
 * @brief 边缘有效行计算：左/右
 *
 */
void Tracking::validRowsCal(void)
{
    // 左边有效行
    validRowsLeft = 0;
    if (pointsEdgeLeft.size() > 1)
    {
        for (size_t i = pointsEdgeLeft.size() - 1; i >= 1; i--)
        {
            if (pointsEdgeLeft[i].y > 2 && pointsEdgeLeft[i - 1].y >= 2)
            {
                validRowsLeft = i + 1;
                break;
            }
            if (pointsEdgeLeft[i].y < 2 && pointsEdgeLeft[i - 1].y >= 2)
            {
                validRowsLeft = i + 1;
                break;
            }
        }
    }

    // 右边有效行
    validRowsRight = 0;
    if (pointsEdgeRight.size() > 1)
    {
        for (size_t i = pointsEdgeRight.size() - 1; i >= 1; i--)
        {
            if (pointsEdgeRight[i].y <= COLSIMAGE - 2 &&
                pointsEdgeRight[i - 1].y <= COLSIMAGE - 2)
            {
                validRowsRight = i + 1;
                break;
            }
            if (pointsEdgeRight[i].y >= COLSIMAGE - 2 &&
                pointsEdgeRight[i - 1].y < COLSIMAGE - 2)
            {
                validRowsRight = i + 1;
                break;
            }
        }
    }
}

/**
 * @brief 冒泡法求取集合中值
 *
 * @param vec 输入集合
 * @return int 中值
 */
int Tracking::getMiddleValue(vector<int> vec)
{
    if (vec.size() < 1)
        return -1;
    if (vec.size() == 1)
        return vec[0];

    int len = vec.size();
    while (len > 0)
    {
        bool sort = true; // 是否进行排序操作标志
        for (int i = 0; i < len - 1; ++i)
        {
            if (vec[i] > vec[i + 1])
            {
                swap(vec[i], vec[i + 1]);
                sort = false;
            }
        }
        if (sort) // 排序完成
            break;

        --len;
    }

    return vec[(int)vec.size() / 2];
}

/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
// 左手迷宫巡线
void Tracking::findline_lefthand_adaptive(cv::Mat img, int block_size, int clip_value,
                                          int x, int y, int pts[][2], int *num)
{
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < *num) && (half <= x) && (x <= img.cols - half - 1) &&
           (half <= y) && (y <= img.rows - half - 1) && (turn < 4))
    {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++)
        {
            for (int dx = -half; dx <= half; dx++)
            {
                local_thres += img.at<uint8_t>(y + dy, x + dx);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value =
            img.at<uint8_t>(y + dir_front[dir][1], x + dir_front[dir][0]);
        int frontleft_value =
            img.at<uint8_t>(y + dir_frontleft[dir][1], x + dir_frontleft[dir][0]);
        if (front_value < local_thres)
        {
            dir = (dir + 1) % 4;
            turn++;
        }
        else if (frontleft_value < local_thres)
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
        else
        {
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
// 右手迷宫巡线
// 该函数用于在二值化图像img上，从(x, y)点出发，采用右手法则自适应地跟踪黑线，
// 并将路径点存入pts，最终路径点数量存入*num。
// 参数说明：
//   img         - 输入的二值化图像
//   block_size  - 局部阈值计算的块大小（奇数）
//   clip_value  - 局部阈值修正值
//   x, y        - 起始点坐标
//   pts         - 输出路径点数组
//   *num        - 输入为最大点数，输出为实际点数
void Tracking::findline_righthand_adaptive(cv::Mat img, int block_size, int clip_value,
                                           int x, int y, int pts[][2], int *num)
{
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2; // 块半径
    int step = 0;              // 已采样点数
    int dir = 0;               // 当前前进方向（0上1右2下3左）
    int turn = 0;              // 连续转向次数（防止死循环）

    // 循环条件：未超出最大点数、未越界、未连续转向4次
    while ((step < *num) && (half <= x) && (x <= img.cols - half - 1) &&
           (half <= y) && (y <= img.rows - half - 1) && (turn < 4))
    {
        // 计算当前位置的局部阈值
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++)
        {
            for (int dx = -half; dx <= half; dx++)
            {
                local_thres += img.at<uint8_t>(y + dy, x + dx);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        // 获取前方和右前方像素值
        int front_value =
            img.at<uint8_t>(y + dir_front[dir][1], x + dir_front[dir][0]);
        int frontright_value =
            img.at<uint8_t>(y + dir_frontright[dir][1], x + dir_frontright[dir][0]);

        // 右手法则决策
        if (front_value < local_thres)
        {
            // 前方为黑，右转（顺时针），尝试新方向
            dir = (dir + 3) % 4;
            turn++;
        }
        else if (frontright_value < local_thres)
        {
            // 右前方为黑，直行，采样新点
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
        else
        {
            // 右前方为白，右斜前进并左转（逆时针），采样新点
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step; // 返回实际采样点数
}




// 点集三角滤波
void Tracking::blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) {
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
void Tracking::resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2,
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
void Tracking::local_angle_points(float pts_in[][2], int num, float angle_out[],
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
void Tracking::nms_angle(float angle_in[], int num, float angle_out[], int kernel) {
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

// 左边线跟踪中线
void Tracking::track_leftline(float pts_in[][2], int num, float pts_out[][2],
                    int approx_num, float dist) {
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] -
                   pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] -
                   pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);

        dx /= dn;
        dy /= dn;

        pts_out[i][0] = pts_in[i][0] - dy * dist;
        pts_out[i][1] = pts_in[i][1] + dx * dist;
    }
}

// 右边线跟踪中线
void Tracking::track_rightline(float pts_in[][2], int num, float pts_out[][2],
                     int approx_num, float dist) {
    for (int i = 0; i < num; i++) { // 遍历每个点
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] -
                   pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] -
                   pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy); // 计算距离

        dx /= dn; // 归一化
        dy /= dn;

        pts_out[i][0] = pts_in[i][0] + dy * dist; // 右边线点向中线偏移
        pts_out[i][1] = pts_in[i][1] - dx * dist;
    }
}

// 直线拟合 (返回平均绝对误差)
float Tracking::fit_line(float pts[][2], int num, int cut_h) {
    if (num != 0) {
        std::vector<cv::Point> points;
        cv::Vec4f line_para;
        float k, b, mea = 0.0f;
        float trans[2];
        int y_counter = 0;

        for (int i = 0; i < num; i++, y_counter++) {
            _imgprocess.mapPerspective(pts[i][0], pts[i][1], trans, 1);
            if (trans[1] < cut_h)
                break;

            points.push_back(cv::Point(trans[0], trans[1]));
        }

        cv::fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

        k = line_para[1] / line_para[0];
        b = line_para[3] - k * line_para[2];

        for (int i = 0; i < y_counter; i++)
            mea += fabs(k * points[i].x + b - points[i].y);

        return (float)(mea / y_counter);
    }

    return 100.0f;
}

string Tracking::trackstateToString(TrackState state)
{
    switch (state)
    {
    case TRACK_LEFT:
        return "LEFT";
    case TRACK_RIGHT:
        return "RIGHT";
    case TRACK_MIDDLE:
        return "MIDDLE";

    }
}
