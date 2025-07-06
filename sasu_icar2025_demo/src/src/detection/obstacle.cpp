#include "../../include/obstacle.hpp"

using namespace std;
using namespace cv;

bool Obstacle::process(vector<PredictResult> &predict, bool is_straight0, bool is_straight1)
{
    if (!is_straight0 && !is_straight1) // 非直道
    {
        return enable; // 无障碍
    }
    if (current_state == state::StateNone)
    {
        vector<PredictResult> resultsObs; // 锥桶AI检测数据
        for (size_t i = 0; i < predict.size(); i++)
        {
            if ((predict[i].type == LABEL_CONE || predict[i].type == LABEL_BLOCK || predict[i].type == LABEL_PEDESTRIAN) && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.4) // AI标志距离计算
                resultsObs.push_back(predict[i]);
        }
    
        if (resultsObs.size() <= 0) // 无障碍物检测结果
            enable = false; // 场景检测使能标志
        else
        {
            enable = true; // 有障碍物
        }
    }
    return enable;
}

int Obstacle::run(vector<PredictResult> &predict, float rpts0s[ROWSIMAGE][2], float rpts1s[ROWSIMAGE][2])
{
    if (!enable) // 场景检测使能标志
        return 0;

    vector<PredictResult> resultsObs; // 锥桶AI检测数据
    for (size_t i = 0; i < predict.size(); i++)
    {
        if ((predict[i].type == LABEL_CONE || predict[i].type == LABEL_BLOCK || predict[i].type == LABEL_PEDESTRIAN) && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.4) // AI标志距离计算
            resultsObs.push_back(predict[i]);
    }

    // if (resultsObs.size() <= 0) // 无障碍物检测结果
    // {
    //     enable = false;
    //     return 0;
    // }

    // 选取距离最近的锥桶
    int areaMax = 0; // 框面积
    int index = 0;   // 目标序号
    for (size_t i = 0; i < resultsObs.size(); i++)
    {
        int area = resultsObs[i].width * resultsObs[i].height;
        if (area >= areaMax)
        {
            index = i;
            areaMax = area;
        }
    }
    resultObs = resultsObs[index];

    if (current_state == state::StateNone && resultsObs.size() > 0) // 无障碍，遭遇障碍
    {
        current_state = state::EnterObstacle;      // 进入障碍区
        if (resultsObs[index].type == LABEL_BLOCK) // 黑色路障特殊处理
        {
            flag_obstacle_type = Obstacle::ObstacleType::Block;
        }
        else if (resultsObs[index].type == LABEL_PEDESTRIAN) // 行人特殊处理
        {
            flag_obstacle_type = Obstacle::ObstacleType::Pedestrian;
        }
        else if (resultsObs[index].type == LABEL_CONE) // 锥桶特殊处理
        {
            flag_obstacle_type = Obstacle::ObstacleType::Cone;
        }
        else
        {
            flag_obstacle_type = Obstacle::ObstacleType::ObstacleTypeNone;
        }
    }
    if (current_state == state::EnterObstacle) // 进入障碍区前
    {

        if (resultsObs.size() <= 0) // 丢失检测
        {
            obstacle_counter++; // 增加障碍计数器
            if (obstacle_counter > 20) // 滤波器，连续丢失进入下一阶段
            {
                current_state = state::InObstacle; // 进入障碍区中
                obstacle_counter = 0;              // 重置障碍计数器
            }
        }
        else
        {
            obstacle_counter = 0; // 重置障碍计数器
            // 障碍框底部两点进行透视变换
            _imgprocess.mapPerspective(resultObs.x, resultObs.y + resultObs.height, pointLeftTrans, 0);    // 左侧点透视变换
            _imgprocess.mapPerspective(resultObs.x + resultObs.width, resultObs.y + resultObs.height, pointRightTrans, 0); // 右侧点透视变换

            // rpts0s为左边线点集，找y最接近的x距离
            float minDistLeft = 1000.0f; // 左侧点距离左边线最近距离
            int leftIndex = 0;           // 左侧点索引
            float minYleft = 1000.0f;
            for (int i = 0; i < ROWSIMAGE; i++)
            {
                float minY = abs(pointLeftTrans[1] - rpts0s[i][1]); // 计算y轴距离
                if (minY < minYleft)
                {
                    minDistLeft = pointLeftTrans[0] - rpts0s[i][0];
                    leftIndex = i;
                    minYleft = minY; // 记录y最接近的点
                }
            }
    
            // 计算右侧点距离右边线最近距离 rpts1s为右边线点集
            float minDistRight = 1000.0f; // 右侧点距离右边线最近距离
            int rightIndex = 0;           // 右侧点索引
            float minYright = 1000.0f;
            for (int i = 0; i < ROWSIMAGE; i++)
            {
                float minY = abs(pointRightTrans[1] - rpts1s[i][1]); // 计算y轴距离
                if (minY < minYright)
                {
                    minDistRight = pointRightTrans[0] - rpts1s[i][0];
                    rightIndex = i;
                    minYright = minY; // 记录y最接近的点
                }
            }
    
            // 赛道外检测
            if (pointLeftTrans[0] > rpts1s[rightIndex][0] || pointRightTrans[0] < rpts0s[leftIndex][0])
            {
                flag_obstacle_pos = Obstacle::ObstaclePos::ObstaclePosNone; // 无障碍
                current_state = state::StateNone;                     // 无障碍
                obstacle_counter = 0;                            // 重置障碍计数器
                return 0;
            }
    
            // 确定避障方向：左侧或右侧
            // 规则：看那一侧的minDist较大
            if (minDistLeft >= minDistRight) // 左侧障碍
            {
                flag_obstacle_pos = Obstacle::ObstaclePos::Left;
                // 计算偏移量，使用滑动平均增加准确性
                if (track_offset == 0.0f)
                    track_offset = minDistLeft / PIXEL_PER_METER; // 左侧点偏移量
                else
                    track_offset = (track_offset + minDistLeft / PIXEL_PER_METER) / 2.0f; // 左侧点偏移量
            }
            else // 右侧障碍
            {
                flag_obstacle_pos = Obstacle::ObstaclePos::Right;
                // 计算偏移量
                if (track_offset == 0.0f)
                    track_offset = minDistRight / PIXEL_PER_METER; // 右侧点偏移量
                else
                    track_offset = (track_offset + minDistRight / PIXEL_PER_METER) / 2.0f; // 右侧点偏移量
            }
            printf("Obstacle: EnterObstacle, pos=%d, type=%d, track_offset=%.2f\n", flag_obstacle_pos, flag_obstacle_type, track_offset);
        }

    }
    if (current_state == state::InObstacle) // 在障碍区
    {
        if (resultsObs.size() <= 0) // 丢失检测
            obstacle_counter++;     // 增加障碍计数器
        else                        // 仍然有障碍物
            obstacle_counter = 0;   // 重置障碍计数器

        if (obstacle_counter > 100) // 离开障碍区
        {
            current_state = state::ExitObstacle;
        }
    }
    if (current_state == state::ExitObstacle) // 离开障碍区
    {
        // 重置变量
        current_state = state::StateNone; // 无障碍
        flag_obstacle_pos = Obstacle::ObstaclePos::ObstaclePosNone;
        flag_obstacle_type = Obstacle::ObstacleType::ObstacleTypeNone;
        track_offset = ROAD_WIDTH / 2.0f;
        enable = false;       // 禁用障碍检测
        obstacle_counter = 0; // 重置障碍计数器
    }

    return obstacle_counter;
}

float Obstacle::getTrackOffset()
{
    if (enable) // 场景检测使能标志
    {
        return track_offset; // 返回赛道偏移量
    }
    return ROAD_WIDTH / 2.0f; // 无障碍物，返回0
}

/**
 * @brief 图像绘制禁行区识别结果
 *
 * @param img 需要叠加显示的图像
 */
void Obstacle::drawImage(Mat &img)
{
    if (enable)
    {
        putText(img, "[2] Obstacle - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }
}

// void Obstacle::curtailTracking(Tracking &track, bool left)
// {
//     if (left) // 向左侧缩进
//     {
//         if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
//             track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

//         for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
//         {
//             track.pointsEdgeRight[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
//         }
//     }
//     else // 向右侧缩进
//     {
//         if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
//             track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

//         for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
//         {
//             track.pointsEdgeLeft[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
//         }
//     }
// }
