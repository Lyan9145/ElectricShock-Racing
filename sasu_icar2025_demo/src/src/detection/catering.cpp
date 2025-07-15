#include "../../include/catering.hpp"

using namespace cv;
using namespace std;


bool Catering::process(vector<PredictResult> predict)
{
    detected = false; // 无检测结果
    if (predict.empty())
    {
        return false; // 无检测结果
    }
    for (int i = 0; i < predict.size(); i++)
    {
        if (predict[i].type == LABEL_BURGER) // 检测到汉堡标志
        {
            if (predict[i].y + predict[i].height > ROWSIMAGE * 0.3) // 汉堡标志在下半部分
            {
                state = CateringState::Enter;
            }
            detected = true; // 有检测结果
            return true;
        }
    }
    return false; // 无汉堡标志检测结果
}


int Catering::run(vector<PredictResult> predict, UartStatus &status)
{
    if (state == CateringState::Enter) // 进入快餐店状态
    {
        if (!process(predict)) // 丢失检测
        {
            counter++;
            if (counter > 3) // 连续3帧无检测结果
            {
                state = CateringState::In; // 进入通道
                counter = 0; // 重置计数器
                start_odometer = status.distance; // 记录起始里程
                cout << "Catering: Entering, direction: " << (direction == CateringDirection::Left ? "Left" : "Right") << endl;
            }
        }
        else // 有检测结果
        {
            counter = 0; // 重置计数器
            for (int i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_BURGER) // 检测到汉堡标志
                {
                    direction = ((predict[i].x + predict[i].width / 2) < COLSIMAGE / 2) ? CateringDirection::Right : CateringDirection::Left; // 判断方向
                }
            }
        }
    }
    else if (state == CateringState::In) // 在快餐店状态
    {
        if (status.distance - start_odometer >= stop_distance) // 距离达到停车距离
        {
            state = CateringState::Stopping; // 准备减速停车状态
            cout << "Catering: Stopping" << endl;
        }
    }
    else if (state == CateringState::Stopping) // 减速停车状态
    {
        if (status.speed < 0.01f) // 是否已经停车
        {
            counter++;
            if (counter > 3) // 连续3帧停车状态
            {
                state = CateringState::Leave; // 准备离开快餐店状态
                start_odometer = status.distance; // 记录离开时的里程
                cout << "Catering: Stopped, preparing to leave" << endl;
            }
        }
        else
            counter = 0;

    }
    else if (state == CateringState::Leave) // 离开快餐店状态
    {
        if (status.distance - start_odometer >= stop_distance)
        {
            state = CateringState::None; // 重置为无快餐店状态
            direction = CateringDirection::Unknown; // 重置方向
            counter = 0; // 重置计数器
            start_odometer = 0.0f; // 重置起始里程
            cout << "Catering: Exit" << endl;
        }
    }

    return 1; // 返回操作成功标志
}


