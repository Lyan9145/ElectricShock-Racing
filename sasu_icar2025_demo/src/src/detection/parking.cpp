#include "../../include/parking.hpp"

using namespace cv;
using namespace std;

bool Parking::process(vector<PredictResult> &predict)
{
    detected = false; // 重置检测标志
    // 停车场检测逻辑
    for (const auto &result : predict)
    {
        if (result.type == LABEL_BATTERY && result.y > ROWSIMAGE * 0.2) // 检测到停车场标志
        {
            counter++;
            detected = true;
            if (counter > 3) // 连续3帧检测到停车场标志
            {
                if (result.x + result.width / 2 < COLSIMAGE / 2) // 判断停车场位置
                {
                    position = Position::Left; // 停车在左侧
                }
                else
                {
                    position = Position::Right; // 停车在右侧
                }
                cout << "Parking: Entering parking lot" << endl;
                counter = 0;
                return true;
            }
        }
    }
    return false; // 无停车场标志检测结果
}

void Parking::run(vector<PredictResult> &predict, UartStatus &status)
{
    process(predict);
    if (state == State::None && detected) // 初始状态且检测到停车
    {
        state = State::Enter; // 进入停车状态
        cout << "Parking: Entering parking lot" << endl;
    }
    else if (state == State::Enter)
    {
        if (!detected) // 停车场标志丢失
        {
            counter++;
            if (counter > 3) // 连续3帧无检测结果
            {
                startOdometer = status.distance; // 记录起始里程
                state = State::In; // 进入停车状态
                cout << "Parking: In parking lot" << endl;
            }
        }
        else // 有检测结果
        {
            counter = 0; // 重置计数器
        }
    }
    else if (state == State::In) // 在停车状态
    {
        if (status.distance - startOdometer >= stopDistance) // 距离达到停车距离
        {
            cout << "Parking: Stopped at parking lot" << endl;
            state = State::None; // 停车完成，回到初始状态
            counter = 0; // 重置计数器
        }
    }
}

