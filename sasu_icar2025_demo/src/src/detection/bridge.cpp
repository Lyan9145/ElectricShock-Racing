#include "../../include/bridge.hpp"

using namespace cv;
using namespace std;

bool Bridge::process(vector<PredictResult> predict)
{
    bridgeEnable = false; // 桥区域使能标志
    if (predict.size() <= 0)
        return false;
        
    for (const auto &result : predict)
    {
        if (result.type == LABEL_BRIDGE && result.y + result.height > ROWSIMAGE * 0.4) // 桥区域检测
        {
            detectionCounter++;
            if (detectionCounter >= 2)
                bridgeEnable = true; // 桥区域使能标志
                return true;
        }
    }

    return false;
}

void Bridge::run(vector<PredictResult> predict)
{
    process(predict);
    if (state == State::None && bridgeEnable)
    {
        counter = 0; // 重置计数器
        state = State::Enter; // 进入桥区域
        cout << "Bridge: Enter" << endl;
    }
    else if (state == State::Enter)
    {
        if (!bridgeEnable)
        {
            counter++;
            if (counter >= 3) // 连续3帧未检测到桥区域
            {
                state = State::Up; // 桥区域上升
                cout << "Bridge: Up" << endl;
                counter = 0; // 重置计数器
            }
        }
        else
        {
            counter = 0;
        }
    }
    else if (state == State::Up)
    {
        counter++;
        if (counter >= accframes)
        {
            state = State::Down; // 桥区域下降
            cout << "Bridge: Down" << endl;
            counter = 0; // 重置计数器
        }
    }
    else if (state == State::Down)
    {
        counter++;
        if (counter >= 4)
        {
            state = State::None; // 桥区域结束
            cout << "Bridge: Out" << endl;
            counter = 0; // 重置计数器
            detectionCounter = 0; // 重置检测计数器
        }
    }
}


