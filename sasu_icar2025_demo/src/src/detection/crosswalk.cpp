#include "../../include/crosswalk.hpp"

using namespace std;
using namespace cv;


bool StopArea::process(vector<PredictResult> predict)
{
    detected = false; // 重置检测标志
    if (predict.size() <= 0)
        return false;

    for (const auto &result : predict)
    {
        if (result.type == LABEL_CROSSWALK && result.y + result.height > ROWSIMAGE * 0.7) // 斑马线检测
        {
            // cout << "Crosswalk detected at (" << result.x << ", " << result.y << ")" << endl;
            detected = true; // 检测到斑马线
            return true;
        }
    }
    return false;
}

void StopArea::run(vector<PredictResult> predict, UartStatus &status)
{
    process(predict);
    if (state == State::Startup && detected)
    {
        state = State::Firstdet;
        cout << "Crosswalk: First detection" << endl;
    }
    else if (state == State::Firstdet)
    {
        if (!detected) // 斑马线检测
        {
            counter++;
            if (counter > 3)
            {
                startDistance = status.distance; // 记录起始距离
                lapstartTime = std::chrono::high_resolution_clock::now(); // 记录首次检测时间
                state = State::Firstpass; // 首次通过终点线
                cout << "Crosswalk: First pass" << endl;
            }
        }
        else
        {
            counter = 0; // 重置计数器
        }
    }
    else if (state == State::Firstpass)
    {
        if (status.distance - startDistance > passDistance)
        {
            // 进入飞行圈状态
            state = State::Flyinglap;
            cout << "Crosswalk: Flying lap" << endl;
            counter = 0; // 重置计数器
            startDistance = status.distance; // 记录起始距离
        }
    }
    else if (state == State::Flyinglap)
    {
        lapendTime = std::chrono::high_resolution_clock::now(); // 更新时间
        if (detected && status.distance - startDistance > 5) // 第二次检测斑马线
        {
            state = State::Seconddet; // 准备进入第二次检测斑马线
            cout << "Crosswalk: Second detection" << endl;
            startDistance = status.distance; // 记录起始距离
        }
    }
    else if (state == State::Seconddet)
    {
        lapendTime = std::chrono::high_resolution_clock::now(); // 更新时间
        if (!detected && status.distance - startDistance > passDistance) // 斑马线检测
        {
            counter++;
            if (counter > 2)
            {
                lapendTime = std::chrono::high_resolution_clock::now(); // 记录第二次检测时间
                state = State::Stop; // 第二次通过终点线
                cout << "Crosswalk: Stopping" << endl;
            }
        }
        else
        {
            counter = 0; // 重置计数器
        }
    }
    else if (state == State::Secondpass)
    {
        lapendTime = std::chrono::high_resolution_clock::now(); // 更新时间
        state = State::Stop; // 进入停车状态
        cout << "Crosswalk: Second pass" << endl;
    }
    if (state == State::Stop)
    {
        park = true; // 停车标志
        // cout << "Crosswalk: Stopped" << endl;
    }
}

void StopArea::drawUI(Mat &img)
{
    if (state < State::Flyinglap)
        return;

    // mm:ss:xxx
    string lapTimeStr = format("Lap Time: %02d:%02d:%03d",
        std::chrono::duration_cast<std::chrono::minutes>(
            lapendTime - lapstartTime).count(),
        std::chrono::duration_cast<std::chrono::seconds>(
            lapendTime - lapstartTime).count() % 60,
        std::chrono::duration_cast<std::chrono::milliseconds>(
            lapendTime - lapstartTime).count() % 1000);

    putText(img, lapTimeStr, Point(140, 220), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 255, 0), 2);
}



