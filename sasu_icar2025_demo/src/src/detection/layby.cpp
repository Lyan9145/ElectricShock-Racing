#include "../../include/layby.hpp"

using namespace cv;
using namespace std;

bool Layby::process(vector<PredictResult> &predict)
{
    detected = false;
    if (predict.empty())
        return false;
    for (int i = 0; i < predict.size(); i++)
    {
        if (predict[i].type == LABEL_COMPANY || predict[i].type == LABEL_SCHOOL)
        {
            detected = true;
            target = predict[i];
            return true;
        }
    }
}

int Layby::run(vector<PredictResult> &predict, UartStatus &status)
{
    if (state == LaybyState::None && detected)
    {
        cout << "Layby: Entering" << endl;
        state = LaybyState::Enter;
    }
    if (state == LaybyState::Enter)
    {
        if (!process(predict))
        {
            counter++;
            if (counter > 3)
            {
                cout << "Layby: In" << endl;
                state = LaybyState::In;
                counter = 0;
                start_odom = status.distance;
            }
        }
        else
        {
            counter = 0;
            // 确定方向
            if (target.x + target.width / 2 < COLSIMAGE / 2)
            {
                direction = LaybyDirection::Left;
                cout << "Layby: Direction Left" << endl;
            }
            else
            {
                direction = LaybyDirection::Right;
                cout << "Layby: Direction Right" << endl;
            }
        }
    }
    if (state == LaybyState::In)
    {
        if (status.distance - start_odom > stop_distance)
        {
            cout << "Layby: Stopping" << endl;
            state = LaybyState::Stopping;
        }
    }
    if (state == LaybyState::Stopping)
    {
        if (status.speed < 0.01f)
        {
            counter++;
            if (counter > 3)
            {
                cout << "Layby: Leave" << endl;
                state = LaybyState::Leave;
                counter = 0;
                start_odom = status.distance;
            }
        }
    }
    if (state == LaybyState::Leave)
    {
        if (status.distance - start_odom > stop_distance)
        {
            cout << "Layby: Out" << endl;
            state = LaybyState::None;
            direction = LaybyDirection::Unknown;
            counter = 0;
            start_odom = 0.0f;
        }
    }
}
