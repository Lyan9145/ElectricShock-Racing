#include "../include/controlcenter.hpp"

using namespace cv;
using namespace std;

/**
 * @brief 控制中心计算
 *
 * @param pointsEdgeLeft 赛道左边缘点集
 * @param pointsEdgeRight 赛道右边缘点集
 */

void ControlCenter::fitting(Tracking &track)
{
  sigmaCenter = 0;
  controlCenter = COLSIMAGE / 2;
  centerEdge.clear();
  vector<POINT> v_center(4); // 三阶贝塞尔曲线
  style = "STRIGHT";

  // 边缘斜率重计算（边缘修正之后）
  track.stdevLeft = track.stdevEdgeCal(track.pointsEdgeLeft, ROWSIMAGE);
  track.stdevRight = track.stdevEdgeCal(track.pointsEdgeRight, ROWSIMAGE);

  // 边缘有效行优化
  // if ((track.stdevLeft < 80 && track.stdevRight > 50) ||
  //     (track.stdevLeft > 60 && track.stdevRight < 50)) {
  //   validRowsCal(track.pointsEdgeLeft,
  //                track.pointsEdgeRight); // 边缘有效行计算
  //   track.pointsEdgeLeft.resize(validRowsLeft);
  //   track.pointsEdgeRight.resize(validRowsRight);
  // }

  if (track.pointsEdgeLeft.size() > 10 &&
      track.pointsEdgeRight.size() > 10) // 通过双边缘有效点的差来判断赛道类型
  {
    v_center[0] = {
        (track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2,
        (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};

    v_center[1] = {
        (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x +
         track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x) /
            2,
        (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y +
         track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y) /
            2};

    v_center[2] = {
        (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x +
         track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x) /
            2,
        (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y +
         track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y) /
            2};

    v_center[3] = {
        (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 0.9].x +
         track.pointsEdgeRight[track.pointsEdgeRight.size() * 0.9].x) /
            2,
        (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 0.9].y +
         track.pointsEdgeRight[track.pointsEdgeRight.size() * 0.9].y) /
            2};

    centerEdge = Bezier(0.03, v_center);

    style = "STRIGHT";
  }
  // 左单边
  else if ((track.pointsEdgeLeft.size() > 0 &&
            track.pointsEdgeRight.size() <= 10) ||
           (track.pointsEdgeLeft.size() > 0 &&
            track.pointsEdgeRight.size() > 0 &&
            track.pointsEdgeLeft[0].x - track.pointsEdgeRight[0].x >
                ROWSIMAGE / 2))
  {
    style = "RIGHT";
    centerEdge = centerCompute(track.pointsEdgeLeft, 0);
  }
  // 右单边
  else if ((track.pointsEdgeRight.size() > 0 &&
            track.pointsEdgeLeft.size() <= 10) ||
           (track.pointsEdgeRight.size() > 0 &&
            track.pointsEdgeLeft.size() > 0 &&
            track.pointsEdgeRight[0].x - track.pointsEdgeLeft[0].x >
                ROWSIMAGE / 2))
  {
    style = "LEFT";
    centerEdge = centerCompute(track.pointsEdgeRight, 1);
  }
  else if (track.pointsEdgeLeft.size() > 10 &&
           track.pointsEdgeRight.size() == 0) // 左单边
  {
    v_center[0] = {track.pointsEdgeLeft[0].x,
                   (track.pointsEdgeLeft[0].y + COLSIMAGE - 1) / 2};

    v_center[1] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x,
                   (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y +
                    COLSIMAGE - 1) /
                       2};

    v_center[2] = {
        track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x,
        (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y +
         COLSIMAGE - 1) /
            2};

    v_center[3] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x,
                   (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y +
                    COLSIMAGE - 1) /
                       2};

    centerEdge = Bezier(0.02, v_center);

    style = "RIGHT";
  }
  else if (track.pointsEdgeLeft.size() == 0 &&
           track.pointsEdgeRight.size() > 10) // 右单边
  {
    v_center[0] = {track.pointsEdgeRight[0].x,
                   track.pointsEdgeRight[0].y / 2};

    v_center[1] = {track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x,
                   track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y /
                       2};

    v_center[2] = {
        track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x,
        track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y / 2};

    v_center[3] = {track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x,
                   track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y /
                       2};

    centerEdge = Bezier(0.02, v_center);

    style = "LEFT";
  }

  // 加权控制中心计算
  int controlNum = 1;
  for (auto p : centerEdge)
  {
    if (p.x < ROWSIMAGE / 2)
    {
      controlNum += ROWSIMAGE / 2;
      controlCenter += p.y * ROWSIMAGE / 2;
    }
    else
    {
      controlNum += (ROWSIMAGE - p.x);
      controlCenter += p.y * (ROWSIMAGE - p.x);
    }
  }
  if (controlNum > 1)
  {
    controlCenter = controlCenter / controlNum;
  }

  if (controlCenter > COLSIMAGE)
    controlCenter = COLSIMAGE;
  else if (controlCenter < 0)
    controlCenter = 0;

  // 控制率计算
  if (centerEdge.size() > 20)
  {
    vector<POINT> centerV;
    int filt = centerEdge.size() / 5;
    for (size_t i = filt; i < centerEdge.size() - filt;
         i++) // 过滤中心点集前后1/5的诱导性
    {
      centerV.push_back(centerEdge[i]);
    }
    sigmaCenter = sigma(centerV);
  }
  else
    sigmaCenter = 1000;
}

/**
 * @brief 车辆冲出赛道检测（保护车辆）
 *
 * @param track
 * @return true
 * @return false
 */
bool ControlCenter::derailmentCheck(Tracking track)
{
  if (track.pointsEdgeLeft.size() < 30 &&
      track.pointsEdgeRight.size() < 30) // 防止车辆冲出赛道
  {
    countOutlineA++;
    countOutlineB = 0;
    if (countOutlineA > 20)
      return true;
  }
  else
  {
    countOutlineB++;
    if (countOutlineB > 50)
    {
      countOutlineA = 0;
      countOutlineB = 50;
    }
  }
  return false;
}

/**
 * @brief 显示赛道线识别结果
 *
 * @param centerImage 需要叠加显示的图像
 */
void ControlCenter::drawImage(Tracking track, Mat &centerImage)
{
  // 赛道边缘绘制
  for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
  {
    circle(centerImage,
           Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
           Scalar(0, 255, 0), -1); // 绿色点
  }
  for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
  {
    circle(centerImage,
           Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
           Scalar(0, 255, 255), -1); // 黄色点
  }

  // 绘制中心点集
  for (size_t i = 0; i < centerEdge.size(); i++)
  {
    circle(centerImage, Point(centerEdge[i].y, centerEdge[i].x), 1,
           Scalar(0, 0, 255), -1);
  }

  // 绘制加权控制中心：方向
  Rect rect(controlCenter, ROWSIMAGE - 20, 10, 20);
  rectangle(centerImage, rect, Scalar(0, 0, 255), CV_FILLED);

  // 详细控制参数显示
  int dis = 20;
  string str;
  putText(centerImage, style, Point(COLSIMAGE - 60, dis), FONT_HERSHEY_PLAIN,
          1, Scalar(0, 0, 255), 1); // 赛道类型

  str = "Edge: " + formatDoble2String(track.stdevLeft, 1) + " | " +
        formatDoble2String(track.stdevRight, 1);
  putText(centerImage, str, Point(COLSIMAGE - 150, 2 * dis),
          FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右

  str = "Center: " + formatDoble2String(sigmaCenter, 2);
  putText(centerImage, str, Point(COLSIMAGE - 120, 3 * dis),
          FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 中心点方差

  putText(centerImage, to_string(controlCenter),
          Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 40), FONT_HERSHEY_PLAIN, 1.2,
          Scalar(0, 0, 255), 1); // 中心
}

/**
 * @brief 搜索十字赛道突变行（左下）
 *
 * @param pointsEdgeLeft
 * @return uint16_t
 */
uint16_t ControlCenter::searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
{
  uint16_t counter = 0;

  for (size_t i = 0; i < pointsEdgeLeft.size() - 10; i++)
  {
    if (pointsEdgeLeft[i].y >= 2)
    {
      counter++;
      if (counter > 3)
      {
        return i - 2;
      }
    }
    else
      counter = 0;
  }

  return 0;
}

/**
 * @brief 搜索十字赛道突变行（右下）
 *
 * @param pointsEdgeRight
 * @return uint16_t
 */
uint16_t ControlCenter::searchBreakRightDown(vector<POINT> pointsEdgeRight)
{
  uint16_t counter = 0;

  for (size_t i = 0; i < pointsEdgeRight.size() - 10; i++) // 寻找左边跳变点
  {
    if (pointsEdgeRight[i].y < COLSIMAGE - 2)
    {
      counter++;
      if (counter > 3)
      {
        return i - 2;
      }
    }
    else
      counter = 0;
  }

  return 0;
}

/**
 * @brief 赛道中心点计算：单边控制
 *
 * @param pointsEdge 赛道边缘点集
 * @param side 单边类型：左边0/右边1
 * @return vector<POINT>
 */
vector<POINT> ControlCenter::centerCompute(vector<POINT> pointsEdge, int side)
{
  int step = 4;                    // 间隔尺度
  int offsetWidth = COLSIMAGE / 2; // 首行偏移量
  int offsetHeight = 0;            // 纵向偏移量

  vector<POINT> center; // 控制中心集合

  if (side == 0) // 左边缘
  {
    uint16_t counter = 0, rowStart = 0;
    for (size_t i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
    {
      if (pointsEdge[i].y > 1)
      {
        counter++;
        if (counter > 2)
        {
          rowStart = i - 2;
          break;
        }
      }
      else
        counter = 0;
    }

    offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
    counter = 0;
    for (size_t i = rowStart; i < pointsEdge.size(); i += step)
    {
      int py = pointsEdge[i].y + offsetWidth;
      if (py > COLSIMAGE - 1)
      {
        counter++;
        if (counter > 2)
          break;
      }
      else
      {
        counter = 0;
        center.emplace_back(pointsEdge[i].x - offsetHeight, py);
      }
    }
  }
  else if (side == 1) // 右边沿
  {
    uint16_t counter = 0, rowStart = 0;
    for (size_t i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
    {
      if (pointsEdge[i].y < COLSIMAGE - 1)
      {
        counter++;
        if (counter > 2)
        {
          rowStart = i - 2;
          break;
        }
      }
      else
        counter = 0;
    }

    offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
    counter = 0;
    for (size_t i = rowStart; i < pointsEdge.size(); i += step)
    {
      int py = pointsEdge[i].y - offsetWidth;
      if (py < 1)
      {
        counter++;
        if (counter > 2)
          break;
      }
      else
      {
        counter = 0;
        center.emplace_back(pointsEdge[i].x - offsetHeight, py);
      }
    }
  }

  return center;
  // return Bezier(0.2,center);
}
