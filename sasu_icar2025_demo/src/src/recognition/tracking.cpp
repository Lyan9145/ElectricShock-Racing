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
  // for (int row = rowStart; row > rowCutUp; row--) // 有效行：10~220
  // {
  //   counterBlock = 0; // 色块计数器清空
  //   // 搜索色（block）块信息
  //   if (imageType == ImageType::Rgb) // 输入RGB图像
  //   {
  //     if (imagePath.at<Vec3b>(row, 1)[2] > 0)
  //     {
  //       startBlock[counterBlock] = 0;
  //     }
  //     for (int col = 1; col < COLSIMAGE; col++) // 搜索出每行的所有色块
  //     {
  //       if (imagePath.at<Vec3b>(row, col)[2] > 0 &&
  //           imagePath.at<Vec3b>(row, col - 1)[2] == 0)
  //       {
  //         startBlock[counterBlock] = col;
  //       }
  //       else
  //       {
  //         if (imagePath.at<Vec3b>(row, col)[2] == 0 &&
  //             imagePath.at<Vec3b>(row, col - 1)[2] > 0)
  //         {
  //           endBlock[counterBlock++] = col;
  //           if (counterBlock >= end(endBlock) - begin(endBlock))
  //             break;
  //         }
  //       }
  //     }
  //     if (imagePath.at<Vec3b>(row, COLSIMAGE - 1)[2] > 0)
  //     {
  //       if (counterBlock < end(endBlock) - begin(endBlock) - 1)
  //         endBlock[counterBlock++] = COLSIMAGE - 1;
  //     }
  //   }
  //   if (imageType == ImageType::Binary) // 输入二值化图像

  //   {
  //     if (imagePath.at<uchar>(row, 1) > 127)
  //     {
  //       startBlock[counterBlock] = 0;
  //     }
  //     for (int col = 1; col < COLSIMAGE; col++) // 搜索出每行的所有色块
  //     {
  //       if (imagePath.at<uchar>(row, col) > 127 &&
  //           imagePath.at<uchar>(row, col - 1) <= 127)
  //       {
  //         startBlock[counterBlock] = col;
  //       }
  //       else
  //       {
  //         if (imagePath.at<uchar>(row, col) <= 127 &&
  //             imagePath.at<uchar>(row, col - 1) > 127)
  //         {
  //           endBlock[counterBlock++] = col;
  //           if (counterBlock >= end(endBlock) - begin(endBlock))
  //             break;
  //         }
  //       }
  //     }
  //     if (imagePath.at<uchar>(row, COLSIMAGE - 1) > 127)
  //     {
  //       if (counterBlock < end(endBlock) - begin(endBlock) - 1)
  //         endBlock[counterBlock++] = COLSIMAGE - 1;
  //     }
  //   }

  // 原图找左边线 -------------------------------------------------------
  {
    int x1 = begin_x_l, y1 = begin_y_t;
    // 向左寻找白点
    if (imagePath.at<uint8_t>(y1, x1) < _config.threshold)
      for (x1--; x1 > 0; x1--)
        if (imagePath.at<uint8_t>(y1, x1) >= _config.threshold)
          break;
    // 向左寻找黑点
    for (; x1 > 0; x1--)
      if (imagePath.at<uint8_t>(y1, x1 - 1) < _config.threshold)
        break;
    // 向上寻找黑点
    if (x1 < BLOCK_SIZE / 2)
    {
      x1 = BLOCK_SIZE / 2;
      for (; y1 > IMAGE_HEIGHT * 2 / 3; y1--)
        if (imagePath.at<uint8_t>(y1 - 1, x1) < _config.threshold)
          break;
    }
    if (imagePath.at<uint8_t>(y1, x1) >= _config.threshold)
    {
      ipts0_num = y1 + (IMAGE_HEIGHT - _config.track_row_begin);
      findline_lefthand_adaptive(imagePath, BLOCK_SIZE, CLIP_VALUE,
                                 x1, y1, ipts0, &ipts0_num);
      begin_x_l = x1 + 50 > IMAGE_WIDTH - 1 ? IMAGE_WIDTH - 1 : x1 + 50;
    }
    else
    {
      ipts0_num = 0;
      begin_x_l = _config.threshold;
    }
  }
  // 原图找右边线 -------------------------------------------------------
  {
    int x2 = begin_x_r, y2 = begin_y_t;
    // 向右寻找白点
    if (imagePath.at<uint8_t>(y2, x2) < _config.threshold)
      for (x2++; x2 < IMAGE_WIDTH - 1; x2++)
        if (imagePath.at<uint8_t>(y2, x2) >= _config.threshold)
          break;
    // 向右寻找黑点
    for (; x2 < IMAGE_WIDTH - 1; x2++)
      if (imagePath.at<uint8_t>(y2, x2 + 1) < _config.threshold)
        break;
    // 向上寻找黑点
    if (x2 > IMAGE_WIDTH - BLOCK_SIZE / 2 - 1)
    {
      x2 = IMAGE_WIDTH - BLOCK_SIZE / 2 - 1;
      for (; y2 > IMAGE_HEIGHT * 2 / 3; y2--)
        if (imagePath.at<uint8_t>(y2 - 1, x2) < _config.threshold)
          break;
    }
    if (imagePath.at<uint8_t>(y2, x2) >= _config.threshold)
    {
      ipts1_num = y2 + (IMAGE_HEIGHT - _config.track_row_begin);
      findline_righthand_adaptive(imagePath, BLOCK_SIZE, CLIP_VALUE,
                                  x2, y2, ipts1, &ipts1_num);
      begin_x_r = x2 - 50 < 0 ? 0 : x2 - 50;
    }
    else
    {
      ipts1_num = 0;
      begin_x_r = IMAGE_WIDTH - _config.track_col_begin;
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
  for (size_t i = 0; i < pointsEdgeLeft.size(); i++)
  {
    circle(trackImage, Point(pointsEdgeLeft[i].y, pointsEdgeLeft[i].x), 1,
           Scalar(0, 255, 0), -1); // 绿色点
  }
  for (size_t i = 0; i < pointsEdgeRight.size(); i++)
  {
    circle(trackImage, Point(pointsEdgeRight[i].y, pointsEdgeRight[i].x), 1,
           Scalar(0, 255, 255), -1); // 黄色点
  }

  for (size_t i = 0; i < spurroad.size(); i++)
  {
    circle(trackImage, Point(spurroad[i].y, spurroad[i].x), 3,
           Scalar(0, 0, 255), -1); // 红色点
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
