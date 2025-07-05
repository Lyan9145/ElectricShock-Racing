#include "../include/imgprocess.hpp"

ImageProcess::ImageProcess() {}



//去除高光
cv::Mat ImageProcess::matilluminationChange(cv::Mat src) {
  cv::Mat gray, threshmat, dst;
  //复制出来改为灰度图
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

  //直方图均衡化
  cv::equalizeHist(gray, gray);
  //imshow("equalizeHist", gray);

  //二值化操作，定义大于210的即为高光
  cv::threshold(gray, threshmat, 210, 255, cv::THRESH_BINARY);

  //查找图片中高亮区域轮廓
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(threshmat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
  for (int i = 0; i < contours.size(); ++i) {
    cv::Rect rect = boundingRect(contours[i]);
    cv::rectangle(mask, rect, cv::Scalar(255), -1);
  }

  //imshow("mask", mask);

  //去高光
  cv::illuminationChange(src, mask, dst, 1.0f, 0.1f);

  return dst;
}

cv::Mat ImageProcess::processImage(const cv::Mat & src_img) {
    cv::Mat result_img;
    // cv::resize(src_img, result_img, cv::Size(_config.img_width, _config.img_height));
    cv::cvtColor(src_img, result_img, cv::COLOR_BGR2GRAY);
    
    cv::threshold(result_img, result_img, 100, 255, cv::THRESH_OTSU);
    cv::morphologyEx(result_img, result_img, cv::MORPH_CLOSE, kernel);
    return result_img;
}

// 原图 -> 俯视  透视矩阵
cv::Mat warpMatrix = (cv::Mat_<float>(3, 3) <<
    3.41575344e+00,  6.75223255e-01, -1.67839012e+03,
   -1.48898470e-01,  6.53779083e+00, -1.93995113e+03,
   -1.00614568e-03,  5.05477137e-03,  1.00000000e+00
);

// 俯视 -> 原图  透视矩阵
cv::Mat warpMatrixT = (cv::Mat_<float>(3, 3) <<
    3.44314005e-01, -1.92954325e-01, 2.03571264e+02,
    4.42568546e-02,  3.63836417e-02, 1.44862754e+02,
    1.22721767e-04, -3.78051151e-04, 4.72574244e-01
);

// 透视变换 (0:原图 -> 俯视, 1:俯视 -> 原图)
void ImageProcess::mapPerspective(float x, float y, float loc[2], uint8_t mode) {
    float xx, yy, zz;

    if (mode == 0) {
        zz = warpMatrix.at<float>(2, 0) * x +
             warpMatrix.at<float>(2, 1) * y +
             warpMatrix.at<float>(2, 2);
        xx = (warpMatrix.at<float>(0, 0) * x +
              warpMatrix.at<float>(0, 1) * y +
              warpMatrix.at<float>(0, 2)) /
             zz;
        yy = (warpMatrix.at<float>(1, 0) * x +
              warpMatrix.at<float>(1, 1) * y +
              warpMatrix.at<float>(1, 2)) /
             zz;

        loc[0] = xx;
        loc[1] = yy;
    } else {
        zz = warpMatrixT.at<float>(2, 0) * x +
             warpMatrixT.at<float>(2, 1) * y +
             warpMatrixT.at<float>(2, 2);
        xx = (warpMatrixT.at<float>(0, 0) * x +
              warpMatrixT.at<float>(0, 1) * y +
              warpMatrixT.at<float>(0, 2)) /
             zz;
        yy = (warpMatrixT.at<float>(1, 0) * x +
              warpMatrixT.at<float>(1, 1) * y +
              warpMatrixT.at<float>(1, 2)) /
             zz;

        loc[0] = xx;
        loc[1] = yy;
    }
}
