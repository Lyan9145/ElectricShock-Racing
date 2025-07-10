#include "../include/preprocess.hpp"

using namespace cv;
using namespace std;

/**
**[1] 读取视频
**[2] 图像二值化
*/

/**
 * @brief 图像矫正参数初始化
 *
 */
Preprocess::Preprocess()
{
	// 读取xml中的相机标定参数
	cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参矩阵
	distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));	// 相机的畸变矩阵
	FileStorage file;
	if (file.open("../res/calibration/valid/calibration.xml", FileStorage::READ)) // 读取本地保存的标定文件
	{
		file["cameraMatrix"] >> cameraMatrix;
		file["distCoeffs"] >> distCoeffs;
		cout << "相机矫正参数初始化成功!" << endl;
		enable = true;
	}
	else
	{
		cout << "打开相机矫正参数失败!!!" << endl;
		enable = false;
	}
	const cv::Size image_size(640, 480);

	// Pre-calculate the rectification maps.       
	// CV_16SC2 is a more compact and faster map format than the default CV_32FC1.
	cv::initUndistortRectifyMap(
		PRECOMPUTED_MTX_LOW_RES,
		PRECOMPUTED_DIST,
		cv::Mat(), // Optional rectification (not needed for monocular)
		PRECOMPUTED_NEW_MTX_LOW_RES,
		image_size,
		CV_16SC2, // Map type for remap
		m_map1,
		m_map2
	);
};

/**
 * @brief 图像尺寸标准化：确保输入图像为标准尺寸320x240
 *
 * @param frame 输入原始帧
 * @return Mat  标准化尺寸后的图像
 */
Mat Preprocess::resizeImage(Mat &frame)
{
	Mat resizedFrame;

	// 如果图像尺寸已经是标准尺寸，直接返回
	if (frame.cols == COLSIMAGE && frame.rows == ROWSIMAGE)
	{
		return frame;
	}

	// 将图像缩放到标准尺寸320x240
	resize(frame, resizedFrame, Size(COLSIMAGE, ROWSIMAGE), 0, 0, INTER_LINEAR);

	return resizedFrame;
}

/**
 * @brief 图像二值化
 *
 * @param frame	输入原始帧
 * @return Mat	二值化图像
 */
Mat Preprocess::binaryzation(Mat &frame)
{
	Mat imageGray, imageBinary;

	cvtColor(frame, imageGray, COLOR_BGR2GRAY); // RGB转灰度图

	threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU); // OTSU二值化方法

	return imageBinary;
}

/**
 * @brief 矫正图像
 *
 * @param imagesPath 图像路径
 */
Mat Preprocess::correction(Mat &image)
{
	if (enable)
	{
        // 1. Apply the rectification map. This is much faster than cv::undistort().
        cv::Mat dst;
        cv::remap(image, dst, m_map1, m_map2, cv::INTER_LINEAR);

        // 2. Crop to the valid region using the pre-calculated ROI.
        // .clone() creates a deep copy, making the returned Mat independent.
        cv::Mat cropped_dst = dst(m_roi).clone();      

        return cropped_dst;
	}
	else
	{
		return image;
	}
}
