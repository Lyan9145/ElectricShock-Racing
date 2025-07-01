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
		// Size sizeImage; // 图像的尺寸
		// sizeImage.width = image.cols;
		// sizeImage.height = image.rows;

		// Mat mapx = Mat(sizeImage, CV_32FC1);	// 经过矫正后的X坐标重映射参数
		// Mat mapy = Mat(sizeImage, CV_32FC1);	// 经过矫正后的Y坐标重映射参数
		// Mat rotMatrix = Mat::eye(3, 3, CV_32F); // 内参矩阵与畸变矩阵之间的旋转矩阵

		// // 采用initUndistortRectifyMap+remap进行图像矫正
		// initUndistortRectifyMap(cameraMatrix, distCoeffs, rotMatrix, cameraMatrix, sizeImage, CV_32FC1, mapx, mapy);
		// Mat imageCorrect = image.clone();
		// remap(image, imageCorrect, mapx, mapy, INTER_LINEAR);

		// // 采用undistort进行图像矫正
		// //  undistort(image, imageCorrect, cameraMatrix, distCoeffs);

		// return imageCorrect;
		// 获取图像尺寸
		int w = image.cols;
		int h = image.rows;
		cv::Size imageSize(w, h);
		
		cv::Mat newCameraMtx;
		cv::Rect roi;

		// 1. 计算新的相机矩阵和有效区域
		// Python: newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
		// C++ 中 alpha=1.0 表示保留所有原始像素，可能会有黑边。
		// roi 会被此函数填充，因此需要传递其地址。
		newCameraMtx = cv::getOptimalNewCameraMatrix(mtx, dist, imageSize, 1.0, imageSize, &roi);

		// 2. 去畸变
		// Python: dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
		// C++ 中，直接将 newCameraMtx 作为第五个参数传入。
		cv::Mat dst;
		cv::undistort(image, dst, cameraMatrix, distCoeffs, newCameraMtx);

		// 3. 裁剪掉黑边，仅保留有效区域
		// Python:
		//   x, y, w, h = roi
		//   dst = dst[y:y+h, x:x+w]
		// C++ 中，可以直接使用 roi (cv::Rect) 来创建一个指向子图像的 Mat Header。
		// 使用 .clone() 来创建一个独立的、深拷贝的裁剪后图像，以供返回。
		cv::Mat cropped_dst = dst(roi).clone();

		return cropped_dst;
	}
	else
	{
		return image;
	}
}
