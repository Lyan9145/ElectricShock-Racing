import numpy as np
import cv2
import glob

# 棋盘格尺寸（内角点数量，例如11x8）
chessboard_size = (11, 8)
# 格子物理尺寸，单位如毫米
square_size = 20

# 物理世界坐标点准备
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp = objp * square_size

objpoints = [] # 3D点
imgpoints = [] # 2D点

# 加载所有标定照片
images = glob.glob('./calib_imgs/*.jpg')  # 假设图片都在calib_imgs文件夹

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        objpoints.append(objp)
        # 亚像素优化
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)
        # 可视化检测效果
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(200)
cv2.destroyAllWindows()

# 执行标定
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)
print("内参矩阵:\n", mtx)
print("畸变系数:\n", dist)

# 计算重投影误差
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error
print("平均重投影误差：", mean_error/len(objpoints))

# 保存参数
np.savez('calib_params.npz', mtx=mtx, dist=dist)
