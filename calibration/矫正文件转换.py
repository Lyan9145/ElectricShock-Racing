import cv2
import numpy as np

# 读取保存的标定参数
data = np.load('calib_params.npz')
mtx = data['mtx']
dist = data['dist']

# 保存标定参数为XML文件
fs = cv2.FileStorage('calibration_parameters.xml', cv2.FILE_STORAGE_WRITE)
fs.write('cameraMatrix', mtx)
fs.write('distCoeffs', dist)
fs.release()
print("Camera calibration parameters saved to calibration_parameters.xml")



# # 读一张要校正的照片
# img = cv2.imread('./calib_imgs/calib_01.jpg')  # 换成你实际图片路径
# h, w = img.shape[:2]

# # 计算新的校正映射
# newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# # 去畸变
# dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# # 裁剪有效区域（可选）
# x, y, w, h = roi
# dst = dst[y:y+h, x:x+w]

# cv2.imshow('Undistorted', dst)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# cv2.imwrite('./undistorted_01.jpg', dst)
