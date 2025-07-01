import numpy as np
import cv2
import glob
import os

# 加载标定参数
with np.load('calib_params.npz') as X:
    mtx, dist = [X[i] for i in ('mtx', 'dist')]

# 输入和输出文件夹
src_dir = './calib_imgs/'
dst_dir = './undistort_imgs/'
os.makedirs(dst_dir, exist_ok=True)

# 获取所有待处理图片
images = glob.glob(os.path.join(src_dir, '*.jpg'))

for fname in images:
    img = cv2.imread(fname)
    h, w = img.shape[:2]

    # 计算新的相机矩阵和有效区域
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # 去畸变
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # 可选：裁剪掉黑边，仅保留有效区域
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]

    # 构造输出文件名
    base = os.path.basename(fname)
    out_path = os.path.join(dst_dir, f'undistort_{base}')

    cv2.imwrite(out_path, dst)
    print(f"已保存去畸变图片：{out_path}")

print("全部图片去畸变完成！")
