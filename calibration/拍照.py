import cv2
import os

# 打开摄像头，通常是0或1，如果不对可换成1试试
cap = cv2.VideoCapture(1)

# print("默认分辨率：", cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
# print("设置后分辨率：", cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT))



# 创建保存目录
save_dir = './calib_imgs/'
os.makedirs(save_dir, exist_ok=True)

img_id = 1

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头")
        break
    cv2.imshow('USB Cam', frame)

    # 按空格键拍照保存
    key = cv2.waitKey(1)
    if key == ord(' '):  # 空格键保存
        img_path = os.path.join(save_dir, f'calib_{img_id:02d}.jpg')
        cv2.imwrite(img_path, frame)
        print(f'保存：{img_path}')
        img_id += 1
    elif key == 27:  # ESC键退出
        break

cap.release()
cv2.destroyAllWindows()
