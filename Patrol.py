# OpenMV 主程序：巡线 + PID + 卡尔曼滤波
import sensor, image, time, pyb, math
from pid import PID
from kalman import KalmanFilter  # 需自行实现或导入简化版卡尔曼滤波

# 初始化硬件
sensor.reset()                      # 复位摄像头
sensor.set_pixformat(sensor.RGB565) # 设置色彩格式
sensor.set_framesize(sensor.QVGA)   # 分辨率320x240
sensor.skip_frames(30)              # 跳过初始不稳定帧
uart = pyb.UART(3, 115200)          # 初始化UART3（P4/P5引脚）

# PID参数配置
pid = PID(p=0.8, i=0.05, d=0.1, setpoint=0)
motor_speed = 50  # 基础速度

# 简化卡尔曼滤波器（状态：位置误差）
kf = KalmanFilter(
    A=1,  # 状态转移矩阵（假设误差变化缓慢）
    H=1,  # 观测矩阵
    Q=0.01,  # 过程噪声
    R=0.1    # 观测噪声
)

def find_line_error(img):
    # ROI区域（仅检测下方1/3图像）
    roi = (0, img.height() * 2 // 3, img.width(), img.height() // 3)

    # 二值化（黑线检测）
    img.binary([(0, 50)])  # 阈值需根据实际赛道调整

    # 寻找最大色块
    blobs = img.find_blobs([(100, 255)], roi=roi, merge=True)  # 或 merge=1
    if not blobs:
        return None

    largest = max(blobs, key=lambda b: b.area())
    if not largest:  # 防止空 blob
        return None

    img.draw_rectangle(largest.rect(), color=(255, 0, 0))  # 绘制检测框

    # 计算中心误差（像素）
    line_center = largest.cx()
    img_center = img.width() // 2
    return line_center - img_center

while True:
    img = sensor.snapshot()
    error = find_line_error(img)

    if error is not None:
        # 卡尔曼滤波更新
        kf.predict()
        filtered_error = kf.update(error)

        # PID计算转向量
        steer = pid(filtered_error)
        steer = max(-100, min(100, steer))  # 限制输出范围

        # 发送控制指令（格式：S速度,T转向\n）
        cmd = "S{},T{}\n".format(motor_speed, int(steer))
        uart.write(cmd)

        # 调试显示
        img.draw_string(0, 0, "Err:{:.1f}".format(filtered_error), color=(255,0,0))
    else:
        uart.write("S0,T0\n")  # 未检测到线时停止

    time.sleep_ms(20)  # 控制周期50Hz
