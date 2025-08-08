import sensor, image, time
from pid import PID
from pyb import Servo, LED

red_led = LED(1)
# 初始化舵机
pan_servo = Servo(1)
tilt_servo = Servo(2)
pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# 初始化PID控制器
pan_pid = PID(p=0.031, i=0, imax=90)
tilt_pid = PID(p=0.05, i=0, imax=90)

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

# 获取图像中心坐标
IMAGE_CENTER_X = sensor.width() // 2
IMAGE_CENTER_Y = sensor.height() // 2

# 颜色阈值
WHITE_THRESHOLD = [(70, 95, -10, 10, -10, 10)]  # 白色阈值
BLACK_THRESHOLD = [(0, 40, -10, 10, 10, -10)]  # 黑色阈值

class KalmanFilter:
    def __init__(self, process_noise=0.01, measurement_noise=1, initial_value=0):
        # 状态变量 (位置和速度)
        self.x = initial_value
        self.v = 0

        # 协方差矩阵
        self.P = [[1, 0],
                 [0, 1]]

        # 过程噪声
        self.Q = [[process_noise, 0],
                 [0, process_noise]]

        # 测量噪声
        self.R = measurement_noise

        # 状态转移矩阵
        self.F = [[1, 1],  # 假设每帧时间间隔为1
                  [0, 1]]

        # 观测矩阵 (我们只能观测到位置)
        self.H = [1, 0]

    def predict(self):
        # 预测状态
        self.x = self.F[0][0] * self.x + self.F[0][1] * self.v
        self.v = self.F[1][0] * self.x + self.F[1][1] * self.v

        # 预测协方差
        self.P[0][0] = self.F[0][0] * self.P[0][0] + self.F[0][1] * self.P[1][0] + self.Q[0][0]
        self.P[0][1] = self.F[0][0] * self.P[0][1] + self.F[0][1] * self.P[1][1] + self.Q[0][1]
        self.P[1][0] = self.F[1][0] * self.P[0][0] + self.F[1][1] * self.P[1][0] + self.Q[1][0]
        self.P[1][1] = self.F[1][0] * self.P[0][1] + self.F[1][1] * self.P[1][1] + self.Q[1][1]

        return self.x

    def update(self, measurement):
        # 计算卡尔曼增益
        S = self.H[0] * self.P[0][0] * self.H[0] + self.R
        K = [self.P[0][0] * self.H[0] / S,
             self.P[1][0] * self.H[0] / S]

        # 更新状态估计
        y = measurement - self.H[0] * self.x
        self.x = self.x + K[0] * y
        self.v = self.v + K[1] * y

        # 更新协方差估计
        self.P[0][0] = (1 - K[0] * self.H[0]) * self.P[0][0]
        self.P[0][1] = (1 - K[0] * self.H[0]) * self.P[0][1]
        self.P[1][0] = (1 - K[1] * self.H[0]) * self.P[1][0]
        self.P[1][1] = (1 - K[1] * self.H[0]) * self.P[1][1]

        return self.x

# 初始化卡尔曼滤波器 (x和y坐标各一个)
kalman_x = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_X)
kalman_y = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_Y)

def clamp(value, min_val, max_val):
    """确保值在[min_val, max_val]范围内"""
    return max(min_val, min(value, max_val))

def has_black_border(img, blob, border_width=3):
    """检查白色区域是否有黑色边框"""
    # 获取区域边界并确保不超出图像范围
    x = clamp(blob.x(), 0, img.width()-1)
    y = clamp(blob.y(), 0, img.height()-1)
    w = clamp(blob.w(), 1, img.width()-x)
    h = clamp(blob.h(), 1, img.height()-y)

    # 检查上边框
    roi_top = (clamp(x-border_width, 0, img.width()-1),
               clamp(y-border_width, 0, img.height()-1),
               clamp(w+border_width*2, 1, img.width()-(x-border_width)),
               1)
    if not img.find_blobs(BLACK_THRESHOLD, roi=roi_top, merge=True):
        return False

    # 检查下边框
    roi_bottom = (clamp(x-border_width, 0, img.width()-1),
                  clamp(y+h-1, 0, img.height()-1),
                  clamp(w+border_width*2, 1, img.width()-(x-border_width)),
                  1)
    if not img.find_blobs(BLACK_THRESHOLD, roi=roi_bottom, merge=True):
        return False

    # 检查左边框
    roi_left = (clamp(x-border_width, 0, img.width()-1),
                clamp(y-border_width, 0, img.height()-1),
                1,
                clamp(h+border_width*2, 1, img.height()-(y-border_width)))
    if not img.find_blobs(BLACK_THRESHOLD, roi=roi_left, merge=True):
        return False

    # 检查右边框
    roi_right = (clamp(x+w-1, 0, img.width()-1),
                 clamp(y-border_width, 0, img.height()-1),
                 1,
                 clamp(h+border_width*2, 1, img.height()-(y-border_width)))
    if not img.find_blobs(BLACK_THRESHOLD, roi=roi_right, merge=True):
        return False

    return True

def move_to_initial_position():
    """将云台移动到初始位置"""
    print("Moving to initial position...")
    pan_servo.angle(135)
    tilt_servo.angle(0)
    time.sleep_ms(200)

move_to_initial_position()
last_time = time.ticks_ms()

while True:
    clock.tick()
    img = sensor.snapshot()

    red_led.off()
    current_time = time.ticks_ms()
    dt = (current_time - last_time) / 1000.0
    last_time = current_time

    # 寻找白色区域
    white_blobs = img.find_blobs(WHITE_THRESHOLD, pixels_threshold=20, area_threshold=20, merge=True)

    valid_blobs = []
    if white_blobs:
        # 筛选有黑色边框的白色区域
        for blob in white_blobs:
            if has_black_border(img, blob):
                valid_blobs.append(blob)
                # 绘制黑色边框检测结果
                img.draw_rectangle(blob.rect(), color=(0,255,0))  # 用绿色标记有效区域

        if valid_blobs:
            red_led.on()
            # 找到最大的有效白色区域
            largest_blob = max(valid_blobs, key=lambda b: b.pixels())

            # 在白色区域内寻找圆
            roi = largest_blob.rect()
            circles = img.find_circles(roi, threshold=2000, x_margin=10, y_margin=10, r_margin=10,
                                      r_min=2, r_max=100, r_step=2)

            if circles:
                largest_circle = max(circles, key=lambda c: c.r())
                raw_x = largest_circle.x()
                raw_y = largest_circle.y()

                # 使用卡尔曼滤波平滑坐标
                kalman_x.predict()
                kalman_y.predict()
                filtered_x = kalman_x.update(raw_x)
                filtered_y = kalman_y.update(raw_y)

                img.draw_circle(largest_circle.x(), largest_circle.y(), largest_circle.r())
                img.draw_cross(int(filtered_x), int(filtered_y), color=(255,0,0))  # 红色显示滤波后的中心
                img.draw_cross(raw_x, raw_y, color=(0,255,0))  # 绿色显示原始中心

                pan_error = filtered_x - IMAGE_CENTER_X
                tilt_error = filtered_y - IMAGE_CENTER_Y

                print("pan_error: ", pan_error)
                pan_output = pan_pid.get_pid(pan_error, 1) / 2
                tilt_output = tilt_pid.get_pid(tilt_error, 1)

                pan_servo.angle(pan_servo.angle() - pan_output)
                tilt_servo.angle(tilt_servo.angle() - tilt_output)
            else:
                # 使用白色区域中心
                raw_x = largest_blob.cx()
                raw_y = largest_blob.cy()

                # 使用卡尔曼滤波平滑坐标
                kalman_x.predict()
                kalman_y.predict()
                filtered_x = kalman_x.update(raw_x)
                filtered_y = kalman_y.update(raw_y)

                img.draw_cross(int(filtered_x), int(filtered_y), color=(255,0,0))  # 红色显示滤波后的中心
                img.draw_cross(raw_x, raw_y, color=(0,255,0))  # 绿色显示原始中心

                pan_error = filtered_x - IMAGE_CENTER_X
                tilt_error = filtered_y - IMAGE_CENTER_Y

                print("Using blob center - pan_error: ", pan_error)
                pan_output = pan_pid.get_pid(pan_error, 1) / 2
                tilt_output = tilt_pid.get_pid(tilt_error, 1)

                pan_servo.angle(pan_servo.angle() - pan_output)
                tilt_servo.angle(tilt_servo.angle() - tilt_output)
    else:
        pan_pid.reset_I()
        tilt_pid.reset_I()
        # 没有检测到目标时，重置卡尔曼滤波器
        kalman_x = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_X)
        kalman_y = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_Y)

    # 显示帧率
    img.draw_string(0, 0, "FPS:%.2f" % clock.fps(), color=(255,0,0))
