
import sensor, image, time
from pid import PID
from pyb import Servo, LED,millis

red_led = LED(1)
# 初始化舵机
pan_servo = Servo(1)
tilt_servo = Servo(2)
pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# 初始化PID控制器
pan_pid = PID(p=0.05, i=0.01, imax=90)
tilt_pid = PID(p=0.03, i=0.01, imax=90)

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
WHITE_THRESHOLD = (70, 100, -20, 20, -20, 20) # 白色阈值
BLACK_THRESHOLD = (0, 40, -20, 20, 20, -20)  # 黑色阈值
target_offset = [-1, -1] #倾角补偿，暂时手动填写
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



def move_to_initial_position():
    """将云台移动到初始位置"""
    print("Moving to initial position...")
    pan_servo.angle(90)
    tilt_servo.angle(60)
    time.sleep_ms(200)

move_to_initial_position()
last_time = time.ticks_ms()

while True:
    timer = millis()
    clock.tick()
    img = sensor.snapshot()

    red_led.off()
    current_time = time.ticks_ms()
    dt = (current_time - last_time) / 1000.0
    last_time = current_time

    black_blobs = img.find_blobs([BLACK_THRESHOLD], merge=False)  # 找黑框

    if black_blobs:  # 如果有目标
        for single_black_blob in black_blobs:
            # 找到的目标中，符合阈值的面积和总的区域之间的比值。因为黑框内部不是黑色，所以这个比值不会很大。
            if single_black_blob.pixels() / (single_black_blob.w() * single_black_blob.h()) < 0.3:
                # 在区域内找白色
                white_blobs = img.find_blobs([WHITE_THRESHOLD], area_threshold=2, roi=single_black_blob.rect(), merge=False)
                if white_blobs:  # 如果有目标
                    largest_white = max(white_blobs, key=lambda b: b.area())  # 找到最大的块
                    # 判断条件1：黑色区域面积和白色区域面积的比例；判断条件2：黑框和白色区域中心坐标的差值
                    if (2 < largest_white.pixels() / single_black_blob.pixels() < 4) and \
                            abs(largest_white.cx() - single_black_blob.cx()) < 10 and \
                            abs(largest_white.cy() - single_black_blob.cy()) < 10:
                        red_led.on()
                        target = [largest_white.cx() + target_offset[0], largest_white.cy() + target_offset[1]]  # 白色区域中心坐标
                        img.draw_cross(target, color=(255, 0, 0), thickness=3)  # 绘制在画布上

                        # 在白色区域内寻找圆
                        roi = largest_white.rect()

                        raw_x = target[0]
                        raw_y = target[1]

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
