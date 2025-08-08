import sensor, image, time, struct
from pid import PID
from pyb import Servo, LED, millis, UART, Pin

# 初始化串口通信
uart = UART(3, 9600)
uart.init(9600, bits=8, parity=None, stop=1)

# 初始化LED和舵机
blue_led = LED(3)
red_led = LED(1)
pan_servo = Servo(1)
tilt_servo = Servo(2)
pan_servo.calibration(500, 2500, 1500)  # 校准舵机范围
tilt_servo.calibration(500, 2500, 1500)
button1 = Pin('P3', Pin.IN, Pin.PULL_UP)  # 是否开始
button2 = Pin('P4', Pin.IN, Pin.PULL_UP)  # 激光是否常亮（切题）
button3 = Pin('P5', Pin.IN, Pin.PULL_UP) #辅瞄开关
# 激光控制类
class Laser:
    def __init__(self, pin_name):
        self.pin = Pin(pin_name, Pin.OUT_PP)
        self.off()
    def on(self):
        self.pin.high()
    def off(self):
        self.pin.low()
    def toggle(self):
        self.pin.value(not self.pin.value())
laser = Laser("P2")

# PID控制器参数（保持原始参数）
pan_pid = PID(p=0.016, i=0.035, d=0.008, imax=90)
tilt_pid = PID(p=0.026, i=0.035, d=0.006, imax=90)

# 扫描与追踪状态变量
SCAN_SPEED = 30  # 扫描速度(度/秒)
scan_direction = 1  # 1:顺时针, -1:逆时针
is_scanning = True  # 是否处于扫描状态
missCount = 0
MAX_MISS_COUNT = 50  # 丢失目标后进入扫描的阈值

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()
IMAGE_CENTER_X = sensor.width() // 2
IMAGE_CENTER_Y = sensor.height() // 2

# 颜色阈值（保持原始设置）
WHITE_THRESHOLD = (80, 100, -20, 20, -20, 20)
BLACK_THRESHOLD = (0, 40, -20, 20, 20, -20)
target_offset = [-1, -1]

# 卡尔曼滤波器（保持原始实现）
class KalmanFilter:
    def __init__(self, process_noise=0.01, measurement_noise=1, initial_value=0):
        self.x = initial_value
        self.v = 0
        self.P = [[1, 0], [0, 1]]
        self.Q = [[process_noise, 0], [0, process_noise]]
        self.R = measurement_noise
        self.F = [[1, 1], [0, 1]]
        self.H = [1, 0]
    def predict(self):
        self.x = self.F[0][0] * self.x + self.F[0][1] * self.v
        self.v = self.F[1][0] * self.x + self.F[1][1] * self.v
        self.P[0][0] = self.F[0][0] * self.P[0][0] + self.F[0][1] * self.P[1][0] + self.Q[0][0]
        self.P[0][1] = self.F[0][0] * self.P[0][1] + self.F[0][1] * self.P[1][1] + self.Q[0][1]
        self.P[1][0] = self.F[1][0] * self.P[0][0] + self.F[1][1] * self.P[1][0] + self.Q[1][0]
        self.P[1][1] = self.F[1][0] * self.P[0][1] + self.F[1][1] * self.P[1][1] + self.Q[1][1]
        return self.x
    def update(self, measurement):
        S = self.H[0] * self.P[0][0] * self.H[0] + self.R
        K = [self.P[0][0] * self.H[0] / S, self.P[1][0] * self.H[0] / S]
        y = measurement - self.H[0] * self.x
        self.x = self.x + K[0] * y
        self.v = self.v + K[1] * y
        self.P[0][0] = (1 - K[0] * self.H[0]) * self.P[0][0]
        self.P[0][1] = (1 - K[0] * self.H[0]) * self.P[0][1]
        self.P[1][0] = (1 - K[1] * self.H[0]) * self.P[1][0]
        self.P[1][1] = (1 - K[1] * self.H[0]) * self.P[1][1]
        return self.x

kalman_x = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_X)
kalman_y = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_Y)

# 辅助函数
def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

def move_to_initial_position():
    """移动到初始位置"""
    pan_servo.angle(0)
    tilt_servo.angle(0)
    time.sleep_ms(500)

# 状态切换函数
def enter_scan_mode():
    global is_scanning, missCount
    is_scanning = True


def enter_track_mode():
    global is_scanning
    is_scanning = False
    # 不重置PID积分项，保持原始逻辑

# 初始化位置
move_to_initial_position()
last_time = time.ticks_ms()

while True:
    current_time = time.ticks_ms()
    dt = (current_time - last_time) / 1000.0  # 时间差(秒)
    last_time = current_time
    clock.tick()
    img = sensor.snapshot()
    if (button1.value() == 0) or (button1.value() == 1 and button2.value() == 1):
        laser.off()
    red_led.off()
    valid_target = False

    # 检测黑色区域
    black_blobs = img.find_blobs([BLACK_THRESHOLD], merge=False)
    if black_blobs:
        for black_blob in black_blobs:
            # 保持原始的黑色区域过滤条件
            if black_blob.pixels() / (black_blob.w() * black_blob.h()) < 0.3:
                # 在黑色区域内找白色目标
                white_blobs = img.find_blobs([WHITE_THRESHOLD],
                                           area_threshold=2,
                                           roi=black_blob.rect(),
                                           merge=False)
                if white_blobs:
                    largest_white = max(white_blobs, key=lambda b: b.area())
                    enter_track_mode()
                    # 保持原始的目标验证条件
                    if (2 < largest_white.pixels() / black_blob.pixels() < 4) and \
                       abs(largest_white.cx() - black_blob.cx()) < 10 and \
                       abs(largest_white.cy() - black_blob.cy()) < 10:
                        valid_target = True
                        if button3.value() == 1:
                            red_led.on()


                        # 目标位置计算（使用原始坐标，不依赖滤波结果）
                        target = [largest_white.cx() + target_offset[0], largest_white.cy() + target_offset[1]]
                        raw_x = target[0]
                        raw_y = target[1]

                        # 卡尔曼滤波步骤保留但不影响pan_error计算
                        kalman_x.predict()
                        kalman_y.predict()
                        filtered_x = kalman_x.update(raw_x)
                        filtered_y = kalman_y.update(raw_y)

                        # 绘制目标标记
                        img.draw_cross(raw_x, raw_y, color=(0, 255, 0))  # 原始位置(绿)
                        img.draw_cross(int(filtered_x), int(filtered_y), color=(255, 0, 0))  # 滤波位置(红)

                        # 保持原始的pan_error计算方式（使用raw_x）
                        pan_error = raw_x - IMAGE_CENTER_X - 5
                        tilt_error = raw_y - IMAGE_CENTER_Y - 5

                        # 激光瞄准逻辑（保持原始）
                        if (pan_error <= 5) and (pan_error >= -5) and (tilt_error <= 6) and (tilt_error >= -6):
                            if button1.value() == 1:
                                laser.on()

                        # 进入追踪模式
                        enter_track_mode()

                        # 保持原始的pan_output计算逻辑
                        pan_output = pan_pid.get_pid(pan_error, 1)
                        tilt_output = tilt_pid.get_pid(tilt_error, 1)

                        # 保持原始的输出幅度限制
                        pan_output = clamp(pan_output, -5, 5)
                        tilt_output = clamp(tilt_output, -2, 2)
                        if button1.value() == 0:
                            pan_output = 0
                            new_pan = 0
                            tilt_output = 0
                            new_tilt = 0
                            pan_pid.reset_I()
                            tilt_pid.reset_I()
                            if button3.value() == 1:
                                blue_led.on()
                        else:
                        # 更新舵机位置（不限制最终角度，保持原始逻辑）

                            new_pan = pan_servo.angle() - pan_output
                            new_tilt = tilt_servo.angle() - tilt_output

                            blue_led.off()
                            if button1.value() == 1:
                                pan_servo.angle(new_pan)
                                tilt_servo.angle(new_tilt)  # 保持原始注释状态

                        # 输出调试信息
                        print(f"err_x=%.1f,err_y=%.1f,out_pan=%.1f,out_tilt=%.1f" %
                              (pan_error, tilt_error, pan_output, tilt_output),is_scanning,new_pan)
                        uart.write("%.1f,%.1f,%.1f,%.1f\n" %
                                  (pan_error, tilt_error, pan_output, tilt_output))

    # 扫描模式逻辑（新增的平滑扫描）
    if is_scanning:
        # 计算扫描角度(连续转动)
        turn_angle = SCAN_SPEED * dt * scan_direction * 2.9
        new_pan_angle = pan_servo.angle() - turn_angle

        # 边界检测与方向反转
        if new_pan_angle >= 94:
            new_pan_angle = 94
            scan_direction = 1  # 反转方向
        elif new_pan_angle < -94:
            new_pan_angle = -94
            scan_direction = -1   # 反转方向
        if button1.value() == 1:
            pan_servo.angle(new_pan_angle)
        print(f"扫描中: 角度={new_pan_angle:.1f}, 方向={scan_direction}")

    # 目标丢失处理
    if not valid_target:
        # 只有在追踪模式下，才累加丢失计数并判断是否需要进入扫描
        if not is_scanning:
            missCount += 1
            if missCount >= MAX_MISS_COUNT:
                enter_scan_mode()
                missCount = 0
                print(f"目标丢失，进入扫描模式 (计数={missCount})，当前模式: {is_scanning}")
        # 无目标且无黑色区域时，重置滤波器（仅在追踪模式下执行）
        elif not black_blobs and not is_scanning:
            pan_pid.reset_I()
            tilt_pid.reset_I()
            kalman_x = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_X)
            kalman_y = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_Y)

    # 显示帧率
    img.draw_string(0, 0, f"FPS:{clock.fps():.1f}", color=(255, 0, 0))
