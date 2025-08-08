# 激光笔红斑追踪程序（带PID滤波）
# 适用于白色背景下的红色激光点追踪
# 输出激光点相对于图像中心的坐标（经PID平滑处理）

import sensor, image, time
import uart_send_packet

# PID控制器类
class PIDController:
    def __init__(self, kp=0.3, ki=0.1, kd=0.2, setpoint=0):
        self.kp = kp  # 比例增益
        self.ki = ki  # 积分增益
        self.kd = kd  # 微分增益
        self.setpoint = setpoint  # 目标值

        self.last_error = 0
        self.integral = 0
        self.output = 0

    def update(self, process_value, dt):
        # 计算误差
        error = self.setpoint - process_value

        # 计算积分项
        self.integral += error * dt

        # 计算微分项
        derivative = (error - self.last_error) / dt if dt > 0 else 0

        # 计算PID输出
        self.output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 保存当前误差用于下次计算
        self.last_error = error

        return self.output

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 使用RGB565格式
sensor.set_framesize(sensor.QVGA)    # 设置分辨率为QVGA (320x240)
sensor.skip_frames(time = 2000)      # 跳过2000ms的帧，等待摄像头稳定
sensor.set_auto_gain(False)          # 关闭自动增益，避免颜色跟踪不稳定
sensor.set_auto_whitebal(False)      # 关闭自动白平衡，保持颜色稳定
clock = time.clock()                 # 创建时钟对象，用于计算帧率

# 获取图像中心坐标
IMAGE_CENTER_X = sensor.width() // 2
IMAGE_CENTER_Y = sensor.height() // 2

# 红色激光点的颜色阈值范围(HSV)
RED_THRESHOLD = ((83, 100, -103, 100, -30, 40))  # 可根据实际情况调整

# 初始化PID控制器，用于平滑X和Y方向的输出
pid_x = PIDController(kp=0.8, ki=0.02, kd=0.2)
pid_y = PIDController(kp=0.8, ki=0.02, kd=0.2)

# 用于存储上一时刻的时间和滤波后的坐标
last_time = time.ticks_ms()
smoothed_x = 0
smoothed_y = 0

# 初始化移动平均滤波器
window_size = 5
x_buffer = [0] * window_size
y_buffer = [0] * window_size
index = 0

# 减少数据发送频率
SEND_INTERVAL = 10  # 每10帧发送一次数据
frame_count = 0

while(True):
    clock.tick()
    img = sensor.snapshot()  # 获取一帧图像

    # 计算时间差（用于PID计算）
    current_time = time.ticks_ms()
    dt = (current_time - last_time) / 1000.0  # 转换为秒
    last_time = current_time

    # 在图像中寻找红色区域
    blobs = img.find_blobs([RED_THRESHOLD], pixels_threshold=5, area_threshold=5)

    # 如果找到红色区域
    if blobs:
        # 找到最大的红色区域（假设这是激光点）
        largest_blob = max(blobs, key=lambda b: b.area())

        # 获取激光点中心坐标
        laser_x = largest_blob.cx()
        laser_y = largest_blob.cy()

        # 移动平均滤波
        x_buffer[index] = laser_x
        y_buffer[index] = laser_y
        index = (index + 1) % window_size
        filtered_x = sum(x_buffer) // window_size
        filtered_y = sum(y_buffer) // window_size

        # 计算相对于图像中心的坐标
        relative_x = filtered_x - IMAGE_CENTER_X
        relative_y = filtered_y - IMAGE_CENTER_Y

        # 使用PID进行平滑处理
        smoothed_x = pid_x.update(relative_x, dt)
        smoothed_y = pid_y.update(relative_y, dt)

        # 在图像上绘制相关信息
        img.draw_rectangle(largest_blob.rect())  # 绘制矩形框
        img.draw_cross(laser_x, laser_y)         # 绘制原始激光点十字标记
        img.draw_cross(IMAGE_CENTER_X, IMAGE_CENTER_Y, color=(0, 255, 0))  # 绘制中心十字

        # 绘制平滑后的位置（可选）
        smooth_draw_x = int(IMAGE_CENTER_X - smoothed_x)
        smooth_draw_y = int(IMAGE_CENTER_Y - smoothed_y)
        img.draw_circle(smooth_draw_x, smooth_draw_y, 5, color=(0, 0, 255))  # 蓝色圆圈表示平滑后的位置

        # 每10帧发送一次数据
        if frame_count % SEND_INTERVAL == 0:
            message = f" {relative_x, relative_y, smoothed_x, smoothed_y}\n"
            uart_send_packet.send_packet(message)

        frame_count += 1
    else:
        # 如果未找到激光点，重置PID积分项以避免累积误差
        pid_x.integral = 0
        pid_y.integral = 0

    # 减少打印输出
    # print("FPS:", clock.fps())
