import sensor, image, time
from pid import PID
from pyb import Servo, LED, millis

red_led = LED(1)
# 初始化舵机
pan_servo = Servo(1)
tilt_servo = Servo(2)
pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# 初始化PID控制器
pan_pid = PID(p=0.08, i=0.01, imax=90)
tilt_pid = PID(p=0.05, i=0.01, imax=90)

# 初始化摄像头 - 修改了窗口设置以扩大视野
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)  # VGA分辨率 (640x480)
# sensor.set_windowing([200, 120, 240, 240])  # 注释掉原来的窗口设置，使用完整视野
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

# 获取图像中心坐标 - 根据完整分辨率更新
IMAGE_CENTER_X = sensor.width() // 2  # 现在是320
IMAGE_CENTER_Y = sensor.height() // 2  # 现在是240

# 颜色阈值
thr_white = (60, 100, -20, 20, -20, 20)
thr_black = (0, 35, -20, 20, -20, 20)

target_offset = [-1, -1]  # 目标补偿量，根据视角倾斜角度而设，暂时手动填写
target = [0, 0]  # 目标坐标，自动计算
view_offset = [0, 0]  # 视角偏移补偿量，自动计算 (可能需要调整)
deviation = [0, 0]  # 偏差量，最终输出结果

def clamp(value, min_val, max_val):
    """确保值在[min_val, max_val]范围内"""
    return max(min_val, min(value, max_val))

def move_to_initial_position():
    """将云台移动到初始位置"""
    print("Moving to initial position...")
    pan_servo.angle(90)
    tilt_servo.angle(60)

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

    black_blobs = img.find_blobs([thr_black], merge=False)  # 找黑框

    if black_blobs:  # 如果有目标
        for single_black_blob in black_blobs:
            # 找到的目标中，符合阈值的面积和总的区域之间的比值。因为黑框内部不是黑色，所以这个比值不会很大。
            if single_black_blob.pixels() / (single_black_blob.w() * single_black_blob.h()) < 0.3:
                # 在区域内找白色
                white_blobs = img.find_blobs([thr_white], area_threshold=2, roi=single_black_blob.rect(), merge=False)
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

                        blob_x = target[0]
                        blob_y = target[1]

                        img.draw_cross(blob_x, blob_y)
                        view_offset[0] += round((target[0] - IMAGE_CENTER_X) * 0.5)  # 更新为新的图像中心
                        view_offset[1] += round((target[1] - IMAGE_CENTER_Y) * 0.5)  # 更新为新的图像中心

                        # 根据新的视野范围调整view_offset限制
                        view_offset[0] = min(440, max(0, view_offset[0]))  # 0-440 (640-200)
                        view_offset[1] = min(360, max(0, view_offset[1]))  # 0-360 (480-120)

                        # 计算目标与图像中心的偏差
                        #err_x = target[0] + view_offset[0] - IMAGE_CENTER_X
                        #err_y = target[1] + view_offset[1] - IMAGE_CENTER_Y
                        err_x = target[0] - IMAGE_CENTER_X
                        err_y = target[1] - IMAGE_CENTER_Y
                        #img.draw_circle(err_x + IMAGE_CENTER_X, err_y + IMAGE_CENTER_Y, 5, fill=True, color=(255, 0, 0))  # 绘制偏差点

                        # 使用PID控制器计算舵机输出
                        pan_output = pan_pid.get_pid(err_x, dt)
                        tilt_output = tilt_pid.get_pid(err_y, dt)

                        # 计算新的舵机角度 (90是初始位置)
                        #new_pan_angle = 90 + pan_output
                        #new_tilt_angle = 60 + tilt_output

                        # 限制舵机角度范围
                        #new_pan_angle = clamp(new_pan_angle, 0, 270)
                        #new_tilt_angle = clamp(new_tilt_angle, 0, 180)

                        # 设置舵机角度
                        pan_servo.angle(pan_servo.angle() - pan_output)
                        tilt_servo.angle(tilt_servo.angle() - tilt_output)

                        # 打印调试信息
                        print(f"目标坐标: ({target[0]}, {target[1]})")
                        print(f"偏差: ({err_x:.2f}, {err_y:.2f})")
                        print(f"PID输出: ({pan_output:.2f}, {tilt_output:.2f})")
                        #print(f"舵机角度: ({new_pan_angle:.2f}, {new_tilt_angle:.2f})")

    # 绘制中心十字线 - 更新为新的图像中心
    img.draw_line(0, IMAGE_CENTER_Y, sensor.width(), IMAGE_CENTER_Y)
    img.draw_line(IMAGE_CENTER_X, 0, IMAGE_CENTER_X, sensor.height())

    # 注释掉窗口调整代码，因为现在使用完整视野
    #sensor.set_windowing([200 + view_offset[0], 120 + view_offset[1], 240, 240])
    # print(200+view_offset[0], 120+view_offset[1])
    timer = millis() - timer
    print('用时', timer, '实时帧速', 1000 / timer, '平均帧速', clock.fps())
    print(f"Pan - Error: {err_x:.1f}, Output: {pan_output:.1f}")
