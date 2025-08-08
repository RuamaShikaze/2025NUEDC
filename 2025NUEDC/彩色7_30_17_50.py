import sensor, image, time
from pid import PID
from pyb import Servo,LED

red_led = LED(1)
# 初始化舵机
pan_servo = Servo(1)
tilt_servo = Servo(2)
pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# 初始化PID控制器
pan_pid = PID(p=0.03, i=0.01, imax=90)
tilt_pid = PID(p=0.05, i=0
, imax=90)

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
WHITE_THRESHOLD = [(35, 95, 0, -20, -15, 20)]  # 白色阈值
BLACK_THRESHOLD = [(0, 18, -12, -3, -5, 6)]   # 黑色阈值

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
                circle_x = largest_circle.x()
                circle_y = largest_circle.y()

                pan_error = circle_x - IMAGE_CENTER_X
                tilt_error = circle_y - IMAGE_CENTER_Y

                print("pan_error: ", pan_error)
                img.draw_circle(largest_circle.x(), largest_circle.y(), largest_circle.r())
                img.draw_cross(circle_x, circle_y)

                pan_output = pan_pid.get_pid(pan_error, 1) / 2
                tilt_output = tilt_pid.get_pid(tilt_error, 1)

                pan_servo.angle(pan_servo.angle() - pan_output)
                tilt_servo.angle(tilt_servo.angle() - tilt_output)
            else:
                # 使用白色区域中心
                blob_x = largest_blob.cx()
                blob_y = largest_blob.cy()

                pan_error = blob_x - IMAGE_CENTER_X
                tilt_error = blob_y - IMAGE_CENTER_Y

                print("Using blob center - pan_error: ", pan_error)
                img.draw_cross(blob_x, blob_y)

                pan_output = pan_pid.get_pid(pan_error, 1) / 2
                tilt_output = tilt_pid.get_pid(tilt_error, 1)

                pan_servo.angle(pan_servo.angle() - pan_output)
                tilt_servo.angle(tilt_servo.angle() - tilt_output)
    else:
        pan_pid.reset_I()
        tilt_pid.reset_I()
