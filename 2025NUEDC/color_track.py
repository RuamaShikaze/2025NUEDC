import sensor, image, time, pyb
from pyb import UART, Servo
from pid import PID
# 初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_whitebal(False)
clock = time.clock()

# 串口初始化
uart = UART(3, 115200)

# 云台舵机初始化
pan_servo = Servo(1)  # P7
tilt_servo = Servo(2) # P8

# 目标颜色阈值 (示例：红色)
red_threshold = (30, 100, 15, 127, 15, 127)  # (L Min, L Max, A Min, A Max, B Min, B Max)

# PID参数


def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob

while(True):
    clock.tick()
    img = sensor.snapshot()

    # 颜色阈值分割
    blobs = img.find_blobs([red_threshold],
                          pixels_threshold=100,
                          area_threshold=100,
                          merge=True)

    if blobs:
        max_blob = max(blobs, key=lambda b: b.pixels())

        # 绘制追踪框
        img.draw_rectangle(max_blob.rect())
        img.draw_cross(max_blob.cx(), max_blob.cy())

        # 计算位置误差
        pan_error = max_blob.cx() - img.width()/2
        tilt_error = max_blob.cy() - img.height()/2

        # 发送目标位置数据
        uart.write("X%dY%d\n" % (max_blob.cx(), max_blob.cy()))
