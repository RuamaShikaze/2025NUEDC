import time
import sensor
import image
from pyb import UART, LED

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160x120分辨率（提高帧率）
sensor.skip_frames(30)
sensor.set_auto_gain(False)  # 关闭自动增益（避免光线变化影响）
sensor.set_auto_whitebal(False)  # 关闭自动白平衡

# 初始化UART（P0=RX, P1=TX，波特率19200）
uart = UART(1, 19200, timeout_char=1000)

# LED定义（用于状态指示）
led_red = LED(1)
led_green = LED(2)

# 循迹参数（根据实际赛道调整）
TRACK_COLOR_THRESHOLD = (0, 100, -20, 20, -20, 20)  # 黑色赛道阈值（LAB格式）
END_LINE_THRESHOLD = (50, 100, -20, 20, 20, 100)  # 终点线阈值（如红色）

# 数据发送函数
def send_int(val):
    uart.write(bytes(val.to_bytes(4, 'little', signed=True)))

def send_float(val):
    import struct
    uart.write(struct.pack('<f', val))  # 小端模式打包浮点数

def send_byte(val):
    uart.write(bytes([val]))

def send_end():
    uart.write(b'\x01\xfe\xff')  # 帧尾标记

# 自检响应
def self_check():
    led_green.on()
    time.sleep_ms(200)
    led_green.off()
    time.sleep_ms(200)
    led_green.on()
    time.sleep_ms(200)
    led_green.off()
    uart.write(bytes([0x1B]))  # 发送确认码

# 循迹逻辑
def track_line():
    img = sensor.snapshot()
    # 查找赛道轮廓
    blobs = img.find_blobs([TRACK_COLOR_THRESHOLD], pixels_threshold=50)
    track_error = 0.0  # 偏差值（0为中心，正值偏右，负值偏左）
    end_flag = 0  # 终点标志（0=未检测，1=检测到）

    # 检测终点线
    end_blobs = img.find_blobs([END_LINE_THRESHOLD], pixels_threshold=100)
    if end_blobs:
        end_flag = 1

    # 计算赛道偏差
    if blobs:
        largest_blob = max(blobs, key=lambda b: b.area())
        img.draw_rectangle(largest_blob.rect())
        # 图像中心x坐标：80（QQVGA宽度160）
        track_error = largest_blob.cx() - 80  # 偏差=赛道中心-图像中心

    return track_error, end_flag

# 主循环
tft_flag = 0  # 循迹模式标志
while True:
    # 读取MSPM0发送的命令
    cmd = uart.readchar()

    # 处理自检命令
    if cmd == 0x1A:
        self_check()
        tft_flag = 0

    # 处理循迹命令
    elif cmd == 0xE0 or tft_flag:
        tft_flag = 1
        # 获取循迹数据
        error, end_flag = track_line()
        # 发送数据（浮点数偏差 + 字节终点标志 + 帧尾）
        send_float(error)
        send_byte(end_flag)
        send_end()
        # 若检测到终点，退出循迹模式
        if end_flag:
            tft_flag = 0

    # 处理停止命令
    elif cmd == 0x1C:
        tft_flag = 0
        led_red.on()
        time.sleep_ms(200)
        led_red.off()

    time.sleep_ms(50)
