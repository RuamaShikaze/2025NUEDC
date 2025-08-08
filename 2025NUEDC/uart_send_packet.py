import time
from pyb import UART

# 初始化UART对象（根据实际连接的引脚修改）
uart = UART(3, 9600)  # UART3, 波特率115200
uart.init(9600, bits=8, parity=None, stop=1)

# 帧头和终止符定义
FRAME_HEADER = b'\xaa\x55'  # 帧头: 0xAA, 0x55
TERMINATOR = b'\xcc'          # 终止符: ASCII换行符

def send_packet(data):
    """封装数据并通过UART发送"""
    # 如果data是字符串，转换为bytes
    if isinstance(data, str):
        data = data.encode('utf-8')

    # 构建完整数据包：帧头 + 数据 + 终止符
    packet = FRAME_HEADER + data + TERMINATOR

    # 发送数据包
    uart.write(packet)
    print("Sent:", [hex(b) for b in packet])  # 打印发送的字节（用于调试）

