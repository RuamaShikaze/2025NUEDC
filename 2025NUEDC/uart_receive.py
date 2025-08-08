import time
from pyb import UART

# 配置UART对象 - 根据你的硬件连接选择正确的UART通道和波特率
uart = UART(3, 9600)  # 例如使用UART 3，波特率115200
uart.init(9600, bits=8, parity=None, stop=1)  # 初始化UART参数

# 定义特定字符串和对应的动作函数
def action_hello():
    print("收到Hello命令!")

def action_world():
    print("收到World命令!")

def action_default():
    print("收到未知命令:", received_data)

# 命令映射字典 - 将特定字符串映射到对应的动作函数
command_map = {
    "HELLO": action_hello,
    "WORLD": action_world
}

# 接收缓冲区和相关变量
buffer = ''           # 用于累积接收到的字符
timeout_duration = 1  # 超时时间（秒），用于判断消息结束
last_receive_time = time.ticks_ms()  # 上次接收到数据的时间

print("UART监听已启动，等待数据...")

while(True):
    # 检查UART是否有数据可读
    if uart.any():
        # 读取一个字节数据
        byte_data = uart.readchar()

        # 将字节转换为字符并添加到缓冲区
        if 0 <= byte_data <= 127:  # 只处理ASCII字符
            char_data = chr(byte_data)
            buffer += char_data
            last_receive_time = time.ticks_ms()  # 更新接收时间

            # 简单调试：打印接收到的每个字符
            # print(f"收到字符: {char_data}")

    # 检查是否超时（一段时间没有新数据），或者缓冲区中是否有换行符
    current_time = time.ticks_ms()
    time_elapsed = time.ticks_diff(current_time, last_receive_time) / 1000  # 转换为秒

    if (time_elapsed >= timeout_duration and buffer) or ('\n' in buffer):
        # 处理接收到的数据
        received_data = buffer.strip()  # 去除首尾空白字符和换行符

        if received_data:  # 如果缓冲区不为空
            # 查找匹配的命令并执行对应动作
            matched = False
            for cmd, action in command_map.items():
                if cmd in received_data:
                    action()
                    matched = True
                    break

            # 如果没有匹配的命令，执行默认动作
            if not matched:
                action_default()

        # 清空缓冲区准备接收下一条消息
        buffer = ''
        # print("缓冲区已清空")

    # 短暂延时，避免CPU占用过高
    time.sleep_ms(10)
