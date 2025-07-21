import pyb
# 电机控制通信协议封装
class MotorController:
    def __init__(self, uart_id, tx_pin, rx_pin,**kwargs):
        self.uart = pyb.UART(uart_id, 115200, pins=(tx_pin, rx_pin))
        self.cmd_format = {
            'header': 0xAA,
            'footer': 0x55,
            'speed_range': (-1000, 1000)  # PWM范围
        }
        for key, value in kwargs.items():
                   setattr(self, key, value)
    def _checksum(self, data):
        return sum(data) & 0xFF

    def update(self, speed, steer):
        # 构造数据帧：帧头(1B) + 速度(2B) + 转向(2B) + 校验(1B) + 帧尾(1B)
        speed = max(min(speed, self.cmd_format['speed_range'][1]),
                   self.cmd_format['speed_range'][0])
        steer = int(steer * 100)  # 浮点转整型(放大100倍)

        data = bytearray([
            self.cmd_format['header'],
            (speed >> 8) & 0xFF, speed & 0xFF,
            (steer >> 8) & 0xFF, steer & 0xFF
        ])
        data.append(self._checksum(data[1:]))
        data.append(self.cmd_format['footer'])

        self.uart.write(data)
