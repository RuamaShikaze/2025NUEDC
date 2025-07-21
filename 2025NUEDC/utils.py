import uart
def send_track_data(error):
    data = bytearray([0xAA, 0x03])  # 帧头+长度
    data.extend(error.to_bytes(2, 'big', signed=True))  # 有符号16位
    checksum = sum(data[1:]) & 0xFF
    data.append(checksum)
    data.append(0x55)
    uart.write(data)
