# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Connect Example
#
# This example shows how to connect to a WiFi network.

import network
import time

SSID = "Kazuki"  # 网络名称
KEY = "123qweasdzxc"  # 网络密码

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

# 显式初始化 Wi-Fi 模块
try:
    wlan.connect(SSID, KEY)
except OSError as e:
    print("Wi-Fi 初始化错误:", e)
    time.sleep(5)
    wlan.connect(SSID, KEY)

while not wlan.isconnected():
    print('正在尝试连接到 "{:s}"...'.format(SSID))
    time.sleep_ms(1000)

print("Wi-Fi 已连接 ", wlan.ifconfig())

