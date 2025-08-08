import time
from pyb import Servo, LED, delay

# 测试参数配置
ANGLE_TEST_POINTS = [0, 45,90, 179, 0, -90, -180, 0]  # 角度测试点
ANGLE_DELAY = 3000                             # 每个角度停留时间(ms)
PWM_STOP = 1500                                # 停止PWM(1.5ms)
PWM_FWD = 500                                # 正向PWM(1.0ms)
PWM_REV = 2500                                # 反向PWM(2.0ms)
PWM_DURATION = 10000                            # 每种PWM状态持续时间(ms)

# 初始化硬件
servo = Servo(1)  # 舵机连接到引脚1
led_red = LED(1)  # 角度模式指示
led_green = LED(2)# PWM模式指示

# 角度模式测试
def test_angle_mode():
    print("\n=== 角度模式测试开始 ===")
    led_red.on()
    led_green.off()

    for angle in ANGLE_TEST_POINTS:
        print(f"设置角度: {angle}°")
        servo.angle(angle)
        delay(ANGLE_DELAY)  # 等待舵机响应

    servo.angle(0)  # 回到初始位置
    led_red.off()
    print("=== 角度模式测试结束 ===")
    delay(2000)  # 模式切换间隔

# PWM速度模式测试
def test_pwm_mode():
    print("\n=== PWM模式测试开始 ===")
    led_green.on()
    led_red.off()

    # 正向转动
    print(f"正向转动 (PWM={PWM_FWD}μs)")
    servo.pulse_width(PWM_FWD)
    delay(PWM_DURATION)

    # 停止
    print(f"停止 (PWM={PWM_STOP}μs)")
    servo.pulse_width(PWM_STOP)
    delay(PWM_DURATION // 2)  # 停止时间减半

    # 反向转动
    print(f"反向转动 (PWM={PWM_REV}μs)")
    servo.pulse_width(PWM_REV)
    delay(PWM_DURATION)

    # 最终停止
    print(f"停止 (PWM={PWM_STOP}μs)")
    servo.pulse_width(PWM_STOP)
    led_green.off()
    print("=== PWM模式测试结束 ===")
    delay(2000)  # 模式切换间隔

# 主程序：循环测试两种模式
def main():
    print("360度舵机简易测试程序启动")
    print("自动循环测试：角度模式 → PWM模式 → 角度模式...")

    while True:
        test_angle_mode()
        test_pwm_mode()

if __name__ == "__main__":
    main()
