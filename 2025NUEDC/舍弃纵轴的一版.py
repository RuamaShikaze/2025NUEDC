import sensor, image, time,struct
from pid import PID
from pyb import Servo, LED,millis,UART,Pin
uart = UART(3, 9600)
uart.init(9600, bits=8, parity=None, stop=1)#111
red_led = LED(1)
pan_servo = Servo(1)
tilt_servo = Servo(2)
pan_servo.calibration(500, 2500, 1500)
tilt_servo.calibration(500, 2500, 1300)
class Laser:
    def __init__(self, pin_name):
        self.pin = Pin(pin_name, Pin.OUT_PP)
        self.off()
    def on(self):
        self.pin.high()
    def off(self):
        self.pin.low()
    def toggle(self):
        self.pin.value(not self.pin.value())
laser = Laser("P2")
pan_pid = PID(p=0.015, i=0.017, d = 0.008,imax=90)
tilt_pid = PID(p=0.001, i=0.1, d=0.005, imax=90)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()
IMAGE_CENTER_X = sensor.width() // 2
IMAGE_CENTER_Y = sensor.height() // 2
WHITE_THRESHOLD = (70, 100, -20, 20, -20, 20)
BLACK_THRESHOLD = (0, 50, -20, 20, 20, -20)
target_offset = [-1, -1]
class KalmanFilter:
    def __init__(self, process_noise=0.01, measurement_noise=1, initial_value=0):
        self.x = initial_value
        self.v = 0
        self.P = [[1, 0],
                 [0, 1]]
        self.Q = [[process_noise, 0],
                 [0, process_noise]]
        self.R = measurement_noise
        self.F = [[1, 1],
                  [0, 1]]
        self.H = [1, 0]
    def predict(self):
        self.x = self.F[0][0] * self.x + self.F[0][1] * self.v
        self.v = self.F[1][0] * self.x + self.F[1][1] * self.v
        self.P[0][0] = self.F[0][0] * self.P[0][0] + self.F[0][1] * self.P[1][0] + self.Q[0][0]
        self.P[0][1] = self.F[0][0] * self.P[0][1] + self.F[0][1] * self.P[1][1] + self.Q[0][1]
        self.P[1][0] = self.F[1][0] * self.P[0][0] + self.F[1][1] * self.P[1][0] + self.Q[1][0]
        self.P[1][1] = self.F[1][0] * self.P[0][1] + self.F[1][1] * self.P[1][1] + self.Q[1][1]
        return self.x
    def update(self, measurement):
        S = self.H[0] * self.P[0][0] * self.H[0] + self.R
        K = [self.P[0][0] * self.H[0] / S,
             self.P[1][0] * self.H[0] / S]
        y = measurement - self.H[0] * self.x
        self.x = self.x + K[0] * y
        self.v = self.v + K[1] * y
        self.P[0][0] = (1 - K[0] * self.H[0]) * self.P[0][0]
        self.P[0][1] = (1 - K[0] * self.H[0]) * self.P[0][1]
        self.P[1][0] = (1 - K[1] * self.H[0]) * self.P[1][0]
        self.P[1][1] = (1 - K[1] * self.H[0]) * self.P[1][1]
        return self.x
kalman_x = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_X)
kalman_y = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_Y)
def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))
def move_to_initial_position():
    print("Moving to initial position...")
    tilt_servo.angle(-30)
#move_to_initial_position()
last_time = time.ticks_ms()
while True:
    timer = millis()
    clock.tick()
    img = sensor.snapshot()
    #laser.off()
    red_led.off()
    current_time = time.ticks_ms()
    dt = (current_time - last_time)/1000
    last_time = current_time
    black_blobs = img.find_blobs([BLACK_THRESHOLD], merge=False)
    if black_blobs:
        for single_black_blob in black_blobs:
            if single_black_blob.pixels() / (single_black_blob.w() * single_black_blob.h()) < 0.3:
                white_blobs = img.find_blobs([WHITE_THRESHOLD], area_threshold=2, roi=single_black_blob.rect(), merge=False)
                if white_blobs:
                    largest_white = max(white_blobs, key=lambda b: b.area())
                    if (2 < largest_white.pixels() / single_black_blob.pixels() < 4) and \
                            abs(largest_white.cx() - single_black_blob.cx()) < 10 and \
                            abs(largest_white.cy() - single_black_blob.cy()) < 10:
                        red_led.on()
                        target = [largest_white.cx() + target_offset[0], largest_white.cy() + target_offset[1]]
                        img.draw_cross(target, color=(255, 0, 0), thickness=3)
                        roi = largest_white.rect()
                        raw_x = target[0]
                        raw_y = target[1]
                        kalman_x.predict()
                        kalman_y.predict()
                        filtered_x = kalman_x.update(raw_x)
                        filtered_y = kalman_y.update(raw_y)
                        img.draw_cross(int(filtered_x), int(filtered_y), color=(255,0,0))
                        img.draw_cross(raw_x, raw_y, color=(0,255,0))
                        pan_error = raw_x - IMAGE_CENTER_X - 5
                        tilt_error = raw_y - IMAGE_CENTER_Y - 5
                        if (pan_error<=3)  and (pan_error>= -3) :
                            laser.on()
                            print("Laser on")
                        pan_output = pan_pid.get_pid(pan_error, 1)
                        tilt_output = tilt_pid.get_pid(tilt_error, 1)
                        current_pan = pan_servo.angle()
                        pan_output = clamp(pan_output,-3,3)
                        new_pan = current_pan - pan_output
                        pan_servo.angle(new_pan)
                        current_tilt = tilt_servo.angle()
                        tilt_output = clamp(tilt_output,-2,2)
                        new_tilt = current_tilt - tilt_output
                        #tilt_servo.angle(new_tilt)
                        print("err_x=%.1f,err_y=%.1f,out_pan=%.1f,out_tilt=%.1f"%(pan_error,tilt_error,pan_output,tilt_output))
                        uart.write("%.1f,%.1f,%.1f,%.1f\n"%(pan_error,tilt_error,pan_output,tilt_output))
    else:
        pan_pid.reset_I()
        tilt_pid.reset_I()
        kalman_x = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_X)
        kalman_y = KalmanFilter(process_noise=0.01, measurement_noise=1, initial_value=IMAGE_CENTER_Y)

    img.draw_string(0, 0, "FPS:%.2f" % clock.fps(), color=(255,0,0))
