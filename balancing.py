import pyb
import motor
from mpu6050 import MPU6050
from pyb import Pin, ADC, ExtInt
from oled_938 import OLED_938
from motor import *

oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'},
					height=64, external_vcc=False, i2c_devid=60)

imu = MPU6050(1, False)

pot = ADC(Pin('X11'))

trigger = pyb.Switch()

oled.poweron()
oled.init_display()

motor = DRIVE()

e = 0
g = -2.5
e_sum = 0
pitch = 0
pitch_error = 0
pwm = 0
tic = pyb.millis()
currentTime = 0
previousTime = 0
offset = 0

alpha = 0.98
K_p = 25
K_d = 0.97
K_i = 70

def estimatePitch(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha * (pitch + pitch_dot * dt) + (1 - alpha) * theta
    return (pitch, pitch_dot)

def pidControl(pitch, pitch_dot, target, pitch_error):
    return (K_p * (target - pitch) + K_d * (target - pitch_dot) + K_i * pitch_error)

def tunePid():
    alpha_temp = 0
    K_p_temp = 0
    K_d_temp = 0
    K_i_temp = 0
    global alpha
    global K_p
    global K_d
    global K_i

    pot_temp = round(100 - pot.read() * (100 / 4095))

    motor.stop()

    oled.clear()
    oled.draw_text(45, 0, 'New    Current')

    while trigger.value(): pass
    while not trigger.value():
        pyb.delay(1)
        alpha_temp = round((0.99 - (pot.read() * (9 / 4095)) / 100), 2)
        oled.draw_text(0, 10, 'alpha  {:4.2f}      {:4.2f}'.format(alpha_temp, alpha))
        oled.display()
    if (round(100 - pot.read() * (100 / 4095)) != pot_temp):
        alpha = alpha_temp
        oled.draw_text(0, 10, 'alpha  {:4.2f}      {:4.2f}'.format(alpha_temp, alpha))
        oled.display()
    pot_temp = round(100 - pot.read() * (100 / 4095))

    while trigger.value(): pass
    while not trigger.value():
        pyb.delay(1)
        K_p_temp = round(30 - (pot.read() * (29 / 4095)))
        oled.draw_text(0, 20,'Kp     {:2d}         {:2d}'.format(K_p_temp, K_p))
        oled.display()
    if (round(100 - pot.read() * (100 / 4095)) != pot_temp):
        K_p = K_p_temp
        oled.draw_text(0, 20,'Kp     {:2d}         {:2d}'.format(K_p_temp, K_p))
        oled.display()
    pot_temp = round(100 - pot.read() * (100 / 4095))

    while trigger.value(): pass
    while not trigger.value():
        pyb.delay(1)
        K_d_temp = round((1.2 - (pot.read() * (12 / 4095)) / 10), 2)
        oled.draw_text(0, 30,'Kd     {:4.2f}      {:4.2f}'.format(K_d_temp, K_d))
        oled.display()
    if (round(100 - pot.read() * (100 / 4095)) != pot_temp):
        K_d = K_d_temp
        oled.draw_text(0, 30,'Kd     {:4.2f}      {:4.2f}'.format(K_d_temp, K_d))
        oled.display()
    pot_temp = round(100 - pot.read() * (100 / 4095))

    while trigger.value(): pass
    while not trigger.value():
        pyb.delay(1)
        K_i_temp = round(10 - (pot.read() * (10 / 4095))) * 10
        K_i_temp = 0
        oled.draw_text(0, 40,'Ki     {}        {}'.format(K_i_temp, K_i))
        oled.display()
    if (round(100 - pot.read() * (100 / 4095)) != pot_temp):
        K_i = K_i_temp
        oled.draw_text(0, 40,'Ki     {}        {}'.format(K_i_temp, K_i))
        oled.display()
    pot_temp = round(100 - pot.read() * (100 / 4095))

    tic1 = pyb.micros()
    oled.draw_text(0, 50,'Balancing ...')
    oled.display()

# tunePid()

try:
    tic1 = pyb.micros()
    tic2 = pyb.millis()
    while True:
        if trigger.value(): tunePid()
        # currentTime = pyb.millis()
        # toc = pyb.millis()
        # dt = (toc - tic) / 1000
        dt1 = pyb.micros() - tic1
        dt2 = pyb.millis() - tic2
        # tic = pyb.millis()

        if (dt1 > 3000):
            pitch, pitch_dot = estimatePitch(pitch, (dt1 / 1000000), alpha)
            tic1 = pyb.micros()
            e = g - pitch
            e_sum = e_sum + e * (dt1 / 1000000)
            
            pwm = K_p * e + K_d * (-pitch_dot) + K_i * e_sum
            pwm = max(min(100, pwm), -100)

            if (pwm > 10):
                motor.right_forward(pwm)
                motor.left_forward(pwm)
            elif (pwm < -10):
                motor.right_back(pwm)
                motor.left_back(pwm)
            else:
                motor.stop()
                e_sum = 0

        if (dt2 > 500):
            tic2 = pyb.millis()
            print('Pitch: {:+6.1f} deg, Pitch Dot: {:+6.1f} deg, PWM: {:6.1f}, E-Sum: {:6.1f}, e: {}'.format(pitch, pitch_dot, pwm, e_sum, e))

        # if(currentTime - previousTime >= 0):
        #     pwm = K_p * e + K_d * (-pitch_dot)
        #     if (pwm > 0):
        #         motor.right_forward(pwm + offset)
        #         motor.left_forward(pwm + offset)
        #     else:
        #         motor.right_back(pwm + offset)
        #         motor.left_back(pwm + offset)
        #     print('PWM: {:6.1f}'.format(pwm))

        # pwm = pidControl(pitch, pitch_dot, 0, 0)

        
        # if(currentTime - previousTime > 500):
        #     previousTime = currentTime
        #     print('Pitch: {:+6.1f} deg, Pitch Dot: {:+6.1f} deg'.format(pitch, pitch_dot))
        #     print('PWM: {:6.1f}, E-Sum: {:6.1f}'.format(pwm, e_sum))

finally:
    motor.stop()