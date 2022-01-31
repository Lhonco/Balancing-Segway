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

pitch_offset = -2.5
target = 0
error_sum = 0
pitch = 0
pitch_error = 0
pwm = 0

mode = 0
interval = 0
timestampOne = 0
timestampTwo = 0

alpha = 0.98
K_p = 25
K_d = 0.97
K_i = 80

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
        K_p_temp = round(40 - (pot.read() * (39 / 4095)))
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
        K_i_temp = round(20 - (pot.read() * (20 / 4095))) * 10
        oled.draw_text(0, 40,'Ki     {}        {}'.format(K_i_temp, K_i))
        oled.display()
    if (round(100 - pot.read() * (100 / 4095)) != pot_temp):
        K_i = K_i_temp
        oled.draw_text(0, 40,'Ki     {}        {}'.format(K_i_temp, K_i))
        oled.display()
    pot_temp = round(100 - pot.read() * (100 / 4095))

    oled.draw_text(0, 50,'Starting ...')
    oled.display()

oled.draw_text(45, 0, 'New    Current')
oled.draw_text(0, 10, 'alpha   -        {:4.2f}'.format(alpha))
oled.draw_text(0, 20,'Kp      -         {:2d}'.format(K_p))
oled.draw_text(0, 30,'Kd      -        {:4.2f}'.format(K_d))
oled.draw_text(0, 40,'Ki      -         {}'.format(K_i))
oled.display()

try:
    tic1 = pyb.micros()
    tic2 = pyb.millis()
    tic3 = pyb.millis()
    while True:
        dt1 = pyb.micros() - tic1
        dt2 = pyb.millis() - tic2
        currentTime = pyb.millis()        

        if (dt1 > 3000):
            # if (mode == 1 or mode == 5):
            #     target += 1
            #     target = max(min(1, target), -1)
            #     target = target / 1000
            # elif (mode == 2 or mode == 4):
            #     target -= 1
            #     target = max(min(1, target), -1)
            #     target = target / 1000
            # else:
            #     target = 0
            if trigger.value(): tunePid()
            pitch, pitch_dot = estimatePitch(pitch, (dt1 / 1000000), alpha)
            tic1 = pyb.micros()
            pitch_error = (target + pitch_offset) - pitch
            error_sum = error_sum + pitch_error * (dt1 / 1000000)
            
            pwm = K_p * pitch_error + K_d * (-pitch_dot) + K_i * error_sum
            pwm = max(min(100, pwm), -100)

            if (pwm > 10):
                motor.right_forward(pwm)
                motor.left_forward(pwm)
            elif (pwm < -10):
                motor.right_back(pwm)
                motor.left_back(pwm)
            else:
                motor.stop()
                error_sum = 0
            
        # if (currentTime - timestampOne >= interval):
        #     timestampOne = currentTime
        #     if (mode == 0):
        #         mode = 1
        #         interval = 4000
        #         pitch_offset = -2.5
        #         oled.draw_text(0, 50,'Staying ...       ')
        #         oled.display()
        #     elif (mode == 1):
        #         mode = 0
        #         interval = 4000
        #         pitch_offset = -3.5
        #         oled.draw_text(0, 50,'Going forward ...       ')
        #         oled.display()

        # if (currentTime - timestampOne >= interval):
        #     timestampOne = currentTime
        #     if (mode == 0):
        #         mode = 1
        #         interval = 2000
        #         oled.draw_text(0, 50,'Going forward ...       ')
        #         oled.display()
        #     elif (mode == 1):
        #         mode = 2
        #         interval = 2000
        #         oled.draw_text(0, 50,'Going forward ...       ')
        #         oled.display()
        #     elif (mode == 2):
        #         mode = 3
        #         interval = 5000
        #         oled.draw_text(0, 50,'Staying ...             ')
        #         oled.display()
        #     elif (mode == 3):
        #         mode = 4
        #         interval = 2000
        #         oled.draw_text(0, 50,'Going backwards ...     ')
        #         oled.display()
        #     elif (mode == 4):
        #         mode = 5
        #         interval = 2000
        #         oled.draw_text(0, 50,'Going backwards ...     ')
        #         oled.display()
        #     elif (mode == 5):
        #         mode = 0
        #         interval = 5000
        #         oled.draw_text(0, 50,'Staying ...             ')
        #         oled.display()
        
        if (dt2 > 250):
            tic2 = pyb.millis()
            # print('Pitch: {:+6.1f} deg, Pitch Dot: {:+6.1f} deg, PWM: {:6.1f}, E-Sum: {:6.1f}, e: {}, target: {}, mode: {}'.format(pitch, pitch_dot, pwm, error_sum, pitch_error, target, mode))
            print('P: {}, D: {}, I: {}, target: {}, pitch: {}, pitch_dot: {}, error_sum: {}, pitch_offset: {}'.format(K_p * pitch_error, K_d * (-pitch_dot), K_i * error_sum, target, pitch, pitch_dot, error_sum, pitch_offset))

finally:
    motor.stop()