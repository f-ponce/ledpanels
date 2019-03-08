#!/usr/bin/python

from __future__ import print_function
import time
from led_pwm_control import LEDController 
import numpy as np

dev = LEDController('/dev/led-device')

suns = [3,5,6,9]

shuffle_suns = suns
np.random.shuffle(shuffle_suns)
print(shuffle_suns)

time_stripe = 31
time_dark = 300
sun_time = 300 
sun_intensity = 50

#stripe+dark
dev.set_value(shuffle_suns[0], 0)
time.sleep(time_stripe+time_dark)

#sun 1
dev.set_value(shuffle_suns[0], sun_intensity)
print(shuffle_suns[0])
time.sleep(sun_time*2)
dev.set_value(suns[0], 0)

#sun 2
dev.set_value(shuffle_suns[1], sun_intensity)
print(shuffle_suns[0])
time.sleep(sun_time*2)
dev.set_value(suns[0], 0)

#sun 1 again
dev.set_value(shuffle_suns[0], sun_intensity)
print(shuffle_suns[0])
time.sleep(sun_time)
dev.set_value(suns[0], 0)
