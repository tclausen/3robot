#!/usr/bin/python
from __future__ import division
import time
import sys

# Import the PCA9685 module.
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

pulse = 366
if len(sys.argv) > 1:
    pulse = int(sys.argv[1])

print "Setting servos to ", pulse
time.sleep(0.05)
pwm.set_pwm(8, 0, pulse)

