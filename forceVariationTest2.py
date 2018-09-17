#!/usr/bin/python
from __future__ import division
import time
import sys

from Motors import *

m1 = Motor("m1", 15, 0, 365)
m2 = Motor("m2", 14, 1, 366)

motors = Motors([m1, m2])

motors.start()

f = open("state.txt", "w")
try:
    while True:
        m1.setTarget(2000)
        m2.setTarget(2000)
        time.sleep(2.0)
        m1.setTarget(8000)
        m2.setTarget(8000)
        time.sleep(2.0)
        m1.setTarget(5000)
        m2.setTarget(5000)
        time.sleep(2.0)
except KeyboardInterrupt:
    motors.stop()
    print ("Closing state.txt")
    f.close()

    
