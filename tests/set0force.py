#!/usr/bin/python
from __future__ import division
import time
import sys
import spidev
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

def readadc(adcnum):
    v = mcp.read_adc(adcnum)
    # return a value proportional til to potentiometer value (serial resistance
    # will return approx [0:10000]
    vf = float(v)
    vf = vf/(1.0 - (vf/1024.0))
    return vf

class Motor:
    def __init__(self, name, channel, adc, neutral):
        self.channel = channel
        self.adc = adc
        self.neutral = neutral
        self._targetForce = 0
        self.name = name

        self.ep = 0
        self.totalError = 0

    def force(self):
        return readadc(self.adc)

    def setTarget(self, targetForce):
        self._targetForce = targetForce

    def move(self, v):
        #print "Correction: ", self.name, v
        pwm.set_pwm(self.channel, 0, self.neutral + v)

    def update(self, dt):
        f = self.force()
        #print "force: ", self.name, f
        e = self._targetForce - f
        # P term
        p = -0.005 * e
        # D term
        d = 0.0009 * (self.ep-e)/dt
        # I term
        self.totalError = self.totalError + e*dt
        i = 0.0 * self.totalError
        i = 0
        correction = int(p + d + i)
        if f > 9000:
            print ("Motor force overload!!!!")
            correction = 5
        self.move(correction)
        self.ep = e
        return correction

    def setNeutral(self):
        self.move(0)

m1 = Motor("m1", 15, 0, 365)
m2 = Motor("m2", 14, 1, 366)
m1.setTarget(5000)
m2.setTarget(5000)

def runWithTarget(m, tStart, period, target):
    print ("Run with target %d for %f seconds, starting at %f" % (target, period, m.force()))
    m.setTarget(target)
    t0 = time.time()
    tp = time.time()
    t = time.time()
    time.sleep(0.05)
    while t - t0 < period:
        t = time.time()
        dt = t - tp 
        c = m.update(dt)
        time.sleep(0.05)
        print ("%f %f %f %f\n" % (t-tStart, target, m.force(), c))
        tp = t
    print ("Ended at %f" % (m.force()))


t = time.time()
try:
    runWithTarget(m1, t, 3, 10)
    runWithTarget(m2, t, 3, 10)
except KeyboardInterrupt:
    print ("Set neutral: %s" % m1.name)
    print ("Set neutral: %s" % m2.name)
    m1.setNeutral()
    m2.setNeutral()

    
