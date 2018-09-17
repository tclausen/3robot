from __future__ import division
import time
import sys
import spidev
import Adafruit_PCA9685
from threading import Thread

from Log import *

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

class Motors:
    def __init__(self, motors):
        self.motors = motors
        self.log = Log(time.time(), "motors")
        self.state = "stopped"
        self.runner = MotorThread(motors)
        
    def start(self):
        self.log.logt("# Motors started")
        self.state = "running"
        print "START motors"
        self.setNeutral()
        self.runner.start()

    def stop(self):
        self.log.logt("# Motors stopped")
        print "STOP motors"
        self.runner.running = False
        self.setNeutral()
        self.runner.join()

    def setNeutral(self):
        for m in self.motors:
            m.setNeutral()

class MotorThread(Thread):
    def __init__(self, motors):
        Thread.__init__(self)
        self.motors = motors
        self.running = False
        self.dt = 0.05
        
    def run(self):
        self.running = True
        t0 = time.time()
        tp = time.time()
        t = time.time()
        time.sleep(self.dt)
        while self.running:
            for m in self.motors:
                t = time.time()
                dt = t - tp
                c = m.update(dt)
                print ("%s %f %f %f %f" % (m.name, t-t0, m.target(), m.force(), c))
            time.sleep(self.dt)
            #f.write("%f %f %f %f\n" % (t-tStart, target, m.force(), c))
            #print ("%f %f %f %f\n" % (t-tStart, target, m.force(), c))
            tp = t

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

    def target(self):
        return self._targetForce
    
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
        #self.totalError = self.totalError + e*dt
        #i = 0.0 * self.totalError
        i = 0
        correction = int(p + d + i)
        if f > 9000:
            print ("Motor " + self.name + " force overload!!!!")
            correction = 5
        self.move(correction)
        self.ep = e
        return correction

    def setNeutral(self):
        self.move(0)
