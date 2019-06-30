# -*- coding: utf-8 -*-
"""
Created on Sun Jun 30 13:22:29 2019

@author: Justus
"""

from ev3dev2.motor import *
import time

m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)
m3 = LargeMotor(OUTPUT_C)
m4 = LargeMotor(OUTPUT_D)

m1.on(SpeedRPS(1))
m2.on(SpeedRPS(-1))
m3.on(SpeedRPS(1))
m4.on(SpeedRPS(-1))

time.sleep(6)

m1.off()
m2.off()
m3.off()
m4.off()