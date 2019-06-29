# -*- coding: utf-8 -*-
"""
Created on Sat Jun 29 16:59:58 2019

@author: Justus
"""

from ev3dev2.motor import *

m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)
m3 = LargeMotor(OUTPUT_C)
m4 = LargeMotor(OUTPUT_D)

m1.off()
m2.off()
m3.off()
m4.off()