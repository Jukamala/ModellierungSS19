# -*- coding: utf-8 -*-
"""
Created on Sat Jun 29 16:32:42 2019

@author: Justus
"""
import logging as lg
import time
import steuerung as st
from numpy import pi

if __name__ == '__main__':
    #Logger set to info
    from imp import reload
    reload(lg)
    lg.basicConfig(level=lg.INFO)

    #init
    seq = st.readSequenz('cur.seq')
    lg.info(seq)
    maxRPS = 2.5
    H = 18.5
    B = 17.7
    T = 6
    D = 6
    maxVel = maxRPS*pi*D*min(1/T,1)
    robot = st.Fahrzeug(seq, maxVel, 10, T, D)
    last = time.time()
    fertig = False
    while not fertig:
        #vergangene Zeit
        now = time.time()
        dt = now - last
        last = now
        fertig = robot.update(dt)
        lg.info("| dt = %.4f, timer = %.4f"%(dt,robot.timer))
        lg.debug(robot.status('hud'))
    robot.status()
    