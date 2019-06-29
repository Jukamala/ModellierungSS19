# -*- coding: utf-8 -*-
"""
Created on Sat Jun 29 16:32:42 2019

@author: Justus
"""
import logging as lg
import time
import steuerung as st

if __name__ == '__main__':
    #Logger set to info
    from imp import reload
    reload(lg)
    lg.basicConfig(level=lg.INFO)

    #init
    seq = st.readSequenz('cur.seq')
    lg.info(seq)
    robot = st.Fahrzeug(seq, 200, 10, 6, 6)
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
    