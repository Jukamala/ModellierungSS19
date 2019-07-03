# -*- coding: utf-8 -*-
"""
Created on Sat Jun 29 16:32:42 2019

@author: Justus
"""
import argparse as ap
import logging as lg
import time
import steuerung as st
from numpy import pi

if __name__ == '__main__':
    parser = ap.ArgumentParser(description='Führe eine Sequenz aus einer Datei ab')
    parser.add_argument('--filename', default='cur.seq', help = 'inpitfile for seq')
    parser.add_argument('--level', default='INFO', help = 'level for logging - DEBUG / INFO')
    args = parser.parse_args()
    #Logger set to info
    from imp import reload
    reload(lg)
    if args.level == 'DEBUG':
        lg.basicConfig(level=lg.DEBUG)
    else:
        lg.basicConfig(level=lg.INFO)

    #init
    seq = st.readSequenz(args.filename)
    lg.info("Sequenz: %s"%seq)
    maxRPS = 2.5
    H = 18.5
    B = 17.7
    T = 6
    D = 6
    maxVel = maxRPS*pi*D*min(1/T,1)
    robot = st.Fahrzeug(seq, maxVel, 10, T, D)
    robot.setSpeed(1)
    last = time.time()
    fertig = False
    wait = 0
    while not fertig:
        #vergangene Zeit
        now = time.time()
        dt = now - last
        last = now
        fertig = robot.update(dt)
        lg.debug("| dt = %.4f, timer = %.4f"%(dt,robot.timer))
        lg.debug(robot.status('hud'))
    robot.status()
    