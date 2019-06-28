# -*- coding: utf-8 -*-
"""
Created on Fri Jun 28 18:23:28 2019

@author: Justus
"""

from ev3dev2.motor import *
import time
import numpy as np

R = 3    #Raddurchmesser
T = 6    #Radumdrehungen für Vollkreisdrehung
#W = 1.9  #%s/cm um Geschwindigkeiten in % der Leistunf an den Motoren umzurechnen


m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)
m3 = LargeMotor(OUTPUT_C)
m4 = LargeMotor(OUTPUT_D)

def set_motion(v, dt):
    print('Moving %s for %.2f s'%(v,dt))
    m1.on(SpeedRPM(60*v[0]), brake = False)  #oben links
    m2.on(SpeedRPM(60*v[1]), brake = False)  #oben rechts
    m3.on(SpeedRPM(-60*v[2]), brake = False)  #unten links
    m4.on(SpeedRPM(-60*v[3]), brake = False)  #unten rechts
    time.sleep(dt)
    

'''def move(v, dt):
    set_motion(v)
    start = time.time()
    end = start
    while end - start < dt:
        end = time.time()
        print(end - start)
    set_motion([0,0,0,0])'''
    
#%%
def rot(phi,dt):
    'Bewegungskomponente für Rotation'
    return phi/360 * T * np.array([-1,1,1,-1])/dt   # Umdreungen/s
    
    
def trans(v, dt):
    '''
    Bewegungskomponente für Translation
    
    v = [dx,dy] - Richtung in cm
    dt          - Zeit 
    '''
    
    return (v[0] * np.array([1,-1,1,-1]) + v[1] * np.array([1,1,1,1]))/(2*np.pi*R*dt)         #Umdrehungen/s
    

def circle(dx,dy,alpha,phi,dt):
    '''
    dx, dy - Abstand zum nächsten Punkt
    alpha  - aktuelle Ausrichtung des Roboters
    phi    - zu drehender Winkel in Grad  < 45°
    dt     - Zeit für das Drehen
    '''
    dreh = rot(phi,dt)                        #Rotation
    
    sgn = -np.sign(phi)
    phi = np.abs(phi)
    [phi,alpha] = np.deg2rad([phi,alpha])    #Bogenmaß
    v = np.sqrt(dx**2+dy**2)
    r = v/(2*np.sin(phi/2))
    b = phi * r 
    c = np.sign(dx)                          #links oder rechts?
    rho = (c*np.arccos(dy/v) - sgn*phi/2 + alpha)
    tx = np.sin(rho)*b
    ty = np.cos(rho)*b
    print(r,b,rho/(2*np.pi),np.rad2deg(rho))
    print(tx,ty)
    v = dreh + trans([tx,ty], dt)
    print(dreh,trans([tx,ty], dt))
#%%
    set_motion(v,dt)
    set_motion([0,0,0,0],0)

def upwardspiral():
    '''for i in range(0,100):
        circle(0,1,-3.6*i,3.6,0.2)
    set_motion([0,0,0,0],0)'''
    
    for i in range(0,8):
        circle(0,12.5,45*i,45,5)
    set_motion([0,0,0,0],0)
    
def klee():
    i = 0
    for t in [(10,10), (-10,-10), (10,-10), (-10,10), (-10,-10), (10,10), (-10,10), (10,-10)]:
        if i % 2 == 0:
            circle(2*t[0],2*t[1],0,90,7)
        else:
            circle(2*t[0],2*t[1],90,-90,7)
        i = i + 1
    set_motion([0,0,0,0],0)
    
if __name__ == '__main__':
    while(True):
        str = input()
        print('%s received'%str)
        if str == 'q':
            break
        else:
            v = str.split(',')
            inp = np.array(v).astype(np.float)
            #move(inp[1:],inp[0])
            #circle(inp[0],inp[1],inp[2],inp[3],inp[4])
            upwardspiral()
            #klee()
        