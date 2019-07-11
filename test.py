from ev3dev2.motor import *
import time
import numpy as np
import logging as lg

R = 3    #Raddurchmesser
T = 6    #Radumdrehungen für Vollkreisdrehung
#W = 1.9  #%s/cm um Geschwindigkeiten in % der Leistunf an den Motoren umzurechnen

end=0

m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)
m3 = LargeMotor(OUTPUT_C)
m4 = LargeMotor(OUTPUT_D)

def set_motion(v, dt):
    global end
    lg.info('Moving %s for %.2f s'%(v,dt))
    start = time.time()
    if end>0:
        lg.debug("dt for comp=%.4fs"%(end-start))
        
    m1.on(SpeedRPS(v[0]), brake = False)  #oben links
    m2.on(SpeedRPS(v[1]), brake = False)  #oben rechts
    m3.on(SpeedRPS(-v[2]), brake = False)  #unten links
    m4.on(SpeedRPS(-v[3]), brake = False)  #unten rechts
    
    mid = time.time()
    lg.debug("dt for setting=%.4fs"%(mid - start))
    time.sleep(dt)
    end = time.time()
    lg.debug("dt error for sleep=%.4fs"%(end - mid - dt))

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
    lg.debug("r=%.4f, b=%.4f, rho[%%]=%.4f, rho[°]=%.4f"%(r,b,rho/(2*np.pi),np.rad2deg(rho)))
    lg.debug("t=[%.4f,%.4f]"%(tx,ty))
    v = dreh + trans([tx,ty], dt)
    lg.debug("rot=%s, tran=%s"%(np.around(dreh, decimals=4),np.around(trans([tx,ty], dt), decimals=4)))
#%%
    set_motion(v,dt)
    #set_motion([0,0,0,0],0)

def upwardspiral():
    lg.info("MOVE=UPWARDSPIRAL")
    for i in range(0,8):
        circle(0,10,45*i,45,3)
    set_motion([0,0,0,0],0)
    
    '''for i in range(0,8):
        circle(0,12.5,45*i,45,5)
    set_motion([0,0,0,0],0)
    '''
    
def klee():
    lg.info("MOVE=KLEE")
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
        lg.basicConfig(level=lg.DEBUG)
        str = input()
        lg.info('%s received'%str)
        if str == 'q':
            break
        else:
            v = str.split(',')
            inp = np.array(v).astype(np.float)
            #move(inp[1:],inp[0])
            #circle(inp[0],inp[1],inp[2],inp[3],inp[4])
            klee()
            time.sleep(5)
            upwardspiral()
        