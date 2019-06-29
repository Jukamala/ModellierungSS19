import logging as lg
import numpy as np
from ev3dev2.motor import *
#%%
def readSequenz(filename):
    '''Erstelle eine Liste von Tasks - (COMMAND,args1,args2,..)'''
    seq = []
    with open(filename) as file:
        for line in file:
            print(line)
            do = line.strip('\n').split(' ')
            #Diskretisieren von langen Drehnungen
            if do[0] == 'MOVE':
                do = np.array(do[1:5]).astype(float)
                #Aufteilen in Abschnitte mit phi <= 45°
                k = np.abs(int(np.ceil(do[2]/45)))
                for i in range(0,k):
                    print(do/k)
                    seq.append(['MOVE'] + list(do/k))
            else:
                print(do)
                seq.append(do)
    return seq
#%%
m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)
m3 = LargeMotor(OUTPUT_C)
m4 = LargeMotor(OUTPUT_D)
    
class Fahrzeug():
    '''
    vel = | [velx,vely] - Geschwindigkeit in cm/s
          | [velb,      -          -"-
             r,         - Radius
             mx,my,     - Vektor zum Mittelpunkt vom Start
             beta]      - Winkel seit Start
    tmpvel              - vel der letzten Runde (für Berechnung von errpos)
    pos = [x,y], phi  - Postion in [cm,cm,Grad]
    errpos = Feler der pos zur eigentlich vorgesehenen pos
    
    maxVel, maxAcc - maximale Beschleunigung und Geschwindigkeit in [cm/s, cm/s^2]
    R - Raddurchmesser
    T - Radumdrehungen für Vollkreisdrehung
    
    subs - Alle Fahrzeuge die noch untergeordnet sind
    seq  - Die auszuführende Sequenz
    mode - aktueller Ausführungsmodus
           SMOOTH - glatter Übergang
           NULL   - zwischen Tasks stehen bleiben
    finished - Sequenz fertig?
    change   - | 2 - neuer Task gestartet
               | 1 - wird von vorbereitet
               | 0 - fertig
    timer    - Zeit, bis akzueler Task vorbei ist
    lastTask                     - vorheriger Task
    curTask = [dx, dy, dphi, dt] - aktueller Task
    deltaNext = [ddx,ddy,ddphi]  - Unterschied zwischen dx,dy,dphi von aktuellem/nächsten Task
    
    '''
    
    def __init__(self, seq, maxVel, maxAcc, T, D, fahrzeuglist=None):
        self.maxVel = maxVel
        self.maxAcc = maxAcc
        self.T = T
        self.D = D
        self.pos = np.zeros(2)
        self.errpos = np.zeros(3)
        self.phi = 0
        self.vel = np.zeros(2)
        self.tmpvel = np.zeros(2)
        self.finished = len(seq) == 0
        self.mode = 'NULL'
        self.seq = seq
        self.timer = 0
        self.lastTask = [0,0,0,0]
        self.curTask = [0,0,0,0]
        self.subs = fahrzeuglist
        self.lookahead()
        self.nextTask(0)
        
    def nextTask(self, delay):
        '''
        Beende den aktuellen Task und beginne Anpassung an neue dx,dy,dphi
        
        delay - Verspätung aus dem letzten Task
        '''
        if not self.finished:
            #Fehler mitberücksichtigen
            self.lastTask = self.curTask
            self.curTask = np.zeros(4)
            self.curTask[:-1] = self.errpos
            #Skip Tasks till next is found
            skip = True
            while skip:
                self.curTask += np.array([float(x) for x in self.seq.pop(0)[1:5]])
                #TODO: Bessere Formel für Geschw
                #Hier nur Translationsgeschw.
                ndt = self.curTask[3] - delay
                if ndt > 0 and np.sqrt(self.curTask[0]**2 + self.curTask[1]**2)/ndt < self.maxVel:
                    self.curTask[3] -= delay
                    skip = False
                
            self.change = 2
            self.move(self.curTask)
            self.timer = 0
            self.status()
        
    def lookahead(self):
        '''Nachdem die Anpassung vorbei ist (Zielgeschw. erreicht ist), berachte das nächste Ziel'''
        #Wechsel in neuen Mode
        while not self.finished and self.seq[0][0] == 'MODE':
            curTask = self.seq.pop(0)
            self.finished = len(self.seq) == 0
            self.mode = curTask[1]
        #Berechne den Abstand zum nächsten MOVE
        if not self.finished:
            nextTask = list(self.seq[0])[1:4]
            nextTask = np.array([float(x) for x in nextTask])
        else:
            nextTask = np.array([0,0,0])
        self.deltaNext = nextTask - self.curTask[0:3]
        #Wann soll Anpassung anfangen?
        if self.mode == 'NULL':
            #Am Ende bremsen:
            self.ende = self.curTask[3]-self.timer
        else:
            lg.info('TODO')
            #TODO
    
    def update(self, dt):
        '''Update with deltatime in s'''
        #Todo: Change
        
        if self.change == 2:
            #Nach dem ersten Update kann wieder aktuelles vel für updatesPos verwendet werden 
            self.tmpvel = self.vel
            self.tmpTask = self.curTask
            self.change = 1
            
        #Anpassung fertig?
        if self.change == 1:
            #Gehts noch weiter?
            self.finished = len(self.seq) == 0
            self.change = 0
            self.lookahead()
        
        #Update timer und pos
        self.timer += dt
        dpos = self.updatePos(dt)
        self.pos += dpos[0:2]
        self.phi += dpos[2]
        
        #Anpassung starten
        if self.timer > self.ende:
            if self.mode == 'NULL' or self.finished:
                #Bremsen
                self.set_motion(np.zeros(4),0)
                #self.vel = np.zeros(2) | vel wird noch für errpos gebraucht
            else:
                #TODO
                lg.info('TODO')
        if self.timer > self.curTask[3]:
            #Berechne zu weit gefahrene Distanz
            self.errpos = self.updatePos(-(self.timer-self.curTask[3]))
            self.nextTask(self.timer - self.curTask[3])
            if self.finished:
                return True
        return False
        
    def updatePos(self, dt):
        '''
        Berechnet die gefahrene Strecke nach dt.
        Für negative dt ist der Vektor umgedreht.
        Verwendet tmpvel und tmpTask, die erst im ersten Update nach dem 
        Einstellen des neuen Tasks auf den aktuellen Wert gesetzt werden.
        '''
        if self.tmpTask[2] == 0:
            dpos = np.zeros(3)
            dpos[:-1] = self.tmpvel*dt
            return dpos
        else:
            #Unnötig kompliziert
            lg.debug("tmpvel=%s"%np.around(self.tmpvel, decimals=4))
                
            beta = np.deg2rad(self.tmpvel[4])
            lg.debug("sgn= %d, %d"%(np.sign(dt),np.sign(self.tmpTask[2])))
            beta2 = beta + np.sign(self.tmpTask[2]) * (self.tmpvel[0]*dt/self.tmpvel[1])
            lg.debug("beta=%.4f, beta2=%.4f"%(beta,beta2))
            c1,c2 = np.cos([beta,beta2])
            s1,s2 = np.sin([beta,beta2])
            #Vektor zu Mitte(beta) + Vektor von Mitte(beta2)
            self.tmpvel[4] = np.rad2deg(beta2)
            return [(c1-c2)*self.tmpvel[2] + (s2-s1)*self.tmpvel[3],
                    (s1-s2)*self.tmpvel[2] + (c1-c2)*self.tmpvel[3], np.rad2deg(beta2-beta)]
        
    def status(self, printing='info'):
        #Konsole
        if printing == 'hud':
            return "[%.2f,%.2f,%.2f°] @ %s"%(self.pos[0], self.pos[1], self.phi%360, np.around(self.vel,decimals=2))
        stat = "----------\npos = [%.2f,%.2f,%.2f°]\nvel = %s\nMode = %s,  changing = %s,  finished = %s,  timer= %.2f,  ende= %.2f\nerrpos = %s\nAktueller Task     - %s\nDelta zum nächsten - %s"%\
              (self.pos[0], self.pos[1], self.phi%360, np.around(self.vel,decimals=4), self.mode, self.change, self.finished, self.timer, self.ende, np.around(self.errpos,decimals=4), self.curTask, self.deltaNext)
        
        if printing == 'debug':
            lg.debug(stat)
        else:
            lg.info(stat)
            
    
    def move(self, task):
        '''
        Ausführung der Grundbewegung.
        |Gerade                        falls dphi=0
        |Kreisegment mit Winkel phi    sonst
        
        task = [dx, dy, dphi, dt]
        '''
        lg.debug('Do Task: %s'%task)
        self.tmpvel = self.vel
        if task[2] == 0:
            self.vel = task[0:2]/task[3]
            self.set_motion(self.trans(task[0:2], task[3]), task[3])
            
        else:
            self.circle(task[0],task[1],task[2],task[3])
            
    def circle(self, dx,dy,alpha,dt):
        '''
        dx, dy - Abstand zum nächsten Punkt
        phi    - aktuelle Ausrichtung des Roboters
        alpha  - zu drehender Winkel in Grad  < 90°
        dt     - Zeit für das Drehen
        '''
        dreh = self.rot(alpha,dt)                        #Rotation
        
        sgn = np.sign(alpha)
        alpha = np.abs(alpha)
        [alpha,phi] = np.deg2rad([alpha,self.phi])    #Bogenmaß
        v = np.sqrt(dx**2+dy**2)
        r = v/(2*np.sin(alpha/2))
        b = alpha * r 
        c = np.sign(dx)                          #links oder rechts?
        rho = (c*np.arccos(dy/v) + sgn*alpha/2 + phi)
        tx = np.sin(rho)*b
        ty = np.cos(rho)*b
        lg.debug("r=%.4f, b=%.4f, rho[%%]=%.4f, rho[°]=%.4f"%(r,b,rho/(2*np.pi),np.rad2deg(rho)))
        lg.debug("t=[%.4f,%.4f]"%(tx,ty))
        
        [mx,my] = r*sgn*np.array([ty,tx])/(np.sqrt(ty**2+tx**2))   #relativ
        c = np.cos(phi)
        s = np.sin(phi)
        mxn = c*mx-s*my
        myn = s*mx+c*my                                             #absolut
        self.vel = np.array([b/dt,r,mxn,myn,0])
        
        bew = self.trans([tx,ty], dt)
        v = dreh + bew
        lg.debug("rot=%s, tran=%s"%(np.around(dreh, decimals=4),np.around(bew, decimals=4)))
        self.set_motion(v,dt)
       
        
    def set_motion(self, v, dt):
        '''
        set the power of the motors
        
        v = [v1,v2,v3,v4] in RPS
        '''
        lg.info('Moving %s for %.2f s'%(v,dt))
        
        m1.on(SpeedRPS(v[0]), brake = False)  #oben links
        m2.on(SpeedRPS(v[1]), brake = False)  #oben rechts
        m3.on(SpeedRPS(-v[2]), brake = False)  #unten links
        m4.on(SpeedRPS(-v[3]), brake = False)  #unten rechts
        
    def rot(self, alpha,dt):
        'Bewegungskomponente für Rotation'
        return alpha/360 * self.T * np.array([-1,1,1,-1])/dt   # Umdreungen/s
        
        
    def trans(self, v, dt):
        '''
        Bewegungskomponente für Translation
        
        v = [dx,dy] - Richtung in cm
        dt          - Zeit 
        '''
        return (v[0] * np.array([1,-1,1,-1]) + v[1] * np.array([1,1,1,1]))/(np.pi*self.D*dt)         #Umdrehungen/s