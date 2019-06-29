import logging as lg
import numpy as np
from ev3dev2.motor import *

def readSequenz(filename):
    '''Erstelle eine Liste von Tasks - (COMMAND,args1,args2,..))'''
    seq = []
    with open(filename) as file:
        for line in file:
            do = line.strip('\n').split(' ')
            seq.append(do)
    return seq
    
class Fahrzeug():
    '''
    vel = [velx,vely] - Geschwindigkeit in cm/s
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
    change   - neuer Task wird von vorbereitet
    timer    - Zeit, bis akzueler Task vorbei ist
    curTask = [dx, dy, dphi, dt] - aktueller Task
    deltaNext = [ddx,ddy,ddphi] - Unterschied zwischen dx,dy,dphi von aktuellem/nächsten Task
    
    '''
    
    def __init__(self, seq, maxVel, maxAcc, T, R, fahrzeuglist=None):
        self.maxVel = maxVel
        self.maxAcc = maxAcc
        self.T = T
        self.R = R
        self.pos = np.zeros(2)
        self.errpos = np.zeros(2)
        self.phi = 0
        self.vel = np.array([0,0]).astype(float)
        self.finished = len(seq) == 0
        self.mode = 'SMOOTH'
        self.seq = seq
        self.timer = 0
        self.curTask = [0,0,0,0]
        self.subs = fahrzeuglist
        self.lookahead()
        self.nextTask()
        
        #TODO
        m1 = LargeMotor(OUTPUT_A)
        m2 = LargeMotor(OUTPUT_B)
        m3 = LargeMotor(OUTPUT_C)
        m4 = LargeMotor(OUTPUT_D)
        
    def nextTask(self):
        '''Beende den aktuellen Task und beginne Anpassung an neue dx,dy,dphi'''
        if not self.finished:
            self.curTask = np.array([float(x) for x in self.seq.pop(0)[1:5]])
            self.change = True
            
            #Fehler mitberücksichtigen
            tmp = np.zeros(4)
            tmp[0:-2] = self.errpos
            #TODO
            self.move(self.curTask+tmp)
            self.timer = 0
            self.finished = len(self.seq) == 0
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
        #Anpassung fertig?
        if self.change:
            self.change = False
            self.lookahead()
        
        #Update timer und pos
        self.timer += dt
        self.pos += self.updatePos(dt)
        
        #Anpassung starten
        if self.timer > self.ende:
            if self.mode == 'NULL' or self.finished:
                #Bremsen
                self.set_motion(np.zeros(4),0)
                self.vel = np.zeros(2)
            else:
                #TODO
                lg.info('TODO')
        if self.timer > self.curTask[3]:
            #Berechne zu weit gefahrene Distanz
            self.errpos = -self.updatePos(self.timer-self.curTask[3])
            self.nextTask()
            if self.finished:
                return True
        return False
        
    def updatePos(self, dt):
        if self.curTask[2] == 0:
            return self.vel*dt
        else:
            #TODO
            lg.info('TODO')
        
    def status(self, printing='info'):
        #Konsole
        if printing == 'hud':
            return "[%.2f,%.2f,%.2f°] @ %s"%(self.pos[0], self.pos[1], self.phi%360, np.around(self.vel,decimals=2))
        stat = "----------\npos = [%.2f,%.2f,%.2f°]\nvel = %s\nMode = %s,  changing = %s,  finished = %s,  timer= %.2f,  ende= %.2f\nAktueller Task     - %s\nDelta zum nächsten - %s"%\
              (self.pos[0], self.pos[1], self.phi%360, np.around(self.vel,decimals=4), self.mode, self.change, self.finished, self.timer, self.ende, self.curTask, self.deltaNext)
        
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
        lg.debug('Move %s'%task)
        if task[2] == 0:
            self.vel = task[0:2]/task[3]
            self.set_motion(self.trans(task[0:2], task[3]), task[3])
        
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
        return (v[0] * np.array([1,-1,1,-1]) + v[1] * np.array([1,1,1,1]))/(np.pi*self.R*dt)         #Umdrehungen/s