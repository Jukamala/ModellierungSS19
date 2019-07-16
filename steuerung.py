import logging as lg
import numpy as np
import time
from ev3dev2.motor import *
#%%
def readSequenz(filename):
    '''
    Erstelle eine Liste von Tasks - (COMMAND,args1,args2,..)
    MOVE        wird zerteilt mit dphi > 22.5° in gleiche Teile mit dphi < 22.5°
    CIRC        wird nicht geändert
    WAIT        wird als [MOVE 0 0 0 dt] hinzugefügt
    '''
    seq = []
    with open(filename) as file:
        for line in file:
            lg.debug("line - %s :"%line)
            do = line.strip('\n').split(' ')
            #Diskretisieren von langen Drehnungen für direkte Strecken
            if do[0] == 'MOVE':
                do = np.array(do[1:5]).astype(float)
                #Aufteilen in Abschnitte mit phi <= 12°
                k = max(1,np.abs(int(np.floor(do[2]/12))))
                for i in range(0,k):
                    lg.debug("add - %s"%(['MOVE'] + list(do/k)))
                    seq.append(['MOVE'] + list(do/k))
            elif do[0] == 'CIRC':
                do[0] = 'MOVE'
                seq.append(do)
                lg.debug("add - %s"%do)
            #Warten
            elif do[0] == 'WAIT':
                do = ['MOVE','0','0','0',str(do[1])]
                seq.append(do)
                lg.debug("add - %s"%do)
            #Keine Diskretisierung für Kreisbögen
            elif do[0] == 'MODE':
                lg.debug("add - %s"%do)
                seq.append(do)
    return seq
#%%
m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)
m3 = LargeMotor(OUTPUT_C)
m4 = LargeMotor(OUTPUT_D)
    
class Fahrzeug():
    '''
    vel = | [velx,vely] - Geschwindigkeit     in cm/s
          | [alpha]       - Drehgeschwindigkeit in °/s
          | [velb,      - Geschwindigkeit     in cm/s
             r,         - Radius
             mx,my,     - Vektor zum Mittelpunkt vom Start
             beta]      - Winkel seit Start
          für Berechnung von errpos
    compvel             - vel der nächsten Runde
    pos = [x,y,phi]     - Postion in [cm,cm,Grad]
    errpos = Feler der pos zur eigentlich vorgesehenen pos
    
    maxVel, maxAcc - maximale Beschleunigung und Geschwindigkeit in [cm/s, cm/s^2]
    D - Raddurchmesser
    T - Radumdrehungen für Vollkreisdrehung
    subs - Alle Fahrzeuge die noch untergeordnet sind
    seq  - Die auszuführende Sequenz
    
    mode - aktueller Ausführungsmodus
           SMOOTH - glatter Übergang
           NULL   - zwischen Tasks stehen bleiben
           NOBRAKE - NULL ohne Bremsen an den Ecken (Fehler in Kauf nehmen)
    finished - Sequenz fertig?
    change   - | 2 - neuer Task gestartet
               | 1 - wird von vorbereitet
               | 0 - fertig
    timer    - Zeit, bis akzueler Task vorbei ist
    curTask = [dx, dy, dphi, dt] - aktueller Task
    nxtTask                      - nächster Task
    deltaNext = [ddx,ddy,ddphi]  - Unterschied zwischen dx,dy,dphi von aktuellem/nächsten Task
    
    speedmult - Faktor mit der die Zeit vergeht um zu Verlangsamen(<1) oder Beschleunigen(>1)
    testime/testdt - Zeitmessung für Debugging
    comptime  - Idle/Wartezeit des letzen Schrittes während berechnet wurde (eig. vernachlässigbar geworden)
    comp = [v1, v2, v3, v4] - Ansteuerung der Motoren im nächsten Schritt in Umd/s
    '''
    
    def __init__(self, seq, maxVel, maxAcc, T, D, fahrzeuglist=None):
        self.maxVel = maxVel
        self.maxAcc = maxAcc
        self.T = T
        self.D = D
        self.testtime = None
        self.testdt = None
        self.comptime = 0
        self.comp = None
        self.change = 1
        self.pos = np.zeros(3)
        self.errpos = np.zeros(3)
        self.compvel = np.zeros(2)
        self.vel = np.zeros(2)
        self.finished = len(seq) == 0
        self.mode = 'NULL'
        self.seq = seq
        self.timer = 0
        self.curTask = np.zeros(4)
        self.subs = fahrzeuglist
        self.lookahead()
        self.nextTask(0)
        self.speedmult = 1
        
    def nextTask(self, delay, start=None):
        '''
        Beende den aktuellen Task und beginne Anpassung an neue dx,dy,dphi
        
        delay - Verspätung aus dem letzten Task
        start - Zeitpunkt beim Aufruf von update()
        
        return: comp - Zeit zwischen Aufruf von update() und starten der Motoren,
                       also Berechenzeit, in der der Roboter still steht
        '''
        if not self.finished:
            self.curTask = np.zeros(4)
            self.curTask[:-1] = self.errpos
            #Skip Tasks till next is found
            skip = True
            skipped = False
            while skip:
                task = np.array([float(x) for x in self.seq.pop(0)[1:5]])
                self.curTask += task
                #TODO: Bessere Formel für Geschw
                #Hier nur Translationsgeschw.
                ndt = self.curTask[3] - delay
                nvel = np.sqrt(self.curTask[0]**2 + self.curTask[1]**2)/ndt
                if ndt > 0 and  nvel < self.maxVel:
                    #alte Zeit speichern und anpassen
                    odt = self.curTask[3]
                    self.curTask[3] -= delay
                    skip = False
                else:
                    lg.warning("%s skipped, ndt= %.4f, nvel= %.4f"%(task,ndt,nvel))
                    skipped = True
                
            self.change = 2
            if not skipped and self.comp is not None:
                #Skallieren, da ndt sich geändert hat
                self.comp *= odt/ndt
                #Erste Komponente von vel ist Geschw.
                self.compvel[0] *= odt/ndt
                #Außer wenn vel = [vx, vy]
                if self.comp[2] == 0:
                    self.compvel[1] *= odt/ndt
            else:
                #Neu berechnen
                self.comp = self.move(self.curTask)
            
            self.status()
            
            #Bewegung starten und Berechnungszeit timen
            self.set_motion(self.comp,ndt)

            if start is not None:
                comp = (time.time() - start)
            else:
                comp = 0
            self.timer = 0
            lg.debug("comptime = %.4f"%comp)
            return comp
        
    def lookahead(self):
        '''Nachdem die Anpassung vorbei ist (Zielgeschw. erreicht ist), berachte das nächste Ziel'''
        #Wechsel in neuen Mode
        while not self.finished and self.seq[0][0] == 'MODE':
            curTask = self.seq.pop(0)
            self.finished = len(self.seq) == 0
            self.mode = curTask[1]
        #Berechne den Abstand zum nächsten MOVE
        if not self.finished:
            self.nxtTask = list(self.seq[0])[1:5]
            self.nxtTask = np.array([float(x) for x in self.nxtTask])
        else:
            self.nxtTask = np.zeros(4)
        self.deltaNext = self.nxtTask[0:3] - self.curTask[0:3]
        #Wann soll Anpassung anfangen?
        if self.mode == 'NULL' or self.mode == 'NOBRAKE':
            #Am Ende bremsen:
            self.ende = self.curTask[3]-self.timer
        else:
            lg.info('TODO')
            #TODO
    
    def update(self, dt, realpos = None):
        '''
        Update with deltatime in s (factored with speedmult)
        realpos = | [x,y,phi] - Optional tatsächliche Postion (von Sensoren, ...) zur Korrektur
                  | None      - Keine Korrektur
        '''
        #Berechnungszeit vernachlässigen
        start = time.time()
        dt *= self.speedmult
        dt -= self.comptime
        self.comptime = 0
        
        #Korrektur
        if realpos is not None:
            self.errpos = realpos - self.pos
        
        
        #TODO: change,ende
        
        if self.change == 2:
            #Nach dem ersten Update kann wieder aktuelles vel für updatesPos verwendet werden 
            self.vel = self.compvel
            self.tmpTask = self.curTask
            self.change = 1
            
        #Anpassung fertig?
        if self.change == 1:
            #Gehts noch weiter?
            self.finished = len(self.seq) == 0
            self.change = 0
            self.lookahead()
            #Nächsten Task berechnen
            lg.debug("nxtTask = %s, errpos = %s"%(self.nxtTask, self.errpos))
            nxt = self.nxtTask
            nxt[:-1] = nxt[:-1] + self.errpos
            self.comp = self.move(nxt,nphi = self.pos[2] + self.curTask[2])
            lg.info("comp=%s"%self.comp)
        
        #Update timer und pos
        self.timer += dt
        dpos = self.updatePos(dt)
        self.pos += dpos
        
        #Anpassung starten
        if self.timer > self.ende:
            if self.mode == 'NULL' or self.finished:
                    self.set_motion(np.zeros(4),0)
            else:
                if self.mode != 'NOBRAKE':
                    #TODO
                    lg.info('TODO')
                
        if self.timer > self.curTask[3]:
            t = self.timer
            self.comptime = self.nextTask(self.timer - self.curTask[3],start=start)
            #Berechne zu weit gefahrene Distanz (fließt erst in übernächste Berechung ein)
            self.errpos = self.updatePos(-(t-self.tmpTask[3]))
            if self.finished:
                return True
        return False
        
    def updatePos(self, dt):
        '''
        Berechnet die gefahrene Strecke nach dt.
        Für negative dt ist der Vektor umgedreht.
        Verwendet vel und tmpTask, die erst im ersten Update nach dem 
        Einstellen des neuen Tasks auf den aktuellen Wert gesetzt werden.
        '''
        lg.debug("vel = %s, tmpTask = %s"%(np.around(self.vel, decimals = 4), self.tmpTask))
        if self.tmpTask[2] == 0:
            dpos = np.zeros(3)
            dpos[:-1] = self.vel*dt
            return dpos
        elif self.tmpTask[0] == 0 and self.tmpTask[1] == 0:
            dpos = np.zeros(3)
            dpos[2] = self.vel[0]*dt
            return dpos
        else:
            #Unnötig kompliziert
            beta = np.deg2rad(self.vel[4])
            #lg.debug("sgn= %d, %d"%(np.sign(dt),np.sign(self.tmpTask[2])))
            beta2 = beta + np.sign(self.tmpTask[2]) * (self.vel[0]*dt/self.vel[1])
            #lg.debug("beta=%.4f, beta2=%.4f"%(beta,beta2))
            c1,c2 = np.cos([beta,beta2])
            s1,s2 = np.sin([beta,beta2])
            #Vektor zu Mitte(beta) + Vektor von Mitte(beta2)
            self.vel[4] = np.rad2deg(beta2)
            return [(c1-c2)*self.vel[2] + (s2-s1)*self.vel[3],
                    (s1-s2)*self.vel[2] + (c1-c2)*self.vel[3], np.rad2deg(beta2-beta)]
        
    def status(self, printing='info'):
        #Konsole
        if printing == 'hud':
            return "[%.2f,%.2f,%.2f°] @ %s"%(self.pos[0], self.pos[1], (self.pos[2]+180)%360-180, np.around(self.vel,decimals=2))
        stat = "----------\npos = [%.3f,%.3f,%.3f°]\nvel = %s\nMode = %s,  changing = %s,  finished = %s,  timer= %.2f,  ende= %.2f\nerrpos = %s\nAktueller Task     - %s\nNächster Task  - %s"%\
              (self.pos[0], self.pos[1], (self.pos[2]+180)%360-180, np.around(self.vel,decimals=4), self.mode, self.change, self.finished, self.timer, self.ende, np.around(self.errpos,decimals=4), self.curTask, self.nxtTask)
        
        if printing == 'debug':
            lg.debug(stat)
        else:
            lg.info(stat)
            
    
    def move(self, task, nphi=None):
        '''
        Ausführung der Grundbewegungen.
        |Gerade                        falls dphi=0
        |Drehung     mit Winkel alpha  falls dx=dy=0
        |Kreisegment        -"-        sonst
        
        task = [dx, dy, dalpha, dt]
        nphi - Ausrichtung zum Startzeitpunkt, kann von phi=pos[2] abweichen, da vorgerechnet wird
        
        return: [v1, v2, v3, v4] Umd/s der Motoren
        '''
        if nphi is None:
            nphi = self.pos[2]
        lg.debug('Compute Task: %s'%task)
        if task[2] == 0:
            self.compvel = task[0:2]/task[3]
            return self.trans(task[0:2], task[3])
        elif task[0] == 0 and task[1] == 0:
            self.compvel = [task[2]/task[3]]
            return self.rot(task[2],task[3])
        else:
            return self.circle(task[0],task[1],task[2],task[3],nphi = nphi)
            
    def circle(self, dx,dy,alpha,dt, nphi):
        '''
        dx, dy - Abstand zum nächsten Punkt
        phi    - aktuelle Ausrichtung des Roboters
        alpha  - zu drehender Winkel in Grad  < 90°
        dt     - Zeit für das Drehen
        nphi   - Ausrichtung zum Startzeitpunkt, kann von phi=pos[2] abweichen, da vorgerechnet wird
        '''
        dreh = self.rot(alpha,dt)                        #Rotation
        
        sgn = np.sign(alpha)
        alpha = np.abs(alpha)
        [alpha,phi] = np.deg2rad([alpha,nphi])    #Bogenmaß
        v = np.sqrt(dx**2+dy**2)
        r = v/(2*np.sin(alpha/2))
        b = alpha * r 
        c = np.sign(dx)                          #links oder rechts?
        rho = (c*np.arccos(dy/v) + sgn*alpha/2 + phi)
        tx = np.sin(rho)*b
        ty = np.cos(rho)*b
        lg.debug("r=%.4f, b=%.4f, rho[%%]=%.4f, rho[°]=%.4f"%(r,b,rho/(2*np.pi),np.rad2deg(rho)))
        lg.debug("t=[%.4f,%.4f]"%(tx,ty))
        
        [mx,my] = r*sgn*np.array([-ty,tx])/(np.sqrt(ty**2+tx**2))   #relativ
        c = np.cos(phi)
        s = np.sin(phi)
        mxn = c*mx-s*my
        myn = s*mx+c*my                                             #absolut
        self.compvel = np.array([b/dt,r,mxn,myn,0])
        
        bew = self.trans([tx,ty], dt)
        v = dreh + bew
        lg.debug("rot=%s, tran=%s"%(np.around(dreh, decimals=4),np.around(bew, decimals=4)))
        lg.debug("compvel = %s"%self.compvel)
        return v
       
        
    def set_motion(self, v, dt):
        if self.testtime is not None:
            lg.info("delta in Zeit: %.4f vs %.4f"%(time.time() - self.testtime, self.testdt))
        '''
        set the power of the motors
        
        v = [v1,v2,v3,v4] in RPS
        '''
        lg.info('Moving %s for %.2f s'%(v,dt))
        self.testtime = time.time()
        self.testdt = dt
        
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
    
    def setSpeed(self, speed):
        '''
        Setzt Faktor mit dem die Zeit vergeht
        | <1 - Verlangsamen
        | >1 - Beschleunigen
        '''
        
        self.speedmult = speed
    