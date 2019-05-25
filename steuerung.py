import numpy as np
import time

#Erstelle eine Liste von Tasks - (COMMAND,args1,args2,..))
def readSequenz(filename):
    seq = []
    with open(filename) as file:
        for line in file:
            do = line.strip('\n').split(' ')
            seq.append(do)
    return seq
    
class Fahrzeug():
    '''
    vel       - Geschwindigkeit in cm/s
    x, y, phi - Postion in [cm,cm,Grad]
    
    maxVel, maxAcc - maximale Beschleunigung und Geschwindigkeit in [cm/s, cm/s^2]
    
    subs - Alle Fahrzeuge die noch untergeordnet sind
    seq  - Die auszuführende Sequenz
    mode - aktueller Ausführungsmodus
           SMOOTH - glatter Übergang
           NULL   - zwischen Tasks stehen bleiben
    finished - Sequenz fertig?
    change   - neuer Task wird von vorbereitet
    curTask = [dx, dy, dphi, dt] - aktueller Task
    deltaNext = [ddx,ddy,ddphi] - Unterschied zwischen dx,dy,dphi von aktuellem/nächsten Task
    
    '''
    def __init__(self, seq, maxVel, maxAcc, fahrzeuglist=None):
        self.maxVel = maxVel
        self.maxAcc = maxAcc
        self.x = 0
        self.y = 0
        self.phi = 0
        self.vel = 0
        self.finished = len(seq) == 0
        self.mode = 'SMOOTH'
        self.seq = seq
        self.curTask = [0,0,0,0]
        self.subs = fahrzeuglist
        self.lookahead()
        self.nextTask()
        
    #Beende den aktuellen Task und beginne Anpassung an neue dx,dy,dphi
    def nextTask(self):
        if not self.finished:
            self.curTask = np.array([float(x) for x in self.seq.pop(0)[1:5]])
            self.change = True
            self.finished = len(self.seq) == 0
        
    #Nachdem aktuelle v erreicht ist:
    def lookahead(self):
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
    
    #update with deltatime in s
    def update(self, dt):
        #Anpassung fertig
        if self.change:
            self.change = False
            self.lookahead()
        else:
            self.nextTask()
        
    def status(self, printing=False):
        #Konsole
        if printing:
            print("----------\npos = [%.2f,%.2f,%.2f°]\nvel = [%.2f]\nMode = %s,  changing = %s,  finished = %s\nAktueller Task - %s\nDelta zum nächsten - %s"%\
              (self.x, self.y, self.phi%360, self.vel, self.mode, self.change, self.finished, self.curTask, self.deltaNext))
        #Hud
        else:
            return "[%.2f,%.2f,%.2f°] @ %.2f"%(self.x, self.y, self.phi%360, self.vel)
        
        
if __name__ == '__main__':
    #init
    seq = readSequenz('cur.seq')
    print(seq)
    robot = Fahrzeug(seq, None, 200, 10)
    robot.status()
    last = time.time()
    while True:
        #vergangene Zeit
        now = time.time()
        dt = now - last
        last = now
        print("| dt = %f"%dt)
        
        robot.update(dt)
        robot.status(True)
        time.sleep(2)