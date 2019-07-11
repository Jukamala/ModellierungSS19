import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D

#Kreis in 20 Sekunden ohne Drehung mit r = 10
def f(t):
    return [20*np.sin(0.1*np.pi*t),10*np.cos(0.1*np.pi*t),0*t]

def diskretisieren(f,t,file_=None,min_refresh_rate=0.2, mode='NOBRAKE'):
    '''
    Diskretisieren von f d.h. ist eine Funktion f gegeben, wird der Zeitabschnitt t in 
    |f Funktion
    |t Zeit bis zu der diskretisiert werden soll
    |file_ Datei in die Skript geschrieben werden soll
    |refresh_rate Minimale Zeit zwischen zewie Aktualisierungen
    return: Skript, das f abfährt.
    '''
    #beste Refreshrate berechnen
    k=np.floor(t/min_refresh_rate)
    refresh_rate = t/k
    doc='MODE %s\n'%mode
    cur=f(0)
    for i in range(1,int(k)+1):
        cur_=f(i*refresh_rate)
        tp=[cur_[0]-cur[0],cur_[1]-cur[1],cur_[2]-cur[2]]
        cur=cur_
        doc+='MOVE '+str(np.around(tp[0],4))+' '+str(np.around(tp[1],4))+' '+str(np.around(tp[2],4))+' '+str(np.around(refresh_rate,4))+'\n'
    
    if(file_):
        text_file = open(file_, "w")
        text_file.write(doc)
        text_file.close()
    return doc

if __name__ == '__main__':
    #Beisipeilplot für Kreis
    print(diskretisieren(f,40,file_="demo3.seq",min_refresh_rate=0.7))
    
    x = np.linspace(0,40)
    y = f(x)
    #Komponenten
    fig, ax = plt.subplots(3,1, sharex=True)
    fig.canvas.set_window_title("Komponentenfunktionen")
    ax[0].plot(x,y[0])
    ax[0].set_ylabel(r'$x(t)$')
    ax[1].plot(x,y[1])
    ax[1].set_ylabel(r'$x(t)$')
    ax[2].plot(x,y[2])
    ax[2].set_ylabel(r'$\varphi(t)$')
    ax[2].set_xlabel(r"$t$ in $s$")
    #Funktion
    fig = plt.figure()
    fig.canvas.set_window_title("Funktion")
    ax = fig.gca(projection='3d')
    ax.plot(y[0],y[1],y[2])
    #Startpunkt
    ax.plot([y[0][0]], [y[1][0]], [y[2][0]], "r*")
    ax.set_xlabel(r'$x(t)$')
    ax.set_ylabel(r'$y(t)$')
    ax.set_zlabel(r'$\varphi(t)$')
    
