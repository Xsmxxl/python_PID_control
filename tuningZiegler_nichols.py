# -*- coding: utf-8 -*-
"""
Created on Mon Jul 27 21:56:50 2020

@author: Santiago D. Salas, PhD sdsalas@espol.edu.ec
"""

## Ecuaciones diferenciales con ODE int
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

# condición inicial
y0 = 0 #m

# función de la altura dinámica
def tank(h,t):
    dhdt = F0/A - beta/A*np.sqrt(h) # m/min
    return dhdt

# time points
t = np.linspace(0,100,101)
F0 = 1 # m**3/min
A = 10 # m**2
beta = 2
# solve ODE
# y = odeint(tank,y0,t)

ns = len(t)

########################################################
ysp = 0.5
hs = ysp
taup = 2*A*np.sqrt(hs)/beta
kp = 2*np.sqrt(hs)/beta
td = 1 # min

def fun1(wco):
    b = np.arctan(-taup*wco) + (-td*wco) + np.pi # phi = -pi
    return b
wco = fsolve(fun1,np.pi)
Pu = 2*np.pi/float(wco)

def fun2(kcu):
    a = kp*kcu/np.sqrt(taup**2 * wco**2 + 1) - 1 # AR = 1
    return a
kcu = fsolve(fun2,20)

print("kcu: ",kcu, "Pu: ", Pu)
epp_0 = 0
ep_0 = 0
ep = 0
# kc = 20
# tauI = 1E6
# tauD = 0

# kcu = 20
# Pu = 2

### Ziegler - Nichols PI
kc = 0.45*kcu
tauI = Pu/1.2
tauD = 0

# ### Ziegler - Nichols PID
# kc = 0.6*kcu
# tauI = Pu/2
# tauD = Pu/8

# ### Tyreus - Luyben PI
# kc = 0.31*kcu
# tauI = Pu*2.2
# tauD = 0

# ### Tyreus - Luyben PID
# kc = 0.45*kcu
# tauI = Pu*2.2
# tauD = Pu/6.3

F0_ctrl = 0

yd0 = y0
ydf = []
F0_ctrl_t = []
F0_ctrl_t.append(float(0))
ydf.append(float(y0))

for i in range(0,ns-1):

    ts = [t[i],t[i+1]]
    F0 = F0_ctrl
    
    yd = odeint(tank,yd0,ts)
    yd0 = yd[-1]
    ydf.append(float(yd0))
    
    ## PID
    delta_t = t[i+1]-t[i]
    
    epp_0 = ep_0
    ep_0 = ep
    ep = ysp - yd0
    delta_u = kc*((ep-ep_0) + ep/float(tauI)*delta_t + tauD/float(delta_t)*(ep-2*ep_0+epp_0))
    F0_ctrl = F0_ctrl + delta_u
    if F0_ctrl < 0: F0_ctrl = 0
    F0_ctrl_t.append(float(F0_ctrl))

t_sp = [0,t[-1]]
y_sp = [ysp,ysp]

# plot results
plt.plot(t_sp,y_sp,t,ydf)
plt.xlabel('time, min')
plt.ylabel('h(t), m')
plt.show()

plt.plot(t,F0_ctrl_t)
plt.xlabel('time, min')
plt.ylabel('F(t), m3/min')
plt.show()