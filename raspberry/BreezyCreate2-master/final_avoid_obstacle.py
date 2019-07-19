# -*- coding: utf-8 -*-
"""
Created on Fri Jul 19 16:49:00 2019

Adapted from original code by Nikolai K.
"""
#Import Libraries:
import sys
import time                #used to keep track of time
import math

from breezycreate2 import Robot
import time

import serial
import re

eliaquinose = False

ser = serial.Serial('/dev/ttyUSB1', 9600)
#ser.write(b'5') #Prefixo b necessario se estiver utilizando Python 3.X

re_compiled = re.compile('[0-9]+')
def read_sonar():
        S = [None,None,None,None]
        while (S[0] == None or S[1] == None or S[2] == None or S[3] == None):
                V_SERIAL = ser.readline()
                valores = re_compiled.findall(str(V_SERIAL))
                S[int(valores[0])] = int(valores[1])
        return S

time.sleep(2)
print('INICIANDO')

#Pre-Allocation
PI=math.pi  #pi=3.14..., constant

# Create a Create2. This will automatically try to connect to your robot over serial
#bot = Robot("COM8")
bot = Robot()

# Play a note to let us know you're alive!
bot.playNote('A4', 100)

bot.setForwardSpeed(-50) 
time.sleep(0.1)

# 0 = -75, 1 = -30, 3 = 30, 2 = 75
#orientation of all the sensors: 
# sensor_loc=np.array([60*PI/180, 15*PI/180, -15*PI/180, -75*PI/180])
sensor_loc=np.array([-75*PI/180, -15*PI/180, 15*PI/180, 60*PI/180]) 

t = time.time()

while (time.time()-t)<60:
    #Loop Execution
    sensor_val=read_sonar()
    if eliaquinose:
        print(sensor_val)
        
    min_ind=sensor_val.index(min(sensor_val))
    if sensor_val[min_ind]< (5*5):
        steer=-1/sensor_loc[min_ind]
    else:
        steer=0

    v=1	#forward velocity
    kp=0.5	#steering gain
    vl=v+kp*steer
    vr=v-kp*steer

    if eliaquinose:
        print("V_l =",vl)
        print("V_r =",vr)

    velo = -200/3.0
    vl = vl * velo
    vr = vr * velo
    bot.setWheelPWM(vr, vl)

    time.sleep(0.1) #loop executes once every 0.2 seconds (= 5 Hz)