# -*- coding: utf-8 -*-
"""
Created on Tue Jan 06 22:00:39 2015

@author: Nikolai K.
"""
#Import Libraries:
#import vrep                  #V-rep library
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math
#import matplotlib as mpl   #used for image plotting

from breezycreate2 import Robot
import time

import serial
import re

ser = serial.Serial('/dev/ttyUSB1', 9600)
#ser.write('5')
#ser.write(b'5') #Prefixo b necessario se estiver utilizando Python 3.X
#ser.read()

def read_sonar():
        S = [None,None,None,None]

        while (S[0] == None or S[1] == None or S[2] == None or S[3] == None):
                V_SERIAL = ser.readline()
                valores = re.findall('[0-9]+', str(V_SERIAL))
                S[int(valores[0])] = int(valores[1])

        return S

# while(True):

# #       VALUE_SERIAL = ser.readline()

# #       print(f'Retorno da porta serial: {VALUE_SERIAL}')

#         new = read_sonar()

        # print(new)


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

# vrep.simxFinish(-1) # just in case, close all opened connections

# clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

# if clientID!=-1:  #check if client connection successful
#     print('Connected to remote API server')
    
# else:
#     print('Connection not successful')
#     sys.exit('Could not connect')


#retrieve motor  handles
# errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
# errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)


sensor_h=[] #empty list for handles
sensor_val=np.array([]) #empty array for sensor measurements

#orientation of all the sensors: 
# sensor_loc=np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 
# sensor_loc=np.array([-PI/2, -PI/4, PI/4, PI/2) 
# 0 = -75, 1 = -30, 3 = 30, 2 = 75
 
sensor_loc=np.array([-75*PI/180, -30*PI/180, 30*PI/180, 75*PI/180]) 

#for loop to retrieve sensor arrays and initiate sensors
# for x in range(1,16+1):
#         errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(x),vrep.simx_opmode_oneshot_wait)
#         sensor_h.append(sensor_handle) #keep list of handles        
#         errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming)                
#         sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #get list of values
        

t = time.time()


while (time.time()-t)<60:
    #Loop Execution
    sensor_val=np.array([])
    sensor_val=read_sonar()
    print(sensor_val) 
    # for x in range(1,16+1):
        # errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h[x-1],vrep.simx_opmode_buffer)                
        # sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #get list of values
        # sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #get list of values
        # sensor_val=read_sonar()

    
    #controller specific
#    sensor_sq=sensor_val[0:4]*sensor_val[0:4] #square the values of front-facing sensors 1-8
    sensor_sq=np.array([])
        
    min_ind=np.where(sensor_val==np.min(sensor_val))
    print(min_ind[0])
    min_ind=min_ind[0][0]
#    print("HMM-> [0] ", sensor_sq[0])
    if sensor_val[min_ind]< (6*6):
        steer=-1/sensor_loc[min_ind]
        bot.setForwardSpeed(0)
        bot.setTurnSpeed(100*steer)
        time.sleep(0.1)
        bot.setForwardSpeed(-50)
    else:
        steer=0
            
    
    v=1	#forward velocity
    kp=0.5	#steering gain
    vl=v+kp*steer
    vr=v-kp*steer
    print("V_l =",vl)
    print("V_r =",vr)

    # errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,vl, vrep.simx_opmode_streaming)
    # errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,vr, vrep.simx_opmode_streaming)
    

    time.sleep(0.1) #loop executes once every 0.2 seconds (= 5 Hz)

#Post ALlocation
# errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
# errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
    

