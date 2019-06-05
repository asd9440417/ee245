#!/usr/bin/env python

import math
import numpy as np
from crazyflieParser import CrazyflieParser

if __name__ == '__main__':

    index = 1   # for cf1
    initialPosition = [0,0,0] # x,y,z coordinate for this crazyflie
    cfs = CrazyflieParser(index, initialPosition)
    cf = cfs.crazyflies[0]
    time = cfs.timeHelper

    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2) # Use EKF
    cf.setParam("stabilizer/controller", 2) # Use mellinger controller
    cf.setParam("ring/effect", 7)

    cf.takeoff(targetHeight = 0.5, duration = 3.0)
    time.sleep(3.0)

    # FILL IN YOUR CODE HERE
    # Please try both goTo and cmdPosition
    # =============================================================================
    
    choose = str(input('line for 1, circle for 2, 2D 8 flying for 3, 3D 8 flying for 4:'))

    if choose=='1':
        waypoint = [[1,0,0], [0, 1, 0], [0, 0, 1], [-1, -1, 0], [0, 0, -1]]
        for i in waypoint:
            cf.goTo(goal=i,yaw=0,duration=3,relative=True)
            time.sleep(3)
        
        cf.land(targetHeight = 0.0, duration = 5.0)
        time.sleep(5.0)
##############################################################################
    if choose=='2':
        cf.goTo(goal=[1,0,0,0.5],yaw=0,duration=3.0,relative=True)
        time.sleep(3.0)
        for i in range(0,51):
            x=np.cos(2*(np.pi)/50*i)
            y=np.sin(2*(np.pi)/50*i)
            cf.cmdPosition(pos=[x,y,0.5],yaw=0)
            time.sleep(0.1)
            
        for i in range(0,10):
            
            cf.cmdPosition(pos=[0.1*(10-i-1),0,0.5],yaw=0)
            time.sleep(0.1)
            
        cf.land(targetHeight = 0.0, duration = 5.0)
        time.sleep(5.0)
#############################################################################33    
    if choose=='3':
        cf.goTo(goal=[1.2,0,0,0.5],yaw=0,duration=3.0,relative=True)
        time.sleep(3.0)
        for i in range(0,51):
            x=1.2*np.cos(2*(np.pi)/50*i)
            y=np.sin(2*(np.pi)/50*i)
            cf.cmdPosition(pos=[x,y,0.5],yaw=0)
            time.sleep(0.1)
        for i in range(0,51):
            x=1.2*np.cos(2*np.pi-((2*np.pi)/50*i+np.pi))
            y=np.sin(2*np.pi-((2*np.pi)/50*i+np.pi))
            cf.cmdPosition(pos=[x+2.4,y,0.5],yaw=0)
            time.sleep(0.1)

        for i in range(0,10):
            cf.cmdPosition(pos=[1.2/10*(10-i-1),0,0.5],yaw=0)
            time.sleep(0.1)
        cf.land(targetHeight = 0.0, duration = 5.0)
        time.sleep(5.0)
        
# ============================================================================
    if choose=='4':
        la=np.pi/4
        laz=[]
        for i in range(0,51):
            laz.append(2*np.pi/50*i)
        cf.goTo(goal=[np.cos(la),0,0],yaw=0,duration=3.0,relative=True)    
        time.sleep(3.0)
            
        for i in range(0,51):
            x=np.cos(2*(np.pi)/50*i)*np.cos(la)
            y=np.sin(2*(np.pi)/50*i)*np.sin(la)
            z=0.5+0.3*np.sin(laz[i])
            cf.cmdPosition(pos=[x,y,z],yaw=0)
            time.sleep(0.1)
        laz2=[]
        for i in range(0,51):
            laz2.append(np.pi-(2*np.pi/50*i))
        for i in range(0,51):
            x=np.cos(2*np.pi-((2*np.pi)/50*i+np.pi))*np.cos(la)
            y=np.sin(2*np.pi-((2*np.pi)/50*i+np.pi))*np.sin(la)
            z=0.5+0.3*np.sin(laz2[i])
            cf.cmdPosition(pos=[x+2*np.cos(la),y,z],yaw=0)
            time.sleep(0.1)
        for i in range(0,10):
            cf.cmdPosition(pos=[(np.cos(la)/10)*(10-i-1),0,0.5],yaw=0)
            time.sleep(0.1)
        cf.land(targetHeight = 0.0, duration = 5.0)
        time.sleep(5.0)
#%%
#for i in range(0,100):
#    a=np.cos((2*(np.pi)/100)*i)
#    b=np.sin((2*(np.pi)/100)*i)