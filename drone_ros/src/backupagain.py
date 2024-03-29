#!/usr/bin/env python

# Author: Tony Sik Chiu Chow
# Description: Python code for auto pathing control ANAFI by MPC 

import rospy
from geometry_msgs.msg import PoseStamped

import cvxpy as cp
import numpy as np
import scipy
import olympe
import olympe.media
import olympe_deps as od
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
from olympe.enums.ardrone3.GPSSettings import HomeType_Type
from olympe.messages.ardrone3.PilotingState import moveToChanged, FlyingStateChanged, PositionChanged, AttitudeChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged, ResetHomeChanged
from olympe.messages.ardrone3.GPSSettings import HomeType, SetHome, ResetHome
from olympe.messages.ardrone3.GPSState import HomeTypeChosenChanged
from olympe.messages.ardrone3.PilotingState import GpsLocationChanged, AltitudeChanged
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, moveTo, Circle, PCMD
import os
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET
import time

#current = PoseStamped()

ANAFI_IP = "192.168.42.1"
ANAFI1_IP = "192.168.44.1"
ANAFI2_IP = "192.168.45.1"
SPHINX_IP = "10.202.0.1"
check_flag = False
arrive = False
call = True
data = 0

class TakeOff_Class:

    def __init__(self):
        self.DRONE_IP = os.environ.get(SPHINX_IP)
        self.drone = olympe.Drone(ANAFI1_IP)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.gaz = 0
        self.current_pos = np.array([[0.],[0.],[0.]])
        self.count = 0

    def mpc_controll(self, pos):
            global check_flag   
            m = 1
            T = 1
            
            A = np.array([
                [1, 0, 0, T, 0, 0],
                [0, 1, 0, 0, T, 0],
                [0, 0, 1, 0, 0, T],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]]

                )
            B = np.array([
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [1/m, 0, 0],
                [0, 1/m, 0],
                [0, 0, -4/(14 * m)]
            ])

            # System parameters
            n = A.shape[0]  # number of states
            m = B.shape[1]  # number of inputs
            N = 10  # prediction horizon
            if (self.count == 0 or self.count == 20):
                Q = np.eye(n)  # weight matrix Q
            else :
                Q = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0]])
            R = np.eye(m)  # weight matrix R
            P = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]])  # weight matrix P  

            # Condense
            Ma = A
            M = A
            for i in range(1, N):
                Ma = np.matmul(Ma, A)
                M = np.concatenate((M, Ma), axis=0)  # Matrix M
            for j in range(1, N + 1):
                for k in range(1, N + 1):
                    if k <= j:
                        V_cell = np.matmul(np.linalg.matrix_power(A, j - k), B)
                    else:
                        V_cell = np.zeros((n, m))
                    if k == 1:
                        V_x = V_cell
                    else:
                        V_x = np.concatenate((V_x, V_cell), axis=1)
                if j == 1:
                    V = V_x
                else:
                    V = np.concatenate((V, V_x), axis=0)  # Matrix V
            for vale in range(1, N):
                if vale == 1:
                    E = Q
                else:
                    E = scipy.linalg.block_diag(E, Q)
            E = scipy.linalg.block_diag(E, P)  # Matrix E
            for val in range(1, N + 1):
                if val == 1:
                    T = R
                else:
                    T = scipy.linalg.block_diag(T, R)  # Matrix T
            H = 2 * (T + np.matmul(np.matmul(V.T, E), V))  # Matrix H
            F = 2 * np.matmul(np.matmul(V.T, E.T), M)  # Matrix F

            # Input constraints
            u_min = -2.1 * np.ones((m, 1))
            u_max = 2.1 * np.ones((m, 1))
            z_min = np.matmul(np.kron(np.ones((N, 1)), np.eye(m)), u_min)
            z_max = np.matmul(np.kron(np.ones((N, 1)), np.eye(m)), u_max)

            #Refrence Value
            xref = np.array([
                [0.97, 0.47, 0.13, -0.03, -0.07, -0.07, -0.07, 0.42, 0.53, 0.78, 0.81, 1.07, 1.1, 1, 0.8, 0.5, 0.2, 0, 0, 0],
                [-1.55, -1.38, -1.38, -1.34, -0.9, -0.42, 0.16, 0.53, 0.68, 0.882, 1.2, 1.5, 1.8, 2, 1.8, 1.6, 1.4, 1.2, 1, 0],
                [1.09, 1.2, 1.4, 1.4, 1.5, 1.6, 1.8, 2, 1.8, 1.6, 1.4, 1.4, 1.2, 1.09, 1.09, 1.09, 1.09, 1.09, 1.09, 1.09],
            ])

            # Initial value
            x0 = np.array([
                [0.],
                [0.],
                [0.],
                [0.],
                [0.],
                [0.]
            ])

            for i in range(3):
                x0[i] = pos[i] - xref[i, self.count] 

            # Define optimization variables
            z = cp.Variable((N * m, 1))

            # Define constraints
            constraints = [z_min <= z, z <= z_max]
            u_opt_a = []
            u_opt = []
            flag = True

            while flag :
                obj = (1/2) * cp.quad_form(z, H)+ cp.matmul(cp.matmul(x0.T, F.T), z)

                # Define and solve the optimization problem
                prob = cp.Problem(cp.Minimize(obj), constraints)
                prob.solve(warm_start=True)
                u_opt = z[0:m].value
                x0 = A.dot(x0) + B.dot(u_opt)
                #print(x0)
                for i in range (3):
                    u_opt[i] = int(u_opt[i] // 0.14)
                u_opt_a.extend(u_opt)
                
                if (np.linalg.norm(x0) <= 1e-2):
                    flag = False
                    self.roll = int(u_opt_a[0])
                    self.pitch = int(u_opt_a[1])
                    self.gaz = - int(u_opt_a[2])
                    print(self.roll)
                    print(self.pitch)
                    print(self.gaz)
                else:
                    flag = True
                    #print(x0)'''
    
    def position_callback(self, msg):
        #print("position_callback")
        global check_flag
        if check_flag == False:
            check_flag = True
            self.MPC(msg)

    def move(self):
        pass_time = 0
        start_time = time.time()
        while pass_time < 2:
            self.drone(PCMD(1, self.roll, self.pitch, self.yaw, self.gaz, 0))
            pass_time = time.time() - start_time
        print(pass_time)
        #self.drone(PCMD(0, 0, 0, 0, 0, 0))

    def MPC(self, data):
        #print("MPC")
        global arrive, call, check_flag
        
        xref = np.array([[0],
                        [0],
                        [1.09]])

        self.current_pos[0] = round(data.pose.position.x,2)
        self.current_pos[1] = round(data.pose.position.y,2)
        self.current_pos[2] = round(data.pose.position.z,2)
        print(self.current_pos)
        call = False
        #(abs(self.current_pos[2] - xref[2]) <= 1e-1)
        
        if  (abs(self.current_pos[0]) - xref[0] <= 1e-2) and (abs(self.current_pos[1]) - xref[1] <= 1e-2) and self.count == 19:
            arrive = True
            print("arrive")
            return
        else:
            #print("else")
            self.mpc_controll(self.current_pos)
            self.move()
            if self.count < 19:
                self.count += 1
            else:
                pass
            #print("done")
            call = True
            check_flag = False

    def disconnection(self):
        self.drone(Landing()).wait().success()
        self.drone.disconnect()

    def connection(self):
        self.drone.connect()
        self.drone(TakeOff()).wait().success()

    def listener(self):
        global data
        #sub = rospy.Subscriber('/natnet_ros/Drone1/pose', PoseStamped, self.position_callback)
        data = rospy.wait_for_message('/natnet_ros/Drone1/pose', PoseStamped,)
        #data = rospy.wait_for_message('/natnet_ros/Drone2/pose', PoseStamped,)
        #print(data)
        #rospy.spin()

if __name__ == "__main__":
    rospy.init_node('drone_takeoff_node', anonymous=False)
    rospy.loginfo("Takeoff Initiated!")
    takeoff = TakeOff_Class()
    takeoff.connection()
    
    time.sleep(5)
    
    while arrive != True:
        if call == True:
            takeoff.listener()
            takeoff.position_callback(data)
        time.sleep(1)
    '''   
    for i in range(7):
        if call == True:
            takeoff.listener()
            takeoff.position_callback(data)
        time.sleep(2)
    '''
    #takeoff.move()    
    #time.sleep(5)
    #while check_flag == False:
    takeoff.disconnection()
    print("Landed")
    #time.sleep(2.5)
    #takeoff.test_takeoff()


'''
xref = np.array([
                [0.97, 0.47, -0.03, -0.07, -0.07, -0.09, 0.42, 0.81,  0.78, 1.07, 1.1, 1.1],
                [-1.55, -1.38, -1.34, -0.9, -0.42, 0.16, 0.53, 0.882, 1.53, 1.83, 2.19, 2.19],
                [1.09, 1.2, 1.4, 1.5, 1.6, 1.8, 2, 1.8, 1.6, 1.4, 1.2, 1.09],
            ])

xref = np.array([
                [0.97, 0.47, 0.13, -0.03, -0.07, -0.07, -0.07, 0.42, 0.53, 0.78, 0.81, 1.07, 1.1, 1, 0.8, 0.5, 0.2, 0, 0, 0],
                [-1.55, -1.38, -1.38, -1.34, -0.9, -0.42, 0.16, 0.53, 0.68, 0.882, 1.2, 1.5, 1.8, 2, 1.8, 1.6, 1.4, 1.2, 1, 0],
                [1.09, 1.2, 1.4, 1.4, 1.5, 1.6, 1.8, 2, 1.8, 1.6, 1.4, 1.4, 1.2, 1.09, 1.09, 1.09, 1.09, 1.09, 1.09, 1.09],
            ])'''
        

'''self.error.extend([round(math.sqrt((abs(self.current_pos[0]) - abs(self.xref[0][self.count]))**2 + (abs(self.current_pos[1]) - abs(self.xref[1][self.count]))**2), 2)]) '''