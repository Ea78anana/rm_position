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
import matplotlib.pyplot as plt 
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
import math

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
        self.drone = olympe.Drone(ANAFI_IP)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.gaz = 0
        self.current_pos = np.array([[0.],[0.],[0.],[0.]])
        self.count = 0
        self.path_x = []
        self.path_y = []
        self.path_error_x = []
        self.path_error_y =[]
        self.vx = []
        self.vy = []
        self.error = []
        
        self.xref = np.array([
                [0.8, 0.78, 0.75, 0.69, 0.61, 0.51, 0.4, 0.27, 0.14, 0, -0.14, -0.27, -0.4, -0.51, -0.61, -0.69, -0.75, -0.78, -0.8, -0.78, -0.75, -0.69, -0.61, -0.51, -0.4, -0.27, -0.14, 0, 0.14, 0.27, 0.4, 0.51, 0.61, 0.69, 0.75, 0.78],
                [0, 0.14, 0.27, 0.4, 0.51, 0.61, 0.69, 0.75, 0.78, 0.8, 0.78, 0.75, 0.69, 0.61, 0.51, 0.4, 0.27, 0.14, 0, -0.14, -0.27, -0.4, -0.51, -0.61, -0.69, -0.75, -0.78, -0.8, -0.78, -0.75, -0.69, -0.61, -0.51, -0.4, -0.27, -0.14],
                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
            ])
        
        # self.xref = np.array([
        #     [0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #     [0.3, 0.6, 0.85, 1.1, 1.25, 1.4, 1.625, 1.85, 1.9, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95],
        #     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        # ])
        
        self.ref_path_x = self.xref[0]
        self.ref_path_y = self.xref[1]

    def mpc_controll(self, pos):
            global check_flag   
            m = 0.7
            T = 0.6
            pos = np.array(pos)
            D = np.array([
                [math.cos(pos[3] * 180 * math.pi / 180), math.sin(pos[3] * 180 * math.pi / 180), 0],
                [-math.sin(pos[3] * 180 * math.pi / 180), math.cos(pos[3] * 180 * math.pi / 180), 0],
                [0, 0, 1]]
            )

            A = np.array([
                [1, 0, 0, T, 0, 0],
                [0, 1, 0, 0, T, 0],
                [0, 0, 1, 0, 0, T],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]])

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
            if (self.count == 0 or self.count == 36):
                Q = 1000 * np.array([ # weight matrix Q
                [3, 0, 0, 0, 0, 0],
                [0, 3, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]])  
            else :
                Q = 1000 * np.array([
                [3, 0, 0, 0, 0, 0],
                [0, 3, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0]])
            R = np.eye(m)  # weight matrix R
            P = 1000 * np.array([
                [3, 0, 0, 0, 0, 0],
                [0, 3, 0, 0, 0, 0],
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
            u_min = -2.8 * np.ones((m, 1))
            u_max = 2.8 * np.ones((m, 1))
            z_min = np.matmul(np.kron(np.ones((N, 1)), np.eye(m)), u_min)
            z_max = np.matmul(np.kron(np.ones((N, 1)), np.eye(m)), u_max)

            #Refrence Value
            '''
            self.xref = np.array([
                [0.8, 0.8, 0.8, 0.8, 0.8,  0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8],
                [0.3, 0.6, 0.85, 1.1, 1.25, 1.4, 1.625, 1.85, 1.9, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95],
                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
            ])
            '''

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
                x0[i] = pos[i] - self.xref[i][self.count] 

            # Define optimization variables
            z = cp.Variable((N * m, 1))

            # Define constraints
            constraints = [z_min <= z, z <= z_max]
            u_opt_a = []
            u_opt = []
            v = np.array([
                [0.],
                [0.],
                [0.]])
            flag = True

            while flag :
                obj = (1/2) * cp.quad_form(z, H)+ cp.matmul(cp.matmul(x0.T, F.T), z)

                # Define and solve the optimization problem
                prob = cp.Problem(cp.Minimize(obj), constraints)
                prob.solve(warm_start=True)
                u_opt = z[0:m].value
                x0 = A.dot(x0) + B.dot(u_opt)
                for i in range(3):
                    v[i] = x0[i + 3]
                #v0 = np.matmul(D, v)

                for i in range (3):
                    u_opt[i] = int(round(v[i][0] / 0.14))
                u_opt_a.extend(u_opt)
                
                if (np.linalg.norm(x0) <= 1e-2):
                    flag = False
                    self.roll = int(u_opt_a[0])
                    self.pitch = int(u_opt_a[1])
                    self.gaz = int(u_opt_a[2])
                    self.vx.extend(u_opt_a[0])
                    self.vy.extend(u_opt_a[1])
                    #print(self.roll)
                    #print(self.pitch)
                    #print(self.gaz)
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
        while pass_time < 0.4:
            self.drone(PCMD(1, self.roll, self.pitch, self.yaw, self.gaz, 0))
            pass_time = time.time() - start_time
        print(pass_time)
        #self.drone(PCMD(0, 0, 0, 0, 0, 0))

    def MPC(self, update):
        #print("MPC")
        global arrive, call, check_flag, data

        self.current_pos[0] = round(update.pose.position.x,2)
        self.current_pos[1] = round(update.pose.position.y,2)
        self.current_pos[2] = round(update.pose.position.z,2)
        self.current_pos[3] = round(update.pose.orientation.w,2)

        self.path_x.extend(self.current_pos[0])
        self.path_y.extend(self.current_pos[1])
        print(self.current_pos)
        call = False
        #(abs(self.current_pos[2] - self.xref[2]) <= 1e-1)
        
        if ((abs(self.current_pos[0]) - abs(self.xref[0][self.count]) <= 1e-1) and (abs(self.current_pos[1]) - abs(self.xref[1][self.count]) <= 1e-1) and self.count == 35):
            arrive = True
            print("arrive")
            self.listener()
            self.current_pos[0] = round(data.pose.position.x,2)
            self.current_pos[1] = round(data.pose.position.y,2)
            self.current_pos[2] = round(data.pose.position.z,2)
            self.current_pos[3] = round(data.pose.orientation.w,2)
            # self.path_error_x.extend(self.current_pos[0])
            # self.path_error_y.extend(self.current_pos[1])
            self.error.extend([round(math.sqrt((abs(self.current_pos[0]) - abs(self.xref[0][self.count]))**2 + (abs(self.current_pos[1]) - abs(self.xref[1][self.count]))**2), 2)]) 
            return
        else:
        #print("else")
            self.mpc_controll(self.current_pos)
            self.move()
            print(self.count)
            self.listener()
            self.current_pos[0] = round(data.pose.position.x,2)
            self.current_pos[1] = round(data.pose.position.y,2)
            self.current_pos[2] = round(data.pose.position.z,2)
            self.current_pos[3] = round(data.pose.orientation.w,2)
            self.error.extend([round(math.sqrt((abs(self.current_pos[0]) - abs(self.xref[0][self.count]))**2 + (abs(self.current_pos[1]) - abs(self.xref[1][self.count]))**2), 2)]) 
            if (abs(self.current_pos[0]) - abs(self.xref[0][self.count]) <= 1e-1) and (abs(self.current_pos[1]) - abs(self.xref[1][self.count]) <= 1e-1):
                # self.path_error_x.extend(self.current_pos[0])
                # self.path_error_y.extend(self.current_pos[1])
                # self.error.extend([round(math.sqrt((abs(self.current_pos[0]) - abs(self.xref[0][self.count]))**2 + (abs(self.current_pos[1]) - abs(self.xref[1][self.count]))**2), 2)]) 
                if self.count < 35:
                    self.count += 1
                else:
                    pass
            else:
                print(abs(self.current_pos[0]) - abs(self.xref[0][self.count]))
                print(abs(self.current_pos[1]) - abs(self.xref[1][self.count]))
                pass
        #print("done")
        call = True
        check_flag = False

    def plot_graph(self):
        fig, ax = plt.subplots(2,2)

        ax[0,0].plot(self.ref_path_x, self.ref_path_y, 'bo-', label = 'Path1')
        ax[0,0].plot(self.path_x, self.path_y, 'g^-', label = 'Path2')

        ax[0,0].set_xlim([-2, 2])
        ax[0,0].set_ylim([-2, 2])
        ax[0,0].set_xlabel('X position')
        ax[0,0].set_ylabel('Y position')

        ax[0,0].legend()

        #error = [math.sqrt((self.ref_path_x[i] - self.path_error_x[i])**2 + (self.ref_path_y[i] - self.path_error_y[i])**2 ) for i in range(15)]

        ax[0,1].plot(self.error)
        ax[0,1].set_xlabel('Iteration')
        ax[0,1].set_ylabel('Error')
        ax[0,1].legend()

        ax[1,0].plot(self.vx, 'y*-')
        ax[1,0].set_xlabel('Time')
        ax[1,0].set_ylabel('Vx')
        ax[1,0].legend()

        ax[1,1].plot(self.vy, 'ro-')
        ax[1,1].set_xlabel('Time')
        ax[1,1].set_ylabel('Vy')
        ax[1,1].legend()

        plt.show()

    def disconnection(self):
        self.drone(Landing()).wait().success()
        #np.savetxt('/home/ehsan/Desktop/drone.txt', np.array(self.path_x + self.path_y))
        self.drone.disconnect()

    def connection(self):
        self.drone.connect()
        self.drone(TakeOff()).wait().success()

    def listener(self):
        global data
        data = rospy.wait_for_message('/natnet_ros/Drone1/pose', PoseStamped,)
        #data = rospy.wait_for_message('/natnet_ros/Drone2/pose', PoseStamped,)


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
        time.sleep(0.8)
    '''   
    for i in range(36):
        if call == True:
            takeoff.listener()
            takeoff.position_callback(data)
        time.sleep(1)
    '''
    takeoff.disconnection()
    takeoff.plot_graph()
    print("Landed")



'''
self.xref = np.array([
                [0.97, 0.47, -0.03, -0.07, -0.07, -0.09, 0.42, 0.81,  0.78, 1.07, 1.1, 1.1],
                [-1.55, -1.38, -1.34, -0.9, -0.42, 0.16, 0.53, 0.882, 1.53, 1.83, 2.19, 2.19],
                [1.09, 1.2, 1.4, 1.5, 1.6, 1.8, 2, 1.8, 1.6, 1.4, 1.2, 1.09],
            ])
self.xref = np.array([
                [0.97, 0.47, 0.13, -0.03, -0.07, -0.07, -0.07, 0.42, 0.53, 0.78, 0.81, 1.07, 1.1, 1, 0.8, 0.5, 0.2, 0, 0, 0],
                [-1.55, -1.38, -1.38, -1.34, -0.9, -0.42, 0.16, 0.53, 0.68, 0.882, 1.2, 1.5, 1.8, 2, 1.8, 1.6, 1.4, 1.2, 1, 0],
                [1.09, 1.2, 1.4, 1.4, 1.5, 1.6, 1.8, 2, 1.8, 1.6, 1.4, 1.4, 1.2, 1.09, 1.09, 1.09, 1.09, 1.09, 1.09, 1.09],
            ])
self.xref = np.array([
                [0.8, 0.8, 0.8, 0.8, 0.8,  0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8],
                [0.3, 0.6, 0.85, 1.1, 1.25, 1.4, 1.625, 1.85, 1.9, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95],
                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
            ])'''