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

current = PoseStamped()

ANAFI_IP = "192.168.42.1"
SPHINX_IP = "10.202.0.1"

class TakeOff_Class:

    def __init__(self):
        self.DRONE_IP = os.environ.get(SPHINX_IP)
        self.drone = olympe.Drone(ANAFI_IP)
        self.roll = 0
        self.pich = 0
        self.yaw = 0
        self.gaz = 0

    def mpc_controll(self, x0):
        m = 1
        T = 0.1
        
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
        Q = np.eye(n)  # weight matrix Q
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
        u_min = -5 * np.ones((m, 1))
        u_max = 5 * np.ones((m, 1))
        z_min = np.matmul(np.kron(np.ones((N, 1)), np.eye(m)), u_min)
        z_max = np.matmul(np.kron(np.ones((N, 1)), np.eye(m)), u_max)

        # Initial value
        x0 = np.array([
            [1],
            [3],
            [-1],
            [0.0],
            [0.0],
            [0.0]
        ])
        z_0 = np.zeros((N * m, 1))

        # Define optimization variables
        z = cp.Variable((N * m, 1))

        # Define constraints
        constraints = [z_min <= z, z <= z_max]

        # Store data for plotting
        x_traj = [x0]
        u_traj = []

        count = 0
        flag = True

        while flag :
            obj = (1/2) * cp.quad_form(z, H)+ cp.matmul(cp.matmul(x0.T, F.T), z)

            # Define and solve the optimization problem
            prob = cp.Problem(cp.Minimize(obj), constraints)
            prob.solve(warm_start=True)
            u_opt = z[0:m].value
            u_traj.append(u_opt)
            x0 = A.dot(x0) + B.dot(u_opt)
            x_traj.append(x0)
            
            if count < 500:
                count += 1
            else:
                break
            
            if np.linalg.norm(x0) <= 1e-2 :
                flag = False
            else:
                flag = True
    
    def position_callback(self, msg):
        self.MPC(msg)

    def MPC(self, data):
        current.pose.position.x = round(data.pose.position.x,2)
        current.pose.position.y = round(data.pose.position.y,2)
        current.pose.position.z = round(data.pose.position.z,2)
        
        print(current.pose.position)
        '''
        if (current.pose.position.x != 0.06 and current.pose.position.y != -0.08 and current.pose.position.z != 0.39):
            self.drone(PCMD(1,0,10,0,0,0))
            #time.sleep(0.05)
        else:
            self.drone(PCMD(0,0,0,0,0,0))
        '''
        if (current.pose.position.y != -0.08):
            self.drone(PCMD(1,0,1,0,0,0))
            #time.sleep(0.05)
        else:
            self.drone(PCMD(0, 0, 0, 0, 0, 0))
            takeoff.disconnection()

    def disconnection(self):
        self.drone(Landing()).wait().success()
        self.drone.disconnect()

    def connection(self):
        self.drone.connect()
        self.drone(TakeOff()).wait().success()

    def listener(self):
        rospy.init_node('drone_takeoff_node', anonymous=False)
        rospy.loginfo("Takeoff Initiated!")
        sub = rospy.Subscriber('/natnet_ros/Drone1/pose', PoseStamped, takeoff.position_callback)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('drone_takeoff_node', anonymous=False)
    rospy.loginfo("Takeoff Initiated!")
    takeoff = TakeOff_Class()
    takeoff.connection()
    try :
        takeoff.listener()
        #time.sleep(0.05)
        #takeoff.test_takeoff()
    except rospy.ROSInterruptException:
            pass
