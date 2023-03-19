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
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
import os
import time

ANAFI_IP = "192.168.42.1"
ANAFI1_IP = "192.168.44.1"
ANAFI2_IP = "192.168.45.1"
SPHINX_IP = "10.202.0.1"
check_flag = False
arrive = False
call = True
data_drone1 = 0
data_drone2 = 0

class TakeOff_Class:

    def __init__(self):
        self.DRONE_IP = os.environ.get(SPHINX_IP)
        self.drone1 = olympe.Drone(ANAFI1_IP)
        self.roll1 = 0
        self.pitch1 = 0
        self.yaw1 = 0
        self.gaz1 = 0
        self.current_drone1_pos = np.array([[0.],[0.],[0.]])
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
            if (self.count == 0 or self.count == 30):
                Q = 1000 * np.eye(n)  # weight matrix Q
            else :
                Q = 1000 * np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0]])
            R = np.eye(m)  # weight matrix R
            P = 1000 * np.array([
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
            u_min = -14 * np.ones((m, 1))
            u_max = 14 * np.ones((m, 1))
            z_min = np.matmul(np.kron(np.ones((N, 1)), np.eye(m)), u_min)
            z_max = np.matmul(np.kron(np.ones((N, 1)), np.eye(m)), u_max)

            # Refrence value
            xref1 = np.array([
                [0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9],
                [0.3, 0.3, 0.6, 0.6, 0.85,0.85, 1.1, 1.1, 1.25, 1.25, 1.4, 1.4, 1.625, 1.625, 1.85, 1.85, 1.9, 1.9, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95],
                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            ])

            xref2 = np.array([
                [-0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4],
                [0.3, 0.3, 0.6, 0.6, 0.85,0.85, 1.1, 1.1, 1.25, 1.25, 1.4, 1.4, 1.625, 1.625, 1.85, 1.85, 1.9, 1.9, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95, 1.95],
                [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
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
                x0[i] = pos[i] - xref1[i, self.count]

            # Define optimization variables
            z = cp.Variable((N * m, 1))

            # Define constraints
            constraints = [z_min <= z, z <= z_max]
            u_opt = []
            u_opt_out = []
            flag = True
            while flag :
                obj = (1/2) * cp.quad_form(z, H)+ cp.matmul(cp.matmul(x0.T, F.T), z)

                # Define and solve the optimization problem
                prob = cp.Problem(cp.Minimize(obj), constraints)
                prob.solve(warm_start=True)
                u_opt = z[0:m].value
                x0 = A.dot(x0) + B.dot(u_opt)
                for i in range (3):
                    u_opt[i] = int(u_opt[i] // 0.14)
                u_opt_out.extend(u_opt)
                    
                if (np.linalg.norm(x0) <= 1e-2):
                    flag = False
                    self.roll1 = int(u_opt_out[0])
                    self.pitch1 = int(u_opt_out[1])
                    self.gaz1 = - int(u_opt_out[2])
                    print(self.roll1)
                    print(self.pitch1)
                    print(self.gaz1)
                else:
                    flag = True
    
    def position_callback(self, msg1): #call the function to convert the position and call the MPC to calculate the control input and put into the move function
        global arrive, call, check_flag
        
        xref1 = np.array([[0.8],
                        [1.85],
                        [1]])
        
        self.current_drone1_pos[0] = round(msg1.pose.position.x,2)
        self.current_drone1_pos[1] = round(msg1.pose.position.y,2)
        self.current_drone1_pos[2] = round(msg1.pose.position.z,2)
        print(self.current_drone1_pos)
        call = False
        
        if  (abs(self.current_drone1_pos[0]) - xref1[0] <= 1e-2) and (abs(self.current_drone1_pos[1]) - xref1[1] <= 1e-2) and self.count == 29:
            arrive = True
            print("arrive")
            return
        else:
            self.mpc_controll(self.current_drone1_pos)
            self.move()
            if self.count < 29:
                self.count += 1
            else:
                pass
            call = True


    def move(self):  #Move function for the drones
        pass_time = 0
        start_time = time.time()
        while pass_time < 0.5:
            self.drone1(PCMD(1, self.roll1, self.pitch1, self.yaw1, self.gaz1, 0))
            pass_time = time.time() - start_time
        print(pass_time)
        

    def disconnection(self):
        self.drone1(Landing()).wait().success()
        self.drone1.disconnect()

    def connection(self):
        self.drone1.connect()
        self.drone1(TakeOff()).wait().success()

    def listener(self): #Use for collecting the position of the drones
        global data_drone1, data_drone2
        data_drone1 = rospy.wait_for_message('/natnet_ros/Drone1/pose', PoseStamped,)
        #data_drone1 = rospy.wait_for_message('/natnet_ros/Drone2/pose', PoseStamped,)


if __name__ == "__main__":
    rospy.init_node('drone_takeoff_node', anonymous=False)
    rospy.loginfo("Takeoff Initiated!")
    takeoff = TakeOff_Class()
    takeoff.connection()
    time.sleep(5)
    for i in range(30):
        if call == True:
            takeoff.listener()
            takeoff.position_callback(data_drone1)
        time.sleep(1)   
    takeoff.disconnection()
    print("Landed")

