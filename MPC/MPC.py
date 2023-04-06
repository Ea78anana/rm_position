#!/usr/bin/env python

# Author: Tony Sik Chiu Chow
# Description: Python code for auto pathing control ANAFI by MPC 

import cvxpy as cp
import numpy as np
import scipy
check_flg = True
count = 0

def mpc_controll(pos):
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
        if (count == 0 or count == 30):
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
            x0[i] = pos[i] - xref1[i, count]

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
                roll1 = int(u_opt_out[0])
                pitch1 = int(u_opt_out[1])
                gaz1 = - int(u_opt_out[2])
                print(roll1)
                print(pitch1)
                print(gaz1)
            else:
                flag = True

if __name__ == "__main__":
    mpc_controll()