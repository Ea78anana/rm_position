import olympe
import cvxpy as cp
import numpy as np
import scipy
import time
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, moveTo, Circle, PCMD

ANAFI_IP = "192.168.42.1"
SPHINX_IP = "10.202.0.1"

def MPC():
        m = 1
        T = 1

        # System model
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
        u_min = -0.14 * np.ones((m, 1))
        u_max = 0.14 * np.ones((m, 1))
        z_min = np.matmul(np.kron(np.ones((N, 1)), np.eye(m)), u_min)
        z_max = np.matmul(np.kron(np.ones((N, 1)), np.eye(m)), u_max)

        # Refrence value
        xref = np.array([
            [0.0],
            [0.0],
            [10],
        ])
        # Initial value
        x0 = np.array([
            [10.0],
            [30.0],
            [5.0],
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
        u_opt_a = [0, 0, 0]

        count = 0
        flag = True

        while flag :
            obj = (1/2) * cp.quad_form(z, H)+ cp.matmul(cp.matmul(x0.T, F.T), z)

            # Define and solve the optimization problem
            prob = cp.Problem(cp.Minimize(obj), constraints)
            result = prob.solve(warm_start=True)
            u_opt = z[0:m].value
            u_traj.append(u_opt)
            x0 = A.dot(x0) + B.dot(u_opt)
            x_traj.append(x0)
            for i in range (3):
                u_opt_a[i] = int(u_opt[i] // 0.14)
            print(u_opt_a)
            start_time = time.time()
                pass_time = 0
                if pass_time < 1: 
                    self.drone(PCMD(1, u_opt_a[0], u_opt_a[1], 0, u_opt_a[2], 0))
                    cur_time = time.time()
                    pass_time = cur_time - start_time
            
            if count < 5:
                count += 1
                #print(count)
            else:
                drone(PCMD(0, 0, 0, 0, 0, 0))
                break
            
            if (x0[0] - xref[0] <= 1e-1) and (x0[1] - xref[1] <= 1e-1) and (x0[2] - xref[2] <= 1e-1):
                flag = False
                drone(PCMD(0, 0, 0, 0, 0, 0))
            else:
                flag = True
                #print(x0)

def takeoff():
    drone.connect()
    drone(TakeOff()).wait().success()
    time.sleep(5)

    MPC()


    drone(Landing()).wait().success()
    drone.disconnect()

if __name__ == "__main__":
    drone = olympe.Drone(SPHINX_IP)
    takeoff()