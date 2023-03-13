
import cvxpy as cp
import numpy as np
import scipy
import matplotlib.pyplot as plt

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
u_opt_a = []

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
      u_opt_a = u_opt[i] // 0.14
  print(u_opt_a)
  
  if count < 500:
      count += 1
      #print(count)
  else:
      break
  
  if np.linalg.norm(x0) <= 1e-2 :
    flag = False
  else:
    flag = True
    #print(x0)


# Plot results
t = np.arange(0, len(u_traj))
u_traj = np.array(u_traj).reshape(-1, m)
x_traj = np.array(x_traj).reshape(-1, n)

fig, axs = plt.subplots(2, 1, figsize=(8, 8))

# Plot state trajectories
axs[0].plot(x_traj[:, 0], label='x1')
axs[0].plot(x_traj[:, 1], label='x2')
axs[0].plot(x_traj[:, 2], label='x3')
axs[0].plot(x_traj[:, 3], label='x4')
axs[0].plot(x_traj[:, 4], label='x5')
axs[0].plot(x_traj[:, 5], label='x6')
axs[0].set_ylabel('State')
axs[0].set_xlabel('Iteration')
axs[0].legend()

# Plot input trajectories
axs[1].plot(t, u_traj[:, 0], label='u1')
#axs[1].plot(t, u_traj[:, 1], label='u2')
axs[1].set_ylabel('Input')
axs[1].set_xlabel('Iteration')
axs[1].legend()

plt.show()

'''
[
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0]]
'''
'''[
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1]]'''