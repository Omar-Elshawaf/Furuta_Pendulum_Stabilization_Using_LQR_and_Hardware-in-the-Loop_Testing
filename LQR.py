import numpy as np
from scipy.linalg import solve_continuous_are, eigvals

# Constants
g = 9.81

# 1 - Arm
# 2 - Pendulum
m1 = 0.03217
m2 = 0.041
L1 = 0.095
L2 = 0.08
M  = 0.01

J = 0.007022721987053
kb_p = 4.7940e-04
kb_m = 0.013983279899632
ke = 0.967774441589921
Re = 4.857292094830934

alpha = J + (M + m1/3 + m2) * L1**2
beta  = (M + m2/3) * L2**2
gamma = (M + m2/2) * L2 * L1
sigma = (M + m2/2) * g * L2

# Linearization
A = np.zeros((4, 4))
A[0, 1] = 1
A[1, 2] = -(sigma * gamma) / (alpha * beta - gamma**2)
A[2, 3] = 1
A[3, 2] = (alpha * sigma) / (alpha * beta - gamma**2)

B = np.zeros((4, 2))
B[1, 0] = beta  / (alpha * beta - gamma**2)
B[1, 1] = -gamma / (alpha * beta - gamma**2)
B[3, 0] = -gamma / (alpha * beta - gamma**2)
B[3, 1] = alpha / (alpha * beta - gamma**2)

C = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# Pseudo Linear System
Ap = np.zeros((4, 4))

Ap[0, 1] = 1

Ap[1, 1] = -B[1, 0] * (ke**2 / Re + kb_m)
Ap[1, 2] = A[1, 2]
Ap[1, 3] = -B[1, 1] * kb_p

Ap[2, 3] = 1

Ap[3, 1] = -B[3, 0] * (ke**2 / Re + kb_m)
Ap[3, 2] = A[3, 2]
Ap[3, 3] = -B[3, 1] * kb_p

Bp = np.zeros((4, 1))
Bp[1, 0] = B[1, 0] * ke / Re
Bp[3, 0] = B[3, 0] * ke / Re

# LQR weights
Q = np.array([
    [40, 0, 0, 0],
    [0, 400, 0, 0],
    [0, 0, 3000, 0],
    [0, 0, 0, 800]
])

R = np.array([[1.51]])

# Solve CARE (Continuous-time Riccati)
P = solve_continuous_are(Ap, Bp, Q, R)

# Compute gain K
K = np.linalg.inv(R) @ (Bp.T @ P)

# Closed-loop eigenvalues
E = eigvals(Ap - Bp @ K)

print("LQR Gain K:")
print(K)

print("\nClosed-loop eigenvalues E:")
print(E)
