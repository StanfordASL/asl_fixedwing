"""
    @file mpc_synth.py
    Computes required components for ROMPC optimal
    control problem
"""
import sys
from os.path import dirname, abspath, join, isdir
from os import mkdir
import numpy as np
from scipy.linalg import solve_discrete_are, expm, block_diag
import pdb

class Polyhedron(object):
    def __init__(self, A, b):
        self.A = A
        self.b = b

    def contains(self, x):
        """
        Returns true if x is contained in the Polyhedron
        """
        return False if np.max(np.matmul(self.A, x) - self.b) > 0 else True

class HyperRectangle(Polyhedron):
    def __init__(self, ub, lb):
        n = len(ub)
        A = np.block(np.kron(np.eye(n), np.array([[1], [-1]])))
        b = np.hstack([np.array([ub[i], -lb[i]]) for i in range(n)])
        super(HyperRectangle, self).__init__(A, b)

def zoh(dt, A, B):
    """
    Compute zero order hold discrete equivalent of continuous
    time system xd = Ax + Bu with sampling time dt seconds.
    """
    n, m = B.shape
    M = np.vstack((np.hstack((A, B)), np.zeros((m, n+m))))
    M = expm(dt*M)
    Ad = M[0:n, 0:n]
    Bd = M[0:n, n:]
    return Ad, Bd

def cost(Ad, Bd, H, Wz, Wu):
    """
    Compute controller gains by solving Riccati equations
    assuming the model is discrete time.
    """
    R = np.matmul(Wu.T, Wu)
    Qz = np.matmul(Wz, H)
    Q = np.matmul(Qz.T, Qz) + 0.00001*np.eye(Ad.shape[0])
    P = solve_discrete_are(Ad, Bd, Q, R)
    return Q, R, P

def to_standard_QP(Ad, Bd, P, Q, R, U, N):
    """
    Convert problem:

    min xn^T P xn + sum_j=0^N-1 xj^T Q xj + uj^T R uj
    s.t. x_k+1 = Ad x_k + Bd u_k
         u_k in U

    into the dense standard form:

    min 0.5 * U^T F U + x_0^T G U
    s.t. E U <= ub

    where U = [u_0, ..., u_N-1]^T and x_0 is the initial
    condition parameter.
    """
    n, m = Bd.shape
    Qrep = [Q]*(N-1)
    Qrep.append(P)
    Qf = block_diag(*Qrep)
    Rf = block_diag(*[R]*(N))
    Su = np.zeros((N*n, N*m))
    Sx = np.zeros((N*n, n))
    M = np.eye(n)
    for i in range(1,N+1):
        Mblk = block_diag(*[np.matmul(M, Bd)]*(N-i+1))
        Su[(i-1)*n:, :(N-i+1)*m] += Mblk
        M = np.matmul(M, Ad)

        Sx[(i-1)*n:i*n, :] = M

    F = np.matmul(Su.T, np.matmul(Qf, Su)) + Rf
    G = np.matmul(Sx.T, np.matmul(Qf, Su))

    E = block_diag(*[U.A]*N)
    ub = np.array([U.b]*N).flatten()
    return F, G, E, ub


if __name__ == '__main__':

    if len(sys.argv) <= 3 or sys.argv[1] not in ['sgf', 'slf', 'stf'] \
        or sys.argv[2] not in ['gazebo', 'skywalker'] \
        or sys.argv[3] not in ['ctrl_surf', 'body_rate']:
        print('Correct usage: python rompc_utils.py {sgf, slf, stf} {gazebo, skywalker} {ctrl_surf, body_rate}')
        sys.exit()
   
    # Generate save directory if it doesn't exist
    path = dirname(abspath(__file__))
    savepath = join(path, join(sys.argv[2], sys.argv[1]))
    if not isdir(savepath):
        mkdir(savepath)
        print(savepath)
        print('The save directory does not exist')
        sys.exit()

    # Load continuous time state matrices
    A = np.loadtxt(join(savepath, "A.csv"), delimiter=",")
    B = np.loadtxt(join(savepath, "B.csv"), delimiter=",")
    H = np.loadtxt(join(savepath, "H.csv"), delimiter=",")
    u_eq = np.loadtxt(join(savepath, "u_eq.csv"), delimiter=",")

    # Compute discrete time system
    dt = 0.05
    Ad, Bd = zoh(dt, A, B)

    # Compute cost matrices
    sys.path.append(join(path, sys.argv[2]))
    from rompc_synth import cost_weights
    Wz, Wu = cost_weights(sys.argv[3])
    Q, R, P = cost(Ad, Bd, H, Wz, Wu)

    # Control constraints
    if (sys.argv[2] == 'skywalker' or sys.argv[2] == 'gazebo') \
         and sys.argv[3] == 'body_rate':
        Tref = u_eq[0]
        uUB = np.array([10.0, np.deg2rad(20), np.deg2rad(20), np.deg2rad(20)])
        uLB = np.array([-Tref, -np.deg2rad(20), -np.deg2rad(20), -np.deg2rad(20)])
        U = HyperRectangle(uUB, uLB)
    else:
        print('Control constraints are not defined for this input set')
        sys.exit()

    # Define matrices
    N = 20
    F, G, E, ub = to_standard_QP(Ad, Bd, P, Q, R, U, N)

    print('Defining OCP with N = %d and dt = %.2f' % (N, dt))
    params = np.array([N, dt])

    np.savetxt(join(savepath, "F.csv"), F, delimiter=",")
    np.savetxt(join(savepath, "G.csv"), G, delimiter=",")
    np.savetxt(join(savepath, "E.csv"), E, delimiter=",")
    np.savetxt(join(savepath, "ub.csv"), ub, delimiter=",")
    np.savetxt(join(savepath, "params.csv"), params, delimiter=",")
