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

def to_standard_QP(Ad, Bd, H, P, Q, R, U, N, Z=None):
    """
    Convert problem:

    min xN^T P xN + sum_j=0^N-1 xj^T Q xj + uj^T R uj
    s.t. x_k+1 = Ad x_k + Bd u_k
         u_k in U
         Hx_k in Z
         xN = 0

    into the dense standard form:

    min 0.5 * U^T F U + x_0^T G U
    s.t. E1 U <= e + E2 x_0

    where U = [u_0, ..., u_N-1]^T and x_0 is the initial
    condition parameter.

    Note that if H and Z are none then no terminal constraint
    is needed but P is needed. If H and Z are present then
    the terminal constraint guarantees recursive feasibility
    without having to compute Xf (challenging). In this case
    P ends up not being needed anyways.
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

    # Cost function J = U^T F U + 2 x0^T G U
    F = np.matmul(Su.T, np.matmul(Qf, Su)) + Rf
    G = np.matmul(Sx.T, np.matmul(Qf, Su))

    # Control constraints
    E_u = block_diag(*[U.A]*N)
    ub_u = np.array([U.b]*N).flatten()

    # Performance and terminal constraints
    if Z is not None:
        E_z = block_diag(*[np.matmul(Z.A, H)]*(N-1))
        ub_z = np.array([Z.b]*(N-1)).flatten()
        E_z = block_diag(E_z, np.eye(n))
        ub_z = np.hstack((ub_z, np.zeros(n)))

        # Constraints E1 U < e + E2 x_0
        E1 = np.vstack((E_u, np.matmul(E_z, Su)))
        e = np.hstack((ub_u, ub_z))
        E2 = np.vstack((np.zeros((ub_u.shape[0], n)), 
                       -np.matmul(E_z, Sx)))

    else:
        # Constraints E1 U < e + E2 x_0
        E1 = E_u
        e = ub_u
        E2 = np.zeros((ub_u.shape[0], n))

    return F, G, E1, e, E2


if __name__ == '__main__':

    if len(sys.argv) <= 3 or sys.argv[1] not in ['sgf', 'slf', 'stf'] \
        or sys.argv[2] not in ['gazebo', 'skywalker'] \
        or sys.argv[3] not in ['ctrl_surf', 'body_rate']:
        print('Correct usage: python mpc_synth.py {sgf, slf, stf} {gazebo, skywalker} {ctrl_surf, body_rate}')
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
    dt = 0.1
    Ad, Bd = zoh(dt, A, B)

    # Compute cost matrices
    sys.path.append(join(path, sys.argv[2]))
    from rompc_synth import cost_weights
    Wz, Wu = cost_weights(sys.argv[3])
    Q, R, P = cost(Ad, Bd, H, Wz, Wu)

    # Control constraints
    Z = None
    if sys.argv[2] == 'skywalker' and sys.argv[3] == 'body_rate':

        # Control u = [T, th1d, th2d, th3d] in [N, rad/s, rad/s, rad/s]
        Tref = u_eq[0]
        uUB = np.array([10.0, np.deg2rad(100), np.deg2rad(100), np.deg2rad(100)])
        uLB = np.array([-Tref, -np.deg2rad(100), -np.deg2rad(100), -np.deg2rad(100)])
        U = HyperRectangle(uUB, uLB)

        # Performance z = [dxdot, dydot, dzdot, dx, dy, dz, dthx, dthy, dthz]
        zUB = np.array([20, 20, 20, 100, 100, 100, np.deg2rad(45), np.deg2rad(45), np.deg2rad(45)])
        zLB = -zUB
        Z = HyperRectangle(zUB, zLB)

    elif sys.argv[2] == 'skywalker' and sys.argv[3] == 'body_rate':
        
        # Control u = [T, ad, ed, rd] in [N, rad/s, rad/s, rad/s]
        Tref = u_eq[0]
        uUB = np.array([10.0, np.deg2rad(1000), np.deg2rad(1000), np.deg2rad(1000)])
        uLB = np.array([-Tref, -np.deg2rad(1000), -np.deg2rad(1000), -np.deg2rad(1000)])
        U = HyperRectangle(uUB, uLB)

        # # Performance z = [pos_rate, rot_rate, pos, rot, ctrl_surf_def]
    else:
        print('Control constraints are not defined for this input set')
        sys.exit()

    # Define matrices
    N = 50
    F, G, E1, e, E2 = to_standard_QP(Ad, Bd, H, P, Q, R, U, N, Z=Z)

    print('Defining OCP with N = %d and dt = %.2f' % (N, dt))
    params = np.array([N, dt])

    np.savetxt(join(savepath, "F.csv"), F, delimiter=",")
    np.savetxt(join(savepath, "G.csv"), G, delimiter=",")
    np.savetxt(join(savepath, "E1.csv"), E1, delimiter=",")
    np.savetxt(join(savepath, "e.csv"), e, delimiter=",")
    np.savetxt(join(savepath, "E2.csv"), E2, delimiter=",")
    np.savetxt(join(savepath, "ocp_params.csv"), params, delimiter=",")
