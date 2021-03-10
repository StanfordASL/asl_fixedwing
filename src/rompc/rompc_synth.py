"""
    @file rompc_synth.py
    Computes required components for ROMPC controller
    given the linear ROM matrices A, B, C, H
"""
import sys
from os.path import dirname, abspath, join, isdir
from os import mkdir
import numpy as np
from scipy.linalg import solve_continuous_are

def reducedOrderRiccati(A, B, C, H, Wz, Wu):
    """
    Compute controller gains by solving Riccati equations
    assuming the model is continuous time.
    """
    
    R = np.matmul(Wu.T, Wu)
    Qz = np.matmul(Wz, H)
    Qz = np.matmul(Qz.T, Qz) + .001*np.eye(A.shape[0])
    Qw = .0001*np.eye(A.shape[0])

    # Solve Riccati equations
    X = solve_continuous_are(A, B, Qz, R)
    Y = solve_continuous_are(A.T, C.T, Qw, np.eye(C.shape[0]))

    # Compute controller gains
    K = -np.matmul(np.linalg.inv(R), np.matmul(B.T, X))
    L = np.matmul(Y, C.T)

    return K, L

if __name__ == '__main__':
    if len(sys.argv) <= 1 or sys.argv[1] not in ['gazebo','skywalker']:
        print('Correct usage: python controller.py {gazebo, skywalker}')
        sys.exit()

    # Generate save directory if it doesn't exist
    path = dirname(abspath(__file__))
    savepath = join(path, sys.argv[1])
    if not isdir(savepath):
        mkdir(savepath)

    # Append path to load the models   
    root = dirname(dirname(path))
    sys.path.append(join(root, "models/" + sys.argv[1]))
    from model import linearized_aircraft_slf

    if sys.argv[1] == 'gazebo':
        S = 25
        psi = 0.0
        A, B, C, H, x_eq, u_eq = linearized_aircraft_slf(S, psi)

    elif sys.argv[1] == 'skywalker':
        print('Skywalker model not implemented')
        sys.exit()

    # Compute controller gains
    # z = [p, q, r, ex, ey, ez]
    # u = [T, al, ar, e, r]
    Wz = np.diag([1, 1, 1, 1, 1, 1])
    Wu = np.diag([0.5, 1, 1, 1, 1])
    K, L = reducedOrderRiccati(A, B, C, H, Wz, Wu)

    # Save files to be loaded by ROMPC controller
    np.savetxt(join(savepath, "A.csv"), A, delimiter=",")
    np.savetxt(join(savepath, "B.csv"), B, delimiter=",")
    np.savetxt(join(savepath, "C.csv"), C, delimiter=",")
    np.savetxt(join(savepath, "H.csv"), H, delimiter=",")
    np.savetxt(join(savepath, "K.csv"), K, delimiter=",")
    np.savetxt(join(savepath, "L.csv"), L, delimiter=",")
    np.savetxt(join(savepath, "x_eq.csv"), x_eq, delimiter=",")
    np.savetxt(join(savepath, "u_eq.csv"), u_eq, delimiter=",")
