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

def cost_weights(ctrl_type):
    if ctrl_type == 'ctrl_surf':
        print('ctrl_surf not implemented for skywalker')
        sys.exit()
    elif ctrl_type == 'body_rate':
        # z = [xd_r, yd_r, zd_r, x_r, y_r, z_r, th1, th2, th3]
        # u = [T, th1d, th2d, th3d]
        Wz = np.diag([0.01, 0.01, 0.01, 10, 10, 10, 0.1, 0.1, 0.1])
        Wu = np.diag([10, 100, 100, 100])
    else:
        print('Control type %s not known.' % ctrl_type)
        sys.exit()
    
    return Wz, Wu

def reducedOrderRiccati(A, B, C, H, Wz, Wu):
    """
    Compute controller gains by solving Riccati equations
    assuming the model is continuous time.
    """
    R = np.matmul(Wu.T, Wu)
    Qz = np.matmul(Wz, H)
    Qz = np.matmul(Qz.T, Qz) + .000001*np.eye(A.shape[0])
    
    # Increase Qw to make measurements have more influence
    Qw = 10*np.eye(A.shape[0])

    # Solve Riccati equations
    X = solve_continuous_are(A, B, Qz, R)
    Y = solve_continuous_are(A.T, C.T, Qw, np.eye(C.shape[0]))

    # Compute controller gains
    K = -np.matmul(np.linalg.inv(R), np.matmul(B.T, X))
    L = np.matmul(Y, C.T)

    return K, L

if __name__ == '__main__':
    if len(sys.argv) <= 1 or sys.argv[1] not in ['sgf', 'slf', 'stf']:
        print('Correct usage: python rompc_synth.py {sgf, slf, stf}')
        sys.exit()

    if sys.argv[1] in ['sgf', 'stf']:
        print('SGF and STF not yet supported')
        sys.exit()

    # Generate save directory if it doesn't exist
    root = dirname(dirname(dirname(dirname(abspath(__file__)))))
    path = join(root, "src/rompc/skywalker")
    savepath = join(path, sys.argv[1])
    if not isdir(savepath):
        mkdir(savepath)

    # Load matrices
    A = np.loadtxt(join(savepath, "A.csv"), delimiter=",")
    B = np.loadtxt(join(savepath, "B.csv"), delimiter=",")
    C = np.loadtxt(join(savepath, "C.csv"), delimiter=",")
    H = np.loadtxt(join(savepath, "H.csv"), delimiter=",")

    # Compute controller gains
    Wz, Wu = cost_weights('body_rate')
    K, L = reducedOrderRiccati(A, B, C, H, Wz, Wu)
    np.savetxt(join(savepath, "K.csv"), K, delimiter=",")
    np.savetxt(join(savepath, "L.csv"), L, delimiter=",")

    # Append path to load the models
    sys.path.append(join(root, "models/skywalker"))
    from sysid import aircraft_ctrl_params

    # Get additional aircraft parameters
    p = aircraft_ctrl_params()
    np.savetxt(join(savepath, "ctrl_params.csv"), p, delimiter=",");
