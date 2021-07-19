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
        # z = [xd_r, yd_r, zd_r, th1d, th2d, th3d, x_r, y_r, z_r, th1, th2, th3, a, e, r]
        # u = [T, ad, ed, rd]
        # y = z
        Wz = np.diag([0.01, 0.01, 0.01, 100, 100, 100, 10, 10, 10, 0.1, 0.1, 0.1, 1, 1, 1])
        Wu = np.diag([10, 1, 1, 1])
        Wy = np.diag([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.00001, 0.00001, 0.00001])
    elif ctrl_type == 'body_rate':
        # z = [xd_r, yd_r, zd_r, x_r, y_r, z_r, th1, th2, th3]
        # u = [T, th1d, th2d, th3d]
        # y = z
        Wz = np.diag([0.01, 0.01, 0.01, 10, 10, 10, 0.1, 0.1, 0.1])
        Wu = np.diag([10, 200, 200, 200])
        Wz = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    else:
        print('Control type %s not known.' % ctrl_type)
        sys.exit()
    
    return Wz, Wu, Wy

def reducedOrderRiccati(A, B, C, H, Wz, Wu, Wy):
    """
    Compute controller gains by solving Riccati equations
    assuming the model is continuous time.
    """
    R = np.matmul(Wu.T, Wu)
    Qz = np.matmul(Wz, H)
    Qz = np.matmul(Qz.T, Qz) + .000001*np.eye(A.shape[0])
    Qy = np.matmul(Wy.T, Wy)
    
    # Increase Qw to make measurements have more influence
    Qw = 100*np.eye(A.shape[0])

    # Solve Riccati equations
    X = solve_continuous_are(A, B, Qz, R)
    Y = solve_continuous_are(A.T, C.T, Qw, Qy)

    # Compute controller gains
    K = -np.matmul(np.linalg.inv(R), np.matmul(B.T, X))
    L = np.matmul(Y, C.T)

    return K, L

if __name__ == '__main__':
    if len(sys.argv) <= 2 or sys.argv[1] not in ['sgf', 'slf', 'stf'] \
            or sys.argv[2] not in ['ctrl_surf', 'body_rate']:
        print('Correct usage: python rompc_synth.py {sgf, slf, stf} {ctrl_surf, body_rate}')
        sys.exit()

    if sys.argv[1] in ['stf']:
        print('STF not yet supported')
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
    if C.shape[0] == 15 and sys.argv[2] == 'ctrl_surf':
        print('This does appear to be a ctrl_surf model')
    elif C.shape[0] == 9 and sys.argv[2] == 'body_rate':
        print('This does appear to be a body_rate model')
    else:
        print('Verify model is actually %s' % sys.argv[2])
        sys.exit()

    # Compute controller gains
    print('Saving controller to %s' % savepath)
    Wz, Wu, Wy = cost_weights(sys.argv[2])
    K, L = reducedOrderRiccati(A, B, C, H, Wz, Wu, Wy)
    np.savetxt(join(savepath, "K.csv"), K, delimiter=",")
    np.savetxt(join(savepath, "L.csv"), L, delimiter=",")

    # Append path to load the models
    sys.path.append(join(root, "models/skywalker"))
    from sysid import aircraft_ctrl_params

    # Get additional aircraft parameters
    p = aircraft_ctrl_params()
    np.savetxt(join(savepath, "ctrl_params.csv"), p, delimiter=",");
