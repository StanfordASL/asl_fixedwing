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
    
    # Increase Qw to make measurements have more influence
    Qw = 1*np.eye(A.shape[0])

    # Solve Riccati equations
    X = solve_continuous_are(A, B, Qz, R)
    Y = solve_continuous_are(A.T, C.T, Qw, np.eye(C.shape[0]))

    # Compute controller gains
    K = -np.matmul(np.linalg.inv(R), np.matmul(B.T, X))
    L = np.matmul(Y, C.T)

    return K, L

if __name__ == '__main__':
    if len(sys.argv) <= 2 or sys.argv[1] not in ['sgf', 'slf', 'stf'] \
            or sys.argv[2] not in ['ctrl_surf', 'body_rate']:
        print('Correct usage: python rompc_synth.py {sgf, slf, stf} {ctrl_surf, body_rate}')
        sys.exit()

    # Generate save directory if it doesn't exist
    root = dirname(dirname(dirname(dirname(abspath(__file__)))))
    path = join(root, "src/rompc/gazebo")
    savepath = join(path, sys.argv[1])
    if not isdir(savepath):
        mkdir(savepath)

    # Append path to load the models
    sys.path.append(join(root, "models/gazebo"))
    from sysid import aircraft_ctrl_params
    from model import linearized_aircraft_sgf, linearized_aircraft_stf, simplify_control
    
    # Compute system dynamics and equilibrium
    S = 13 # m/s
    if sys.argv[1] == 'sgf':
        gamma = np.deg2rad(-3.5) # flight path angle
        A, B, C, H, x_eq, u_eq = linearized_aircraft_sgf(S, gamma)
        th = x_eq[7] # pitch angle [rad]
        target = np.array([S, gamma, th])
    elif sys.argv[1] == 'slf':
        gamma = np.deg2rad(0.0)
        A, B, C, H, x_eq, u_eq = linearized_aircraft_sgf(S, gamma)
        th = x_eq[7] # pitch angle [rad]
        target = np.array([S, th])
    elif sys.argv[1] == 'stf':
        R = 50 # m
        A, B, C, H, x_eq, u_eq = linearized_aircraft_stf(S, R)
        u, v, w = x_eq[0:3]
        p, q, r = x_eq[3:6]
        phi, th = x_eq[6:8]
        target = np.array([u, v, w, p, q, r, phi, th, R])
        
    if sys.argv[2] == 'body_rate':
        # Simplify controls to be aircraft body rates
        A, B, C, H, x_eq, u_eq = simplify_control(A, B, x_eq, u_eq)

        # z = [u, v, w, phi, th, psi, x_r, y_r, z_r]
        # u = [T, p, q, r]
        Wz = np.diag([1, 1, 1, np.deg2rad(1), np.deg2rad(1), np.deg2rad(1), 
                      1, 1, 1])
        Wu = np.diag([0.5, 100, 100, 100])

    elif sys.argv[2] == 'ctrl_surf':
        # z = [u, v, w, p, q, r, phi, th, psi, x_r, y_r, z_r]
        # u = [T, a, e, r]
        Wz = np.diag([1, 1, 1, np.deg2rad(5), np.deg2rad(5), np.deg2rad(5), 
                      np.deg2rad(1), np.deg2rad(1), np.deg2rad(1), 1, 1, 1])
        Wu = np.diag([0.5, np.deg2rad(1), np.deg2rad(1), np.deg2rad(1)])

    # Compute controller gains
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

    # Save parameters for defining the target
    np.savetxt(join(savepath, "target.csv"), target, delimiter=",")

    # Get additional aircraft parameters
    p = aircraft_ctrl_params()
    np.savetxt(join(savepath, "ctrl_params.csv"), p, delimiter=",");
