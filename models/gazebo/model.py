"""
    @file gazebo/model.py
    Defines a model of the gazebo simulator aircraft
    and provides functionality to linearize about a
    steady level flight condition.
"""

import numpy as np
from casadi import *

# Vehicle parameters
M = 1.5 # kg
IX = 0.19756 # kg m^2
IY = 0.14589 # kg m^2
IZ = 0.14770 # kg m^2
G = 9.8066 # m/s^2
RHO = 1.2041 # kg/m^3
D_MAX = 0.53 # rad, control surface deflection limits
T_MAX = 10 # N, max thrust

# Position vectors from CM to center of pressure of each
# element, written in body frame coordinates in meters
P_LW = np.array([-0.05, -0.3, -0.05])
P_RW = np.array([-0.05, 0.3, -0.05])
P_E = np.array([-0.5, 0.0, -0.05])
P_R = np.array([-0.5, 0.0, 0.0])

# Wing Aero
S_W = 0.12 # m^2, wing area
A0_W = -0.0598 # rad, zero lift AoA
CLA_W = 4.75280 # lift curve slope
CDA_W = 0.64171 # drag curve slope
CLD_W = -0.5 # lift coeff vs control surface deflection

# Elevator Aero
S_E = 0.01 # m^2
A0_E = 0.2
CLA_E = 4.75280
CDA_E = 0.64171
CLD_E = -4.0

# Rudder Aero
S_R = 0.02 # m^2
A0_R = 0.0
CLA_R = 4.75280
CDA_R = 0.64171
CLD_R = 4.0


def aerodynamics(x, u):
    """ Compute Body frame aerodynamic forces
        x = [u, v, w, p, q, r, phi, th, psi]
        u = [T, al, ar, e, r]
    """
    q = 0.5*RHO*(x[0]**2 + x[2]**2)
    a = atan(x[2]/x[0])

    # Wings
    D_lw = fabs(CDA_W*(a - A0_W))*q*S_W
    L_lw = (CLA_W*(a - A0_W) + CLD_W*u[1])*q*S_W
    F_lw = vertcat(-D_lw*cos(a) + L_lw*sin(a),
                    0.0,
                    -D_lw*sin(a) - L_lw*cos(a))
    D_rw = fabs(CDA_W*(a - A0_W))*q*S_W
    L_rw = (CLA_W*(a - A0_W) + CLD_W*u[2])*q*S_W
    F_rw = vertcat(-D_rw*cos(a) + L_rw*sin(a),
                    0.0,
                    -D_rw*sin(a) - L_rw*cos(a))

    # Elevator
    D_e = fabs(CDA_E*(a - A0_W))*q*S_E
    L_e = (CLA_E*(a - A0_W) + CLD_E*u[3])*q*S_E
    F_e = vertcat(-D_e*cos(a) + L_e*sin(a),
                    0.0,
                    -D_e*sin(a) - L_e*cos(a))

    # Rudder
    q_r = 0.5*RHO*(x[0]**2 + x[1]**2)
    a_r = atan(x[1]/x[0])
    D_r = fabs(CDA_R*(a_r - A0_R))*q_r*S_R
    L_r = (CLA_R*(a_r - A0_R) + CLD_R*u[4])*q_r*S_R
    F_r = vertcat(-D_r*cos(a_r) + L_r*sin(a_r),
                    -D_r*sin(a_r) - L_r*cos(a_r),
                    0.0)

    # Combined
    F = F_lw + F_rw + F_e + F_r
    M = cross(P_LW, F_lw) + cross(P_RW, F_rw) + \
            cross(P_E, F_e) + cross(P_R, F_r)
    return vertcat(F, M)

def aircraft(x, u):
    """ Aircraft dynamics ODE, computes xdot = f(x,u):
        x = [u, v, w, p, q, r, phi, th, psi]
        u = [T, al, ar, e, r],  

    u, v, w: velocity components in body frame coord
    p, q, r: body rates
    phi, th, psi: euler angles of body rel. to inertial
    T: thrust
    al, ar: aileron deflection angle, left and right wing
    e, r: elevator and rudder deflection angles 
    """
    f_aero = aerodynamics(x, u)
    return vertcat(
        (u[0] + f_aero[0])/M - G*sin(x[7]) + x[5]*x[1] - x[4]*x[2],
        f_aero[1]/M + G*cos(x[7])*sin(x[6]) - x[5]*x[0] + x[3]*x[2], 
        f_aero[2]/M + G*cos(x[7])*cos(x[6]) + x[4]*x[0] + x[3]*x[1], 
        (1.0/IX)*(f_aero[3] + (IY - IZ)*x[4]*x[5]),
        (1.0/IY)*(f_aero[4] + (IZ - IX)*x[3]*x[5]),
        (1.0/IZ)*(f_aero[5] + (IX - IY)*x[3]*x[4]),
        x[3] + tan(x[7])*(x[4]*sin(x[6]) + x[5]*cos(x[6])),
        x[4]*cos(x[6]) - x[5]*sin(x[7]),
        (1.0/cos(x[7]))*(x[4]*sin(x[6]) + x[5]*cos(x[6])),
        )

def equilibrium(S, heading):
    """ Compute an equilibrium state and control for the aircraft
    to fly with speed S in steady-level flight with constant altitude.
    Additionally, respecting control constraints and trying
    to minimize the control surface deflections. """

    # Create symbolic state and control vars
    uvw = MX.sym('uvw', 3)
    pqr = MX.sym('pqr', 3)
    phi = MX.sym('phi')
    th = MX.sym('th')
    psi = MX.sym('psi')
    u = MX.sym('u', 5)

    # Create constraints for the equilibrium condition
    x = vertcat(uvw, pqr, phi, th, psi)
    const = vertcat(aircraft(x, u)[0:6],              # equilibrium
                    norm_2(uvw) - S,                  # specified velocity
                    uvw[1],                           # no sideways motion
                    uvw[2]*cos(th) - uvw[0]*sin(th))  # constant altitude

    # Bounds on the decision variables u, v, w, th, u[0:5]
    lb = [-inf, -inf, -inf, -inf, 0, -D_MAX, -D_MAX, -D_MAX, -D_MAX]
    ub = [inf, inf, inf, inf, T_MAX, D_MAX, D_MAX, D_MAX, D_MAX]

    # Find a equilibrium point by solving a nonlinear
    # optimization problem by minimizing the control effort
    # at the solution
    f = (u[1]/D_MAX)**2 + (u[2]/D_MAX)**2 \
        + (u[3]/D_MAX)**2 + (u[4]/D_MAX)**2
    nlp = {'x':vertcat(uvw, th, u),
           'p':vertcat(pqr, phi, psi),
           'f':f,
           'g':const}
    opt = nlpsol('opt', 'ipopt', nlp)

    # Solve the NLP
    z_guess = np.zeros(9)
    z_guess[0] = S
    tol = .00001
    sol = opt(x0=z_guess, p=np.zeros(5), 
            lbg=-tol*np.ones(9), ubg=tol*np.ones(9),
            lbx=lb, ubx=ub)

    # Extract solution
    print('\n\nEquilibrium solution results:')
    sol = np.array(sol['x']).flatten()
    xsol = np.hstack((sol[0:3],    # uvw
                      np.zeros(3), # pqr
                      0.0,         # phi  
                      sol[3],      # th
                      heading))    # psi
    usol = sol[4:]
    print('speed S = {} m/s'.format(S))
    print('u = {:.2f} m/s, v = 0 m/s, w = {:.2f} m/s'.format(sol[0], sol[2]))
    print('p = 0 rad/s, q = 0 rad/s, r = 0 rad/s')
    print('phi = 0 deg, th = {:.2f} deg, psi = {:.2f} deg'.format(np.rad2deg(sol[3]), np.rad2deg(heading)))
    print('T = {:.2f} N'.format(sol[4]))
    print('d_lw = {:.2f} deg, d_rw = {:.2f} deg'.format(*np.rad2deg(sol[5:7])))
    print('d_e = {:.2f} deg, d_r = {:.2f} deg'.format(*np.rad2deg(sol[7:])))
    
    print('If control surface deflections are too large try increasing speed!')
    return xsol, usol

def aircraft_slf(x0, u0):
    """ Aircraft dynamics with additional navigation equations
    to include error from a steady level flight corresponding
    to the equilibrium (x0, u0). Returns a
    casadi function. New state for this model is
    x = [u, v, w, p, q, r, phi, th, psi, ex, ey, ez]

    Note that the heading psi can be changed arbitrarily without
    affecting the equilibrium.
    """
    x = MX.sym('x', 12)
    u = MX.sym('u', 5)

    # Define some useful rotation matrices
    R_I2R = np.array([[np.cos(x0[8])*np.cos(x0[7]), np.sin(x0[8])*np.cos(x0[7]), -np.sin(x0[7])],
                      [-np.sin(x0[8]), np.cos(x0[8]), 0.],
                      [np.cos(x0[8])*np.sin(x0[7]), np.sin(x0[8])*np.sin(x0[7]), np.cos(x0[7])]])
    

    R_00 = cos(x[8])*cos(x[7])
    R_01 = cos(x[8])*sin(x[7])*sin(x[6]) - cos(x[6])*sin(x[8])
    R_02 = sin(x[8])*sin(x[6]) + cos(x[8])*cos(x[6])*sin(x[7])
    R_10 = cos(x[7])*sin(x[8])
    R_11 = cos(x[8])*cos(x[6]) + sin(x[8])*sin(x[7])*sin(x[6])
    R_12 = cos(x[6])*sin(x[8])*sin(x[7]) - cos(x[8])*sin(x[6])
    R_20 = -sin(x[7])
    R_21 = cos(x[7])*sin(x[6])
    R_22 = cos(x[7])*cos(x[6])
    R_B2I = vertcat(horzcat(R_00, R_01, R_02),
                    horzcat(R_10, R_11, R_12),
                    horzcat(R_20, R_21, R_22))

    # Navigation equation dynamics
    nav = mtimes(mtimes(R_I2R, R_B2I), vertcat(x[0], x[1], x[2])) - x0[0:3]
    return Function('f', [x, u], [vertcat(aircraft(x, u), nav)], ['x', 'u'], ['xd'])

def linearized_aircraft_slf(S, psi):
    """
    Given speed S and heading psi, compute a linearized model

    dx_dot = A dx + B du

    where dx and du are perturbations from a steady level flight
    equilibrium condition with:

    x = [u, v, w, p, q, r, phi, th, psi, ex, ey, ez]
    u = [T, al, ar, e, r].

    Returns the matrices A and B and also the equilibrium x0 and u0
    """

    # Compute equilibrium and round solutions
    x0, u0 = equilibrium(S, psi)
    x0 = x0.round(8)
    u0 = u0.round(8)

    # Add navigation equations and build model function
    x0 = np.hstack((x0, np.zeros(3)))
    f = aircraft_slf(x0, u0)

    # Compute Jacobians
    J = np.array(f.jacobian()(x0, u0, np.zeros(12)))
    A = J[0:12, 0:12]
    B = J[0:12, 12:]
    return A, B, x0, u0


if __name__ == "__main__":
    # Compute an equilibrium
    S = 25. # m/s
    psi = np.deg2rad(0.) # rad
    A, B, x0, u0 = linearized_aircraft_slf(S, psi)

