"""
    @file gazebo/model.py
    Defines a model of the gazebo simulator aircraft
    and provides functionality to linearize about
    steady level and steady turning flight conditions.
"""

import numpy as np
from casadi import *
import sys

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
        u = [T, a, e, r]
    """
    q = 0.5*RHO*(x[0]**2 + x[2]**2)
    a = atan(x[2]/x[0])

    # Wings
    D_lw = fabs(CDA_W*(a - A0_W))*q*S_W
    L_lw = (CLA_W*(a - A0_W) - CLD_W*u[1])*q*S_W
    F_lw = vertcat(-D_lw*cos(a) + L_lw*sin(a),
                    0.0,
                    -D_lw*sin(a) - L_lw*cos(a))
    D_rw = fabs(CDA_W*(a - A0_W))*q*S_W
    L_rw = (CLA_W*(a - A0_W) + CLD_W*u[1])*q*S_W
    F_rw = vertcat(-D_rw*cos(a) + L_rw*sin(a),
                    0.0,
                    -D_rw*sin(a) - L_rw*cos(a))

    # Elevator
    D_e = fabs(CDA_E*(a - A0_E))*q*S_E
    L_e = (CLA_E*(a - A0_E) + CLD_E*u[2])*q*S_E
    F_e = vertcat(-D_e*cos(a) + L_e*sin(a),
                    0.0,
                    -D_e*sin(a) - L_e*cos(a))

    # Rudder
    q_r = 0.5*RHO*(x[0]**2 + x[1]**2)
    a_r = atan(x[1]/x[0])
    D_r = fabs(CDA_R*(a_r - A0_R))*q_r*S_R
    L_r = (CLA_R*(a_r - A0_R) + CLD_R*u[3])*q_r*S_R
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
        x = [u, v, w, p, q, r, phi, th]
        u = [T, a, e, r],  

    u, v, w: velocity components in body frame coord
    p, q, r: body rates
    phi, th: euler angles of body rel. to inertial
    T: thrust
    a: aileron deflection angle, left and right wing (anti-symmetrically)
    e, r: elevator and rudder deflection angles

	Note this ignores yaw angle and navigation equations because
	they are not relevant to the remaining equations.
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
        x[4]*cos(x[6]) - x[5]*sin(x[7])
		)

def sgf_equilibrium(S, gamma):
    """ Compute an equilibrium state and control for the aircraft
    to fly with speed S in steady-glidelsope flight.
    Additionally, respecting control constraints and trying
    to minimize the control surface deflections. """
    S = float(S)
    gamma = float(gamma)

    # Create symbolic state and control vars
    uvw = MX.sym('uvw', 3)
    pqr = MX.sym('pqr', 3)
    phi = MX.sym('phi')
    th = MX.sym('th')
    dpsi = MX.sym('dpsi')
    u = MX.sym('u', 4)

    # Create constraints for the equilibrium condition
    x = vertcat(uvw, pqr, phi, th, dpsi)
    const = vertcat(aircraft(x, u)[0:6],              # equilibrium
                    norm_2(uvw) - S,                  # specified velocity
                    uvw[1],                           # no sideways motion
                    uvw[2]*cos(th) - uvw[0]*sin(th) + S*sin(gamma)) # glideslope

    # Bounds on the decision variables u, v, w, th, u[0:4]
    lb = [-inf, -inf, -inf, -inf, 0, -D_MAX, -D_MAX, -D_MAX]
    ub = [inf, inf, inf, inf, T_MAX, D_MAX, D_MAX, D_MAX]

    # Find a equilibrium point by solving a nonlinear
    # optimization problem by minimizing the control effort
    # at the solution
    f = (u[1]/D_MAX)**2 + (u[2]/D_MAX)**2 + (u[3]/D_MAX)**2
    nlp = {'x':vertcat(uvw, th, u),
           'p':vertcat(pqr, phi, dpsi),
           'f':f,
           'g':const}
    opt = nlpsol('opt', 'ipopt', nlp)

    # Solve the NLP
    z_guess = np.zeros(8)
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
                      0.0))        # dpsi
    usol = sol[4:]
    residual = np.sum(np.abs(np.array(aircraft(xsol,usol))))
    print('residual (should be zero): {:.6f}'.format(residual))
    
    if residual > .0001:
        print('Equilibrium calculation did not converge.')
        sys.exit()
    print('speed S = {} m/s'.format(S))
    print('u = {:.2f} m/s, v = 0 m/s, w = {:.2f} m/s'.format(sol[0], sol[2]))
    print('p = 0 rad/s, q = 0 rad/s, r = 0 rad/s')
    print('phi = 0 deg, th = {:.2f} deg, dpsi = 0.0 deg'.format(np.rad2deg(sol[3])))
    print('T = {:.2f} N'.format(sol[4]))
    print('d_a = {:.2f} deg'.format(np.rad2deg(sol[5])))
    print('d_e = {:.2f} deg, d_r = {:.2f} deg'.format(*np.rad2deg(sol[6:])))
    
    return xsol, usol

def stf_equilibrium(S, R):
    """ Compute an equilibrium state and control for the aircraft
    to fly with speed S in steady-turning flight with constant altitude.
    Additionally, respecting control constraints and trying
    to minimize the control surface deflections. 
	
	S: speed in m/s
	R: turning radius in m
	"""
    S = float(S)
    R = float(R)

    # Create symbolic state and control vars
    uvw = MX.sym('uvw', 3)
    pqr = MX.sym('pqr', 3)
    phi = MX.sym('phi')
    th = MX.sym('th')
    dpsi = MX.sym('dpsi')
    u = MX.sym('u', 4)

    # Create constraints for the equilibrium condition
    x = vertcat(uvw, pqr, phi, th, dpsi)
    xd = aircraft(x, u)
    yaw = (1.0/cos(x[7]))*(x[4]*sin(x[6]) + x[5]*cos(x[6])) - S/R # yaw rate equation
    const = vertcat(xd[0:8],              		  # equilibrium
                    norm_2(uvw) - S,              # specified velocity
                    yaw,                          # turn rate
                    uvw[2]*cos(th)*cos(phi) \
					 + uvw[1]*cos(th)*sin(phi) \
					 - uvw[0]*sin(th))            # constant altitude

    # Bounds on the decision variables u, v, w, p, q, r, phi, th, u[0:4]
    lb = [-inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, 0, -D_MAX, -D_MAX, -D_MAX]
    ub = [inf, inf, inf, inf, inf, inf, inf, inf, T_MAX, D_MAX, D_MAX, D_MAX]

    # Find a equilibrium point by solving a nonlinear
    # optimization problem by minimizing the control effort
    # at the solution
    f = (u[1]/D_MAX)**2 + (u[2]/D_MAX)**2 + (u[3]/D_MAX)**2
    nlp = {'x':vertcat(uvw, pqr, phi, th, u),
           'p':vertcat(dpsi),
           'f':f,
           'g':const}
    opt = nlpsol('opt', 'ipopt', nlp)

	# Solve the NLP
    z_guess = np.zeros(12)
    z_guess[0] = S
    tol = .00001
    sol = opt(x0=z_guess, p=np.zeros(1),
            lbg=-tol*np.ones(11), ubg=tol*np.ones(11),
            lbx=lb, ubx=ub)

    # Extract solution
    print('\n\nEquilibrium solution results:')
    sol = np.array(sol['x']).flatten()
    xsol = np.hstack((sol[0:3],    # uvw
                      sol[3:6],    # pqr
                      sol[6],      # phi
                      sol[7],      # th
                      0.0))        # dpsi
    usol = sol[8:]
    residual = np.sum(np.abs(np.array(aircraft(xsol, usol))))
    print('residual (should be zero): {:.6f}'.format(residual))
    
    if residual > .0001:
        print('Equilibrium calculation did not converge.')
        sys.exit()
    print('speed S = {} m/s'.format(S))
    print('u = {:.2f} m/s, v = {:.2f} m/s, w = {:.2f} m/s'.format(sol[0], sol[1], sol[2]))
    print('p = {:.2f} deg/s, q = {:.2f} deg/s, r = {:.2f} deg/s'.format(*np.rad2deg(sol[3:6])))
    print('phi = {:.2f} deg, th = {:.2f} deg, dpsi = 0.0 deg'.format(np.rad2deg(sol[6]), np.rad2deg(sol[7])))
    print('T = {:.2f} N'.format(sol[8]))
    print('d_a = {:.2f} deg'.format(np.rad2deg(sol[9])))
    print('d_e = {:.2f} deg, d_r = {:.2f} deg'.format(*np.rad2deg(sol[10:])))

    print('psi_dot_0 = {:.2f} deg/s'.format(np.rad2deg(S/R)))
    return xsol, usol

def aircraft_sgf(x0, u0):
    """ Aircraft dynamics with additional yaw and navigation equations
    to include error from a steady glideslope flight corresponding
    to the equilibrium (x0, u0). Returns a
    casadi function. New state for this model is
    x = [u, v, w, p, q, r, phi, th, dpsi, x_r, y_r, z_r]

    dpsi is relative heading psi - psi_0 and (x_r,y_r,z_r)
	are relative position coordinates w.r.t R frame.
    """
    x = MX.sym('x', 12)
    u = MX.sym('u', 4)

    # Converting inertial to R frame, assuming phi0 = 0
    R_R_I = np.array([[np.cos(x0[7]), 0.0, -np.sin(x0[7])],
                      [0.0, 1.0, 0.0],
                      [np.sin(x0[7]), 0.0, np.cos(x0[7])]])
    
	# Converting body to inertial (with yaw offset)
    R_00 = cos(x[8])*cos(x[7])
    R_01 = cos(x[8])*sin(x[7])*sin(x[6]) - cos(x[6])*sin(x[8])
    R_02 = sin(x[8])*sin(x[6]) + cos(x[8])*cos(x[6])*sin(x[7])
    R_10 = cos(x[7])*sin(x[8])
    R_11 = cos(x[8])*cos(x[6]) + sin(x[8])*sin(x[7])*sin(x[6])
    R_12 = cos(x[6])*sin(x[8])*sin(x[7]) - cos(x[8])*sin(x[6])
    R_20 = -sin(x[7])
    R_21 = cos(x[7])*sin(x[6])
    R_22 = cos(x[7])*cos(x[6])
    R_I_B = vertcat(horzcat(R_00, R_01, R_02),
                    horzcat(R_10, R_11, R_12),
                    horzcat(R_20, R_21, R_22))

	# Yaw angle dynamics
    yaw = (1.0/cos(x[7]))*(x[4]*sin(x[6]) + x[5]*cos(x[6]))

    # Navigation equation dynamics
    nav = mtimes(mtimes(R_R_I, R_I_B), vertcat(x[0], x[1], x[2])) - x0[0:3]

    return Function('f', [x, u], [vertcat(aircraft(x, u), yaw, nav)], ['x', 'u'], ['xd'])

def aircraft_stf(x0, u0):
    """ Aircraft dynamics with additional yaw and navigation equations
    to include error from a steady turning flight corresponding
    to the equilibrium (x0, u0). Returns a
    casadi function. New state for this model is
    x = [u, v, w, p, q, r, phi, th, dpsi, x_r, y_r, z_r]

    dpsi is relative heading psi - psi_0 and (x_r,y_r,z_r)
    are relative position coordinates w.r.t R frame.
    """
    x = MX.sym('x', 12)
    u = MX.sym('u', 4)

    # Converting inertial to R frame, assuming phi0 = 0
    R_R_I = np.array([[np.cos(x0[7]), 0.0, -np.sin(x0[7])],
                      [np.sin(x0[7])*np.sin(x0[6]), np.cos(x0[6]), np.cos(x0[7])*np.sin(x0[6])],
                      [np.sin(x0[7])*np.cos(x0[6]), -np.sin(x0[6]), np.cos(x0[7])*np.cos(x0[6])]])

    # Converting body to inertial (with yaw offset)
    R_00 = cos(x[8])*cos(x[7])
    R_01 = cos(x[8])*sin(x[7])*sin(x[6]) - cos(x[6])*sin(x[8])
    R_02 = sin(x[8])*sin(x[6]) + cos(x[8])*cos(x[6])*sin(x[7])
    R_10 = cos(x[7])*sin(x[8])
    R_11 = cos(x[8])*cos(x[6]) + sin(x[8])*sin(x[7])*sin(x[6])
    R_12 = cos(x[6])*sin(x[8])*sin(x[7]) - cos(x[8])*sin(x[6])
    R_20 = -sin(x[7])
    R_21 = cos(x[7])*sin(x[6])
    R_22 = cos(x[7])*cos(x[6])
    R_I_B = vertcat(horzcat(R_00, R_01, R_02),
                    horzcat(R_10, R_11, R_12),
                    horzcat(R_20, R_21, R_22))

    # Yaw angle dynamics
    yaw_0 = (1.0/np.cos(x0[7]))*(x0[4]*np.sin(x0[6]) + x0[5]*np.cos(x0[6]))
    yaw = (1.0/cos(x[7]))*(x[4]*sin(x[6]) + x[5]*cos(x[6])) - yaw_0 

    # Navigation equation dynamics
    g = x0[0:3] + vertcat(x0[4]*x[11] - x0[5]*x[10],
						  x0[5]*x[9] - x0[3]*x[11],
						  x0[3]*x[10] - x0[4]*x[9])
    nav = mtimes(mtimes(R_R_I, R_I_B), vertcat(x[0], x[1], x[2])) - g

    return Function('f', [x, u], [vertcat(aircraft(x, u), yaw, nav)], ['x', 'u'], ['xd'])

def linearized_aircraft_sgf(S, gamma):
    """
    Given speed S [m/s], and glideslope angle gamma [rad] (negative
    glideslope angle has velocity below horizoin), compute a linearized model

    dx_dot = A dx + B du

    where dx and du are perturbations from a steady glideslope flight
    equilibrium condition with:

    x = [u, v, w, p, q, r, phi, th, dpsi, x_r, y_r, z_r]
    u = [T, a, e, r].

    Returns the matrices A and B and also the equilibrium x0 and u0
    """

    # Compute equilibrium and round solutions
    x0, u0 = sgf_equilibrium(S, gamma)
    x0 = x0.round(8)
    u0 = u0.round(8)

    # Add navigation equations and build model function
    x0 = np.hstack((x0, np.zeros(3)))
    f = aircraft_sgf(x0, u0)

    # Compute Jacobians
    J = np.array(f.jacobian()(x0, u0, np.zeros(12)))
    A = J[0:12, 0:12]
    B = J[0:12, 12:]

    # Compute output matrices
    C = np.eye(12)
    H = np.eye(12)
    return A, B, C, H, x0, u0

def linearized_aircraft_stf(S, R):
    """
    Given speed S [m/s] and turning radius R [m], compute a linearized model

    dx_dot = A dx + B du

    where dx and du are perturbations from a steady turning flight
    equilibrium condition with:

    x = [u, v, w, p, q, r, phi, th, dpsi, x_r, y_r, z_r]
    u = [T, a, e, r].

    Returns the matrices A and B and also the equilibrium x0 and u0
    """

    # Compute equilibrium and round solutions
    x0, u0 = stf_equilibrium(S, R)
    x0 = x0.round(8)
    u0 = u0.round(8)
    
    # Add navigation equations and build model function
    x0 = np.hstack((x0, np.zeros(3)))
    f = aircraft_stf(x0, u0)

    # Compute Jacobians
    J = np.array(f.jacobian()(x0, u0, np.zeros(12)))
    A = J[0:12, 0:12]
    B = J[0:12, 12:]

    # Compute output matrices
    C = np.eye(12)
    H = np.eye(12)
    return A, B, C, H, x0, u0

def simplify_control(A, B, x0, u0):
    """
	Given the linarized A and B matrices corresponding to the state:

    x = [u, v, w, p, q, r, phi, th, dpsi, x_r, y_r, z_r]
    u = [T, a, e, r]

	this function simplifies the model to assume the body rates p, q, r
	are directly controlled, by essentially just removing the rotational
	equations such that the new state and control are:

	x = [u, v, w, phi, th, dpsi, x_r, y_r, z_r]
    u = [T, p, q, r].
    """
    x0 = np.hstack((x0[:3], x0[6:]))
    u0 = np.hstack((u0[0], x0[3:6]))
    
    # Remove columns
    B = np.hstack((B[:,0].reshape(12,1), A[:,3:6]))
    A = np.hstack((A[:,0:3], A[:,6:]))
    
    # Remove rows
    B = np.vstack((B[0:3,:],B[6:,:]))
    A = np.vstack((A[0:3,:],A[6:,:]))

    C = np.eye(9)
    H = np.eye(9)
    return A, B, C, H, x0, u0
	

if __name__ == "__main__":
    # Compute an equilibrium
    #S = 13. # m/s
    #gamma = np.deg2rad(-3.5)
    #A, B, C, H, x0, u0 = linearized_aircraft_sgf(S, gamma)
    
    S = 13. # m/s
    R = 50 # m
    A, B, C, H, x0, u0 = linearized_aircraft_stf(S, R)

    # Simplify the model to have body rate control
    #A, B, C, H, x0, u0 = simplify_control(A, B, x0, u0)
