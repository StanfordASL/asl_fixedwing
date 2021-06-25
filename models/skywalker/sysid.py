"""
System identification for Skywalker plane
"""
import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit
import pdb

# Static thrust stand testing with APC 10x6E Prop
# Slow is with 11.2V battery and Shi is 12.2V (both after test)
# Units of S are grams
U_T = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
S_LOW = np.array([0, 73, 243, 450, 634, 940, 1246, 1540, 1910, 1975, 1975])
S_HI = np.array([0, 84, 278, 508, 754, 1090, 1430, 1808, 2220, 2320, 2290])

# Thrust stand geometry
L_MOTOR = 0.23 # meters from pivot to thrust stand axis
L_SCALE = 0.1625 # meters from pivot to scale
R = L_SCALE/L_MOTOR

# Convert g to N
C = 9.81/1000.0

def static_thrust(u_T, c_T0):
    return c_T0*u_T**2

def thrust(u_T, c_T0, c_TVom, V):
    return c_T0*(1.0 - c_TVom*V/u_T)*u_T**2

def aircraft_ctrl_params():
    """
    Compute the parameters that define the aircraft's command to control mappings
    
    delta = c_delta*u_delta + delta_0 maps [-1,1] to radians
    T = c_T0*(1 - C_TVom*V/u_T)*u_T^2 maps [0,1] to Newtons
    """

    # Convert scale measurement to thrust S*L_SCALE = T*L_MOTOR
    # Thrust model parameters
    idx = 9 # only take first data points for fit
    u_T = np.concatenate((U_T[0:idx], U_T[0:idx]))
    T = C*R*np.concatenate((S_LOW[0:idx], S_HI[0:idx]))
    popt, _ = curve_fit(static_thrust, u_T, T)
    c_T0 = popt[0]
    c_TVom = 0.04

    # TODO Determine mapping for control surface deflection
    a_0 = 0.0
    e_0 = 0.0
    r_0 = 0.0
    c_a = 1.0
    c_e = 1.0
    c_r = 1.0
    print("WARNING: Control surface deflection map not defined, using default.")
    
    return np.array([c_T0, c_TVom, a_0, c_a, e_0, c_e, r_0, c_r])

if __name__ == '__main__':
    p = aircraft_ctrl_params()
    plt.figure()
    plt.plot(U_T, C*R*S_LOW, label='11.2V')
    plt.plot(U_T, C*R*S_HI, label='12.2V')
    plt.plot(U_T, static_thrust(U_T, p[0]), label='Quadratic Fit')
    plt.xlabel('u_T')
    plt.ylabel('T [N]')
    plt.legend()

    plt.figure()
    plt.plot(U_T, thrust(U_T, p[0], p[1], 15.0))
    plt.xlabel('u_T')
    plt.ylabel('T [N]')
    plt.title('Thrust Curve at V = 15.0 m/s')
    plt.show()
