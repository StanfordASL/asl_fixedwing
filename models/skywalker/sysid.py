"""
System identification for Skywalker plane
"""
from os.path import dirname, abspath, join
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import curve_fit
from scipy.interpolate import interp1d
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

def load_prop_data():
    path = dirname(abspath(__file__))
    fpath = join(path, 'thrust/PER3_10x6E.dat')
    RPM = []
    data = {}
    with open(fpath) as f:
        for line in f:
            if line.split('=')[0].strip(' ') == 'PROP RPM':
                rpm = int(line.split('=')[1].strip(' '))
                RPM.append(rpm)
                data[rpm] = {'V':[], 'J':[], 'T':[], 'CT':[]}
            try:
                d = line.split()
                data[rpm]['V'].append(0.44704*float(d[0])) # convert to m/s
                data[rpm]['J'].append(float(d[1]))
                data[rpm]['T'].append(4.4482216*float(d[7])) # convert to N
                data[rpm]['CT'].append(float(d[3]))
            except:
                continue
    return RPM, data

def compute_c_om(rpm, data, idx):
    T0 = []
    for rot in rpm:
        T0.append(data[rot]['T'][0])
    f = interp1d(T0, 0.10472*np.array(rpm)) # convert to rad/s
    T0_exp = np.array(C*R*S_HI[1:]) # use only HI voltage value
    rot_exp = f(T0_exp)
    rot_exp = np.hstack(([0.0], rot_exp))
    popt, _ = curve_fit(uT_to_rot, U_T[0:idx], rot_exp[0:idx])
    return popt[0]

def compute_cT0_cTVom(rpm, data, c_om):
    Vmax = 20
    x = np.empty((2,0))
    y = np.empty((0))
    for rot in rpm:
        rot_rad = 0.10472*rot # convert to rad/s
        if rot_rad < 0.8*c_om:
            u_T = rot_rad/c_om # compute the normalized command
            V = np.array(data[rot]['V'])

            # Only take data points with V < Vmax
            idx = np.where(V < Vmax)[0]
            V = V[idx]
            T = np.array(data[rot]['T'])[idx]

            x = np.hstack((x, np.vstack((V, np.tile(u_T, V.shape[0])))))
            y = np.hstack((y, T))
    popt, _ = curve_fit(thrust2, x, y)
    c_T0 = popt[0]
    c_TVom = popt[1]
    return c_T0, c_TVom, x, y

def uT_to_rot(u_T, c_om):
    return c_om*u_T

def static_thrust(u_T, c_T0):
    return c_T0*u_T**2

def thrust(u_T, c_T0, c_TVom, V):
    return c_T0*(1.0 - c_TVom*V/u_T)*u_T**2

def thrust2(x, c_T0, c_TVom):
    """
    x = [V, u_T]
    """
    return c_T0*(1.0 - c_TVom*x[0]/x[1])*x[1]**2

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
    print("c_T0 = %.2f when using experimental data only." % c_T0)

    # Now use propeller data combined with thrust stand data
    # to estimate mapping between commanded value and RPM
    rpm, data = load_prop_data()
    c_om = compute_c_om(rpm, data, idx)
    c_T0, c_TVom, _, _ = compute_cT0_cTVom(rpm, data, c_om)
    print("c_T0 = %.2f, c_TVom = %.2f with data fit" % (c_T0, c_TVom))

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
    # Look at raw data and compare to performance table
    rpm, data = load_prop_data()
    c_om = compute_c_om(rpm, data, 9)
    c_T0, c_TVom, x, y = compute_cT0_cTVom(rpm, data, c_om)

    # Create plot for advance ratio J vs C_T
    plt.figure()
    for rot in rpm:
        if 0.10472*rot < c_om:
            plt.plot(data[rot]['J'], data[rot]['CT'], label='RPM = {}'.format(rot))

    plt.xlabel('J')
    plt.ylabel('C_T')
    plt.legend()

    # Create plot for entire thrust mapping fit
    y_fit = thrust2(x, c_T0, c_TVom)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x[0,:], x[1,:], y, c='r', label='Data')
    ax.scatter(x[0,:], x[1,:], y_fit, c='b', label='Fit')
    ax.set_xlabel('Velocity [m/s]')
    ax.set_ylabel('Command')
    ax.set_zlabel('Thrust [N]')

    p = aircraft_ctrl_params()
    plt.figure()
    plt.plot(U_T, C*R*S_LOW, label='11.2V')
    plt.plot(U_T, C*R*S_HI, label='12.2V')
    plt.plot(U_T, static_thrust(U_T, p[0]), label='Quadratic Fit')
    plt.xlabel('u_T')
    plt.ylabel('T [N]')
    plt.legend()

    plt.figure()
    plt.plot(U_T[1:], thrust(U_T[1:], p[0], p[1], 15.0))
    plt.xlabel('u_T')
    plt.ylabel('T [N]')
    plt.title('Thrust Curve at V = 15.0 m/s')
    plt.show()