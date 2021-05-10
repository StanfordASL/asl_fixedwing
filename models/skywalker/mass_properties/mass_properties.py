import numpy as np
import csv
from os.path import dirname, abspath, join
from matplotlib import pyplot as plt
from scipy.fftpack import fft, fftfreq
from scipy.interpolate import interp1d
import pdb

PATH = dirname(abspath(__file__))
G = 9.80665
M_CRADLE = 1.042 # kg, mass of cradle
M_ASSEMBLY = 3.512 # kg, mass of cradle + aircraft
M_AIRCRAFT = 2.470 # kg, mass of aicraft

def compose_quats(p, q):
    """
    Composes two rotations parameterized by the unit quaternions:

    p = (pw, px, py, pz) = pw + (px i, py j, pz k)
    q = (qw, qx, qy, qz) = qw + (qx i, qy j, qz k) 

    and outputs a single quaternion representing the composed rotation.
    This performs rotation p first and then rotation q after.
    """
    return np.array([p[0]*q[0] - (p[1]*q[1] + p[2]*q[2] + p[3]*q[3]),
                     p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
                     p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
                     p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0]])

def invert_quat(q):
    """
    Inverts a unit quaternion, gives the opposite rotation
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_to_euler(q):
    """
    Converts a unit quaternion:

    q = (w, x, y, z) = w + (x i, y j, z k) 

    into the aircraft Euler angles (roll, pitch, yaw) = (phi, th, psi).
    """
    
    R00 = 1 - 2*q[2]**2 - 2*q[3]**2
    R10 = 2*q[1]*q[2] + 2*q[3]*q[0]
    if np.sqrt(R00**2 + R10**2) >= .000001:
        phi = np.arctan2(2*q[2]*q[3] + 2*q[1]*q[0], 1 - 2*q[1]**2 - 2*q[2]**2)
        th = np.arcsin(-2*q[1]*q[3] + 2*q[2]*q[0])
        psi = np.arctan2(R10, R00)
    else:
        phi = np.arctan2(-2*q[2]*q[3] - 2*q[1]*q[0], 1 - 2*q[1]**2 - 2*q[3]**2)
        th = np.arcsin(-2*q[1]*q[3] + 2*q[2]*q[0])
        psi = 0.0
    return phi, th, psi

def load_data(filename):
    fpath = join(PATH, filename)
    t = []
    qx = []
    qy = []
    qz = []
    qw = []
    with open(fpath, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        record = False
        for row in reader:
            if len(row) > 0:
                if record:
                    try:
                        row = [float(val) for val in row]
                        t.append(float(row[1]))
                        qx.append(float(row[2]))
                        qy.append(float(row[3]))
                        qz.append(float(row[4]))
                        qw.append(float(row[5]))
                    except:
                        pass
                if row[0] == 'Frame':
                    record = True
    q = np.vstack((qw, qx, qy, qz))
    t = np.array(t)

    # Compute Euler angles and do some corrections
    q_correct = np.array([1, 1, 0, 0])
    q_correct = q_correct/np.linalg.norm(q_correct)
    q0 = invert_quat(q[:,0])
    euler0 = quat_to_euler(compose_quats(q_correct, compose_quats(q0, q[:,0])))
    euler = np.zeros((3, t.shape[0]))
    for i in range(t.shape[0]):
        euler[:,i] = quat_to_euler(compose_quats(q_correct, compose_quats(q0, q[:,i])))
        euler[:,i] -= euler0
    return t, euler

def linear_param_est(t, theta, verbose=False):
    """
    Given a damped second order linear system respone (t, theta), compute
    the systems damping ratio zeta and natural frequency wn
    """
    # Start data from second peak
    i0 = np.argmax(theta)
    t = t[i0:] - t[i0]
    theta = theta[i0:]
    i0 = np.argmin(theta)
    t = t[i0:] - t[i0]
    theta = theta[i0:]
    i0 = np.argmax(theta)
    t = t[i0:] - t[i0]
    theta = theta[i0:]

    # Interpolate to downsample
    f = interp1d(t, theta)
    t = np.linspace(0, t[-1], 4000)
    dt = t[1] - t[0]
    theta = f(t)

    # FFT to get frequency
    f_s = 1.0/dt # sampling freq [Hz]
    X = fft(theta)
    freq = fftfreq(theta.shape[0]) * f_s

    main_freq = freq[np.argmax(np.abs(X))] # Hz
    T = 1.0/main_freq
    wd = 2*np.pi*main_freq # rad/s

    # Find peak values
    di = int(T/dt)
    th_max = [theta[0]]
    t_max = [t[0]]
    i = di
    w = 100
    while i < len(theta) - w:
        # Find max within a window of current i
        imax = np.argmax(theta[i-w:i+w]) + (i-w)
        th_max.append(theta[imax])
        t_max.append(t[imax])
        i = imax + di
    th_max = np.array(th_max)
    th_max_ratio = np.mean(th_max[1:]/th_max[:-1])

    # Compute desired values
    log_ratio = np.log(th_max_ratio)
    zeta = np.abs(log_ratio)/np.sqrt(4.0*np.pi**2 + log_ratio**2)
    wn = wd/np.sqrt(1.0 - zeta**2)

    if verbose:
        print("Period of oscillation: %.3f seconds" % T)
        print("Damped natural frequency estimate: %.3f rad/s" % wd)
        print("Peak decay ratio mean: %.3f" % th_max_ratio)
        print("Natural frequency estimate: %.3f rad/s" % wn)
        plt.plot(t, np.rad2deg(theta), label='theta')
        plt.plot(t_max, np.rad2deg(th_max), 'r*')
        plt.legend()
        plt.show()

    return zeta, wn

def compound_pendulum_moi(t, theta, m, l, verbose=False):
    """
    t: time data [s]
    theta: swing angle data [rad]
    m: mass of the object [kg]
    l: distance from pivot axis of pendulum to center of mass of the object [m]

    Returns:
    Ip: moment of inertia of the object about the compound pendulum swing axis [kg m^2]
    I: moment of inertia fo the object about the object c.m. [kg m^2]
    """
    _, wn = linear_param_est(t, theta, verbose=verbose)

    # Compute moment of inertia estimate about pendulum swing axis
    Ip = (m*G*l)/(wn**2)

    # Compute moment of inertia about the center of mass of the object
    I = Ip - m*l**2

    # Plot results
    if verbose:
        print("Moment of inertia about swing axis: %.3f kg m^2" % Ip)
        print("Moment of inertia about center of mass: %.3f kg m^2" % I)
    return Ip, I

def bifilar_pendulum_moi(t, theta, m, D, h, verbose=False):
    """
    t: time data [s]
    theta: swing angle data [rad]
    m: mass of the object [kg]
    D: distance between filars [m]
    h: length of filars [m]

    Returns:
    Ip: moment of inertia of the object about the pivot axis [kg m^2]
    """
    _, wn = linear_param_est(t, theta, verbose=verbose)

    # Compute moment of inertia estimate about pendulum swing axis
    Ip = (m*G*D**2)/(4.0*h*wn**2)

    # Plot results
    if verbose:
        print("Moment of inertia about pivot axis: %.3f kg m^2" % Ip)
    return Ip

def compute_pitch_moi(verbose=False):
    # Compute cradle MOI
    l_cradle = 1.79 # m, distance from pivot to cradle c.m.
    t, euler = load_data('cradle/cradle_yaxis_1.csv')
    _, I1 = compound_pendulum_moi(t, euler[1,:], M_CRADLE, l_cradle, verbose=verbose)
    t, euler = load_data('cradle/cradle_yaxis_2.csv')
    _, I2 = compound_pendulum_moi(t, euler[1,:], M_CRADLE, l_cradle, verbose=verbose)
    I_y_cradle = (I1 + I2)/2.0

    # Compute assembly MOI
    l_assembly = 1.86 # m, distance from pivot to assembly c.m.
    t, euler = load_data('aircraft_cradle/yaxis_1.csv')
    Ip1, _ = compound_pendulum_moi(t, euler[1,:], M_ASSEMBLY, l_assembly, verbose=verbose)
    t, euler = load_data('aircraft_cradle/yaxis_2.csv')
    Ip2, _ = compound_pendulum_moi(t, euler[1,:], M_ASSEMBLY, l_assembly, verbose=verbose)
    Ip = (Ip1 + Ip2)/2.0

    # Compute aircraft MOI
    Ip_cradle = I_y_cradle + M_CRADLE*l_cradle**2
    Ip_aircraft = Ip - Ip_cradle
    l_aircraft = l_assembly - .165 + .1925
    I_y_aircraft = Ip_aircraft - M_AIRCRAFT*l_aircraft**2

    print("Pitch MOI of aircraft about center of mass: %.3f kg m^2" % I_y_aircraft)
    return I_y_aircraft

def compute_roll_moi(verbose=False):
    # Compute cradle MOI
    D_cradle = 0.49 # m
    h_cradle = 1.44 # m
    t, euler = load_data('cradle/cradle_xaxis_1.csv')
    Ip1 = bifilar_pendulum_moi(t, euler[0,:], M_CRADLE, D_cradle, h_cradle, verbose=verbose)
    t, euler = load_data('cradle/cradle_xaxis_2.csv')
    Ip2 = bifilar_pendulum_moi(t, euler[0,:], M_CRADLE, D_cradle, h_cradle, verbose=verbose)
    I_x_cradle = (Ip1 + Ip2)/2.0

    # Compute assembly MOI
    D_assembly = 0.49 # m
    h_assembly = 1.145 # m
    t, euler = load_data('aircraft_cradle/xaxis_1.csv')
    Ip1 = bifilar_pendulum_moi(t, euler[0,:], M_ASSEMBLY, D_assembly, h_assembly, verbose=verbose)
    t, euler = load_data('aircraft_cradle/xaxis_2.csv')
    Ip2 = bifilar_pendulum_moi(t, euler[0,:], M_ASSEMBLY, D_assembly, h_assembly, verbose=verbose)
    t, euler = load_data('aircraft_cradle/xaxis_3.csv')
    Ip3 = bifilar_pendulum_moi(t, euler[0,:], M_ASSEMBLY, D_assembly, h_assembly, verbose=verbose)
    I_x_assembly = (Ip1 + Ip2 + Ip3)/3.0

    I_x_aircraft = I_x_assembly - I_x_cradle

    print("Roll MOI of aircraft about center of mass: %.3f kg m^2" % I_x_aircraft)
    return I_x_aircraft

def compute_yaw_moi(verbose=False):
    # Compute cradle MOI
    D_cradle = 0.49 # m
    h_cradle = 1.44 # m
    t, euler = load_data('cradle/cradle_zaxis_1.csv')
    Ip1 = bifilar_pendulum_moi(t, euler[2,:], M_CRADLE, D_cradle, h_cradle, verbose=verbose)
    t, euler = load_data('cradle/cradle_zaxis_2.csv')
    Ip2 = bifilar_pendulum_moi(t, euler[2,:], M_CRADLE, D_cradle, h_cradle, verbose=verbose)
    I_z_cradle = (Ip1 + Ip2)/2.0

    # Compute assembly MOI
    D_assembly = 0.49 # m
    h_assembly = 1.91 # m
    t, euler = load_data('aircraft_cradle/zaxis_1.csv')
    Ip1 = bifilar_pendulum_moi(t, euler[2,:], M_ASSEMBLY, D_assembly, h_assembly, verbose=verbose)
    t, euler = load_data('aircraft_cradle/zaxis_2.csv')
    Ip2 = bifilar_pendulum_moi(t, euler[2,:], M_ASSEMBLY, D_assembly, h_assembly, verbose=verbose)
    t, euler = load_data('aircraft_cradle/zaxis_3.csv')
    Ip3 = bifilar_pendulum_moi(t, euler[2,:], M_ASSEMBLY, D_assembly, h_assembly, verbose=verbose)
    I_z_assembly = (Ip1 + Ip2 + Ip3)/3.0

    I_z_aircraft = I_z_assembly - I_z_cradle

    print("Yaw MOI of aircraft about center of mass: %.3f kg m^2" % I_z_aircraft)



compute_pitch_moi()
compute_roll_moi()
compute_yaw_moi()
