import numpy as np

"""
All of the rotation parameterizations here represent rotations
of an initial fixed frame to a new frame. Therefore using these
parameterizations to operate on a vector is the same as transforming
the new frame coordinates into the original fixed frame.

Examples if rotation describes relation of body w.r.t. inertial
(how to rotate inertial to body):
R: coverts body frame coordinates into inertial frame coordinates
euler: usual roll-pitch-yaw of body frame w.r.t inertial frame
"""
def R_to_euler(R):
    """ Converts the rotation matrix R to the Euler angles (ZYX)
    (Yaw-Pitch-Roll) (psi, th, phi) used for aircraft conventions.

    Note to compute the Euler angles for the aircraft this should be
    the R matrix that converts a vector from body frame coordinates
    to a vector in inertial frame coordinates.
    """
    if np.sqrt(R[0,0]**2 + R[1,0]**2) >= .000001:
        phi = np.arctan2(R[2,1], R[2,2])
        th = np.arcsin(-R[2,0])
        psi = np.arctan2(R[1,0], R[0,0])
    else:
        phi = np.arctan2(-R[1,2], R[1,1])
        th = np.arcsin(-R[2,0])
        psi = 0.0
    return phi, th, psi

def euler_to_R(phi,th,psi):
    """ 
    Converts Euler angles (ZYX) (Yaw-Pitch-Roll) (psi, th, phi) 
    into the inertial to body rotation matrix """
    return np.array([[np.cos(th)*np.cos(psi), 
                    np.cos(th)*np.sin(psi), 
                    -np.sin(th)],
                    [np.sin(phi)*np.sin(th)*np.cos(psi) - np.cos(phi)*np.sin(psi),
                    np.cos(phi)*np.cos(psi) + np.sin(phi)*np.sin(th)*np.sin(psi),
                    np.sin(phi)*np.cos(th)],
                    [np.sin(phi)*np.sin(psi) + np.cos(phi)*np.sin(th)*np.cos(psi),
                    np.cos(phi)*np.sin(th)*np.sin(psi) - np.sin(phi)*np.cos(psi),
                    np.cos(phi)*np.cos(th)]]).T

def quat_to_R(q):
    """
    Converts a unit quaternion:

    q = (w, x, y, z) = w + (x i, y j, z k) 

    into an equivalent rotation matrix.
    """
    return np.array([[1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[3]*q[0], 2*q[1]*q[3] + 2*q[2]*q[0]],
                  [2*q[1]*q[2] + 2*q[3]*q[0], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[1]*q[0]],
                  [2*q[1]*q[3] - 2*q[2]*q[0], 2*q[2]*q[3] + 2*q[1]*q[0], 1 - 2*q[1]**2 - 2*q[2]**2]])

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

def axis_to_quat(th, a):
    """
    Converts an axis a = (x, y, z) and angle th into a unit quaternion

    q = (w, x, y, z) = w + (x i, y j, z k) 
    """
    a = a/np.linalg.norm(a)
    return np.array([np.cos(th/2),
                     a[0]*np.sin(th/2),
                     a[1]*np.sin(th/2),
                     a[2]*np.sin(th/2)])

def quat_to_axis(q):
    """
    Converts a unit quaternion:

    q = (w, x, y, z) = w + (x i, y j, z k) 

    into a normalized axis a = (x, y, z) and angle th.

    Returns th, a
    """
    return 2*np.arccos(q[0]), np.array([q[1]/np.sqrt(1 - q[0]**2),
                                        q[2]/np.sqrt(1 - q[0]**2),
                                        q[3]/np.sqrt(1 - q[0]**2)])

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


if __name__ == '__main__':
    # # Testing quat to euler
    # a1 = np.array([0., 0., 1.])
    # th1 = np.pi/4

    # a2 = np.array([0., 1., 0.])
    # th2 = np.pi/4

    # q1 = axis_to_quat(th1, a1)
    # q2 = axis_to_quat(th2, a2)

    # print quat_to_euler(compose_quats(q1, q2))

    # # Testing inverse quaternion
    # a1 = np.array([1., 0., 0.])
    # th1 = np.pi/4
    # q1 = axis_to_quat(th1, a1)
    # R1 = quat_to_R(q1)
    # print R1.T
    # print quat_to_R(invert_quat(q1))

    # Testing R to euler failure
    phi = -np.pi/4
    th = -np.pi/2
    psi = 0.0
    R = euler_to_R(phi, th, psi)
    print phi, th, psi
    print R_to_euler(R)
