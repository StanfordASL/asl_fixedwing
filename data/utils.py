from os.path import dirname, abspath, join, isfile, isdir
import sys
import rosbag
import numpy as np

"""
    @brief Computes operator T such that om = T(aa) * aa_dot which
    transforms the rate of change of axis/angle representation of
    rotation from A to B into the angular velocity of B w.r.t A written
    in B coordinates.

    @param[in] aa  axis angle representation aa = (th1, th2, th3)
"""
def tangential_transf(aa):
    th2 = np.linalg.norm(aa)**2

    if th2 < 5e-6:
        th4 = th2 * th2
        c1 = 1.0  - th2/6   + th4/120
        c2 = 1/2. - th2/24  + th4/720
        c3 = 1/6. - th2/120 + th4/5040
    else:
        th = np.sqrt(th2)
        c1 = np.sin(th)/th
        c2 = (1 - np.cos(th))/th2
        c3 = (th - np.sin(th))/(th * th2)

    skew_th = np.array([[0.0, -aa[2], aa[1]],
                        [aa[2], 0.0, -aa[0]],
                        [-aa[1], aa[0], 0.0]])


    T = c1*np.eye(3) - c2*skew_th + c3*np.outer(aa, aa)
    return T

"""
    @brief Converts the angular velocity vector om for a frame B rotating w.r.t A
    into the rate of change of the axis/angle parameters that represent the
    rotation from A to B.

    @param[in] aa     axis/angle repr. of rotation from A to B frames [rad]
    @param[in] om     angular velocity of B w.r.t A [rad/s]
    @param[in] aadot  time derivative of aa [rad/s]
"""
def om_to_aadot(aa, om):
    T = tangential_transf(aa)
    aadot = np.matmul(np.linalg.inv(T), om)
    return aadot

"""
    @brief Given current axis/angle parameters for rotation from A to B and their
    time derivatives, computes the angular velocity vector of B w.r.t A.

    @param[in] aa     axis/angle repr. of rotation from A to B frames [rad]
    @param[in] aadot  time derivative of aa [rad/s]
    @param[in] om     angular velocity of B w.r.t A [rad/s]
"""
def aadot_to_om(aa, aadot):
    T = tangential_transf(aa)
    om = np.matmul(T, aadot);
    return om

def get_data_dir():
    return dirname(abspath(__file__))

def get_models_dir():
    return join(dirname(dirname(abspath(__file__))), 'models')

class pointStream:
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        self.t = []

    def add_point(self, t, x, y, z):
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)
        self.t.append(t)


class ctrlStream:
    def __init__(self):
        self.u = [[], [], [], []]
        self.t = []

    def add_point(self, t, u):
        self.t.append(t)
        for i in range(4):
            self.u[i].append(u[i])


class zStream:
    def __init__(self):
        self.z = []
        self.t = []

    def add_point(self, t, z):
        self.t.append(t)
        if not self.z:
            self.z = [[z[i]] for i in range(len(z))]
        else:
            for i in range(len(z)):
                self.z[i].append(z[i])


class planeData:
    """
    A class to extract data from Plane topic messages
    """
    def __init__(self):
        self.pos = pointStream() # inertial pos x_i, y_i, z_i [m]
        self.vel = pointStream() # body frame vel u, v, w [m/s]
        self.euler = pointStream() # euler angle phi, th, psi [rad]
        self.om = pointStream() # body rate p, q, r [rad/s]
        self.act = ctrlStream() # thrust [N] and ctrl srf def [rad]
        self.nrmlzd_act = ctrlStream() # normalized actuators

        # Total velocity
        self.vel.V = []

    def add_msg(self, topic, msg, t):
        """
        Add a piece of data from a ROS message
        """
        if topic == 'position':
            self.pos.add_point(t, msg.point.x, msg.point.y, msg.point.z)
        elif topic == 'velocity':
            self.vel.add_point(t, msg.point.x, msg.point.y, msg.point.z)
            self.vel.V.append(np.sqrt(msg.point.x**2 + msg.point.y**2 + msg.point.z**2))
        elif topic == 'euler':
            self.euler.add_point(t, msg.point.x, msg.point.y, msg.point.z)
        elif topic == 'bodyrate':
            self.om.add_point(t, msg.point.x, msg.point.y, msg.point.z)
        elif topic == 'actuators':
            self.act.add_point(t, msg.controls)
        elif topic == 'target_actuator_control':
            self.nrmlzd_act.add_point(t, msg.controls)


class rompcData:
    """
    A class to extract data from ROMPC topic messages

    Note e_att is either euler angles or axis/angle param depending
    on the type of model used.

    Note e_attrate is either body rates p,q,r or axis/angle rates depending
    on the type of model used.
    """
    def __init__(self):
        self.e_pos = pointStream() # pos error x_r, y_r, z_r [m]
        self.e_vel = pointStream() # body frame vel error [m/s]
        self.e_att = pointStream() # attitude error [rad]
        self.e_attrate = pointStream() # attitude rate error [rad/s]
        self.ubar = ctrlStream() # nominal control minus eq. control
        self.u = ctrlStream() # control minus eq. control
        self.zbar = zStream()
        self.zhat = zStream()
        self.u_prev = ctrlStream() # Control used in state estimator
        self.y = zStream()
        self.qp_solve_time = zStream()

    def add_msg(self, topic, msg, t):
        """
        Add a piece of data from a ROS message
        """
        if topic == 'pos_error':
            self.e_pos.add_point(t, msg.point.x, msg.point.y, msg.point.z)
        elif topic == 'vel_error':
            self.e_vel.add_point(t, msg.point.x, msg.point.y, msg.point.z)
        elif topic == 'att_error':
            self.e_att.add_point(t, msg.point.x, msg.point.y, msg.point.z)
        elif topic == 'attrate_error':
            self.e_attrate.add_point(t, msg.point.x, msg.point.y, msg.point.z)
        elif topic == 'ubar':
            self.ubar.add_point(t, msg.data)
        elif topic == 'u':
            self.u.add_point(t, msg.data)
        elif topic == 'zbar':
            self.zbar.add_point(t, msg.data)
        elif topic == 'zhat':
            self.zhat.add_point(t, msg.data)
        elif topic == 'u_prev':
            self.u_prev.add_point(t, msg.data)
        elif topic == 'y':
            self.y.add_point(t, msg.data)
        elif topic == 'qp_solve_time':
            self.qp_solve_time.add_point(t, [msg.data])


class RosbagData:
    """
    This class extracts rosbag data
    """
    def __init__(self, fpath):
        self.plane = planeData()
        self.rompc = rompcData()
        self.t0 = None

        bag = rosbag.Bag(fpath)
        topics = ['/plane/position', '/plane/velocity', 
                  '/plane/euler', '/plane/bodyrate', 
                  '/plane/actuators', '/rompc/pos_error', 
                  '/rompc/vel_error', '/rompc/att_error',
                  '/rompc/attrate_error',
                  '/rompc/ubar', '/rompc/u', 
                  '/rompc/zbar', '/rompc/zhat',
                  '/rompc/u_prev', '/rompc/y',
                  '/rompc/qp_solve_time',
                  '/mavros/target_actuator_control']
        for topic, msg, t in bag.read_messages(topics=topics):
            self.add_msg(msg, topic)
    
    def extract_time(self, msg):
        t = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def add_msg(self, msg, topic):
        main, sub = topic.split('/')[1:3]
        if sub == 'qp_solve_time':
            t = 0
        else:
            t = self.extract_time(msg)

        if main == 'plane' or main == 'mavros':
            self.plane.add_msg(sub, msg, t)
        elif main == 'rompc':
            self.rompc.add_msg(sub, msg, t)

if __name__ == '__main__':
    data_dir = get_data_dir()
    fpath = join(data_dir, 'rompc.bag')
    data = RosbagData(fpath)
