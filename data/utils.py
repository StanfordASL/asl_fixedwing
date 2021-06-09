from os.path import dirname, abspath, join, isfile, isdir
from os import remove, mkdir
import sys
import rosbag
import pdb

def get_data_dir():
    return dirname(abspath(__file__))


class posStream:
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


class velStream:
    def __init__(self):
        self.u = []
        self.v = []
        self.w = []
        self.t = []

    def add_point(self, t, u, v, w):
        self.u.append(u)
        self.v.append(v)
        self.w.append(w)
        self.t.append(t)


class eulerStream:
    def __init__(self):
        self.phi = []
        self.th = []
        self.psi = []
        self.t = []

    def add_point(self, t, phi, th, psi):
        self.phi.append(phi)
        self.th.append(th)
        self.psi.append(psi)
        self.t.append(t)


class omStream:
    def __init__(self):
        self.p = []
        self.q = []
        self.r = []
        self.t = []

    def add_point(self, t, p, q, r):
        self.p.append(p)
        self.q.append(q)
        self.r.append(r)
        self.t.append(t)


class ctrlStream:
    def __init__(self):
        self.u = [[], [], [], []]
        self.t = []

    def add_point(self, t, u):
        self.t.append(t)
        for i in range(4):
            self.u[i].append(u[i])


class planeData:
    """
    A class to extract data from Plane topic messages
    """
    def __init__(self):
        self.pos = posStream() # inertial pos x_i, y_i, z_i [m]
        self.vel = velStream() # body frame vel u, v, w [m/s]
        self.euler = eulerStream() # euler angle phi, th, psi [deg]
        self.om = omStream() # body rate p, q, r [deg/s]
        self.act = ctrlStream() # thrust [N] and ctrl srf def [deg]

    def add_msg(self, topic, msg, t):
        """
        Add a piece of data from a ROS message
        """
        if topic == 'position':
            self.pos.add_point(t, msg.x, msg.y, msg.z)
        elif topic == 'velocity':
            self.vel.add_point(t, msg.x, msg.y, msg.z)
        elif topic == 'euler':
            self.euler.add_point(t, msg.x, msg.y, msg.z)
        elif topic == 'bodyrate':
            self.om.add_point(t, msg.x, msg.y, msg.z)
        elif topic == 'actuators':
            self.act.add_point(t, msg.controls)


class rompcData:
    """
    A class to extract data from ROMPC topic messages
    """
    def __init__(self):
        self.pos = posStream() # inertial pos x_i, y_i, z_i [m]
        self.vel = velStream() # body frame vel u, v, w [m/s]
        self.euler = eulerStream() # euler angle phi, th, psi [deg]
        self.om = omStream() # body rate p, q, r [deg/s]
        self.act = ctrlStream() # thrust [N] and ctrl srf def [deg]

    def add_msg(self, topic, msg, t):
        """
        Add a piece of data from a ROS message
        """
        if topic == 'position':
            self.pos.add_point(t, msg.x, msg.y, msg.z)
        elif topic == 'velocity':
            self.vel.add_point(t, msg.x, msg.y, msg.z)
        elif topic == 'euler':
            self.euler.add_point(t, msg.x, msg.y, msg.z)
        elif topic == 'bodyrate':
            self.om.add_point(t, msg.x, msg.y, msg.z)
        elif topic == 'actuators':
            self.act.add_point(t, msg.controls)


class RosbagData:
    """
    This class extracts rosbag data
    """
    def __init__(self, fpath):
        self.plane = planeData()
        #self.rompc = ctrlData()
        self.t0 = None

        bag = rosbag.Bag(fpath)
        topics = ['/plane/position', '/plane/velocity', 
                  '/plane/euler', '/plane/bodyrate', 
                  '/plane/actuators']
        for topic, msg, t in bag.read_messages(topics=topics):
            self.add_msg(t, msg, topic)

    def extract_time(self, msg):
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def add_msg(self, t, msg, topic):
        t = t.secs + t.nsecs/1e9
        if self.t0 is None:
            self.t0 = t;
        t -= self.t0
        main, sub = topic.split('/')[1:3]
        if main == 'plane':
            self.plane.add_msg(sub, msg, t)
        elif main == 'rompc':
            self.rompc.add_msg(sub, msg, t)

if __name__ == '__main__':
    data_dir = get_data_dir()
    fpath = join(data_dir, 'rompc.bag')
    data = RosbagData(fpath)
    pdb.set_trace()
