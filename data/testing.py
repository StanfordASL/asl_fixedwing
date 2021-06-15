from os.path import join, dirname
from matplotlib import pyplot as plt
import sys
import numpy as np
from scipy.interpolate import interp1d
import pdb
sys.path.append('/home/jlorenze/lab/asl_fixedwing/src/utils')

from utils import get_data_dir, RosbagData, aadot_to_om
from rotation import axis_to_quat, quat_to_euler

if __name__ == '__main__':

    data_dir = get_data_dir()
    fpath = join(data_dir, 'rompc.bag')
    data = RosbagData(fpath)

    # y measurements
    v = np.vstack((data.rompc.y.z[0], data.rompc.y.z[1], data.rompc.y.z[2]))
    p = np.vstack((data.rompc.y.z[3], data.rompc.y.z[4], data.rompc.y.z[5]))
    th = np.vstack((data.rompc.y.z[6], data.rompc.y.z[7], data.rompc.y.z[8]))
    t = data.rompc.y.t
    y = np.vstack((v, p, th))

    # control
    u_prev = np.vstack((data.rompc.u_prev.u[0], data.rompc.u_prev.u[1], data.rompc.u_prev.u[2], data.rompc.u_prev.u[3]))
    f = interp1d(data.rompc.u_prev.t, u_prev)
    u = f(t)
    T = u[0,:]
    thd = u[1:,:]

    euler = np.zeros(th.shape)
    for i in range(th.shape[1]):
        euler[:,i] = quat_to_euler(axis_to_quat(th[:,i]))

    om = np.zeros(thd.shape)
    for i in range(om.shape[1]):
        om[:,i] = aadot_to_om(th[:,i], thd[:,i])


    # Simulate
    path = join(dirname(data_dir), 'src/rompc/skywalker/slf')
    A = np.loadtxt(join(path, "A.csv"), delimiter=",")
    B = np.loadtxt(join(path, "B.csv"), delimiter=",")
    C = np.loadtxt(join(path, "C.csv"), delimiter=",")
    H = np.loadtxt(join(path, "H.csv"), delimiter=",")
    L = np.loadtxt(join(path, "L.csv"), delimiter=",")
    n = A.shape[0]
    o = H.shape[0]

    N = 100
    xhat = np.zeros(n)
    zhat = np.zeros((o, N))
    AL = A - np.matmul(L, C)
    for i in range(N):
        zhat[:,i] = np.matmul(H, xhat)
        dt = t[i+1] - t[i]
        xhat = np.linalg.solve(np.eye(n) - dt*AL, xhat + dt*np.matmul(B, u[:,i]) + dt*np.matmul(L, y[:,i]))
        # xhat += dt*(np.matmul(A, xhat) + np.matmul(B, u[:,i]) + np.matmul(L, y[:,i] - np.matmul(C, xhat)))

    pdb.set_trace()


    # plt.figure()
    # plt.plot(t, np.rad2deg(euler[0,:]), label='Roll')
    # plt.plot(t, np.rad2deg(euler[1,:]), label='Pitch')
    # plt.plot(t, np.rad2deg(euler[2,:]), label='Yaw')
    # plt.ylabel('deg')
    # plt.legend()

    # plt.figure()
    # plt.plot(t, np.rad2deg(om[0,:]), label='Roll Rate, p')
    # plt.plot(t, np.rad2deg(om[1,:]), label='Pitch Rate, q')
    # plt.plot(t, np.rad2deg(om[2,:]), label='Yaw Rate, r')
    # plt.ylabel('deg/s')
    # plt.legend()

    # plt.figure()
    # plt.plot(t, thd[0,:], 'r', label='thxd')
    # plt.plot(t, thd[1,:], 'g', label='thyd')
    # plt.plot(t, thd[2,:], 'b', label='thzd')
    # plt.ylabel('rad/s')
    # plt.legend()

    # plt.figure()
    # plt.plot(t, T, label='Thrust')
    # plt.ylabel('N')
    # plt.legend()

    # plt.figure()
    # plt.plot(t, v[0,:], label='xd')
    # plt.plot(t, v[1,:], label='yd')
    # plt.plot(t, v[2,:], label='zd')
    # plt.ylabel('m/s')
    # plt.legend()


    plt.show()
