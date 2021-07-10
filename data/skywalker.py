from os.path import join, isfile, dirname
from matplotlib import pyplot as plt
import sys
import numpy as np
from scipy.interpolate import interp1d
import pdb

from utils import get_data_dir, RosbagData, get_models_dir, get_utils_dir

if __name__ == '__main__':
    if len(sys.argv) == 2:
        t0 = tf = None
    elif len(sys.argv) == 4:
        t0 = float(sys.argv[2])
        tf = float(sys.argv[3])
    else:
        print('Correct usage: python skywalker.py {filename}.bag')
        print('or: python skywalker.py {filename}.bag t0 t1')
        sys.exit()

    data_dir = get_data_dir()
    fpath = join(data_dir, sys.argv[1])
    if not isfile(fpath):
        print('%s was not found' % fpath)
        sys.exit()
    data = RosbagData(fpath)

    # Find beginning and end of flight
    if t0 is None:
        idx = np.where(np.abs(np.array(data.plane.act.u[3])) > 0.0)[0]
        t0 = data.plane.act.t[idx[0]]
        tf = data.plane.act.t[idx[-1]]

    # Compute thrust estimate
    sys.path.append(join(get_models_dir(), "skywalker"))
    from sysid import aircraft_ctrl_params, thrust
    p = aircraft_ctrl_params()
    c_T0, c_TVom = p[0], p[1]
    T_est = []
    f = interp1d(data.plane.vel.t, data.plane.vel.x, bounds_error=False, fill_value='extrapolate')
    for t, u_T in zip(data.plane.nrmlzd_act.t, data.plane.nrmlzd_act.u[3]):
        # if t > 170:
        #     pdb.set_trace()
        if u_T < .0001:
            T_est.append(0.0)
        else:
            T_est.append(thrust(u_T, c_T0, c_TVom, f(t)))

    # Load equilibrium thrust
    fpath = join(dirname(data_dir), 'src/rompc/skywalker/slf/u_eq.csv')
    u_eq = np.loadtxt(fpath, delimiter=",")
    T_rompc = np.array(data.rompc.u.u[0]) + u_eq[0]

    # Convert axis/angle to euler angles relative to target frame
    sys.path.append(get_utils_dir())
    from rotation import aadot_to_om, axis_to_quat, quat_to_euler, quat_to_axis, euler_to_quat
    e_att = np.vstack((data.rompc.e_att.x, data.rompc.e_att.y, data.rompc.e_att.z))
    euler_rompc = np.empty((0,3))
    for i, t in enumerate(data.rompc.e_att.t):
        e_att_euler = quat_to_euler(axis_to_quat(e_att[:,i]))
        euler_rompc = np.vstack((euler_rompc, e_att_euler))

    # Convert axis/angle rate commands to body rates
    # NOTE THAT THIS ASSUMES THE TARGET ANGULAR VELOCITY IS ZERO
    e_att = np.vstack((data.rompc.e_att.x, data.rompc.e_att.y, data.rompc.e_att.z))
    f = interp1d(data.rompc.e_att.t, e_att, bounds_error=False, fill_value='extrapolate')
    om_rompc = np.empty((0,3))
    for i, t in enumerate(data.rompc.u.t):
        aa_dot = np.array([data.rompc.u.u[1][i], 
                           data.rompc.u.u[2][i], 
                           data.rompc.u.u[3][i]])
        om = aadot_to_om(f(t), aa_dot)
        om_rompc = np.vstack((om_rompc, om))

    # Plot plane related data streams
    fig, axs1 = plt.subplots(4, 1, sharex='col')
    axs1[0].plot(data.plane.pos.t, data.plane.pos.x, 'r', label='plane.x')
    axs1[0].plot(data.plane.pos.t, data.plane.pos.y, 'g', label='plane.y')
    axs1[0].plot(data.plane.pos.t, data.plane.pos.z, 'b', label='plane.z')
    axs1[0].set_ylabel('Position [m]')

    axs1[1].plot(data.plane.vel.t, data.plane.vel.x, 'r', label='plane.u')
    axs1[1].plot(data.plane.vel.t, data.plane.vel.y, 'g', label='plane.v')
    axs1[1].plot(data.plane.vel.t, data.plane.vel.z, 'b', label='plane.w')
    axs1[1].set_ylabel('Velocity [m/s]')

    axs1[2].plot(data.plane.euler.t, np.rad2deg(data.plane.euler.x), 'r', label='plane.roll')
    axs1[2].plot(data.plane.euler.t, np.rad2deg(data.plane.euler.y), 'g', label='plane.pitch')
    axs1[2].plot(data.plane.euler.t, np.rad2deg(data.plane.euler.z), 'b', label='plane.yaw')
    axs1[2].set_ylabel('Euler Angles [deg]')

    axs1[3].plot(data.plane.om.t, np.rad2deg(data.plane.om.x), 'r', label='plane.p')
    axs1[3].plot(data.plane.om.t, np.rad2deg(data.plane.om.y), 'g', label='plane.q')
    axs1[3].plot(data.plane.om.t, np.rad2deg(data.plane.om.z), 'b', label='plane.r')
    axs1[3].set_ylabel('Angular Velocity [deg/s]')

    for ax in axs1:
        ax.legend()
    axs1[0].set_xlim(t0, tf)


    fig, axs2 = plt.subplots(4, 1, sharex='col')
    
    axs2[0].plot(data.plane.nrmlzd_act.t, data.plane.nrmlzd_act.u[3], 'r', label='plane.thrust')
    axs2[0].set_ylabel('Throttle')

    axs2[1].plot(data.plane.act.t, np.rad2deg(data.plane.act.u[0]), 'r', label='plane.aileron')
    axs2[1].plot(data.plane.act.t, np.rad2deg(data.plane.act.u[1]), 'g', label='plane.elevator')
    axs2[1].plot(data.plane.act.t, np.rad2deg(data.plane.act.u[2]), 'b', label='plane.rudder')
    axs2[1].set_ylabel('Ctrl Srf Dflct [deg]')

    axs2[2].plot(data.plane.vel.t, data.plane.vel.V, 'r', label='plane.V')
    axs2[2].set_ylabel('Velocity [m/s]')

    axs2[3].plot(data.plane.nrmlzd_act.t, T_est, 'r')
    axs2[3].set_ylabel('Thrust Estimate [N]')


    for ax in axs2:
        ax.legend()
    axs2[0].set_xlim(t0, tf)


    # Plot plane related data streams
    fig, axs3 = plt.subplots(4, 1, sharex='col')
    axs3[0].plot(data.rompc.e_pos.t, data.rompc.e_pos.x, 'r', label='rompc.e_x')
    axs3[0].plot(data.rompc.e_pos.t, data.rompc.e_pos.y, 'g', label='rompc.e_y')
    axs3[0].plot(data.rompc.e_pos.t, data.rompc.e_pos.z, 'b', label='rompc.e_z')
    axs3[0].plot(data.rompc.zhat.t, data.rompc.zhat.z[3], 'r*', label='rompc.e_x_est')
    axs3[0].plot(data.rompc.zhat.t, data.rompc.zhat.z[4], 'g*', label='rompc.e_y_est')
    axs3[0].plot(data.rompc.zhat.t, data.rompc.zhat.z[5], 'b*', label='rompc.e_z_est')
    axs3[0].set_ylabel('Position error [m]')

    axs3[1].plot(data.rompc.e_vel.t, data.rompc.e_vel.x, 'r', label='rompc.e_xd')
    axs3[1].plot(data.rompc.e_vel.t, data.rompc.e_vel.y, 'g', label='rompc.e_yd')
    axs3[1].plot(data.rompc.e_vel.t, data.rompc.e_vel.z, 'b', label='rompc.e_zd')
    axs3[1].plot(data.rompc.zhat.t, data.rompc.zhat.z[0], 'r*', label='rompc.e_xd_est')
    axs3[1].plot(data.rompc.zhat.t, data.rompc.zhat.z[1], 'g*', label='rompc.e_yd_est')
    axs3[1].plot(data.rompc.zhat.t, data.rompc.zhat.z[2], 'b*', label='rompc.e_zd_est')
    axs3[1].set_ylabel('Velocity error [m/s]')

    axs3[2].plot(data.rompc.e_att.t, np.rad2deg(data.rompc.e_att.x), 'r', label='rompc.e_att_x')
    axs3[2].plot(data.rompc.e_att.t, np.rad2deg(data.rompc.e_att.y), 'g', label='rompc.e_att_y')
    axs3[2].plot(data.rompc.e_att.t, np.rad2deg(data.rompc.e_att.z), 'b', label='rompc.e_att_z')
    axs3[2].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[6]), 'r*', label='rompc.e_att_x_est')
    axs3[2].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[7]), 'g*', label='rompc.e_att_y_est')
    axs3[2].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[8]), 'b*', label='rompc.e_att_z_est')
    axs3[2].set_ylabel('Attitude error [deg]')

    axs3[3].plot(data.rompc.e_att.t, np.rad2deg(euler_rompc[:,0]), 'r', label='rompc.e_att_phi')
    axs3[3].plot(data.rompc.e_att.t, np.rad2deg(euler_rompc[:,1]), 'g', label='rompc.e_att_th')
    axs3[3].plot(data.rompc.e_att.t, np.rad2deg(euler_rompc[:,2]), 'b', label='rompc.e_att_psi')
    axs3[3].set_ylabel('Euler angle error [deg]')

    for ax in axs3:
        ax.legend()
    axs3[0].set_xlim(t0, tf)

    fig, axs4 = plt.subplots(2, 1, sharex='col')
    
    axs4[0].plot(data.rompc.u.t, T_rompc, 'r', label='rompc.u[0]')
    axs4[0].set_ylabel('Thrust Command [N]')

    axs4[1].plot(data.rompc.u.t, np.rad2deg(om_rompc[:,0]), 'r', label='rompc.p')
    axs4[1].plot(data.rompc.u.t, np.rad2deg(om_rompc[:,1]), 'g', label='rompc.q')
    axs4[1].plot(data.rompc.u.t, np.rad2deg(om_rompc[:,2]), 'b', label='rompc.r')
    axs4[1].set_ylabel('Body Rate Command [deg/s]')

    for ax in axs4:
        ax.legend()
    axs4[0].set_xlim(t0, tf)

    plt.show()

