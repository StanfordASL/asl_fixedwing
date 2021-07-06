from os.path import join, isfile
from matplotlib import pyplot as plt
import sys
import numpy as np
import pdb

from utils import get_data_dir, RosbagData, aadot_to_om

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print('Correct usage: python skywalker.py {filename}.bag')
        sys.exit()

    data_dir = get_data_dir()
    fpath = join(data_dir, sys.argv[1])
    if not isfile(fpath):
        print('%s was not found' % fpath)
        sys.exit()
    data = RosbagData(fpath)

    # Find beginning and end of flight
    idx = np.where(np.abs(np.array(data.plane.act.u[3])) > 0.0)[0]
    t0 = data.plane.act.t[idx[0]]
    tf = data.plane.act.t[idx[-1]]

    pdb.set_trace()

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

    axs2[3].plot(data.plane.act.t, data.plane.act.u[3], 'r', label='plane.thrust')
    axs2[3].set_ylabel('Thrust Estimate [N]')


    for ax in axs2:
        ax.legend()
    axs2[0].set_xlim(t0, tf)



    # axs[0][0].plot(data.rompc.e_pos.t, data.rompc.e_pos.x, 'b.')
    # axs[1][0].plot(data.rompc.e_pos.t, data.rompc.e_pos.y, 'b.')
    # axs[2][0].plot(data.rompc.e_pos.t, data.rompc.e_pos.z, 'b.')
    # if sys.argv[1] == 'std':
    #     axs[0][0].plot(data.rompc.zhat.t, data.rompc.zhat.z[6], 'g.')
    #     axs[1][0].plot(data.rompc.zhat.t, data.rompc.zhat.z[7], 'g.')
    #     axs[2][0].plot(data.rompc.zhat.t, data.rompc.zhat.z[8], 'g.')
    # elif sys.argv[1] == 'cfd':
    #     axs[0][0].plot(data.rompc.zhat.t, data.rompc.zhat.z[3], 'g.')
    #     axs[1][0].plot(data.rompc.zhat.t, data.rompc.zhat.z[4], 'g.')
    #     axs[2][0].plot(data.rompc.zhat.t, data.rompc.zhat.z[5], 'g.')
    # axs[0][0].set_ylabel('x_r')
    # axs[1][0].set_ylabel('y_r')
    # axs[2][0].set_ylabel('z_r')


    # axs[3][0].plot(data.rompc.e_vel.t, data.rompc.e_vel.x, 'b.')
    # axs[4][0].plot(data.rompc.e_vel.t, data.rompc.e_vel.y, 'b.')
    # axs[5][0].plot(data.rompc.e_vel.t, data.rompc.e_vel.z, 'b.')
    # axs[3][0].plot(data.rompc.zhat.t, data.rompc.zhat.z[0], 'g.')
    # axs[4][0].plot(data.rompc.zhat.t, data.rompc.zhat.z[1], 'g.')
    # axs[5][0].plot(data.rompc.zhat.t, data.rompc.zhat.z[2], 'g.')
    # if sys.argv[1] == 'std':
    #     axs[3][0].set_ylabel('e_u')
    #     axs[4][0].set_ylabel('e_v')
    #     axs[5][0].set_ylabel('e_w')
    # elif sys.argv[1] == 'cfd':
    #     axs[3][0].set_ylabel('xd_r')
    #     axs[4][0].set_ylabel('yd_r')
    #     axs[5][0].set_ylabel('zd_r')
    

    # axs[6][0].plot(data.rompc.e_att.t, np.rad2deg(data.rompc.e_att.x), 'b.')
    # axs[7][0].plot(data.rompc.e_att.t, np.rad2deg(data.rompc.e_att.y), 'b.')
    # axs[8][0].plot(data.rompc.e_att.t, np.rad2deg(data.rompc.e_att.z), 'b.')
    # if sys.argv[1] == 'std':
    #     axs[6][0].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[3]), 'g.')
    #     axs[7][0].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[4]), 'g.')
    #     axs[8][0].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[5]), 'g.')
    #     axs[6][0].set_ylabel('e_phi')
    #     axs[7][0].set_ylabel('e_th')
    #     axs[8][0].set_ylabel('e_psi')
    # elif sys.argv[1] == 'cfd':
    #     axs[6][0].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[6]), 'g.')
    #     axs[7][0].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[7]), 'g.')
    #     axs[8][0].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[8]), 'g.')
    #     axs[6][0].set_ylabel('e_th1')
    #     axs[7][0].set_ylabel('e_th2')
    #     axs[8][0].set_ylabel('e_th3')



    # axs[0][1].plot(data.rompc.u.t, data.rompc.u.u[0], 'b.', label='ROMPC')
    # if sys.argv[1] == 'std':
    #     axs[1][1].plot(data.rompc.u.t, np.rad2deg(data.rompc.u.u[1]), 'b.', label='ROMPC')
    #     axs[2][1].plot(data.rompc.u.t, np.rad2deg(data.rompc.u.u[2]), 'b.', label='ROMPC')
    #     axs[3][1].plot(data.rompc.u.t, np.rad2deg(data.rompc.u.u[3]), 'b.', label='ROMPC')

    plt.legend()

    plt.show()

