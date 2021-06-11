from os.path import join
from matplotlib import pyplot as plt
import sys
import numpy as np
import pdb

from utils import get_data_dir, RosbagData

if __name__ == '__main__':
    data_dir = get_data_dir()
    fpath = join(data_dir, 'rompc.bag')
    data = RosbagData(fpath)

    fig, axs = plt.subplots(9, 1, sharex='col')
    axs[0].plot(data.rompc.e_pos.t, data.rompc.e_pos.x, 'b.')
    axs[0].plot(data.rompc.zhat.t, data.rompc.zhat.z[6], 'g.')
    axs[0].set_ylabel('x_r')
    axs[1].plot(data.rompc.e_pos.t, data.rompc.e_pos.y, 'b.')
    axs[1].plot(data.rompc.zhat.t, data.rompc.zhat.z[7], 'g.')
    axs[1].set_ylabel('y_r')
    axs[2].plot(data.rompc.e_pos.t, data.rompc.e_pos.z, 'b.')
    axs[2].plot(data.rompc.zhat.t, data.rompc.zhat.z[8], 'g.')
    axs[2].set_ylabel('z_r')

    axs[3].plot(data.rompc.e_vel.t, data.rompc.e_vel.x, 'b.')
    axs[3].plot(data.rompc.zhat.t, data.rompc.zhat.z[0], 'g.')
    axs[3].set_ylabel('e_u')
    axs[4].plot(data.rompc.e_vel.t, data.rompc.e_vel.y, 'b.')
    axs[4].plot(data.rompc.zhat.t, data.rompc.zhat.z[1], 'g.')
    axs[4].set_ylabel('e_v')
    axs[5].plot(data.rompc.e_vel.t, data.rompc.e_vel.z, 'b.')
    axs[5].plot(data.rompc.zhat.t, data.rompc.zhat.z[2], 'g.')
    axs[5].set_ylabel('e_w')

    axs[6].plot(data.rompc.e_att.t, np.rad2deg(data.rompc.e_att.x), 'b.')
    axs[6].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[3]), 'g.')
    axs[6].set_ylabel('e_phi')
    axs[7].plot(data.rompc.e_att.t, np.rad2deg(data.rompc.e_att.y), 'b.')
    axs[7].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[4]), 'g.')
    axs[7].set_ylabel('e_th')
    axs[8].plot(data.rompc.e_att.t, np.rad2deg(data.rompc.e_att.z), 'b.')
    axs[8].plot(data.rompc.zhat.t, np.rad2deg(data.rompc.zhat.z[5]), 'g.')
    axs[8].set_ylabel('e_psi')

    
    fig2, axs2 = plt.subplots(4, 1, sharex='col')
    axs2[0].plot(data.rompc.u.t, data.rompc.u.u[0], 'b.')
    axs2[0].set_ylabel('T')
    axs2[1].plot(data.rompc.u.t, data.rompc.u.u[1], 'b.')
    axs2[1].set_ylabel('p')
    axs2[2].plot(data.rompc.u.t, data.rompc.u.u[2], 'b.')
    axs2[2].set_ylabel('q')
    axs2[3].plot(data.rompc.u.t, data.rompc.u.u[3], 'b.')
    axs2[3].set_ylabel('r')

    plt.show()

